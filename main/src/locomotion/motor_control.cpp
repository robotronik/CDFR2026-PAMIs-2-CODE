#include "locomotion/motor_control.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>

static const char* LOGGER_TAG = "MotorControl";

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define WHEEL_DIST 88.0f // distance between the two wheels in mm, TODO: measure this
#define WHEEL_RADIUS 13.0f // radius of the wheels in mm, TODO: measure this
#define WHEEL_CIRCUMFERENCE (2.0f * M_PI * WHEEL_RADIUS) // circumference of the wheels in mm

namespace {
constexpr float POSITION_EPS_MM = 8.0f;
constexpr float HEADING_ALIGN_EPS_DEG = 10.0f;
constexpr float FINAL_ANGLE_EPS_DEG = 4.0f;

// Rotation PID with angular error in deg and output in motor speed percentage.
constexpr float KP_ROT = 4.0f; // % per deg 
constexpr float KI_ROT = 0.0f;  // % per deg*s
constexpr float KD_ROT = 0.20f; // % per deg/s

// Translation PID with distance error in mm and output in motor speed percentage.
constexpr float KP_LIN = 4.0f;  // % per mm
constexpr float KI_LIN = 0.2f;  // % per mm*s
constexpr float KD_LIN = 1.0f;  // % per mm/s

// Heading correction while translating (heading error in deg).
constexpr float KP_STEER = 2.0f; // % per deg
constexpr float KI_STEER = 0.0f;  // % per deg*s
constexpr float KD_STEER = 0.08f; // % per deg/s

// Speed values are motor command percentages in [-100, 100].
constexpr float MAX_TRANSLATION_SPEED = 80.0f;
constexpr float MIN_TRANSLATION_SPEED = 22.0f;
constexpr float MAX_ROTATION_SPEED = 75.0f;
constexpr float MAX_MOTOR_SPEED = 100.0f;
// Integral clamp for both translation and heading loops (error * second units).
constexpr float MAX_INTEGRAL = 400.0f;
}

float MotorControl::normalize_angle_deg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float MotorControl::clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void MotorControl::reset_pid() {
    lin_integral = 0.0f;
    lin_prev_error = 0.0f;
    ang_integral = 0.0f;
    ang_prev_error = 0.0f;
}

MotorControl::MotorControl() 
   : encoder_a(PIN_HALL_A1, PIN_HALL_A2),
     encoder_b(PIN_HALL_B1, PIN_HALL_B2),
     motor_a(PIN_DC_A1, PIN_DC_A2),
      motor_b(PIN_DC_B1, PIN_DC_B2),
      target_pos{0.0f, 0.0f, 0.0f},
      current_pos{0.0f, 0.0f, 0.0f},
      has_target(false),
      lin_integral(0.0f),
      lin_prev_error(0.0f),
      ang_integral(0.0f),
      ang_prev_error(0.0f),
      last_control_us(0)
{
   ESP_LOGD(LOGGER_TAG, "init");
}

void MotorControl::move(coords_t new_target) {
    ESP_LOGD(LOGGER_TAG, "Received move order, coords: x: %f, y: %f, angle: %f", new_target.x, new_target.y, new_target.angle);

    // TODO: safety checks (bounds)
    target_pos = new_target;
    target_pos.angle = fmodf(target_pos.angle, 360.0f);
    if (target_pos.angle < 0.0f) {
        target_pos.angle += 360.0f;
    }

    has_target = true;
    reset_pid();

    // Start motion cleanly from this move command.
    last_control_us = esp_timer_get_time();
}

void MotorControl::update() {
    float count_to_mm = 2.0f * WHEEL_CIRCUMFERENCE / 1050.0f; // convert count to mm
    float delta_right = encoder_a.get_delta() * count_to_mm;
    float delta_left = encoder_b.get_delta() * count_to_mm;

    float heading_rad = current_pos.angle * DEG_TO_RAD;
    float delta_heading_rad = (delta_right - delta_left) / WHEEL_DIST;
    float delta_center_mm = (delta_left + delta_right) * 0.5f;

    // Midpoint integration with 0 deg aligned to +X (standard frame): x=cos(theta), y=sin(theta).
    float heading_mid_rad = heading_rad + (delta_heading_rad * 0.5f);
    current_pos.x += delta_center_mm * cos(heading_mid_rad);
    current_pos.y += delta_center_mm * sin(heading_mid_rad);
    current_pos.angle += delta_heading_rad * RAD_TO_DEG;

    if (current_pos.angle >= 360.0f || current_pos.angle < 0.0f) {
        current_pos.angle = fmodf(current_pos.angle, 360.0f);
        if (current_pos.angle < 0.0f) {
            current_pos.angle += 360.0f;
        }
    }

    // ESP_LOGI(LOGGER_TAG, "delta_left: %.2f mm, delta_right: %.2f mm, delta_heading: %.2f deg", delta_left, delta_right, delta_heading_rad * RAD_TO_DEG);
    // ESP_LOGI(LOGGER_TAG, "Position updated: x: %.2f mm, y: %.2f mm, angle: %.2f deg", current_pos.x, current_pos.y, current_pos.angle);

    if (!has_target) {
        motor_a.set_speed(0.0f);
        motor_b.set_speed(0.0f);
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    float dt_s = 0.01f;
    if (last_control_us != 0) {
        dt_s = (now_us - last_control_us) / 1000000.0f;
        dt_s = clamp(dt_s, 0.001f, 0.1f);
    }
    last_control_us = now_us;

    const float dx = target_pos.x - current_pos.x;
    const float dy = target_pos.y - current_pos.y;
    const float distance_error = sqrtf((dx * dx) + (dy * dy));

    const float angle_to_point = atan2f(dy, dx) * RAD_TO_DEG;
    const float heading_error = normalize_angle_deg(angle_to_point - current_pos.angle);
    const float final_angle_error = normalize_angle_deg(target_pos.angle - current_pos.angle);

    // Stage 1: rotate toward path. Stage 2: move while steering. Stage 3: final orientation on target angle.
    bool at_target_position = distance_error <= POSITION_EPS_MM;
    bool do_rotation_only = at_target_position || (fabsf(heading_error) > HEADING_ALIGN_EPS_DEG);

    if (at_target_position && fabsf(final_angle_error) <= FINAL_ANGLE_EPS_DEG) {
        has_target = false;
        reset_pid();
        motor_a.set_speed(0.0f);
        motor_b.set_speed(0.0f);
        ESP_LOGI(LOGGER_TAG, "Target reached at x: %.1f, y: %.1f, angle: %.1f", current_pos.x, current_pos.y, current_pos.angle);
        return;
    }

    float left_speed = 0.0f;
    float right_speed = 0.0f;

    if (do_rotation_only) {
        float rot_error = at_target_position ? final_angle_error : heading_error;

        ang_integral += rot_error * dt_s;
        ang_integral = clamp(ang_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        float ang_derivative = (rot_error - ang_prev_error) / dt_s;
        ang_prev_error = rot_error;

        float rot_cmd = (KP_ROT * rot_error) + (KI_ROT * ang_integral) + (KD_ROT * ang_derivative);
        rot_cmd = clamp(rot_cmd, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        left_speed = -rot_cmd;
        right_speed = rot_cmd;

        // Do not carry linear integral/derivative through pure rotation phases.
        lin_integral = 0.0f;
        lin_prev_error = 0.0f;
    } else {
        lin_integral += distance_error * dt_s;
        lin_integral = clamp(lin_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        float lin_derivative = (distance_error - lin_prev_error) / dt_s;
        lin_prev_error = distance_error;

        float lin_cmd = (KP_LIN * distance_error) + (KI_LIN * lin_integral) + (KD_LIN * lin_derivative);
        lin_cmd = clamp(lin_cmd, 0.0f, MAX_TRANSLATION_SPEED);
        if (lin_cmd > 0.0f && lin_cmd < MIN_TRANSLATION_SPEED) {
            lin_cmd = MIN_TRANSLATION_SPEED;
        }

        ang_integral += heading_error * dt_s;
        ang_integral = clamp(ang_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        float ang_derivative = (heading_error - ang_prev_error) / dt_s;
        ang_prev_error = heading_error;

        float steer_cmd = (KP_STEER * heading_error) + (KI_STEER * ang_integral) + (KD_STEER * ang_derivative);
        steer_cmd = clamp(steer_cmd, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        left_speed = lin_cmd - steer_cmd;
        right_speed = lin_cmd + steer_cmd;
    }

    left_speed = clamp(left_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    right_speed = clamp(right_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    motor_a.set_speed(right_speed);
    motor_b.set_speed(left_speed);
}

void MotorControl::start() {
    encoder_a.start();
    encoder_b.start();
    motor_a.start();
    motor_b.start();

    ESP_LOGD(LOGGER_TAG, "start");
}

void MotorControl::stop() {
    encoder_a.stop();
    encoder_b.stop();
    motor_a.stop();
    motor_b.stop();

    ESP_LOGD(LOGGER_TAG, "stop");
}
