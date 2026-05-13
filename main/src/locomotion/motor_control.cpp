#include "locomotion/motor_control.h"
#include "main.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>

static const char* LOGGER_TAG = "MotorControl";

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define WHEEL_DIST 88.5f // distance between the two wheels in mm
#define WHEEL_RADIUS 12.0f // radius of the wheels in mm
#define WHEEL_CIRCUMFERENCE (2.0f * M_PI * WHEEL_RADIUS) // circumference of the wheels in mm

namespace {
    constexpr float POSITION_EPS_MM = 5.0f;
    constexpr float APPROACH_EPS_MM = 80.0f; 
    constexpr float HEADING_ALIGN_EPS_DEG = 7.0f;
    constexpr float FINAL_ANGLE_EPS_DEG = 1.0f;

    // Rotation PD-control with angular error in deg and output in motor speed percentage.
    constexpr float KP_ROT = 1.2f; // % per deg
    constexpr float KD_ROT = 0.005f;

    // Translation PD-control with distance error in mm and output in motor speed percentage.
    constexpr float KP_LIN = 1.05f;  // % per mm
    constexpr float KD_LIN = 0.01f;

    // Heading correction while translating (heading error in deg).
    constexpr float KP_STEER = 2.0f; // % per deg
    constexpr float KD_STEER = 0.05f;

    // Derivative low pass filter 
    constexpr float ALPHA = 0.2f;

    // Ramp
    constexpr float SPEED_STEP = 0.5f;

    // Speed values are motor command percentages in [-100, 100].
    constexpr float MAX_TRANSLATION_SPEED = 80.0f;
    constexpr float MAX_ROTATION_SPEED = 40.0f;

    // Physical tune
    #ifdef NINJA
    constexpr float LEFT_WHEEL_TUNE = 1.03f;
    #else
    constexpr float LEFT_WHEEL_TUNE = 1.0f;
    #endif

    // Timeout
    constexpr int64_t TIMEOUT = 5000000.0f; // in us
}


MotorControl::MotorControl() 
    : motor_a(PIN_DC_A1, PIN_DC_A2, PIN_HALL_A1, PIN_HALL_A2),
      motor_b(PIN_DC_B1, PIN_DC_B2, PIN_HALL_B1, PIN_HALL_B2),
    target_pos{0.0f, 0.0f, 0.0f},
    current_pos{0.0f, 0.0f, 0.0f},
    has_target(false),
    doing_final_rotation(false),
    turn_end(false),
    last_control_us(0),
    INVERTED_LEFT_MOTOR(false)
{
   ESP_LOGD(LOGGER_TAG, "init");
}

// Should only be used to set initial coords
void MotorControl::set_coords(coords_t coords) {
    current_pos.x = coords.x;
    current_pos.y = coords.y;
    current_pos.angle = coords.angle;
}

coords_t MotorControl::get_coords() {
    return current_pos;
}

bool MotorControl::goTo(coords_t new_target, bool turnEnd, bool reverse) {
    ESP_LOGD(LOGGER_TAG, "Received goTo order, coords: x: %f, y: %f, angle: %f, turnEnd: %d", new_target.x, new_target.y, new_target.angle, (int)turnEnd);

    // set and normalize
    target_pos = new_target;
    target_pos.angle = fmodf(target_pos.angle, 360.0f);
    if (target_pos.angle < 0.0f) {
        target_pos.angle += 360.0f;
    }

    has_target = true;
    doing_final_rotation = false;
    turn_end = turnEnd;
    is_reversed = reverse;

    previous_lin_cmd = 0.0f;
    previous_rot_cmd = 0.0f;
    previous_steer_cmd = 0.0f;

    // Start motion cleanly from this command.
    last_control_us = esp_timer_get_time();
    begin_us = esp_timer_get_time();

    // Perform one step of control immediately
    return goTo(turnEnd);
}

bool MotorControl::goTo(bool turnEnd) {
    float delta_right = motor_a.get_delta() * WHEEL_CIRCUMFERENCE;
    float delta_left = motor_b.get_delta() * WHEEL_CIRCUMFERENCE * LEFT_WHEEL_TUNE;
    if (INVERTED_LEFT_MOTOR) delta_left *= -1;

    float heading_rad = current_pos.angle * DEG_TO_RAD;
    float delta_heading_rad = (delta_right - delta_left) / WHEEL_DIST;
    float delta_center_mm = (delta_left + delta_right) * 0.5f;

    float heading_mid_rad = heading_rad + (delta_heading_rad * 0.5f);
    current_pos.x += delta_center_mm * cos(heading_mid_rad);
    current_pos.y += delta_center_mm * sin(heading_mid_rad);
    current_pos.angle += delta_heading_rad * RAD_TO_DEG;

    current_pos.angle = normalize_angle_deg(current_pos.angle); 

    if (!has_target) {
        motor_a.set_speed_pid(0.0f);
        motor_b.set_speed_pid(0.0f);
        return true;
    }

    const int64_t now_us = esp_timer_get_time();
    float dt_s = 0.01f;
    if (last_control_us != 0) {
        dt_s = (now_us - last_control_us) / 1000000.0f;
        dt_s = clamp(dt_s, 0.001f, 0.1f);
    }
    last_control_us = now_us; 

    if(now_us - begin_us > TIMEOUT) {
        has_target = false;
    }

    const float dx = target_pos.x - current_pos.x;
    const float dy = target_pos.y - current_pos.y;
    const float distance_error = sqrtf((dx * dx) + (dy * dy));

    float angle_to_point = atan2f(dy, dx) * RAD_TO_DEG;

    // Point our back towards the target instead of our front if we want to go backwards
    if(is_reversed) {
        angle_to_point = normalize_angle_deg(angle_to_point + 180.0f);
    }

    // Do not modify our approach angle if our robot is close enough
    if(!(distance_error <= APPROACH_EPS_MM)) {
        approach_angle = angle_to_point;
    }

    const float heading_error = normalize_angle_deg(approach_angle - current_pos.angle);
    const float final_angle_error = normalize_angle_deg(target_pos.angle - current_pos.angle);

    bool at_target_position = doing_final_rotation ? true : distance_error <= POSITION_EPS_MM;
    bool do_rotation_only = at_target_position || (fabsf(heading_error) > HEADING_ALIGN_EPS_DEG);
    if(at_target_position && !doing_final_rotation) {
        doing_final_rotation = true;
    }

    // Arrival logic: if not required to turn at the end, accept position-only arrival.
    if (at_target_position) {
        if (!turnEnd && !turn_end) {
            has_target = false;
            doing_final_rotation = false;
            motor_a.set_speed_pid(0.0f);
            motor_b.set_speed_pid(0.0f);
            ESP_LOGI(LOGGER_TAG, "Target reached (pos) at x: %.1f, y: %.1f, angle: %.1f", current_pos.x, current_pos.y, current_pos.angle);
            return true;
        }
        // if turn at end required, only finish when final angle within epsilon
        if (fabsf(final_angle_error) <= FINAL_ANGLE_EPS_DEG) {
            has_target = false;
            doing_final_rotation = false;
            motor_a.set_speed_pid(0.0f);
            motor_b.set_speed_pid(0.0f);
            ESP_LOGI(LOGGER_TAG, "Target reached at x: %.1f, y: %.1f, angle: %.1f", current_pos.x, current_pos.y, current_pos.angle);
            return true;
        }
    }

    float left_speed = 0.0f;
    float right_speed = 0.0f;

    if (do_rotation_only) {
        // PD rotation control
        float rot_error = at_target_position ? final_angle_error : heading_error;

        float rot_derivative = (rot_error - last_rot_error) / dt_s;
        rot_derivative = ((1.0f - ALPHA) * last_rot_derivative) + (ALPHA * rot_derivative);
        last_rot_derivative = rot_derivative;
        last_rot_error = rot_error;

        float rot_cmd = (KP_ROT * rot_error + KD_ROT * rot_derivative);
        /* Speed ramp */
        if(rot_cmd > previous_rot_cmd + SPEED_STEP) {
            rot_cmd = previous_rot_cmd + SPEED_STEP;
        } else if(rot_cmd < previous_rot_cmd - SPEED_STEP) {
            rot_cmd = previous_rot_cmd - SPEED_STEP;
        }
        rot_cmd = clamp(rot_cmd, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED); 
        previous_rot_cmd = rot_cmd;

        left_speed = -rot_cmd;
        right_speed = rot_cmd;
    } else {
        // PD translation + PD steering
        
        /* Translation calculations */
        float lin_derivative = (distance_error - last_distance_error) / dt_s;
        lin_derivative = ((1.0f - ALPHA) * last_lin_derivative) + (ALPHA * lin_derivative);
        last_lin_derivative = lin_derivative;

        float lin_cmd = (KP_LIN * distance_error + KD_LIN * lin_derivative);
        last_distance_error = distance_error;
        
        /* Speed ramp */
        if(lin_cmd > previous_lin_cmd + SPEED_STEP) {
            lin_cmd = previous_lin_cmd + SPEED_STEP;
        } else if(lin_cmd < previous_lin_cmd - SPEED_STEP) {
            lin_cmd = previous_lin_cmd - SPEED_STEP;
        }
        lin_cmd = clamp(lin_cmd, 0.0f, MAX_TRANSLATION_SPEED);
        previous_lin_cmd = lin_cmd;

        /* Steering calculations */
        float steer_derivative = (heading_error - last_heading_error) / dt_s;
        steer_derivative = ((1.0f - ALPHA) * last_steer_derivative) + (ALPHA * steer_derivative);
        last_steer_derivative = steer_derivative; 

        float steer_cmd = (KP_STEER * heading_error + KD_STEER * steer_derivative);
        last_heading_error = heading_error;

        /* Speed ramp */
        if(steer_cmd > previous_steer_cmd + SPEED_STEP) {
            steer_cmd = previous_steer_cmd + SPEED_STEP;
        } else if(steer_cmd < previous_steer_cmd - SPEED_STEP) {
            steer_cmd = previous_steer_cmd - SPEED_STEP;
        }
        steer_cmd = clamp(steer_cmd, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        previous_steer_cmd = steer_cmd;

        float applied_lin_cmd = is_reversed ? -lin_cmd : lin_cmd;

        left_speed = applied_lin_cmd - steer_cmd;
        right_speed = applied_lin_cmd + steer_cmd;

        // Debug angle outputs
        /*
        ESP_LOGI(LOGGER_TAG, "Current angle: %lf", current_pos.angle);        
        printf(">Heading_error:%f\n", heading_error);
        printf(">Réponse_angle:%f\n", steer_cmd);
        */
    }

    if (INVERTED_LEFT_MOTOR) left_speed *= -1;

    motor_a.set_speed_pid(right_speed);
    motor_b.set_speed_pid(left_speed);

    return false;
}

void MotorControl::start() {
    motor_a.start();
    motor_b.start();

    ESP_LOGD(LOGGER_TAG, "start");
}

void MotorControl::stop() {
    motor_a.set_speed(0.0f);
    motor_b.set_speed(0.0f);

    ESP_LOGD(LOGGER_TAG, "stop");
}
