#include "locomotion/motor_control.h"
#include <esp_log.h>
#include <math.h>

static const char* LOGGER_TAG = "MotorControl";

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define WHEEL_DIST 88.0f // distance between the two wheels in mm, TODO: measure this
#define WHEEL_RADIUS 13.0f // radius of the wheels in mm, TODO: measure this
#define WHEEL_CIRCUMFERENCE (2.0f * M_PI * WHEEL_RADIUS) // circumference of the wheels in mm

MotorControl::MotorControl() 
   : encoder_a(PIN_HALL_A1, PIN_HALL_A2),
     encoder_b(PIN_HALL_B1, PIN_HALL_B2),
     motor_a(PIN_DC_A1, PIN_DC_A2),
      motor_b(PIN_DC_B1, PIN_DC_B2),
      target_pos{0.0f, 0.0f, 0.0f},
      current_pos{0.0f, 0.0f, 0.0f}
{
   ESP_LOGD(LOGGER_TAG, "init");
}

void MotorControl::move(coords_t new_target) {
    ESP_LOGD(LOGGER_TAG, "Received move order, coords: x: %f, y: %f, angle: %f", new_target.x, new_target.y, new_target.angle);

    // TODO: safety checks (bounds)
    target_pos = new_target;
   
    // do not use pow for faster exponentiation
    double delta_initial = sqrt((target_pos.x - current_pos.x) * (target_pos.x - current_pos.x) + (target_pos.y - current_pos.y) * (target_pos.y - current_pos.y));
    double delta_angle_initial = target_pos.angle - current_pos.angle;

    // First angular target: Move to point
    double real_angle_target = atan2((target_pos.y - current_pos.y), (target_pos.x - current_pos.x))*RAD_TO_DEG;

    // TODO apply PID control
    motor_a.set_speed(0.0f);
    motor_b.set_speed(0.0f);
}

void MotorControl::update() {
    float count_to_mm = 2.0f * WHEEL_CIRCUMFERENCE / 1050.0f; // convert count to mm
    float delta_right = encoder_a.get_delta() * count_to_mm;
    float delta_left = encoder_b.get_delta() * count_to_mm;

    if (delta_left == 0.0f && delta_right == 0.0f) {
        return;
    }

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

    //ESP_LOGD(LOGGER_TAG, "Received move order, coords: x: %f, y: %f", x, y);
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
