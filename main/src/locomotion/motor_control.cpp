#include "locomotion/motor_control.h"
#include <esp_log.h>
#include "math.h"

#define WHEEL_DIST 8; // in cm 

static const char* LOGGER_TAG = "MotorControl";

MotorControl::MotorControl() 
   : encoder_a(PIN_HALL_A1, PIN_HALL_A2),
     encoder_b(PIN_HALL_B1, PIN_HALL_B2),
     motor_a(PIN_DC_A1, PIN_DC_A2),
     motor_b(PIN_DC_B1, PIN_DC_B2)
{
   ESP_LOGD(LOGGER_TAG, "init");
}

void MotorControl::move(coords_t new_target) {
    ESP_LOGD(LOGGER_TAG, "Received move order, coords: x: %f, y: %f, angle: %f", new_target.x, new_target.y, new_target.angle);

    // TODO: safety checks (bounds)
    target_pos = new_target;
}

void MotorControl::update() {
    float delta_a = encoder_a.get_delta();
    float delta_b = encoder_b.get_delta();
    float avg_delta = (delta_a + delta_b) / 2.0f;

    // Update angle
    float delta_angle = (delta_a - delta_b) / WHEEL_DIST;

    current_pos.angle += delta_angle;
    
    // Update position 
    float delta_x = avg_delta * sin(current_pos.angle);
    float delta_y = avg_delta * cos(current_pos.angle);

    current_pos.x += delta_x;
    current_pos.y += delta_y;

    ESP_LOGD(LOGGER_TAG, "Update position - x: %f, y: %f, angle: %f", current_pos.x, current_pos.y, current_pos.angle);

    switch(current_state) {
        case MotorControlState::START:
            start();
            break;
        case MotorControlState::STOP:
            stop(); 
            break;
        case MotorControlState::ROTATION:
            float delta_target_angle = target_pos.angle - current_pos.angle;
            break;
        case MotorControlState::LINEAR:
            float delta_target_x = target_pos.x - current_pos.x;
            float delta_target_y = target_pos.y - current_pos.y;

            
            break;
    }
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
