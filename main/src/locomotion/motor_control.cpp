#include "locomotion/motor_control.h"
#include <esp_log.h>
#include "math.h"
#include "constants.h"

using namespace robot_pins;
using namespace robot_constants;

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
   
    // do not use pow for faster exponentiation
    delta_initial = sqrt((target_pos.x - current_pos.x) * (target_pos.x - current_pos.x) + (target_pos.y - current_pos.y) * (target_pos.y - current_pos.y));
    delta_angle_initial = target_pos.angle - current_pos.angle;

    // First angular target: Move to point
    real_angle_target = atan2((target_pos.y - current_pos.y), (target_pos.x - current_pos.x))*RAD_TO_DEG;

    current_state = MotorControlState::START;
}

void MotorControl::update() {
    float delta_a = encoder_a.get_delta();
    float delta_b = encoder_b.get_delta();
    float avg_delta = (delta_a + delta_b) / 2.0f;

    // Update angle
    float delta_angle = (delta_a - delta_b) / WHEEL_DIST;

    current_pos.angle += delta_angle;
    
    // Update position 
    float delta_x = avg_delta * sin(current_pos.angle*DEG_TO_RAD);
    float delta_y = avg_delta * cos(current_pos.angle*DEG_TO_RAD);

    current_pos.x += delta_x;
    current_pos.y += delta_y;

    ESP_LOGD(LOGGER_TAG, "Update position - x: %f, y: %f, angle: %f", current_pos.x, current_pos.y, current_pos.angle);

    switch(current_state) {
        case MotorControlState::START:
            start();
            // TODO: if our robot can reach its target by just going backwards then it should do that then rotate after
            current_state = MotorControlState::ROTATION;
            break;
        case MotorControlState::STOP:
            stop(); 
            break;
        case MotorControlState::ROTATION:
            float delta_target_angle = real_angle_target - current_pos.angle;
            if(delta_target_angle < ANGLE_ERROR_MARGIN && real_angle_target == target_pos.angle) { 
                current_state = MotorControlState::STOP;
            } else if (delta_target_angle < ANGLE_ERROR_MARGIN) {
                current_state = MotorControlState::LINEAR;
            }

            float angular_speed_percentage = delta_target_angle/delta_angle_initial; 
            if(delta_target_angle < 0.0f && delta_angle_initial < 0.0f) { // TODO: check if correct
                angular_speed_percentage = -angular_speed_percentage;
            }

            motor_a.set_speed(angular_speed_percentage);
            motor_b.set_speed(-angular_speed_percentage);
            break;
        case MotorControlState::LINEAR: // TODO constant phase after ramp? 
            float delta_target_x = target_pos.x - current_pos.x;
            float delta_target_y = target_pos.y - current_pos.y;

            float remaining_distance = sqrt(delta_target_x * delta_target_x + delta_target_y * delta_target_y); 
            if(remaining_distance < DISTANCE_ERROR_MARGIN) {
                // Switch to second rotation phase to align to the real target angle
                real_angle_target = target_pos.angle;
                current_state = MotorControlState::ROTATION;
            }

            // Trapezoidal speed curve 
            float target_speed_percentage = 0.0f;
            float distance_factor = remaining_distance / delta_initial; 
            if(current_speed_percentage < 1.0f && distance_factor > SLOWDOWN_DISTANCE_PERCENTAGE) { // linear accceleration 
                target_speed_percentage = current_speed_percentage + SPEED_STEP;
            } else if(distance_factor <= SLOWDOWN_DISTANCE_PERCENTAGE && target_speed_percentage > MIN_SPEED_PERCENTAGE) {
                target_speed_percentage = current_speed_percentage - SPEED_STEP; // linear deceleration 
            } else {
                target_speed_percentage = current_speed_percentage; // top speed reached
            }

            current_speed_percentage = target_speed_percentage;
            motor_a.set_speed(target_speed_percentage);
            motor_b.set_speed(target_speed_percentage);
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
    motor_a.set_speed(0.0f);
    motor_b.set_speed(0.0f);
    encoder_a.stop();
    encoder_b.stop();
    motor_a.stop();
    motor_b.stop();

    ESP_LOGD(LOGGER_TAG, "stop");
}
