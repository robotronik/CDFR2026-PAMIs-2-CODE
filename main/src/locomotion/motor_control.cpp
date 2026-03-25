#include "locomotion/motor_control.h"
#include <esp_log.h>

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
