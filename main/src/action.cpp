#include "action.h"
#include "main.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* LOGGER_TAG = "Action";

#define NINJA
#ifndef NINJA
// Generic pami action
bool action_step(MotorControl& motor_control) {
    static int step = 0;
    static TickType_t action_start_tick = 0;
    static constexpr TickType_t ACTION_DELAY = pdMS_TO_TICKS(1000);

    switch (step) {
        case 0:
            // Initialize pami action resources here when needed.
            // Start the 85s timer
            action_start_tick = xTaskGetTickCount();
            if (servo_1.write_angle(70) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_1.write_angle(70) failed");
            }
            if (servo_2.write_angle(130) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_2.write_angle(130) failed");
            }
            step = 1;
            break;
        case 1:
        {
            // Wait until near the end of the match (85s)

            // as a test we read the ultrasonic sensor distance and log it
            /*float distance_cm = 0.0f;
            if (ultrasonic.read_distance_cm(distance_cm) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "ultrasonic.read_distance_cm() failed");
            } else {
                ESP_LOGI(LOGGER_TAG, "Ultrasonic distance: %.2f cm", distance_cm);
            }*/

            if ((xTaskGetTickCount() - action_start_tick) >= ACTION_DELAY) {
                step = 2;
            }
        }
            break;
        case 2:
            // Execute one pami action step here.
            if (servo_1.write_angle(130) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_1.write_angle(130) failed");
            }
            if (servo_2.write_angle(70) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_2.write_angle(70) failed");
            }
            step = 3;
            break;
        case 3:
            // Finalize pami action resources here when needed.
            step = 0;
            return true;
    }
    return false;
}

#else
// Specific ninja pami

/* Helpers */
void take_stock() {
    static constexpr TickType_t SERVO_DELAY = pdMS_TO_TICKS(500); 

    // Step 1: Lower claw
    servo_1.write_angle(0);
    vTaskDelay(SERVO_DELAY);

    // Step 2: Tighten
    servo_2.write_angle(360); // TODO: Check claw servo angle to hold stock
    vTaskDelay(SERVO_DELAY);

    // Step 3: Raise claw
    servo_2.write_angle(60);
    vTaskDelay(SERVO_DELAY);
}

void release_stock() {
    static constexpr TickType_t SERVO_DELAY = pdMS_TO_TICKS(500); 

    // Step 1: Lower claw
    servo_1.write_angle(0);
    vTaskDelay(SERVO_DELAY);

    // Step 2: Release
    servo_2.write_angle(0); // TODO: Check claw servo angle to hold stock
    vTaskDelay(SERVO_DELAY);

    // Step 3: Raise claw
    servo_2.write_angle(60);
    vTaskDelay(SERVO_DELAY);
}

/*
Strategy 1: move to stocks, throw them in the nest, put empty nut cases in fridge 
then push last stock over
*/
bool action_step() {
    static int step = 0;
    switch (step) {
        case 0:
            // Initialize ninja action resources here when needed.
            step = 1;
            break;
        case 1: // Move to stock 1 
            motor_control.move({100.0f, 300.0f, 0.0f});
            step = 2;
            break;
        case 2: // Pick up stock 1
            take_stock();
            step = 3;
            break;
        case 3: // Move to border
            motor_control.move({100.0f, 0.0f, -90.0f});
            step = 4;
            break;
        case 4: // Drop stock
            release_stock();
            step = 5;
            break;
        case 5: // Move to stock 2
            motor_control.move({150.0f, 700.0f, 0.0f});
            step = 6;
            break;
        case 6: // Take stock
            take_stock();
            step = 7;
            break;
        case 7: // Move to border
            motor_control.move({150.0f, 0.0f, -90.0f});
            step = 8;
            break;
        case 8: // Drop stock
            release_stock();
            step = 9;
            break;
        case 9: // Move to final stock
            motor_control.move({150.0f, 100.0f, 0.0f});
            step = 10;
            break;
        case 10: // Push (TODO: take empty cases and put in zones)
            motor_control.move({300.0f, 100.0f, 0.0f});
            step = 11;
            break;
        case 11:
            // Finalize ninja action resources here when needed.
            step = 0;
            return true;
    }
    return false;
}

#endif // NINJA
