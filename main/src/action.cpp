#include "action.h"
#include "main.h"
#include "navigation/map.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* LOGGER_TAG = "Action"; 
Map waypoint_map;

#ifndef NINJA
/* -----------------------------
 * GENERIC PAMI ACTION
 * -----------------------------
*/

bool action_state() {
    static int state = 0;
    static TickType_t action_start_tick = 0;
    static constexpr TickType_t ACTION_DELAY = pdMS_TO_TICKS(1000);

    switch (state) {
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
            state = 1;
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
                state = 2;
            }
        }
            break;
        case 2:
            // Execute one pami action state here.
            if (servo_1.write_angle(130) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_1.write_angle(130) failed");
            }
            if (servo_2.write_angle(70) != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "servo_2.write_angle(70) failed");
            }
            state = 3;
            break;
        case 3:
            // Finalize pami action resources here when needed.
            state = 0;
            return true;
    }
    return false;
}

#else
/* -----------------------------
 * NINJA ACTIONS
 * -----------------------------
 */

static constexpr TickType_t SERVO_DELAY = pdMS_TO_TICKS(500); 

/*
Strategy 1: move to stocks, throw them in the nest, put empty nut cases in fridge 
then push last stock over
*/

bool action_state() {
    static PamiAction state = PamiAction::BEGIN;
    static PamiAction next_state;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize ninja action resources here when needed.

            // add every point coords 
            waypoint_map.add_object({150.0f, 450.0f, 0.0f}, "point1", PamiAction::TAKE); // stock 1 
            waypoint_map.add_object({100.0f, 750.0f, 0.0f}, "point2", PamiAction::TAKE); // stock 2
            waypoint_map.add_object({200.0f, 200.0f, 0.0f}, "point3", PamiAction::TAKE); // stock 3

            state = PamiAction::NEXT_STEP;
            break;
        }
        case PamiAction::NEXT_STEP: {
            map_object_t next = waypoint_map.get_next_object();
            motor_control.move(next.coords);
            next_state = next.next_action;
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::MOVING: {
            if(motor_control.target_reached()) {
                state = next_state;
            }
            break;
        }
        case PamiAction::TAKE: {
            take_stock();
            state = PamiAction::NEXT_STEP;
            break;
        }
        case PamiAction::RELEASE: {
            release_stock();
            state = PamiAction::NEXT_STEP;
            break;
        }
        case PamiAction::END: {
            // Shake servos here 
            return true;
        }
    }
    return false;
}

#endif // NINJA

/* Helpers */
void take_stock() {
    // state 1: Lower claw
    servo_1.write_angle(0);
    vTaskDelay(SERVO_DELAY);

    // state 2: Tighten
    servo_2.write_angle(180); // TODO: Check claw servo angle to hold stock
    vTaskDelay(SERVO_DELAY);

    // state 3: Raise claw
    servo_2.write_angle(60);
    vTaskDelay(SERVO_DELAY);
}

void release_stock() {
    // state 1: Lower claw
    servo_1.write_angle(0);
    vTaskDelay(SERVO_DELAY);

    // state 2: Release
    servo_2.write_angle(0); // TODO: Check claw servo angle to release stock
    vTaskDelay(SERVO_DELAY);

    // state 3: Raise claw
    servo_2.write_angle(60);
    vTaskDelay(SERVO_DELAY);
}
