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
    static PamiAction state = PamiAction::BEGIN;
    static PamiAction next_state;
    static TickType_t action_start_tick = 0;
    static constexpr TickType_t ACTION_DELAY = pdMS_TO_TICKS(1000);
    static coords_t next_coords;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize pami action resources here when needed.

            // Start the 85s timer
            action_start_tick = xTaskGetTickCount();
            state = PamiAction::WAIT;
            break;
        }
        case PamiAction::WAIT:{
            // Wait until near the end of the match (85s)
            if ((xTaskGetTickCount() - action_start_tick) >= ACTION_DELAY) {
                state = PamiAction::NEXT_STEP;
            }
            break;
        }
        case PamiAction::NEXT_STEP: {
            map_object_t next = waypoint_map.get_next_object();
            next_coords = next.coords;
            next_state = next.next_action;
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::MOVING: {
            motor_control.goTo(next_coords);
            if(motor_control.target_reached()) {
                state = next_state;
            }
            break;
        }
        case PamiAction::END: {
            // Finalize pami action resources here when needed.
            return true;
        }
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
Strategy 1: goTo to stocks, throw them in the nest, put empty nut cases in fridge 
then push last stock over
*/

bool action_state() {
    static PamiAction state = PamiAction::BEGIN;
    static PamiAction next_state;
    static coords_t next_coords;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize ninja action resources here when needed.

            // add every point coords 
            waypoint_map.add_object({200.0f, 0.0f, 0.0f}, "point1", PamiAction::NEXT_STEP); 
            waypoint_map.add_object({200.0f, 200.0f, 0.0f}, "point2", PamiAction::NEXT_STEP);; 
            waypoint_map.add_object({0.0f, 200.0f, 0.0f}, "point3", PamiAction::NEXT_STEP);
            waypoint_map.add_object({0.0f, 0.0f, 0.0f}, "point4", PamiAction::BEGIN);

            state = PamiAction::NEXT_STEP;
            break;
        }
        case PamiAction::NEXT_STEP: {
            map_object_t next = waypoint_map.get_next_object();
            next_coords = next.coords;
            next_state = next.next_action;
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::MOVING: {
            motor_control.goTo(next_coords, false);
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

#endif // NINJA
