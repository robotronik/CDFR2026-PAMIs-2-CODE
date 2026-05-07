#include "action.h"
#include "esp_err.h"
#include "main.h"
#include "navigation/map.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* LOGGER_TAG = "Action"; 

namespace { 
    constexpr TickType_t SERVO_DELAY = pdMS_TO_TICKS(500); // in ms
    constexpr TickType_t FINAL_PHASE_TIME = pdMS_TO_TICKS(85000); // in ms
    constexpr TickType_t MATCH_END_TIME = pdMS_TO_TICKS(200000); // in ms
    constexpr float OBSTACLE_STOP_DISTANCE = 50.0f; // in mm
}

Map waypoint_map;

/* === Helpers === */

// Check for obstacle using ultrasonic sensor; returns true if there is an obstacle
static bool obstacle_check() {
    uint32_t dist;
    esp_err_t ret = ultrasonic.read_distance_mm(dist);
    if(ret != ESP_OK) {
        ESP_LOGE(LOGGER_TAG, "Obstacle check failed");
        return false;
    }
    return dist < OBSTACLE_STOP_DISTANCE ? true : false; 
}

static void dance() {
    servo_1.write_angle(0);
    vTaskDelay(SERVO_DELAY);
    
    servo_1.write_angle(60);
    vTaskDelay(SERVO_DELAY);
}

#if defined(NINJA) 

static void take_stock() {
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

static void release_stock() {
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

#endif

/* === State machines === */

#ifndef NINJA
/* -----------------------------
 * GENERIC PAMI ACTION
 * -----------------------------
*/

bool action_state() {
    static PamiAction state = PamiAction::BEGIN;
    static PamiAction next_state;
    static bool stopped = false;
    static TickType_t action_start_tick = 0;
    static coords_t next_coords;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize pami action resources here when needed.

            // Start the 85s timer
            action_start_tick = xTaskGetTickCount();
            state = PamiAction::WAIT;
            break;
        }
        case PamiAction::WAIT: {
            // Wait until near the end of the match
            if ((xTaskGetTickCount() - action_start_tick) >= FINAL_PHASE_TIME) {
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
            if(!obstacle_check()) {
                if(stopped) {
                    stopped = false;
                    motor_control.start();
                }
                motor_control.goTo(next_coords, false);
            } else {
                stopped = true;
                motor_control.stop();
            } 
            if(motor_control.target_reached()) {
                state = next_state;
            }
            break;
        }
        case PamiAction::DANCE: {
            dance();
            if ((xTaskGetTickCount() - action_start_tick) >= MATCH_END_TIME) {
                state = PamiAction::END;
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

/*
Strategy 1: move to stocks, throw them in the nest, put empty nut cases in fridge 
then push last stock over
*/

bool action_state() {
    static PamiAction state = PamiAction::BEGIN;
    static PamiAction next_state;
    static bool stopped = false;
    static coords_t next_coords;
    static TickType_t action_start_tick = 0;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize ninja action resources here when needed.

            // add every point coords
            waypoint_map.add_object({200.0f, 0.0f, 0.0f}, "point1", PamiAction::NEXT_STEP); 
            waypoint_map.add_object({200.0f, 200.0f, 0.0f}, "point2", PamiAction::NEXT_STEP);; 
            waypoint_map.add_object({0.0f, 200.0f, 0.0f}, "point3", PamiAction::NEXT_STEP);
            waypoint_map.add_object({0.0f, 0.0f, 0.0f}, "point4", PamiAction::BEGIN);

            /*
            waypoint_map.add_object({0.0f, 0.0f, 180.0f}, "point1", PamiAction::NEXT_STEP);
            waypoint_map.add_object({0.0f, 0.0f, 0.0f}, "point2", PamiAction::BEGIN);
            */

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
            if(!obstacle_check()) {
                if(stopped) {
                    stopped = false;
                    motor_control.start();
                }
                motor_control.goTo(next_coords, false);
            } else {
                stopped = true;
                motor_control.stop();
            }
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
        case PamiAction::DANCE: {
            dance();
            if((xTaskGetTickCount() - action_start_tick) >= MATCH_END_TIME) {
                state = PamiAction::END;
            }
            break;
        }
        case PamiAction::END: {
            // Finalize ressources here if needed 
            return true;
        }
    }
    return false;
}

#endif // NINJA


