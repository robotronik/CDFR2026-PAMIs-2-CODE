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
    constexpr TickType_t FINAL_PHASE_TIME = pdMS_TO_TICKS(10000); // in ms 85 secondes à remettre
    constexpr float OBSTACLE_STOP_DISTANCE = 100.0f; // in mm
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
    return dist < OBSTACLE_STOP_DISTANCE;
}

static void dance() {
    // Non-blocking dance sequence: call repeatedly, it advances on elapsed time.
    static uint8_t step = 0;
    static TickType_t step_start = 0;

    const TickType_t now = xTaskGetTickCount();

    switch (step) {
        case 0: {
            servo_1.write_angle(0);
            step_start = now;
            step = 1;
            break;
        }
        case 1: {
            if ((now - step_start) >= SERVO_DELAY) {
                servo_1.write_angle(60);
                step_start = now;
                step = 2;
            }
            break;
        }
        case 2: {
            if ((now - step_start) >= SERVO_DELAY) {
                step = 0;
            }
            break;
        }
        default: {
            step = 0;
            break;
        }
    }
}

#if N_PAMI == 1
#define OFFSET_START 0
#elif N_PAMI == 2
#define OFFSET_START 1000 
#elif N_PAMI == 3
#define OFFSET_START 2000 
#elif N_PAMI == 4
#define OFFSET_START 3000 
#elif N_PAMI == 5
#define OFFSET_START 4000 
#elif N_PAMI == 6
#define OFFSET_START 5000 
#endif

#if defined(NINJA) 

// Ninja specific servo helpers here
static void take() {
    servo_1.write_angle(0);
}

static void push() {
    servo_1.write_angle(30);
}

static void fold() {
    servo_1.write_angle(180);
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
    static TickType_t action_start_tick = 0;
    static map_object_t current_task;
    static bool has_task = false;

    switch (state) {
        case PamiAction::BEGIN: {
            // Initialize pami action resources here when needed.

            // Start the 85s timer
            action_start_tick = xTaskGetTickCount();
            has_task = false;
            state = PamiAction::SETUP;
            break;
        }
        case PamiAction::SETUP: {
            switch (N_PAMI) {
                case 1: {
                    waypoint_map.add_object({1523.0f, 0.0f, 0.0f}, "PAMI1_1", PamiAction::DANCE, false, false, true);
                    break;
                }
                case 2: {
                    waypoint_map.add_object({1445.0f, 0.0f, 0.0f}, "PAMI2_1", PamiAction::DANCE, false, false, true);
                    break;
                }
                case 3: {
                    waypoint_map.add_object({928.0f, 0.0f, 0.0f}, "PAMI3_1", PamiAction::DANCE, false, false, true);
                    break;
                }
                case 4: {
                    waypoint_map.add_object({1400.0f, 0.0f, 0.0f}, "PAMI4_1", PamiAction::MOVING, false, false, true);
                    waypoint_map.add_object({1830.0f, -1010.0f, 0.0f}, "PAMI4_2", PamiAction::DANCE, false, false, true);
                    break;
                }
                case 5: {
                    waypoint_map.add_object({700.0f, 0.0f, 0.0f}, "PAMI5_1", PamiAction::MOVING, false, false, true);
                    waypoint_map.add_object({1100.0f, -930.0f, 0.0f}, "PAMI5_2", PamiAction::DANCE, false, false, true);
                    break;
                }
                case 6: {
                    waypoint_map.add_object({500.0f, 0.0f, 0.0f}, "PAMI6_1", PamiAction::MOVING, false, false, true);
                    waypoint_map.add_object({500.0f, -585.0f, 0.0f}, "PAMI6_2", PamiAction::DANCE, false, false, true);
                    break;
                }

            }
            state = PamiAction::WAIT;
            break;
        }
        case PamiAction::WAIT: {
            motor_control.goTo();
            // Wait until near the end of the match
            if ((xTaskGetTickCount() - action_start_tick) >= FINAL_PHASE_TIME + pdMS_TO_TICKS(OFFSET_START)) {
                has_task = false;
                state = PamiAction::MOVING;
            }
            break;
        }
        case PamiAction::MOVING: {
            if ((xTaskGetTickCount() - action_start_tick) >= pdMS_TO_TICKS(100000)) {
                state = PamiAction::DANCE;
                ESP_LOGI(LOGGER_TAG, "Final phase reached, switching to DANCE state");
                break;
            }

            if(!has_task) {
                ESP_LOGI(LOGGER_TAG, "Fetching next task");
                // Get new task
                current_task = waypoint_map.get_next_object();
                if(current_task.name.empty()) waypoint_map.add_object({155.0f, -50.0f, -65.0f}, "point12", PamiAction::DANCE, false, false, false); {
                    ESP_LOGW(LOGGER_TAG, "No more tasks available");
                    state = PamiAction::DANCE;
                    break;
                }
                motor_control.goTo(current_task.coords, current_task.turnEnd, current_task.reverse);
                has_task = true;
                ESP_LOGI(LOGGER_TAG, "New task: %s, coords: x: %f, y: %f, angle: %f, turnEnd: %d", current_task.name.c_str(), current_task.coords.x, current_task.coords.y, current_task.coords.angle, (int)current_task.turnEnd);
                break;
            }

            if (current_task.detect && obstacle_check()) {
                motor_control.stop();
                ESP_LOGI(LOGGER_TAG, "Obstacle detected, stopping robot");
            } else {
                if (motor_control.goTo(current_task.turnEnd))
                {
                    // Task reached
                    ESP_LOGI(LOGGER_TAG, "Task %s reached", current_task.name.c_str());
                    has_task = false;
                    if(current_task.next_action != PamiAction::MOVING) {
                        state = current_task.next_action;
                    }
                }
            }
            break;
        }
        case PamiAction::DANCE: {
            motor_control.goTo();
            dance();
            break;
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
    static map_object_t current_task;
    static bool has_task = false;

    switch (state) {
        case PamiAction::BEGIN: { 
            // Initialize ninja action resources here when needed.

            // add every point coords
            // waypoint_map.add_object({1000.0f, 0.0f, 0.0f}, "point1", PamiAction::MOVING, false, false, false);

            waypoint_map.add_object({320.0f, 0.0f, 0.0f}, "point1", PamiAction::TAKE, false, false, false);
            waypoint_map.add_object({320.0f, 140.0f, -90.0f}, "point2", PamiAction::FOLD, false, false, false);
            waypoint_map.add_object({470.0f, 140.0f, -90.0f}, "point3", PamiAction::PUSH, false, false, false);
            waypoint_map.add_object({410.0f, 140.0f, 0.0f}, "point4", PamiAction::MOVING, false, true, false);
            waypoint_map.add_object({410.0f, -90.0f, 0.0f}, "point5", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({410.0f, -80.0f, 0.0f}, "point6", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({650.0f, -80.0f, 0.0f}, "point7", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({650.0f, 230.0f, 0.0f}, "point8", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, 0.0f, 0.0f}, "point9", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, 30.0f, 0.0f}, "point10", PamiAction::TAKE, false, false, false);
            waypoint_map.add_object({155.0f, -50.0f, -65.0f}, "point11", PamiAction::PUSH, true, false, false); 
            waypoint_map.add_object({530.0f, 30.0f, 0.0f}, "point12", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({530.0f, 30.0f, -20.0f}, "point13", PamiAction::FOLD, true, false, false);
            waypoint_map.add_object({155.0f, 0.0f, 0.0f}, "point14", PamiAction::PUSH, false, false, false);
            waypoint_map.add_object({155.0f, 230.0f, 0.0f}, "point15", PamiAction::FOLD, false, false, false);
            waypoint_map.add_object({530.0f, 230.0f, 0.0f}, "point16", PamiAction::DANCE, true, false, false);

            /*
            waypoint_map.add_object({500.0f, 0.0f, 0.0f}, "point1", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({500.0f, 0.0f, 0.0f}, "point2", PamiAction::PUSH, false, false, false);
            waypoint_map.add_object({1000.0f, 0.0f, 0.0f}, "point3", PamiAction::TAKE, false, false, false);
            */

            // move in square 
            /*
            waypoint_map.add_object({200.0f, 0.0f, 0.0f}, "point1", PamiAction::MOVING, false, false, false); 
            waypoint_map.add_object({200.0f, 200.0f, 0.0f}, "point2", PamiAction::MOVING, false, false, false); 
            waypoint_map.add_object({0.0f, 200.0f, 0.0f}, "point3", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({0.0f, 0.0f, 0.0f}, "point4", PamiAction::BEGIN, false, false, false);
            */ 

            // old strat
            /*
            waypoint_map.add_object({155.0f, 0.0f, -90.0f}, "point1", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, 150.0f, 0.0f}, "point2", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, 120.0f, 0.0f}, "point3", PamiAction::MOVING, false, true, false);
            waypoint_map.add_object({470.0f, 100.0f, -90.0f}, "point4", PamiAction::MOVING, false, false, false); 
            waypoint_map.add_object({440.0f, 100.0f, 0.0f}, "point5", PamiAction::MOVING, false, true, false);
            waypoint_map.add_object({440.0f, -90.0f, 0.0f}, "point6", PamiAction::CALIBRATE_Y, false, true, false);
            waypoint_map.add_object({440.0f, -50.0f, -90.0f}, "point7", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({670.0f, -50.0f, 0.0f}, "point8", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({670.0f, 260.0f, 0.0f}, "point9", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({670.0f, 90.0f, 90.0f}, "point10", PamiAction::MOVING, false, true, false);
            waypoint_map.add_object({0.0f,   90.0f, 90.0f}, "point11", PamiAction::CALIBRATE_X, false, false, false);
            waypoint_map.add_object({155.0f, 90.0f, 0.0f}, "point12", PamiAction::MOVING, false, true, false);
            waypoint_map.add_object({155.0f, -90.0f, 0.0f}, "point13", PamiAction::CALIBRATE_Y, false, true, false);
            waypoint_map.add_object({155.0f, 160.0f, 0.0f}, "point14", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, 260.0f, 0.0f}, "point15", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({155.0f, -90.0f, 0.0f}, "point16", PamiAction::CALIBRATE_Y, false, true, false);
            waypoint_map.add_object({155.0f, -50.0f, -65.0f}, "point17", PamiAction::MOVING, false, false, false);
            waypoint_map.add_object({415.0f, 70.0f, -20.0f}, "point18", PamiAction::END, false, false, false);
            */ 

            has_task = false;
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::MOVING: {
            if(!has_task) {
                ESP_LOGI(LOGGER_TAG, "Fetching next task");
                current_task = waypoint_map.get_next_object();
                if(current_task.name.empty()) {
                    ESP_LOGW(LOGGER_TAG, "No more tasks available");
                    state = PamiAction::DANCE;
                    break;
                }
                motor_control.goTo(current_task.coords, current_task.turnEnd, current_task.reverse);
                has_task = true;
                ESP_LOGI(LOGGER_TAG, "New task: %s, coords: x: %f, y: %f, angle: %f, turnEnd: %d", current_task.name.c_str(), current_task.coords.x, current_task.coords.y, current_task.coords.angle, (int)current_task.turnEnd);
                break;
            }

            if (current_task.detect && obstacle_check()) {
                motor_control.stop();
                ESP_LOGI(LOGGER_TAG, "Obstacle detected, stopping robot");
            } else {
                if (motor_control.goTo(current_task.turnEnd)) {
                    ESP_LOGI(LOGGER_TAG, "Task %s reached", current_task.name.c_str());
                    has_task = false;
                    if(current_task.next_action != PamiAction::MOVING) {
                        state = current_task.next_action;
                    }
                }
            }
            break;
        }
        case PamiAction::TAKE:  {
            motor_control.goTo();
            take();
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::PUSH: {
            motor_control.goTo();
            push();
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::FOLD: {
            motor_control.goTo();
            fold();
            state = PamiAction::MOVING;
            break;
        }
        case PamiAction::DANCE: {
            motor_control.goTo();
            // dance();
            break;
        }
    }
    return false;
}

#endif // NINJA


