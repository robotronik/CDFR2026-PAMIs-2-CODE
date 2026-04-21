#include "action.h"
#include "main.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* LOGGER_TAG = "Action";

#ifndef NINJA
// Generic pami action
bool action_step() {
    static int step = 0;
    static TickType_t action_start_tick = 0;
    static constexpr TickType_t ACTION_DELAY = pdMS_TO_TICKS(85000);

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
            // Wait until near the end of the match (85s)
            if ((xTaskGetTickCount() - action_start_tick) >= ACTION_DELAY) {
                step = 2;
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
bool action_step() {
    static int step = 0;
    switch (step) {
        case 0:
            // Initialize ninja action resources here when needed.
            step = 1;
            break;
        case 1:
            // Execute one ninja action step here.
            step = 2;
            break;
        case 2:
            // Finalize ninja action resources here when needed.
            step = 0;
            return true;
    }
    return false;
}

#endif // NINJA