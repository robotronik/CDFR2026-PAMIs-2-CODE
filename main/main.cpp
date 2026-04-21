#include "pins.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "main.h"

static const char* LOGGER_TAG = "MainFSM";
static MainFSM_State current_state = MainFSM_State::INIT;
static constexpr TickType_t MAIN_LOOP_PERIOD = pdMS_TO_TICKS(10); // 100Hz

MotorControl motor_control;
PullSwitch pull_switch(PIN_SW_TIRETTE);
StatusLed status_led(PIN_STATUS_LED);

void main_fsm() {
    TickType_t last_wake_time = xTaskGetTickCount();
    ESP_LOGI(LOGGER_TAG, "Starting main FSM");
    while(true) {
        switch(current_state) {
            case MainFSM_State::INIT: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in init state"); 
                status_led.toggle();
                motor_control.start();
                current_state = MainFSM_State::IDLE;
                break;
            }
            case MainFSM_State::IDLE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in idle state");
                if (pull_switch.read()){
                    ESP_LOGI(LOGGER_TAG, "Pull switch activated, transitioning to active state");
                    current_state = MainFSM_State::ACTIVE;
                    status_led.set(false);
                }
                break;
            }
            case MainFSM_State::ACTIVE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in active state");
                // Nothing to do yet
                // current_state = MainFSM_State::IDLE;
                break;
            }
            case MainFSM_State::DONE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in done state");
                // Do nothing, just wait for reset
                break;
            }
            case MainFSM_State::ERROR: {
                ESP_LOGE(LOGGER_TAG, "ESP32 error! Reinitializing...");
                esp_restart();
                break;
            }
        }

        vTaskDelayUntil(&last_wake_time, MAIN_LOOP_PERIOD);
    } 
} 

extern "C" void app_main(void) {
   main_fsm(); 
}
