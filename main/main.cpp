#include "pins.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_rom_sys.h>

#include "main.h"

static const char* LOGGER_TAG = "MainFSM";
static MainFSM_State current_state = MainFSM_State::INIT;

MotorControl motor_control;
PullSwitch pull_switch(PIN_SW_TIRETTE);
StatusLed status_led(PIN_STATUS_LED);

void main_fsm() {
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
                current_state = pull_switch.read() ? MainFSM_State::ACTIVE : MainFSM_State::IDLE;
                break;
            }
            case MainFSM_State::ACTIVE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in active state");
                // Nothing to do yet
                esp_rom_delay_us(10000);
                current_state = MainFSM_State::IDLE;
                break;
            }
            case MainFSM_State::ERROR: {
                ESP_LOGE(LOGGER_TAG, "ESP32 error! Reinitializing...");
                // TODO further error handling / don't reboot automatically?
                esp_restart();
                break;
            }
        }
    } 
} 

extern "C" void app_main(void) {
   main_fsm(); 
}
