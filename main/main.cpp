#include "pins.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#include "main.h"
#include "action.h"

#if CONFIG_ENABLE_REMOTE_LOGGING
    #include "wireless/wifi.h"
    #include "wireless/web_server.h"
#endif

static const char* LOGGER_TAG = "MainFSM";
static MainFSMState current_state = MainFSMState::INIT;
Team current_team = Team::BLUE;
static constexpr TickType_t MAIN_LOOP_PERIOD = pdMS_TO_TICKS(10); // 100Hz

MotorControl motor_control;
Switch pull_switch(PIN_SW_TIRETTE);
Switch team_switch(PIN_SW_TEAM);
StatusLed status_led(PIN_STATUS_LED);
TeamLed team_led(PIN_TEAM_RGB);
Servo servo_1(PIN_SERVO_1);
Servo servo_2(PIN_SERVO_2);
Ultrasonic ultrasonic(PIN_US_TRIG, PIN_US_ECHO);
BatteryMonitor battery_monitor(ADC_UNIT_1, ADC_CHANNEL_6);

void main_fsm() {
    TickType_t last_wake_time = xTaskGetTickCount();
    ESP_LOGI(LOGGER_TAG, "Starting main FSM");
    while(true) {
        switch(current_state) {
            case MainFSMState::INIT: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in init state"); 
                status_led.toggle(); 
                team_led.set_color(0, 255, 0);
                motor_control.INVERTED_LEFT_MOTOR = (N_PAMI > 0) ;
                motor_control.start();
                esp_err_t err = servo_1.attach();
                if (err != ESP_OK) {
                    ESP_LOGE(LOGGER_TAG, "servo_1.attach() failed: %s", esp_err_to_name(err));
                    current_state = MainFSMState::ERROR;
                    break;
                }

                err = servo_2.attach();
                if (err != ESP_OK) {
                    ESP_LOGE(LOGGER_TAG, "servo_2.attach() failed: %s", esp_err_to_name(err));
                    current_state = MainFSMState::ERROR;
                    break;
                }

                err = ultrasonic.init();
                if (err != ESP_OK) {
                    ESP_LOGE(LOGGER_TAG, "ultrasonic.init() failed: %s", esp_err_to_name(err));
                    current_state = MainFSMState::ERROR;
                    break;
                }

                float battery_level = battery_monitor.readVoltage();
                if(battery_level < 6.5f) {
                    ESP_LOGI(LOGGER_TAG, "Low battery warning!");
                    team_led.set_color(255, 0, 0);
                    vTaskDelay(500);
                }

                current_state = MainFSMState::IDLE;
                break;
            }
            case MainFSMState::IDLE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in idle state");

                if(!team_switch.read()) { // Blue
                    team_led.set_color(255, 255, 0); 
                    current_team = Team::BLUE;
                } else { // Yellow
                    team_led.set_color(0, 255, 255);
                    current_team = Team::YELLOW;
                }
                
                if (pull_switch.read()) {
                    ESP_LOGI(LOGGER_TAG, "Pull switch activated, transitioning to active state");
                    current_state = MainFSMState::ACTIVE;
                    status_led.set(false); 
                    motor_control.goTo({0.0f, 0.0f, 0.0f}, false);
                }
                break;
            }
            case MainFSMState::ACTIVE: {
                // Update position
                ESP_LOGD(LOGGER_TAG, "ESP32 in active state");
                if (action_state()) {
                    ESP_LOGI(LOGGER_TAG, "Action done, transitioning to done state");
                    current_state = MainFSMState::DONE;
                }
                break;
            }
            case MainFSMState::DONE: {
                ESP_LOGD(LOGGER_TAG, "ESP32 in done state");
                // Do nothing, just wait for reset
                break;
            }
            case MainFSMState::ERROR: {
                ESP_LOGE(LOGGER_TAG, "ESP32 error! Reinitializing...");
                esp_restart();
                break;
            }
        }

        vTaskDelayUntil(&last_wake_time, MAIN_LOOP_PERIOD);
    } 
} 

extern "C" void app_main(void) {
    #if DEBUG_LEVEL == 0
        esp_log_level_set("*", ESP_LOG_ERROR);
    #elif DEBUG_LEVEL == 1
        esp_log_level_set("*", ESP_LOG_INFO);
    #else
        esp_log_level_set("*", ESP_LOG_DEBUG);
    #endif

    #if CONFIG_ENABLE_REMOTE_LOGGING
        wifi_init();
        start_webserver(); 
    #endif

    main_fsm(); 
}
