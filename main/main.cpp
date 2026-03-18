#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include "main.h"

static const char* LOGGER_TAG = "MainFSM";
static MainFSM_State current_state = MainFSM_State::INIT;

void main_fsm() {
    while(true) {
        switch(current_state) {
            case MainFSM_State::INIT:
                ESP_LOGI(LOGGER_TAG, "ESP32 Hello"); 
                break;
            case MainFSM_State::IDLE:
 
                break;
            case MainFSM_State::ACTIVE:

                break;
            case MainFSM_State::ERROR:

                break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
} 

extern "C" void app_main(void) {
   main_fsm(); 
}
