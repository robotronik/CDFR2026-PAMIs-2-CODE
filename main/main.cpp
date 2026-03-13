#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "main_fsm.h"

extern "C" void app_main(void) {
   main_fsm(); 
}
