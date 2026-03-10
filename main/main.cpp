#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
    std::cout << "Hello world!" << std::endl;    
}
