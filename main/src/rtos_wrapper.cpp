#include "rtos_wrapper.h"

void RTOSTaskWrapper::task_trampoline(void* pv_parameters) {
    RTOSTaskWrapper* instance = static_cast<RTOSTaskWrapper*>(pv_parameters);
    instance->run();
    instance->task_handle = nullptr;
    vTaskDelete(NULL);
}

void RTOSTaskWrapper::start() {
    stop_task = false;
    xTaskCreatePinnedToCore(&task_trampoline, task_name, stack_depth, this, priority, &task_handle, core_id);  
}

void RTOSTaskWrapper::stop() {
    if(task_handle != nullptr) {
        stop_task = true;
    }
}

RTOSTaskWrapper::~RTOSTaskWrapper() {
    stop();
}
