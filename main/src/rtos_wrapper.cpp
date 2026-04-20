#include "rtos_wrapper.h"
#include <mutex>

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

void SharedPos::set(float new_x, float new_y, float new_angle) {
    std::lock_guard<std::mutex> lock(mutex);
    position.x = new_x;
    position.y = new_y;
    position.angle = new_angle;
}

coords_t SharedPos::get() {
    std::lock_guard<std::mutex> lock(mutex);
}
