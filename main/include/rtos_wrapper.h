#pragma once 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class RTOSTaskWrapper {
    private:
        static void task_trampoline(void* pv_parameters);

        TaskHandle_t task_handle;
        const char* task_name;
        uint32_t stack_depth;
        UBaseType_t priority;
        BaseType_t core_id;
    
    protected: 
        virtual void run() = 0;
        volatile bool stop_task;

    public:
        RTOSTaskWrapper(const char* task_name, uint32_t stack_depth, UBaseType_t priority, BaseType_t core_id)
        : task_handle(nullptr), task_name(task_name), stack_depth(stack_depth), priority(priority), core_id(core_id), stop_task(false) {} 

        void start();
        void stop();
        virtual ~RTOSTaskWrapper();
};
