#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

class BatteryMonitor {
    public:
        BatteryMonitor(adc_unit_t unit, adc_channel_t channel);
        ~BatteryMonitor();
        float readVoltage();

    private:
        adc_unit_t unit;
        adc_channel_t channel;
        float divider_ratio;
        
        adc_oneshot_unit_handle_t adc_handle;
        adc_cali_handle_t cali_handle;
        bool calibrated;
};
