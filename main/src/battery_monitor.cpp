#include "battery_monitor.h"
#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *LOGGER_TAG = "BATTERY_MONITOR";

#define R_TOP 20000.0f
#define R_BOTTOM 10000.0f
#define DIVIDER_RATIO (R_TOP + R_BOTTOM) / R_BOTTOM

BatteryMonitor::BatteryMonitor(adc_unit_t unit, adc_channel_t channel)
    : unit(unit), channel(channel), adc_handle(NULL), cali_handle(NULL), calibrated(false) {
    
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = unit;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12; 
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));

    adc_cali_line_fitting_config_t cali_config = {};
    cali_config.unit_id = unit;
    cali_config.atten = ADC_ATTEN_DB_12;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(LOGGER_TAG, "Calibration setup successful.");
        calibrated = true;
    } else {
        ESP_LOGW(LOGGER_TAG, "Calibration setup failed. Readings will be uncalibrated.");
    }
}

BatteryMonitor::~BatteryMonitor() { 
    if (calibrated) {
        adc_cali_delete_scheme_line_fitting(cali_handle);
    }
    adc_oneshot_del_unit(adc_handle);
}

float BatteryMonitor::readVoltage() {
    int raw_value = 0;
    
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw_value));

    if (calibrated) {
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv));
        return (voltage_mv / 1000.0f) * DIVIDER_RATIO;
    } else { 
        float pin_voltage = (raw_value / 4095.0f) * 3.3f;
        return pin_voltage * DIVIDER_RATIO;
    }
}
