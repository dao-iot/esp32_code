#include "throttle.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define TAG "THROTTLE"

/* -------- CONFIG -------- */
#define THROTTLE_ADC_CHAN     ADC_CHANNEL_6    // GPIO 34
#define THROTTLE_ADC_UNIT     ADC_UNIT_1
#define THROTTLE_ADC_ATTEN    ADC_ATTEN_DB_11

#define ADC_MAX               4095
#define ADC_REF_V             3.3f

#define THROTTLE_IDLE_V       0.8f
#define THROTTLE_FULL_V       3.3f

static adc_oneshot_unit_handle_t adc1_handle;

void throttle_init(void) {
    // 1. ADC Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = THROTTLE_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // 2. ADC Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = THROTTLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, THROTTLE_ADC_CHAN, &config));

    ESP_LOGI(TAG, "Throttle ADC initialized on GPIO 34");
}

float throttle_get_percent(void) {
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, THROTTLE_ADC_CHAN, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC");
        return 0.0f;
    }

    // Convert ADC -> voltage (simplified as per user logic)
    float voltage = ((float)adc_raw * ADC_REF_V) / (float)ADC_MAX;

    // Clamp voltage
    if (voltage < THROTTLE_IDLE_V) voltage = THROTTLE_IDLE_V;
    if (voltage > THROTTLE_FULL_V) voltage = THROTTLE_FULL_V;

    // Voltage -> percentage
    float throttle_percent = ((voltage - THROTTLE_IDLE_V) / (THROTTLE_FULL_V - THROTTLE_IDLE_V)) * 100.0f;

    return throttle_percent;
}
