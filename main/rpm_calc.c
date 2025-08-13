#include "rpm_calc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/adc_types.h"
#include "freertos/FreeRTOS.h"
#include "portmacro.h"
#include "soc/soc_caps.h"
#include "esp_timer.h"
#include "math.h"
#include <stdlib.h>

#define HALL_SENSOR_PIN 33
#define MAX_RPS (3 * 1000 / 60)
#define SCANS_PER_ROUND 360
#define T_SAMPLE (1 / (MAX_RPS * SCANS_PER_ROUND))
#define CALIBRATION_SCANS 1000 // TO DEFINE BETTER
#define TARGET_PERCENTILE 0.95

/*
														[Calculating the RPS]
	1. Calibration stage:
		1.1 Find the 95th percentile of the values reported by the hall meter.
		1.2 To do this, spend the first CALIBRATION_SCANS purely observing and collecting data.
		1.3 The 95th percentile is defined as base_position_value.
	2. Once base_position_value is defined:
		2.1. Keep reading new values and check if a value has a score higher than the 95th percentile from the pre-computed values.
		2.2. If a value is greater and such value has not been seen before, save the current processor time.
		2.3. Otherwise use the previously saved time to calculate RPS.
	
								[Calculating the position of the propeller relative to the base position]

	1. By knowing the time when the propeller was at base position, at any given point the current position can be calculated using the RPS.
		1.1 Formula is (current_time - base_position_time) / (1/RPS) * 2π (in radians)
*/ 

adc_continuous_handle_t h_adc_cont = NULL;
TaskHandle_t cb_task;

uint16_t calibration_stage_values[CALIBRATION_SCANS];
size_t collected_samples = 0;

int16_t base_position_value = 0;

int64_t base_position_time = 0;

float calculated_rps = 0;

bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
	BaseType_t mustYield = pdFALSE;	
	vTaskNotifyGiveFromISR(cb_task, &mustYield);
	return (mustYield == pdTRUE);
}

int comp_function(const void* first_element, const void* second_element) {
	return *(int16_t*)first_element < *(int16_t*) second_element;
}

void cbTask(void *args) {
	uint8_t buff[400];
	uint32_t rx_len = 0;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		adc_continuous_read(h_adc_cont, buff, sizeof(buff), &rx_len, 0);
		
		for (int32_t i = 0; i < rx_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
			adc_digi_output_data_t *data = (adc_digi_output_data_t*)&buff[i];
			
			uint16_t sensor_data = data->type1.data;
			
			if (collected_samples < CALIBRATION_SCANS) {
				calibration_stage_values[collected_samples++] = sensor_data;
				if (collected_samples == CALIBRATION_SCANS) {
					qsort(calibration_stage_values, collected_samples, sizeof(uint16_t), comp_function);

					int16_t target_percentile_of_size = collected_samples * TARGET_PERCENTILE;
					base_position_value = calibration_stage_values[target_percentile_of_size];
				}
			} else if(sensor_data >= base_position_value) {
				int64_t new_time = esp_timer_get_time();
				if (base_position_time != 0) {
					//														  time is in µs
					calculated_rps = 1.f /((new_time - base_position_time) / (1000 * 1000));
					
					// TODO make sure calculated_rps is relatively stable
				}
				base_position_time = new_time;
			}
		}
	}
}

float calculateCurrentPositionInRadians() {
	return (esp_timer_get_time() - base_position_time) / (1.f / calculated_rps) * 2.f * M_PI;
}

float getRPS() {
	return calculated_rps;
}

void ADC_Init() {
	adc_continuous_handle_cfg_t adc_cnt_cfg = {
		.conv_frame_size = 4,
		.max_store_buf_size = 40
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_cnt_cfg, &h_adc_cont));

	adc_continuous_config_t adc_cfg = {
		.sample_freq_hz = 180000,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.pattern_num = 1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
	};

	adc_digi_pattern_config_t channel_cfg = {
		.channel = 	ADC_CHANNEL_5,
		.atten = ADC_ATTEN_DB_12,
		.bit_width = ADC_BITWIDTH_12,
		.unit = ADC_UNIT_1
	};
	adc_cfg.adc_pattern = &channel_cfg;

	ESP_ERROR_CHECK(adc_continuous_config(h_adc_cont, &adc_cfg));

	adc_continuous_evt_cbs_t cbs_cfg = {
		.on_conv_done = callback
	};

	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(h_adc_cont, &cbs_cfg, NULL));
}

void startADC() {
	xTaskCreate(cbTask, "Calculate rpm and find position", 2048, NULL, 0, &cb_task);
	ADC_Init();
	ESP_ERROR_CHECK(adc_continuous_start(h_adc_cont));
}