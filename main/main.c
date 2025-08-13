#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/idf_additions.h"
#include "rpm_calc.h"

void app_main(void) {
	startADC();

	for (;;) {
		printf("ADC Data: %d\n", ADC_DATA);

		vTaskDelay(100);
	}
}
