/*
 * Taskovi.c
 *
 *  Created on: May 12, 2020
 *      Author: peca
 */

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "adc.h"


void ledon(void){
	HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
}

void ledof(void){
	HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
}

void ledtogl(void) {
	HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
}

void trep(uint32_t tickova){
	ledtogl();
	osDelay(tickova/2);
	ledtogl();
	osDelay(tickova/2);
}

void treptrep(){
	trep(200);
	trep(200);
	osDelay(600);
	ledon();
	osDelay(4000);
}






/*
 * Analog Filter template
 */
void analogFilterProcedura_TEMPLATE() {
	// ove promenljive moraju biti raspolozive kao globalne
	uint32_t samplesMax = 10;
	uint32_t samplesKoristi = 7;
	uint32_t ulazaMax = 8;
	uint32_t ulazaKoristi = 5;
	uint32_t istorija[samplesMax][ulazaMax];
	uint32_t najnoviji[ulazaKoristi];
	float filtered[ulazaKoristi];		// rezultat filtriranja

	// samo ovo je privatna promenljiva
	float tmp = 0;

	for (int sempl = 0; sempl < samplesKoristi; ++sempl) {

		// prebaci najnovije merenje u istoriju
		for (int ulaz = 0; ulaz < ulazaKoristi; ++ulaz) {
			// jedan po jedan ulaz
			istorija[sempl][ulaz] = najnoviji[ulaz];

			// kad smo vec tu, za isti taj ulaz nadji average svih semplova do sada
			tmp = 0;
			for (int i = 0; i < samplesKoristi; ++i) {
				tmp = tmp + (float) istorija[i][ulaz];
			}
			// srednja vrednost, sacuvaj je
			filtered[ulaz] = tmp / (float) samplesKoristi;

		}
	}
}




/*
 * Proveri stack trenutnog taska
 * ako je manji nego ikad ranije
 * zapisi ga u low watermark u zadati RTC bekap registar
 *
 */
void proveriStack_TEMPLATE(uint8_t reg){
	extern RTC_HandleTypeDef hrtc;
	uint32_t sad = uxTaskGetStackHighWaterMark(NULL);
	uint32_t prev = HAL_RTCEx_BKUPRead(&hrtc, reg);
	if (prev == 0) {
		prev = 1024;
	}
	if (sad < prev) {
		HAL_RTCEx_BKUPWrite(&hrtc, reg, sad);
	}
}






