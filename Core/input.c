/*
 * input.c
 *
 *  Created on: Mar 4, 2020
 *      Author: sahoe
 */

#include "main.h"
#include "cmsis_os.h"

static const uint8_t BTN_ADDR = 0x20 << 1;
static const uint8_t REG_BTN = 0x09;
static const uint8_t GPINTEN_ADDR = 0x02;
static const uint8_t IODIR_ADDR = 0x00;

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId_t InputSemaphoreHandle;
static void ButtonUpdate(void);

HAL_StatusTypeDef ret;
uint32_t freq; //for synthesizer
uint32_t note; // for MIDI

//GPININT enable

void StartInputTask(void *argument) {
	uint8_t buf[2] = { GPINTEN_ADDR, 0xFF };
	ret = HAL_I2C_Master_Transmit(&hi2c1, BTN_ADDR, buf, 2, HAL_MAX_DELAY);

	buf[0] = REG_BTN;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BTN_ADDR, buf, 1, HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1, BTN_ADDR, buf, 1, HAL_MAX_DELAY);

	/* Infinite loop */
	for (;;) {
		osSemaphoreAcquire(InputSemaphoreHandle, 2000U);
		ButtonUpdate();
		//osDelay(1);

	}
}

//Frequency
static void ButtonUpdate(void) {

	uint8_t buf[2] = { GPINTEN_ADDR, 0xFF };
	ret = HAL_I2C_Master_Transmit(&hi2c1, BTN_ADDR, buf, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK) {

	} else {
		ret = HAL_I2C_Master_Receive(&hi2c1, BTN_ADDR, buf, 1,
		HAL_MAX_DELAY);
		if (ret != HAL_OK) {

		} else {

			buf[0] = ~buf[0];

			//CONVERTING TO 8 BIT BINARY
			uint8_t btn[8];
			uint8_t btnRaw = buf[0];

			uint8_t mask = 1;

			for (int i = 0; i < 8; i++) {
				if (btnRaw & mask) {
					btn[i] = 1;
				} else {
					btn[i] = 0;
					mask = mask * 2;  //initially << 1

				}
			}
		}

	}
	uint32_t b = buf[0];
	if (b & 0x01) {
		freq = 196000;
		note = 67;
	} else if (b & 0x02) {
		freq = 220000;
		note = 69;
	} else if (b & 0x04) {
		freq = 246940;
		note = 71;
	} else if (b & 0x08) {
		freq = 261630;
		note = 72;
	} else if (b & 0x10) {
		freq = 293660;
		note = 74;
	} else if (b & 0x20) {
		freq = 329630;
		note = 76;
	} else if (b & 0x40) {
		freq = 349230;
		note = 77;
	} else if (b & 0x80) {
		freq = 392000;
		note = 79;
	} else {
		freq = 440000;
		note = 81;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	osSemaphoreRelease(InputSemaphoreHandle);
}

