/*
 * soft_i2c_platform.c
 *
 *  Created on: Feb 6, 2021
 *      Author: tims
 */

#include "main.h"
#include "soft_i2c.h"

void soft_i2c_init_platform(void) {
}

void soft_i2c_set_sda(enum i2c_pin_state_t state) {
	if (state == HIGH) {
		LL_GPIO_SetPinMode(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
		LL_GPIO_MODE_INPUT);
		LL_GPIO_SetOutputPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin);
	} else {
		LL_GPIO_SetPinOutputType(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
		LL_GPIO_OUTPUT_OPENDRAIN);
		LL_GPIO_SetPinMode(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
		LL_GPIO_MODE_OUTPUT);
		LL_GPIO_ResetOutputPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin);
	}
}

void soft_i2c_set_scl(enum i2c_pin_state_t state) {
	if (state == HIGH) {
		LL_GPIO_SetPinMode(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
		LL_GPIO_MODE_INPUT);
		LL_GPIO_SetOutputPin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin);
	} else {
		LL_GPIO_SetPinOutputType(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
		LL_GPIO_OUTPUT_OPENDRAIN);
		LL_GPIO_SetPinMode(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
		LL_GPIO_MODE_OUTPUT);
		LL_GPIO_ResetOutputPin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin);
	}
}

enum i2c_pin_state_t soft_i2c_get_sda(void) {
	LL_GPIO_SetPinMode(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
	LL_GPIO_MODE_INPUT);
	if (LL_GPIO_IsInputPinSet(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin)) {
		return HIGH;
	}
	return LOW;
}

enum i2c_pin_state_t soft_i2c_get_scl(void) {
	LL_GPIO_SetPinMode(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
	LL_GPIO_MODE_INPUT);
	if (LL_GPIO_IsInputPinSet(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin)) {
		return HIGH;
	}
	return LOW;
}

uint32_t soft_i2c_get_ms_tick(void) {
	return HAL_GetTick();
}

void soft_i2c_delay(void) {
	for (uint8_t i = 0; i < SOFT_I2C_DELAY_US; i++) {
		asm("nop");
	}
}

#if 0

void soft_i2c_init_platform(void) {
	//LL_TIM_EnableCounter(TIM1);
}

void soft_i2c_set_sda(enum i2c_pin_state_t state) {
	GPIO_InitTypeDef GPIO_InitStruct = { .Pin = SOFT_I2C_SDA_Pin, .Speed =
			GPIO_SPEED_FREQ_HIGH };
	if (state == HIGH) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
				GPIO_PIN_SET);
	} else {
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin,
				GPIO_PIN_RESET);
	}
	HAL_GPIO_Init(SOFT_I2C_SDA_GPIO_Port, &GPIO_InitStruct);
}

void soft_i2c_set_scl(enum i2c_pin_state_t state) {
	GPIO_InitTypeDef GPIO_InitStruct = { .Pin = SOFT_I2C_SCL_Pin, .Speed =
			GPIO_SPEED_FREQ_HIGH };
	if (state == HIGH) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
				GPIO_PIN_SET);
	} else {
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin,
				GPIO_PIN_RESET);
	}
	HAL_GPIO_Init(SOFT_I2C_SCL_GPIO_Port, &GPIO_InitStruct);
}

enum i2c_pin_state_t soft_i2c_get_sda(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { .Pin = SOFT_I2C_SDA_Pin, .Speed =
			GPIO_SPEED_FREQ_HIGH, .Mode =  GPIO_MODE_INPUT };
	HAL_GPIO_Init(SOFT_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

	if (HAL_GPIO_ReadPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin)) {
		return HIGH;
	}
	return LOW;
}

enum i2c_pin_state_t soft_i2c_get_scl(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { .Pin = SOFT_I2C_SCL_Pin, .Speed =
			GPIO_SPEED_FREQ_HIGH, .Mode =  GPIO_MODE_INPUT };
	HAL_GPIO_Init(SOFT_I2C_SCL_GPIO_Port, &GPIO_InitStruct);

	if (HAL_GPIO_ReadPin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin)) {
		return HIGH;
	}
	return LOW;
}

uint32_t soft_i2c_get_ms_tick(void) {
	return HAL_GetTick();
}

void soft_i2c_delay(void) {
	uint32_t start = TIM1->CNT;
	while ((TIM1->CNT - start) < SOFT_I2C_DELAY_US) {
		//asm("nop");
	}
}
#endif
