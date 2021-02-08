/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "soft_i2c.h"
#include "modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t SHT30_I2C_ADDRESS = 0x44;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef DEBUG
#define LOG(...)	printf(__VA_ARGS__); puts("\r\n")
#else
#define LOG(...)
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t mb_rx_byte;
uint16_t mb_holding_registers[2] = { 0, 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);

void mb_tx(uint8_t *data, uint32_t size) {
	LOG("Transmit modbus data");
	HAL_GPIO_WritePin(USART1_DE_GPIO_Port, USART1_DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, data, size, 100);
	HAL_GPIO_WritePin(USART1_DE_GPIO_Port, USART1_DE_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	mb_rx(mb_rx_byte);
	HAL_UART_Receive_IT(&huart1, &mb_rx_byte, 1);
}

uint8_t mb_read_holding_registers(uint16_t start, uint16_t count) {
	uint8_t i;
	LOG("Read holding register %d, count %d", start, count);

	if (start > sizeof(mb_holding_registers) / sizeof(uint16_t)) {
		return MB_ERROR_ILLEGAL_DATA_ADDRESS;
	}

	if (start + count > sizeof(mb_holding_registers) / sizeof(uint16_t)) {
		return MB_ERROR_ILLEGAL_DATA_ADDRESS;
	}

	for (i = start; i < start + count; i++) {
		mb_response_add(mb_holding_registers[i]);
	}

	return MB_NO_ERROR;
}

void read_sht30(void) {
	uint8_t buf[6];

	LOG("Read SHT30");

	buf[0] = 0x2C;
	buf[1] = 0x06;
	uint8_t err = soft_i2c_write(SHT30_I2C_ADDRESS, buf, 2, true);
	if (err) {
		LOG("error: no response from sht30 [%d]", err);
		return;
	}

	int len = soft_i2c_read(SHT30_I2C_ADDRESS, buf, 6, true);
	if (len != 6) {
		LOG("error: invalid data");
		return;
	}

#ifdef DEBUG
	printf("SHT30 raw data: ");
	for (uint8_t i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
	}
	printf("\r\n");
#endif

	//TODO check CRC buf[2] and buf[5]
	uint16_t temperature_raw = (buf[0] << 8) + (buf[1]);
	uint16_t humidity_raw = (buf[3] << 8) + (buf[4]);

	mb_holding_registers[0] = ((float) temperature_raw) * 0.267033 - 4500.;
	mb_holding_registers[1] = ((float) humidity_raw) * 0.15259;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
#ifdef DEBUG
	initialise_monitor_handles();
#endif
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	soft_i2c_init();
	mb_init(99);

	HAL_UART_Receive_IT(&huart1, &mb_rx_byte, 1);
	uint32_t read_timer = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		mb_process();

		if (HAL_GetTick() > read_timer) {
			read_sht30();
			LOG("Temperature: %d", mb_holding_registers[0]);
			LOG("Humidity: %d", mb_holding_registers[1]);
			read_timer = HAL_GetTick() + 5000;
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {

	}
	LL_SetSystemCoreClock(8000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_9B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/**/
	LL_GPIO_SetOutputPin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin);

	/**/
	LL_GPIO_SetOutputPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin);

	/**/
	LL_GPIO_ResetOutputPin(USART1_DE_GPIO_Port, USART1_DE_Pin);

	/**/
	GPIO_InitStruct.Pin = SOFT_I2C_SCL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SOFT_I2C_SCL_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = SOFT_I2C_SDA_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SOFT_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = USART1_DE_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(USART1_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
