/*
 * modbus.c
 *
 *  Created on: Feb 7, 2021
 *      Author: tims
 */

#include "main.h"
#include "modbus.h"

uint32_t mb_get_tick_ms(void){
	return HAL_GetTick();
}
