/*
 * soft_i2c.h
 *
 *  Created on: Feb 6, 2021
 *      Author: tims
 */

#ifndef SOFTI2C_INC_SOFT_I2C_H_
#define SOFTI2C_INC_SOFT_I2C_H_

#include <stdbool.h>
#include <stdint.h>

#define SOFT_I2C_TIMEOUT_MS		100
#define SOFT_I2C_DELAY_US		1

enum i2c_pin_state_t {
	LOW = 0, HIGH = 1
};

enum i2c_result_t {
	ACK = 0, NACK = 1, TIMEOUT = 2
};

enum i2c_mode_t {
	WRITE_MODE = 0, READ_MODE = 1
};

void soft_i2c_init();
enum i2c_result_t soft_i2c_stop(bool allow_clock_stretch);
uint8_t soft_i2c_read(uint8_t address, uint8_t *data, uint8_t len, bool send_stop);
uint8_t soft_i2c_write(uint8_t address, uint8_t *data, uint8_t len, bool send_stop);

#endif /* SOFTI2C_INC_SOFT_I2C_H_ */
