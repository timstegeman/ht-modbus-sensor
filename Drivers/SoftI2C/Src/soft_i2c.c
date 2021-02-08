/*
 * soft_i2c.c
 *
 *  Created on: Feb 6, 2021
 *      Author: tims
 */
#include "soft_i2c.h"

extern void soft_i2c_init_platform(void);
extern void soft_i2c_set_sda(enum i2c_pin_state_t state);
extern void soft_i2c_set_scl(enum i2c_pin_state_t state);
extern enum i2c_pin_state_t soft_i2c_get_sda(void);
extern enum i2c_pin_state_t soft_i2c_get_scl(void);
extern void soft_i2c_delay(void);
extern uint32_t soft_i2c_get_ms_tick(void);

bool soft_i2c_scl_high_and_stretch(uint32_t timeout);

enum i2c_result_t soft_i2c_stop(bool allow_clock_stretch) {
	uint32_t timeout = soft_i2c_get_ms_tick() + SOFT_I2C_TIMEOUT_MS;

	// Force SCL low
	soft_i2c_set_scl(LOW);
	soft_i2c_delay();

	// Force SDA low
	soft_i2c_set_sda(LOW);
	soft_i2c_delay();

	// Release SCL
	if (allow_clock_stretch) {
		if (!soft_i2c_scl_high_and_stretch(timeout))
			return TIMEOUT;
	} else {
		soft_i2c_set_scl(HIGH);
	}
	soft_i2c_delay();

	// Release SDA
	soft_i2c_set_sda(HIGH);
	soft_i2c_delay();

	return ACK;
}

void soft_i2c_init() {
	soft_i2c_init_platform();
	soft_i2c_get_sda();
	soft_i2c_get_scl();
}

bool soft_i2c_scl_high_and_stretch(uint32_t timeout) {
	soft_i2c_set_scl(HIGH);

	// Wait for SCL to actually become high in case the slave keeps
	// it low (clock stretching).
	while (soft_i2c_get_scl() == LOW) {
		if (soft_i2c_get_ms_tick() > timeout) {
			soft_i2c_stop(false); // Reset bus. Do not allow clock stretching here
			return false;
		}
	}

	return true;
}

enum i2c_result_t soft_i2c_ll_write(uint8_t data) {
	uint32_t timeout = soft_i2c_get_ms_tick() + SOFT_I2C_TIMEOUT_MS;

	for (uint8_t i = 8; i; --i) {
		// Force SCL low
		soft_i2c_set_scl(LOW);

		if (data & 0x80) {
			// Release SDA
			soft_i2c_set_sda(HIGH);
		} else {
			// Force SDA low
			soft_i2c_set_sda(LOW);
		}
		soft_i2c_delay();

		// Release SCL
		if (!soft_i2c_scl_high_and_stretch(timeout))
			return TIMEOUT;

		soft_i2c_delay();

		data <<= 1;
		if (soft_i2c_get_ms_tick() > timeout) {
			soft_i2c_stop(true); // Reset bus
			return TIMEOUT;
		}
	}

	// Get ACK
	// Force SCL low
	soft_i2c_set_scl(LOW);

	// Release SDA
	soft_i2c_set_sda(HIGH);

	soft_i2c_delay();

	// Release SCL
	if (!soft_i2c_scl_high_and_stretch(timeout))
		return TIMEOUT;

	enum i2c_result_t res = (soft_i2c_get_sda() == LOW ? ACK : NACK);

	soft_i2c_delay();

	// Keep SCL low between bytes
	soft_i2c_set_scl(LOW);

	return res;
}

enum i2c_result_t soft_i2c_ll_start(uint8_t rawAddr) {

	// Force SDA low
	soft_i2c_set_sda(LOW);
	soft_i2c_delay();

	// Force SCL low
	soft_i2c_set_scl(LOW);
	soft_i2c_delay();
	return soft_i2c_ll_write(rawAddr);
}

enum i2c_result_t soft_i2c_ll_read(uint8_t *data, bool send_ack) {
	uint32_t timeout = soft_i2c_get_ms_tick() + SOFT_I2C_TIMEOUT_MS;
	*data = 0;

	for (uint8_t i = 8; i; --i) {
		*data <<= 1;

		// Force SCL low
		soft_i2c_set_scl(LOW);

		// Release SDA (from previous ACK)
		soft_i2c_set_sda(HIGH);
		soft_i2c_delay();

		// Release SCL
		if (!soft_i2c_scl_high_and_stretch(timeout)) {
			return TIMEOUT;
		}
		soft_i2c_delay();

		// Read clock stretch
		while (soft_i2c_get_scl() == LOW)
		if (soft_i2c_get_ms_tick() > timeout) {
			soft_i2c_stop(true); // Reset bus
			return TIMEOUT;
		}

		if (soft_i2c_get_sda()) {
			*data |= 1;
		}
	}

	// Put ACK/NACK

	// Force SCL low
	soft_i2c_set_scl(LOW);
	if (send_ack) {
		// Force SDA low
		soft_i2c_set_sda(LOW);
	}
	else {
		// Release SDA
		soft_i2c_set_sda(HIGH);
	}

	soft_i2c_delay();

	// Release SCL
	if (!soft_i2c_scl_high_and_stretch(timeout)) {
		return TIMEOUT;
	}
	soft_i2c_delay();

	// Wait for SCL to return high
	while (soft_i2c_get_scl() == LOW)
	if (soft_i2c_get_ms_tick() > timeout) {
		soft_i2c_stop(true); // Reset bus
		return TIMEOUT;
	}

	soft_i2c_delay();

	// Keep SCL low between bytes
	soft_i2c_set_scl(LOW);

	return ACK;
}

uint8_t soft_i2c_write(uint8_t address, uint8_t *data, uint8_t len, bool send_stop) {
	enum i2c_result_t r = soft_i2c_ll_start((address << 1) + WRITE_MODE);
	if (r == NACK)
		return 2;
	else if (r == TIMEOUT)
		return 4;

	for (uint8_t i = 0; i < len; ++i) {
		r = soft_i2c_ll_write(data[i]);
		if (r == NACK)
			return 3;
		else if (r == TIMEOUT)
			return 4;
	}

	if (send_stop)
		soft_i2c_stop(true);

	return 0;
}

uint8_t soft_i2c_read(uint8_t address, uint8_t *data, uint8_t len, bool send_stop) {
	uint8_t bytes_read = 0;

	if (soft_i2c_ll_start((address << 1) + READ_MODE) == 0) {
		for (uint8_t i = 0; i < len; ++i) {
			enum i2c_result_t res = soft_i2c_ll_read(&data[i],
					i != (len - 1));
			if (res != ACK)
				break;
			bytes_read++;
		}
	}

	if (send_stop)
		soft_i2c_stop(true);

	return bytes_read;
}
