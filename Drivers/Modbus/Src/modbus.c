/*
 * modbus.c
 *
 *  Created on: Feb 7, 2021
 *      Author: tims
 */

#include "modbus.h"
#include <string.h>

uint8_t mb_slave_address = 1;
uint8_t mb_request_buf[MB_RX_BUF_SIZE];
uint8_t mb_request_buf_index = 0;
uint8_t mb_response_buf[MB_TX_BUF_SIZE];
uint8_t mb_response_buf_index = 0;
uint32_t mb_timeout;

extern uint32_t mb_get_tick_ms(void);
extern void mb_tx(uint8_t *data, uint32_t len);

__attribute__((weak)) uint8_t mb_read_coil_status(uint16_t start, uint16_t count) {
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_read_input_status(uint16_t start, uint16_t count) {
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_read_holding_registers(uint16_t start, uint16_t count) {
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_read_input_registers(uint16_t start, uint16_t count) {
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

void mb_rx(uint8_t b) {
	mb_request_buf[mb_request_buf_index++] = b;
	mb_timeout = mb_get_tick_ms();
}

uint16_t mb_calc_crc16(uint8_t *buf, uint8_t len) {
	uint16_t crc = 0xFFFF;
	uint8_t i, j = 0;
	while (j < len) {
		crc ^= buf[j];
		for (i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc >>= 1;
				crc ^= 0xA001;
			} else
				crc >>= 1;
		}
		j++;
	}
	return crc;
}

void mb_init(uint8_t slave_address) {
	mb_slave_address = slave_address;
}

mb_state_t mb_check_buf() {
	if (mb_request_buf_index > 4) {
		if (mb_request_buf[0] != mb_slave_address) {
			return MB_INVALID_SLAVE_ADDRESS;
		}

		if (mb_request_buf[1] >= 0x01 && mb_request_buf[1] <= 0x06) {
			if (mb_request_buf_index == 8) {
				return MB_DATA_READY;
			}
		} else if (mb_request_buf[1] == 0x10 || mb_request_buf[1] == 0x0F) {
			if (mb_request_buf_index == mb_request_buf[6] + 9) {
				return MB_DATA_READY;
			}
		} else {
			return MB_INVALID_FUNCTION;
		}
	}

	return MB_DATA_INCOMPLETE;
}

void mb_reset_buf() {
	mb_request_buf_index = 0;
	memset(mb_request_buf, 0, sizeof(mb_request_buf));
}

void mb_response_tx() {
	// Calculate CRC
	uint16_t crc = mb_calc_crc16(mb_response_buf, mb_response_buf_index);
	mb_response_buf[mb_response_buf_index++] = crc & 0xFF;
	mb_response_buf[mb_response_buf_index++] = (crc & 0xFF00) >> 8;

	// Send RTU packet
	mb_tx(mb_response_buf, mb_response_buf_index);
}

void mb_error(uint8_t err) {
	mb_response_buf_index = 0;
	mb_response_buf[mb_response_buf_index++] = mb_slave_address;
	mb_response_buf[mb_response_buf_index++] = mb_request_buf[1] | 0x80;
	mb_response_buf[mb_response_buf_index++] = err;
	mb_response_tx();
}

void mb_response_add(uint16_t value) {
	mb_response_buf[2] += 2;
	mb_response_buf[mb_response_buf_index++] = (value & 0xFF00) >> 8;
	mb_response_buf[mb_response_buf_index++] = value & 0xFF;
}

void mb_response_reset(uint8_t fn) {
	mb_response_buf_index = 0;
	mb_response_buf[mb_response_buf_index++] = mb_slave_address;
	mb_response_buf[mb_response_buf_index++] = fn;
	mb_response_buf[mb_response_buf_index++] = 0;
}

void mb_rx_rtu() {
	// Check CRC
	uint16_t crc = mb_calc_crc16(mb_request_buf, mb_request_buf_index - 2);
	if (memcmp(&crc, &mb_request_buf[mb_request_buf_index - 2], sizeof(crc))
			!= 0) {
		mb_reset_buf();
		return;
	}

	uint8_t res = 0;
	mb_response_reset(mb_request_buf[1]);

	switch (mb_request_buf[1]) {

	case MB_READ_COIL_STATUS:
		res = mb_read_coil_status((mb_request_buf[2] << 8) + mb_request_buf[3],
				(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_INPUT_STATUS:
		res = mb_read_input_status((mb_request_buf[2] << 8) + mb_request_buf[3],
				(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_HOLDING_REGISTERS:
		res = mb_read_holding_registers(
				(mb_request_buf[2] << 8) + mb_request_buf[3],
				(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_INPUT_REGISTERS:
		res = mb_read_input_registers(
				(mb_request_buf[2] << 8) + mb_request_buf[3],
				(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_WRITE_SINGLE_COIL:
		//TODO
	case MB_WRITE_SINGLE_REGISTER:
		//TODO
	case MB_WRITE_MULTIPLE_COILS:
		//TODO
	case MB_WRITE_MULTIPLE_REGISTERS:
		//TODO
	default:
		res = MB_ERROR_ILLEGAL_FUNCTION;
		break;
	}

	if (res == MB_NO_ERROR) {
		mb_response_tx();
	} else {
		mb_error(res);
	}

	mb_reset_buf();
}

void mb_process() {
	if (mb_get_tick_ms() - mb_timeout > 10) {
		mb_reset_buf();
		return;
	}

	mb_state_t mb_state = mb_check_buf();
	switch (mb_state) {

	case MB_INVALID_FUNCTION:
		mb_error(MB_ERROR_ILLEGAL_FUNCTION);
	case MB_INVALID_SLAVE_ADDRESS:
		mb_reset_buf();
		break;
	case MB_DATA_READY:
		mb_rx_rtu();
	case MB_DATA_INCOMPLETE:
		break;
	}

}
