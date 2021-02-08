/*
 * modbus.h
 *
 *  Created on: Feb 7, 2021
 *      Author: tims
 */

#include <stdint.h>

#ifndef MODBUS_INC_MODBUS_H_
#define MODBUS_INC_MODBUS_H_

#define MB_READ_COIL_STATUS         0x01
#define MB_READ_INPUT_STATUS        0x02
#define MB_READ_HOLDING_REGISTERS   0x03
#define MB_READ_INPUT_REGISTERS     0x04
#define MB_WRITE_SINGLE_COIL        0x05
#define MB_WRITE_SINGLE_REGISTER    0x06
#define MB_WRITE_MULTIPLE_COILS     0x10
#define MB_WRITE_MULTIPLE_REGISTERS 0x0F

#define MB_NO_ERROR						0x00
#define MB_ERROR_ILLEGAL_FUNCTION       0x01
#define MB_ERROR_ILLEGAL_DATA_ADDRESS   0x02
#define MB_ERROR_ILLEGAL_DATA_VALUE     0x03
#define MB_ERROR_SLAVE_DEVICE_FAILURE   0x04

#define MB_RX_BUF_SIZE              64
#define MB_TX_BUF_SIZE              64

typedef enum {
    MB_DATA_READY,
    MB_DATA_INCOMPLETE,
    MB_INVALID_SLAVE_ADDRESS,
    MB_INVALID_FUNCTION
} mb_state_t;

void mb_init(uint8_t slave_address);
void mb_rx(uint8_t b);
void mb_response_add(uint16_t value);
void mb_response_reset(uint8_t fn);
void mb_process();

#endif /* MODBUS_INC_MODBUS_H_ */
