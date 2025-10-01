/*
 * CDC.h
 *
 *  Created on: Sep 2, 2025
 *      Author: aparady
 */

#ifndef INC_CDC_H_
#define INC_CDC_H_

#include <stdint.h>

void CDC_write_flag(uint8_t flag);

void CDC_write_status(uint8_t status);

void CDC_write_hexadecimal(uint8_t hex_data);

void CDC_write_address(uint8_t address);

void CDC_write_PEC(uint8_t PEC);

#endif /* INC_CDC_H_ */
