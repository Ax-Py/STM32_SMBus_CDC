/*
 * CDC.c
 *
 *  Created on: Sep 2, 2025
 *      Author: aparady
 */

#include "CDC.h"
#include "tusb.h"

// Lookup table for conversion from hex to ASCII char
const char ASCII_Table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void CDC_write_flag(uint8_t flag){
	tud_cdc_write_char(ASCII_Table[flag & 0x01]);
}

void CDC_write_status(uint8_t status){
	tud_cdc_write_char(ASCII_Table[status >> 4]);
	tud_cdc_write_char(ASCII_Table[status & 0x0F]);
}

void CDC_write_hexadecimal(uint8_t hex_data){
	tud_cdc_write_char(ASCII_Table[hex_data >> 4]);
	tud_cdc_write_char(ASCII_Table[hex_data & 0x0F]);
	tud_cdc_write_char('$');
}

void CDC_write_address(uint8_t address){
	tud_cdc_write_char(ASCII_Table[address >> 4]);
	tud_cdc_write_char(ASCII_Table[address & 0x0F]);
	tud_cdc_write_char(':');
}

void CDC_write_PEC(uint8_t PEC){
	tud_cdc_write_char('|');
	tud_cdc_write_char(ASCII_Table[PEC >> 4]);
	tud_cdc_write_char(ASCII_Table[PEC & 0x0F]);
}
