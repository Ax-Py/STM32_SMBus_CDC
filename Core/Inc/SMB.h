/*
 * SMB.h
 *
 *  Created on: Aug 5, 2025
 *      Author: aparady
 */

#ifndef INC_SMB_H_
#define INC_SMB_H_

#include <stdint.h>

// This forms the high nibble of the current system state
typedef enum {
	SMB_NO_ERROR							= 0x00,	// Default state
	SMB_START_WRITE_ISSUED					= 0x10, // A command to issue a START + WRITE condition was detected
	SMB_START_READ_ISSUED					= 0x20, // A command to issue a START + READ condition was detected
	SMB_WRITE_ISSUED						= 0x30, // A command to transmit a data byte was issued
	SMB_READ_ISSUED							= 0x40, // A command to receive a data byte was issued
	SMB_STOP_ISSUED							= 0x50, // A command to issue a STOP condition was detected
	SMB_ADDRESS_SET							= 0x60, // Address byte was set in CR2
	SMB_NBYTES_SET							= 0x70, // NBYTES data was set in CR2
	CDC_OUT_OF_MEMORY						= 0x80,	// More bytes were received than can fit into data buffer
} SMB_CONTROL_STATE;

// This forms the low nibble of the current system state
typedef enum {
	SMB_TRANSACTION_NO_ERROR				= 0x00, // No error occurred during transaction
	SMB_TRANSACTION_NACK					= 0x01, // NACK was received during transaction
	SMB_TRANSACTION_TIMEOUT					= 0x02, // Timeout elapsed during transaction
	SMB_TRANSACTION_BUS_ERROR				= 0x03, // BERR was set during transaction
	SMB_TRANSACTION_ARBITRATION_LOST		= 0x04, // ARLO was set during transaction
	SMB_TRANSACTION_INCORRECT_BYTE_ORDER	= 0x05, // Bytes in SMB_transaction were out of required order
	SMB_TRANSACTION_NBYTES_ZERO				= 0x06  // NBYTES in SMB_transaction was set to zero
} SMB_TRANSACTION_STATE;

void SMB_FSM_init(void);

SMB_CONTROL_STATE SMB_FSM_update_control_state(SMB_CONTROL_STATE control_state);
SMB_TRANSACTION_STATE SMB_FSM_update_transaction_state(SMB_TRANSACTION_STATE transaction_state);

uint8_t SMB_FSM_get_current_state(void);
void SMB_FSM_update_current_state(SMB_CONTROL_STATE control_state, SMB_TRANSACTION_STATE transaction_state);

// Timeout timer commands
void start_timer(void);
void stop_timer(void);

// Base functions
void SMB_start_write(void);
void SMB_start_read(void);
void SMB_stop(void);
void SMB_reset(void);
void SMB_write_byte(uint8_t data_byte);
uint8_t SMB_read_byte(void);

// Compound functions
SMB_TRANSACTION_STATE SMB_check_flag_set(uint32_t SMB_flag);
SMB_TRANSACTION_STATE SMB_check_flag_cleared(uint32_t SMB_flag);
void SMB_set_speed(char speed_select);
uint8_t SMB_scan_addresses(void);
void SMB_transaction(uint16_t *data_array, uint16_t array_length, uint8_t pec_enabled);

#endif /* INC_SMB_H_ */
