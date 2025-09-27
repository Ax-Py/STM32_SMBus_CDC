/*
 * SMB.c
 *
 *  Created on: Aug 5, 2025
 *      Author: aparady
 */

#include "SMB.h"
#include "CDC.h"
#include "tusb.h"
#include "system_defines.h"
#include "stm32c0xx.h"

// These states are used only within SMB.c and can only be used in other files via the getter function
static uint8_t SMB_current_state;
static SMB_CONTROL_STATE SMB_control_state;
static SMB_TRANSACTION_STATE SMB_transaction_state;

extern uint8_t SMB_discovered_addresses[SMB_valid_address_number];

// Set if a timeout occurred when checking for an SMB flag to be set
volatile uint8_t SMB_timeout = 0;

void SMB_FSM_init(void){
	SMB_control_state = SMB_NO_ERROR;
	SMB_transaction_state = SMB_TRANSACTION_NO_ERROR;
	SMB_current_state = SMB_control_state | SMB_transaction_state;
}

SMB_CONTROL_STATE SMB_FSM_update_control_state(SMB_CONTROL_STATE control_state){
	SMB_control_state = control_state;

	return SMB_control_state;
}

SMB_TRANSACTION_STATE SMB_FSM_update_transaction_state(SMB_TRANSACTION_STATE transaction_state){
	SMB_transaction_state = transaction_state;

	return SMB_transaction_state;
}

void SMB_FSM_update_current_state(SMB_CONTROL_STATE control_state, SMB_TRANSACTION_STATE transaction_state){
	SMB_current_state = control_state + transaction_state;
}

uint8_t SMB_FSM_get_current_state(void){
	return SMB_current_state;
}

void start_timer(){
	// Clear current count, enable update interrupt and enable timer
	TIM14->CNT = 0x00;
	TIM14->CR1 &= ~(TIM_CR1_UDIS);
	TIM14->CR1 |= (TIM_CR1_CEN);
}

void stop_timer(){
	// Disable Timer14 and update interrupt
	TIM14->CR1 &= ~(TIM_CR1_CEN);
	TIM14->CR1 |= (TIM_CR1_UDIS);
}

void SMB_start_write(void){
	I2C1->CR2 |= I2C_CR2_START;
	SMB_FSM_update_control_state(SMB_START_WRITE_ISSUED);
}

void SMB_start_read(void){
	I2C1->CR2 |= I2C_CR2_START;
	SMB_FSM_update_control_state(SMB_START_READ_ISSUED);
}

void SMB_stop(void){
	I2C1->CR2 |= I2C_CR2_STOP;
	SMB_FSM_update_control_state(SMB_STOP_ISSUED);
}

void SMB_reset(void){
	// Disable I2C peripheral and set pins back to GPIO mode
	I2C1->CR1 &= ~(I2C_CR1_PE);
	while(I2C1->CR1 & I2C_CR1_PE);

	// Clear PB7 (SDA) and PB6 (SCL)
	GPIOB->MODER &= ~((GPIO_MODER_MODE7) | (GPIO_MODER_MODE6));

	// Set PB7 to input and PB6 to output
	GPIOB->MODER |= ((0 << GPIO_MODER_MODE7_Pos) | (1 << GPIO_MODER_MODE6_Pos));

	// Set PB6 to push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6);

	// Set PB6 LOW
	GPIOB->ODR &= ~(GPIO_ODR_OD6);
	HAL_Delay(1);

	// Pulse SCL up to 9 times to release slave device(s)
	for(uint8_t SCL_pulse = 0; SCL_pulse < 9; SCL_pulse++){

		// Set PB6 HIGH
		GPIOB->ODR |= GPIO_ODR_OD6;
		HAL_Delay(1);

		// If SDA is HIGH, then quit as the slaves have been released
		if(GPIOB->IDR & GPIO_IDR_ID7){
			break;
		}
		else{
			// Set PB6 LOW
			GPIOB->ODR &= ~(GPIO_ODR_OD6);
			HAL_Delay(1);
		}
	}

	// Set PB6 to open-drain
	GPIOB->OTYPER |= (GPIO_OTYPER_OT6);

	// Clear PB7 and PB6
	GPIOB->MODER &= ~((GPIO_MODER_MODE7) | (GPIO_MODER_MODE6));

	// Set PB7 to input and PB6 to alternate function
	GPIOB->MODER |= ((2 << GPIO_MODER_MODE7_Pos) | (2 << GPIO_MODER_MODE6_Pos));

	I2C1->CR1 |= (I2C_CR1_PE);
	while(!(I2C1->CR1 & I2C_CR1_PE));
}

void SMB_write_byte(uint8_t data_byte){
	I2C1->TXDR = data_byte;
	SMB_FSM_update_control_state(SMB_WRITE_ISSUED);
}

uint8_t SMB_read_byte(void){
	SMB_FSM_update_control_state(SMB_READ_ISSUED);
	return I2C1->RXDR;
}

SMB_TRANSACTION_STATE SMB_check_flag_set(uint32_t SMB_flag){
	// If no error occurred and the flag was successfully set, simply return this value
	SMB_TRANSACTION_STATE state = SMB_TRANSACTION_NO_ERROR;

	SMB_timeout = 0;

	// Start 35ms timer
	start_timer();

	while(!(I2C1->ISR & SMB_flag)){
		if((I2C1->ISR & I2C_ISR_NACKF)){
			I2C1->ICR |= (I2C_ICR_NACKCF);
			state = SMB_TRANSACTION_NACK;
			break;
		}
		else if(SMB_timeout){
			state = SMB_TRANSACTION_TIMEOUT;
			SMB_reset();
			break;
		}

		else if((I2C1->ISR & I2C_ISR_BERR)){
			state = SMB_TRANSACTION_BUS_ERROR;
			SMB_reset();
			break;
		}

		else if((I2C1->ISR & I2C_ISR_ARLO)){
			state = SMB_TRANSACTION_ARBITRATION_LOST;
			SMB_reset();
			break;
		}
	}
	stop_timer();
	return state;
}

SMB_TRANSACTION_STATE SMB_check_flag_cleared(uint32_t SMB_flag){
	// If no error occurred and the flag was successfully set, simply return this value
	SMB_TRANSACTION_STATE state = SMB_TRANSACTION_NO_ERROR;

	SMB_timeout = 0;

	// Start 35ms timer
	start_timer();

	while((I2C1->ISR & SMB_flag)){
		if((I2C1->ISR & I2C_ISR_NACKF)){
			I2C1->ICR |= (I2C_ICR_NACKCF);
			state = SMB_TRANSACTION_NACK;
			break;
		}
		else if(SMB_timeout){
			state = SMB_TRANSACTION_TIMEOUT;
			SMB_reset();
			break;
		}

		else if((I2C1->ISR & I2C_ISR_BERR)){
			state = SMB_TRANSACTION_BUS_ERROR;
			SMB_reset();
			break;
		}

		else if((I2C1->ISR & I2C_ISR_ARLO)){
			state = SMB_TRANSACTION_ARBITRATION_LOST;
			SMB_reset();
			break;
		}
	}
	stop_timer();
	return state;
}

void SMB_set_speed(char speed_select){
	switch(speed_select){
		case 'V':
			I2C1->CR1 &= ~(I2C_CR1_PE);
			while(I2C1->CR1 & I2C_CR1_PE);

			I2C1->TIMINGR = 0x9031DBFF;

			I2C1->CR1 |= (I2C_CR1_PE);
			while(!(I2C1->CR1 & I2C_CR1_PE));
			break;

		case 'S':
			I2C1->CR1 &= ~(I2C_CR1_PE);
			while(I2C1->CR1 & I2C_CR1_PE);

			I2C1->TIMINGR = 0x30812E3E;

			I2C1->CR1 |= (I2C_CR1_PE);
			while(!(I2C1->CR1 & I2C_CR1_PE));
			break;

		case 'T':
			I2C1->CR1 &= ~(I2C_CR1_PE);
			while(I2C1->CR1 & I2C_CR1_PE);

			I2C1->TIMINGR = 0x00B21847;

			I2C1->CR1 |= (I2C_CR1_PE);
			while(!(I2C1->CR1 & I2C_CR1_PE));
			break;

		default:
			break;
	}
}

uint8_t SMB_scan_addresses(void){
	// Number of devices that ACK'd their own address.
	uint8_t discovered_address_count = 0;
	uint8_t retry_counter = 0;

	SMB_FSM_update_transaction_state(SMB_TRANSACTION_NO_ERROR);

	// Valid SMBus addresses range from 0x08 to 0x77 (112 addresses)
	for(uint8_t address = 8; address < 120; address++){

		// If the SMBus peripheral timed out too many times, return only the invalid 0xFF address
		if(retry_counter > SMB_timeout_retries){
			discovered_address_count = 1;
			SMB_discovered_addresses[0] = 0xFF;
			break;
		}

		// Start from known state by clearing CR2
		I2C1->CR2 = 0x00;

		// In 7-bit address mode, bit 0 is not used. Set NBYTES to 0 and keep direction as write.
		I2C1->CR2 |= ((address << 1) | (0 << I2C_CR2_NBYTES_Pos) | (SMB_dir_write << I2C_CR2_RD_WRN_Pos));

		SMB_start_write();
		SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TC));
		SMB_FSM_update_current_state(SMB_control_state, SMB_transaction_state);

		// NACK only needs to check that BUSY is cleared as the hardware triggers a STOP condition automatically. TC will never set if STOP is manually written after a NACK.
		switch(SMB_transaction_state){
			case SMB_TRANSACTION_NO_ERROR:
				SMB_discovered_addresses[discovered_address_count] = address;
				discovered_address_count++;

				SMB_stop();
				SMB_FSM_update_transaction_state(SMB_check_flag_cleared(I2C_ISR_BUSY));
				break;
			case SMB_TRANSACTION_NACK:
				SMB_FSM_update_transaction_state(SMB_check_flag_cleared(I2C_ISR_BUSY));
				break;
			default:
				retry_counter++;
				break;
		}
		if(SMB_transaction_state){
			SMB_FSM_update_current_state(SMB_control_state, SMB_transaction_state);
		}else{
			SMB_FSM_update_control_state(SMB_NO_ERROR);
			SMB_FSM_update_current_state(SMB_control_state, SMB_transaction_state);
		}
	}
	return discovered_address_count;
}

void SMB_transaction(uint16_t *data_array, uint16_t array_length, uint8_t pec_enabled){
	uint8_t address = 0;
	uint8_t instruction_byte = 0;
	uint16_t nbytes = 0;

	for(uint16_t data_index = 0; data_index < array_length; data_index++){

		// Error handling
		if(SMB_transaction_state != SMB_TRANSACTION_NO_ERROR){
			SMB_FSM_update_current_state(SMB_control_state, SMB_transaction_state);
			tud_cdc_write_char('\n');

			if(I2C1->ISR & I2C_ISR_BUSY){
				SMB_stop();
				SMB_check_flag_cleared(I2C_ISR_BUSY);
			}
			SMB_reset();
			break;
		}

		switch(data_array[data_index]){
			// Data Byte
			case 0 ... 255:
				if((SMB_control_state != SMB_START_WRITE_ISSUED) && (SMB_control_state != SMB_WRITE_ISSUED)){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
				I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);

				SMB_write_byte(data_array[data_index]);
				if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;
				break;

			// Address field
			case 1000 ... 1255:
				if(SMB_current_state){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				address = data_array[data_index] - 1000;

				I2C1->CR2 = 0x00;
				I2C1->CR2 |= address << 1;

				SMB_FSM_update_control_state(SMB_ADDRESS_SET);
				break;

			// Write Command
			case 2000 ... 2255:
				if(SMB_control_state != SMB_ADDRESS_SET){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				instruction_byte = data_array[data_index] - 2000;

				// Set RELOAD to true and NBYTES to 1. By doing this, the ST can send an infinite number of bytes without knowing the count.
				I2C1->CR2 |= ((SMB_dir_write << I2C_CR2_RD_WRN_Pos) | (1 << I2C_CR2_RELOAD_Pos) | (1 << I2C_CR2_NBYTES_Pos));

				// Pre-load instruction byte
				I2C1->TXDR = instruction_byte;

				SMB_start_write();
				if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;
				break;

			// Read Command
			case 3000 ... 3255:
				if(SMB_control_state != SMB_ADDRESS_SET){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				instruction_byte = data_array[data_index] - 3000;

				I2C1->CR2 |= ((SMB_dir_write << I2C_CR2_RD_WRN_Pos) | (1 << I2C_CR2_NBYTES_Pos));

				// Pre-load instruction byte
				I2C1->TXDR = instruction_byte;

				// For flow control reasons, this is start_read. Technically this is a start_write
				SMB_start_read();
				if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TC))) break;

				I2C1->CR2 &= ~(1 << I2C_CR2_RD_WRN_Pos);
				I2C1->CR2 |= (SMB_dir_read << I2C_CR2_RD_WRN_Pos);
				break;

			// Non-block command NBYTES field
			case 4000 ... 4255:
				if(SMB_control_state != SMB_START_READ_ISSUED){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				nbytes = data_array[data_index] - 4000;

				if(!(nbytes)){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_NBYTES_ZERO);
					break;
				}

				I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
				I2C1->CR2 |= nbytes << I2C_CR2_NBYTES_Pos;

				CDC_write_address(address);

				// If PEC is disabled, then this does nothing. Otherwise prepare for calling one additional byte at the end of the read transaction
				I2C1->CR2 |= pec_enabled << I2C_CR2_RELOAD_Pos;

				SMB_start_read();

				for(uint16_t read_byte = 0; read_byte < nbytes; read_byte++){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_hexadecimal(SMB_read_byte());
				}

				if(pec_enabled){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;

					// NBYTES must be set first to end clock stretching before RELOAD is cleared, turn off RELOAD for the PEC byte to force NACK
					I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
					I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos;

					I2C1->CR2 &= ~(1 << I2C_CR2_RELOAD_Pos);

					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_PEC(SMB_read_byte());
				}

				tud_cdc_write_char('\n');

				SMB_FSM_update_control_state(SMB_NBYTES_SET);
				break;

			// Block command NBYTES field
			case 5000 ... 5255:
				if(SMB_control_state != SMB_START_READ_ISSUED){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				nbytes = data_array[data_index] - 5000;

				if(!(nbytes)){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_NBYTES_ZERO);
					break;
				}

				// The first byte of a block command is always the packet length
				I2C1->CR2 |= 1 << I2C_CR2_RELOAD_Pos;

				I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
				I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos;

				SMB_start_read();
				CDC_write_address(address);

				if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;

				CDC_write_hexadecimal(SMB_read_byte());

				// NBYTES must be set first to end clock stretching before RELOAD is cleared, turn off RELOAD for the PEC byte to force NACK on last byte
				I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
				I2C1->CR2 |= nbytes << I2C_CR2_NBYTES_Pos;

				// If PEC is enabled, this does nothing and the STM32 will prepare for calling one additional byte at the end of the read transaction
				I2C1->CR2 &= ~((!pec_enabled) << I2C_CR2_RELOAD_Pos);

				for(uint16_t read_byte = 0; read_byte < nbytes; read_byte++){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_hexadecimal(SMB_read_byte());
				}

				if(pec_enabled){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;

					I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
					I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos;
					I2C1->CR2 &= ~(1 << I2C_CR2_RELOAD_Pos);

					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_PEC(SMB_read_byte());
				}

				tud_cdc_write_char('\n');

				SMB_FSM_update_control_state(SMB_NBYTES_SET);
				break;

			// Process call NBYTES field
			case 6000 ... 6255:
				if(SMB_control_state != SMB_WRITE_ISSUED){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_INCORRECT_BYTE_ORDER);
					break;
				}

				nbytes = data_array[data_index] - 6000;

				if(!(nbytes)){
					SMB_FSM_update_transaction_state(SMB_TRANSACTION_NBYTES_ZERO);
					break;
				}

				// Reset CR2 and prepare to perform a block read
				I2C1->CR2 = 0x00;
				I2C1->CR2 |= ((address << 1) | (SMB_dir_read << I2C_CR2_RD_WRN_Pos) | (1 << I2C_CR2_RELOAD_Pos) | (1 << I2C_CR2_NBYTES_Pos));

				SMB_start_read();
				CDC_write_address(address);

				if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;

				CDC_write_hexadecimal(SMB_read_byte());

				// NBYTES must be set first to end clock stretching before RELOAD is cleared, turn off RELOAD for the PEC byte to force NACK on last byte
				I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
				I2C1->CR2 |= nbytes << I2C_CR2_NBYTES_Pos;

				// If PEC is enabled, this does nothing and the STM32 will prepare for calling one additional byte at the end of the read transaction
				I2C1->CR2 &= ~((!pec_enabled) << I2C_CR2_RELOAD_Pos);

				for(uint16_t read_byte = 0; read_byte < nbytes; read_byte++){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_hexadecimal(SMB_read_byte());
				}

				if(pec_enabled){
					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_TCR))) break;

					I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);
					I2C1->CR2 |= 1 << I2C_CR2_NBYTES_Pos;
					I2C1->CR2 &= ~(1 << I2C_CR2_RELOAD_Pos);

					if(SMB_FSM_update_transaction_state(SMB_check_flag_set(I2C_ISR_RXNE))) break;
					CDC_write_PEC(SMB_read_byte());
				}

				tud_cdc_write_char('\n');

				SMB_FSM_update_control_state(SMB_NBYTES_SET);
				break;

			case 7000:
				// Send STOP condition
				if(I2C1->ISR & I2C_ISR_BUSY){
					SMB_stop();
					SMB_FSM_update_transaction_state(SMB_check_flag_cleared(I2C_ISR_BUSY));
				}
				break;

			default:
				break;
		}
	}
}
