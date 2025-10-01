# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:        STM32_SMBus.py
# Purpose:     Driver Class for COCO11 USB to SMBus module by AParady
#
# Author:      AParady, AParady@vicr.com
#
# Created:     09/15/2025
#-------------------------------------------------------------------------------

import serial
import serial.tools.list_ports as sertools
import re

status_codes = {
	'00'   : 'NO_ERROR',
	'01'   : 'SMB_TRANSACTION_NACK',
	'02'   : 'SMB_TRANSACTION_TIMEOUT',
	'03'   : 'SMB_TRANSACTION_BUS_ERROR',
	'04'   : 'SMB_TRANSACTION_ARBITRATION_LOST',
	'05'   : 'SMB_TRANSACTION_INCORRECT_BYTE_ORDER',
	'06'   : 'SMB_TRANSACTION_NBYTES_ZERO',
    
	'10'   : 'SMB_START_WRITE_NO_ERROR',
	'11'   : 'SMB_START_WRITE_NACK',
	'12'   : 'SMB_START_WRITE_TIMEOUT',
	'13'   : 'SMB_START_WRITE_BUS_ERROR',
	'14'   : 'SMB_START_WRITE_ARBITRATION_LOST',
	'15'   : 'SMB_START_WRITE_INCORRECT_BYTE_ORDER',
    '16'   : 'SMB_START_WRITE_NBYTES_ZERO',
    
	'20'   : 'SMB_START_READ_NO_ERROR',
	'21'   : 'SMB_START_READ_NACK',
	'22'   : 'SMB_START_READ_TIMEOUT',
	'23'   : 'SMB_START_READ_BUS_ERROR',
	'24'   : 'SMB_START_READ_ARBITRATION_LOST',
	'25'   : 'SMB_START_READ_INCORRECT_BYTE_ORDER',
    '26'   : 'SMB_START_READ_NBYTES_ZERO',
    
	'30'   : 'SMB_DATA_WRITE_NO_ERROR',
	'31'   : 'SMB_DATA_WRITE_NACK',
	'32'   : 'SMB_DATA_WRITE_TIMEOUT',
	'33'   : 'SMB_DATA_WRITE_BUS_ERROR',
	'34'   : 'SMB_DATA_WRITE_ARBITRATION_LOST',
	'35'   : 'SMB_DATA_WRITE_INCORRECT_BYTE_ORDER',
    '36'   : 'SMB_DATA_WRITE_NBYTES_ZERO',
	'40'   : 'SMB_DATA_READ_NO_ERROR',
    
	'41'   : 'SMB_DATA_READ_NACK',
	'42'   : 'SMB_DATA_READ_TIMEOUT',
	'43'   : 'SMB_DATA_READ_BUS_ERROR',
	'44'   : 'SMB_DATA_READ_ARBITRATION_LOST',
	'45'   : 'SMB_DATA_READ_INCORRECT_BYTE_ORDER',
    '46'   : 'SMB_DATA_READ_NBYTES_ZERO',
    
	'50'   : 'SMB_STOP_NO_ERROR',
	'51'   : 'SMB_STOP_NACK',
	'52'   : 'SMB_STOP_TIMEOUT',
	'53'   : 'SMB_STOP_BUS_ERROR',
	'54'   : 'SMB_STOP_ARBITRATION_LOST',
	'55'   : 'SMB_STOP_INCORRECT_BYTE_ORDER',
    '56'   : 'SMB_STOP_NBYTES_ZERO',
    
    '60'   : 'SMB_ADDRESS_SET_NO_ERROR',
	'61'   : 'SMB_ADDRESS_SET_NACK',
	'62'   : 'SMB_ADDRESS_SET_TIMEOUT',
	'63'   : 'SMB_ADDRESS_SET_BUS_ERROR',
	'64'   : 'SMB_ADDRESS_SET_ARBITRATION_LOST',
	'65'   : 'SMB_ADDRESS_SET_INCORRECT_BYTE_ORDER',
    '66'   : 'SMB_ADDRESS_SET_NBYTES_ZERO',
    
    '70'   : 'SMB_NBYTES_SET_NO_ERROR',
	'71'   : 'SMB_NBYTES_SET_NACK',
	'72'   : 'SMB_NBYTES_SET_TIMEOUT',
	'73'   : 'SMB_NBYTES_SET_BUS_ERROR',
	'74'   : 'SMB_NBYTES_SET_ARBITRATION_LOST',
	'75'   : 'SMB_NBYTES_SET_INCORRECT_BYTE_ORDER',
    '76'   : 'SMB_NBYTES_SET_NBYTES_ZERO',
    
    '80'   : 'CDC_OUT_OF_MEMORY'
}

crc8_table = [
0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]

class SMB:
    
    def __init__(self, comport = None, timeout = 1, baudrate = 2000000):
        
        self.compatible_vid = 0xC0C0
        self.compatible_pid = 0x0011
        
        self.device_connected = 0
        
        if(comport == None):
            raise ValueError ("No COM port selected!")
        
        self.device = serial.Serial()
        self.device.port = comport
        self.device.timeout = timeout
        self.device.baudrate = baudrate
        
        for port in sertools.comports():
            if comport in port and ((port.vid == self.compatible_vid) and (port.pid == self.compatible_pid)):
                self.device_connected = 1
                break
        
        if(self.device_connected != 1):
            raise ValueError ('Device not found!')
            
        self.device.open()
        
        self.set_PEC(False)
            
            
    def connection_status(self) -> bool:
        current_connection_status = False
        
        if(self.device_connected == 1):
            current_connection_status = True
            
        return current_connection_status
        
    
    def close(self):
        """
        This function closes the COM port link between the PC and the Arduino. 
        The link must be closed as it will prevent re-opening.
        
        It is required to call this function at the end of a script. 
        It is best practice is to include this in a finally clause so that it 
        runs no matter what happens when the script runs.

        Returns
        -------
        None.

        """
        
        self.device.reset_input_buffer()
        self.device.reset_output_buffer()
        self.device.close()
        
        
    def write(self, command : str):
        
        self.device.write(str.encode(command.strip('\n') + '\n'))
        
    
    def read(self, num_devices):
        
        returned_data = []
        
        for _ in range(0, num_devices, 1):
            returned_data.append(self.device.readline().decode('ASCII').strip('\n'))
            
        return returned_data
    
    
    def write_read_command_character(self, command_character : str) -> str:
        
        self.write(command_character)
        
        return self.device.readline().decode('ASCII').strip('\n')
    
    
    def write_command_character(self, command_character : str):
        
        self.write(command_character)
        
        
    def get_bus_status(self) -> str:
        return status_codes[self.write_read_command_character('!')]
    
    
    def set_frequency_10khz(self) -> str:
        return self.write_read_command_character('V')
    
    
    def set_frequency_100khz(self) -> str:
        return self.write_read_command_character('S')
    
    
    def set_frequency_400khz(self) -> str:
        return self.write_read_command_character('T')
    
    
    def find_devices(self) -> str:
        return self.write_read_command_character('?')
        
    
    def set_PEC(self, enabled : bool):
        """
        Tell the STM32 to request the PEC byte after each SMBus read.

        Parameters
        ----------
        enabled : bool
            Set PEC read to be enabled (01) or disabled (00).

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        state = ''
        
        if(enabled == True):
            self.write('01=')
            self.pec_enabled = True
            
            state = 'PEC:1'
        else:
            self.write('00=')
            self.pec_enabled = False
            
            state = 'PEC:0'
            
        if(self.read(1)[0] != state):
            raise SMBStatusError (f'A Bus error occurred: {state}')
        
    
    def address_type_check(self, address : str):
        try:
            if(int(address, 16) > 255):
                raise ValueError
        except (ValueError, TypeError):
            raise DataEntryError (f'Input data "{address}" is not a valid 8-bit hexadecimal string or is empty!')
            

    def instruction_type_check(self, instruction : str):
        try:
            if(int(instruction, 16) > 255):
                raise ValueError
        except (ValueError, TypeError):
            raise DataEntryError (f'Instruction byte "{instruction}" is not a valid 8-bit hexadecimal string or is empty!')
        
        
    def data_type_check(self, data : str):
        try:
            if(int(data, 16) > 255):
                raise ValueError
        except (ValueError, TypeError):
            raise DataEntryError (f'Address "{data}" is not a valid 8-bit hexadecimal string or is empty!')
            
    
    def calculate_PEC(self, slave_address: str, instruction_byte : str, data_byte_array : list[str]) -> int:
        """
        Calculate what the PEC byte should be based on received data from slave
        address(es).

        Parameters
        ----------
        slave_address : str
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        data_byte_array : list[str]
            Array of raw data received back from the slave address(es).

        Returns
        -------
        int
            Calculated PEC byte.

        """
        
        calculated_pec_byte = 0
        
        calculated_pec_byte = crc8_table[calculated_pec_byte ^ int(slave_address, 16) << 1]
        calculated_pec_byte = crc8_table[calculated_pec_byte ^ int(instruction_byte, 16)]
        calculated_pec_byte = crc8_table[calculated_pec_byte ^ (int(slave_address, 16) << 1) + 1]
        
        for data_byte in data_byte_array:
            calculated_pec_byte = crc8_table[calculated_pec_byte ^ int(data_byte, 16)]
            
        return calculated_pec_byte
            
            
    def generate_data_dictionary(self, instruction_byte : int, num_devices : int, return_pec : bool = False):
        """
        Parse the data received over USB CDC into a dictionary where each 
        slave device address is the key and the rest of the data is the value. 
        This is the heart of the program.

        Parameters
        ----------
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        num_devices : int
            Number of slave devices that were read from.
        return_pec : bool, optional
            Add PEC data to the dictionary entry for the user to view.
            The default is False.

        Raises
        ------
        SMBPECError
            The PEC byte read back from the slave device does not match the
            value that was calculated using the returned data.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        data_dictionary = {}
        
        raw_data = self.read(num_devices)
        
        for data_packet in raw_data:
            
            data_address = re.findall(r'(.*?)\:', data_packet)[0]
            
            data_dictionary[data_address] = re.findall(r'\:(.*\$)', data_packet)[0]
            
            if(self.pec_enabled == True):
                
                received_pec_byte = 0
                calculated_pec_byte = 0
                
                received_pec_byte = re.findall(r'\|(.*)', data_packet)[0]
                
                calculated_pec_byte = self.calculate_PEC(data_address, instruction_byte, data_dictionary[data_address].split('$')[:-1])
        
                if(calculated_pec_byte != int(received_pec_byte,16)):
                    raise SMBPECError('Calculated PEC byte does not match the received PEC byte!')
                    
                if(return_pec == True):
                    data_dictionary[data_address] = f'{data_dictionary[data_address]}|{received_pec_byte}'
                    
                    
        return data_dictionary
    
    
    def linear11_to_decimal(self, value : int) -> float:
        """
        Convert a Linear11 value read back from a DUT to decimal.
        
        The input of this function is expecting a decimal representation of a 16-bit hexadecimal value.
        
        e.g. F238h = 62008 in decimal. This should convert to 142.

        Parameters
        ----------
        value : int
            Linear11 encoded value to be converted to decimal.

        Returns
        -------
        float
            Decimal interpretation of the Linear11 value.

        """
        
        N = 0 # Exponent
        M = 0 # Mantissa
        
        # Get the exponent by checking if the sign of the MSB is negative and negate 2's complement if true
        if ((value >> 15) & 0x01):
            N = -(((value >> 11) ^ 0x1F) + 1)
        else:
            N = value >> 11
            
        # Get the mantissa by checking if sign of the 11 LSBs is negative and negate 2's complement if true
        if (value & 0x400):
            M = -(((value & 0x3FF) ^ 0x3FF) + 1)
        else:
            M = value & 0x7FF
            
        return M * (2 ** N)


    def decimal_to_linear11(self, value : float) -> tuple[str, str]:
        """
        Encode a decimal float into Linear11.

        Parameters
        ----------
        value : float
            Raw decimal value to be encoded.

        Raises
        ------
        ValueError
            Input value is too big or small to fit into the range specified by
            the Linear11 format.

        Returns
        -------
        tuple[str, str]
            Low byte and high byte to be sent directly over SMBus.

        """
        
        N = 0 # Exponent
        M = 0 # Mantissa

        if (0 <= value < 33521664):
            
            # Find the smallest exponent that equals the input. This will find the finest resolution.
            for exponent in range(-16, 16, 1):
                
                # If the input value is smaller than the maximum calculated value, then this exponent should be used to calculate the mantissa bits.
                if (value < (1023 * (2 ** exponent))):
                    M = round(value / (2 ** exponent))

                    if (exponent < 0):
                        N = (abs(exponent) ^ 0x1F) + 1
                    else:
                        N = exponent
                    break
        
        elif (0 > value > -33554432):
            
            # Find the smallest exponent that equals the input. This will find the finest resolution.
            for exponent in range(-16, 16, 1):
                
                # If the input value is smaller than the minimum calculated value, then this exponent should be used.
                if(value > (-1024 * (2 ** exponent))):
                    M = (abs(round(value / (2 ** exponent))) ^ 0x7FF) + 1

                    if (exponent < 0):
                        N = (abs(exponent) ^ 0x1F) + 1
                    else:
                        N = exponent
                    break
            
        else:
            raise ValueError ("Input value to too big or small to be encoded into Linear11!")

        high_byte = ((N << 11) + M) >> 8
        low_byte = ((N << 11) + M) & 0xFF
        
        return (f'{low_byte:02x}', f'{high_byte:02x}')
            

    def linear16_to_decimal(self, value : int, exponent : int) -> int:
        
        # Exponent comes from VOUT_MODE (17h)
        return int(value) * (2 ** exponent)


    def decimal_to_linear16(self, value : float, exponent : int) -> tuple[str, str]:

        high_byte = round(value / (2 ** (exponent))) >> 8
        low_byte = round(value / (2 ** (exponent))) & 0xFF

        return (f'{low_byte:02x}', f'{high_byte:02x}')
            
            
    def direct_to_decimal (self, value : int, m : int, r : int, b : int) -> float:
        
        return round((1 / m) * ((value * (10 ** (-r))) - b), 4)


    def decimal_to_direct (self, value: float, m : int, r : int, b : int) -> tuple[str, str]:
        
        high_byte = int(((value * m) + b) / (10 ** (-r))) >> 8
        low_byte = int(((value * m) + b) / (10 ** (-r))) & 0xFF

        return (f'{low_byte:02x}', f'{high_byte:02x}')
    
    
    def write_none(self, slave_addresses : tuple[str, ...], instruction_byte : str):
        """
        Write empty data to slave address(es). Used for commands that do not
        accept data to be carried out. Example: CLEAR_FAULTS (03h)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}W'
        
        self.write(command_string)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError (f'A Bus error occurred: {bus_status}')
    
    
    def write_byte(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_byte : str):
        """
        Write a single byte to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        data_byte : str
            Single data byte to be written to the address and instruction code.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        self.data_type_check(data_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}W{data_byte}$'
        
        self.write(command_string)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError (f'A Bus error occurred: {bus_status}')
            
    
    def read_byte(self, slave_addresses : tuple[str, ...], instruction_byte : str, return_pec : int = False) -> dict[str, str]:
        """
        Read a single byte from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        return_pec : bool
            Determines whether or not the PEC byte will be added to dictionary
            value.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}R01#'
        
        self.write(command_string)
        
        
        returned_data = self.generate_data_dictionary(instruction_byte, len(slave_addresses), return_pec)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError(f'A Bus error occurred: {bus_status}')
            
        return returned_data
    
    
    def write_word(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_byte_0 : str, data_byte_1 : str):
        """
        Write a single word to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        low_byte : str
            Low byte of word to be written to the address and instruction code.
        high_byte : str
            High byte of word to be written to the address and instruction code.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        self.data_type_check(data_byte_0)
        self.data_type_check(data_byte_1)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}W{data_byte_0}${data_byte_1}$'
        
        self.write(command_string)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError (f'A Bus error occurred: {bus_status}')
            

    def read_word(self, slave_addresses : tuple[str, ...], instruction_byte : str, return_pec : int = False) -> dict[str, str]:
        """
        Read a single word from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        return_pec : bool
            Determines whether or not the PEC byte will be added to dictionary
            value.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}R02#'
        
        self.write(command_string)
        
        
        returned_data = self.generate_data_dictionary(instruction_byte, len(slave_addresses), return_pec)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError(f'A Bus error occurred: {bus_status}')
            
        return returned_data
    
    
    def write_direct(self, slave_addresses : tuple[str, ...], instruction_byte : str, decimal_data : float, m : int = 0, R : int = 0, b : int = 0):
        """
        Write a single DIRECT encoded word to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        decimal_data : float
            Decimal data to be encoded as DIRECT to be written to the
            address and instruction code.
        m : int
            m parameter of DIRECT format equation
        R : int
            R parameter of DIRECT format equation
        b : int
            b parameter of DIRECT format equation

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        if(type(decimal_data) != float):
            raise DataEntryError('Input decimal data is not a float!')
        
        direct_data = self.decimal_to_direct(decimal_data, m, R, b)
        
        self.write_word(slave_addresses, instruction_byte, direct_data[0], direct_data[1])
        
        
    def read_direct(self, slave_addresses : tuple[str, ...], instruction_byte : str, m : int = 0, R : int = 0, b : int = 0) -> dict[str, str]:
        """
        Read a single DIRECT encoded word from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        exponent : int
            Exponent to be used to convert the ULinear16 data value to decimal.
        m : int
            m parameter of DIRECT format equation
        R : int
            R parameter of DIRECT format equation
        b : int
            b parameter of DIRECT format equation

        Raises
        ------
        None.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
            
        returned_data = self.read_word(slave_addresses, instruction_byte, False)
        
        for address in slave_addresses:
            
            # Flip high byte and low byte around, then join them together into a 16-bit value and decode it into decimal value
            returned_data[address] = self.direct_to_decimal(int(''.join(returned_data[address].split('$')[1::-1]), 16), m, R, b)
            
        return returned_data
        
    
    def write_linear16(self, slave_addresses : tuple[str, ...], instruction_byte : str, decimal_data : float, exponent : int = -9):
        """
        Write a single ULinear16 encoded word to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        decimal_data : float
            Decimal data to be encoded as ULinear16 to be written to the
            address and instruction code.
        exponent : int
            Exponent to be used to convert the decimal data value to ULinear16.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        if(type(decimal_data) != float):
            raise DataEntryError('Input decimal data is not a float!')
            
        if(type(exponent) != int):
            raise DataEntryError('Input exponent is not an integer!')
        
        linear16_data = self.decimal_to_linear16(decimal_data, exponent)
        
        self.write_word(slave_addresses, instruction_byte, linear16_data[0], linear16_data[1])
        
    
    def read_linear16(self, slave_addresses : tuple[str, ...], instruction_byte : str, exponent : int = -9) -> dict[str, str]:
        """
        Read a single ULinear16 encoded word from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        exponent : int
            Exponent to be used to convert the ULinear16 data value to decimal.

        Raises
        ------
        None.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
            
        returned_data = self.read_word(slave_addresses, instruction_byte, False)
        
        for address in slave_addresses:
            
            # Flip high byte and low byte around, then join them together into a 16-bit value and decode it into decimal value
            returned_data[address] = self.linear16_to_decimal(int(''.join(returned_data[address].split('$')[1::-1]), 16), exponent)
            
        return returned_data
    
    
    def write_linear11(self, slave_addresses : tuple[str, ...], instruction_byte : str, decimal_data : float):
        """
        Write a single Linear11 encoded word to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        decimal_data : float
            Decimal data to be encoded as ULinear16 to be written to the
            address and instruction code.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        if(type(decimal_data) != float):
            raise DataEntryError('Input decimal data is not a float!')
        
        linear11_data = self.decimal_to_linear11(decimal_data)
        
        self.write_word(slave_addresses, instruction_byte, linear11_data[0], linear11_data[1])
        
    
    def read_linear11(self, slave_addresses : tuple[str, ...], instruction_byte : str) -> dict[str, str]:
        """
        Read a single Linear11 encoded word from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.

        Raises
        ------
        None.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
            
        returned_data = self.read_word(slave_addresses, instruction_byte, False)
        
        for address in slave_addresses:
            
            # Flip high byte and low byte around, then join them together into a 16-bit value and decode it into decimal value
            returned_data[address] = self.linear11_to_decimal(int(''.join(returned_data[address].split('$')[1::-1]), 16))
            
        return returned_data
    
    
    def write_block(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_block : tuple[str, ...]):
        """
        Write a block of data to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        ascii_string : str
            ASCII encoded text string to be converted to hexadecimal and
            transmitted in a block command.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
            
        if(type(data_block) != tuple):
            data_block = tuple([data_block])

        data_length = len(data_block)
        
        command_string = ''
        
        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}W{data_length:02x}$'
            
            data_string = ''

            for data_byte in data_block:
                self.data_type_check(data_byte)
                
                data_string += f'{data_byte}$'
                
            command_string = f'{command_string}{data_string}'
        
        self.write(command_string)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError (f'A Bus error occurred: {bus_status}')
    
    
    def read_block(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_length : int, return_pec : int = False) -> dict[str, str]:
        
        """
        Read a block of data from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        data_length : int
            The number of bytes to return from the slave device(s). Does not 
            include block length indicator byte.
        return_pec : bool
            Determines whether or not the PEC byte will be added to dictionary
            value.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        if(type(data_length) != int):
            raise DataEntryError('Data length is not an integer!')
            
        if(data_length > 255):
            raise DataEntryError('Data length is greater than 255!')
            
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}R{data_length:02x}&'
        
        self.write(command_string)
        
        
        returned_data = self.generate_data_dictionary(instruction_byte, len(slave_addresses), return_pec)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError(f'A Bus error occurred: {bus_status}')
            
        return returned_data
    
    
    def write_ASCII(self, slave_addresses : tuple[str, ...], instruction_byte : str, ascii_string : str):
        """
        Write an ASCII block to slave address(es).

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        ascii_string : str
            ASCII encoded text string to be converted to hexadecimal and
            transmitted in a block command.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        None.

        """
        
        self.instruction_type_check(instruction_byte)
        
        if(type(ascii_string) != str):
            raise DataEntryError(f'{ascii_string} is not a valid ASCII string!')
            
        data_block = ()
            
        for data_byte in ascii_string:
            data_block += (f'{ord(data_byte):02x}',)
            
        self.write_block(slave_addresses, instruction_byte, data_block)
            
    
    def read_ASCII(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_length : int) -> dict[str, str]:
        """
        Read an ASCII block from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        data_length : int
            The number of bytes to return from the slave device(s). Does not 
            include block length indicator byte.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        returned_data = self.read_block(slave_addresses, instruction_byte, data_length, False)
            
        for address in slave_addresses:
            ascii_string = ''
            
            for data_byte in returned_data[address].split('$')[1:-1]:
                ascii_string += (chr(int(data_byte, 16)))
            
            returned_data[address] = ascii_string
            
        return returned_data
    
    
    def read_32(self, slave_addresses : tuple[str, ...], instruction_byte : str, return_pec : int = False) -> dict[str, str]:
        """
        Read a four bytes from slave address(es)

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        return_pec : bool
            Determines whether or not the PEC byte will be added to dictionary
            value.

        Raises
        ------
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """
        
        self.instruction_type_check(instruction_byte)
        
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])
        
        command_string = ''

        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}R04#'
        
        self.write(command_string)
        
        
        returned_data = self.generate_data_dictionary(instruction_byte, len(slave_addresses), return_pec)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError(f'A Bus error occurred: {bus_status}')
            
        return returned_data
    
    
    def write_read_process_call(self, slave_addresses : tuple[str, ...], instruction_byte : str, data_block : tuple[str, ...], read_data_length : int, return_pec : int = False) -> dict[str, str]:
        """
        Perform a write read process call, where data is written as a block to
        the slave address(es) and then data is read back as a block in a single
        transaction.

        Parameters
        ----------
        slave_addresses : tuple[str, ...]
            A tuple containing all slave device addresses to access.
        instruction_byte : str
            The instruction byte code of the command to be accessed.
        data_block : tuple[str, ...]
            Data to be written in the block write portion of the command.
        data_length : int
            The number of bytes to return from the slave device(s). Does not 
            include block length indicator byte.
        return_pec : bool
            Determines whether or not the PEC byte will be added to dictionary
            value.

        Raises
        ------
        DataEntryError
            Data entered was either not in the right format or it is a value
            that is greater than can be stored in an unsigned 8-bit integer.
        SMBStatusError
            The status returned from the STM32 state machine indicates that
            an error during transmission occured.

        Returns
        -------
        dict[str, str]
            Dictionary where the key is the address of the slave device and the
            value is the data that was read back from said slave. Each slave
            device gets its own entry.

        """    
        
        self.instruction_type_check(instruction_byte)
            
        # This forces a single item to be a tuple
        if(type(slave_addresses) != tuple):
            slave_addresses = tuple([slave_addresses])

        if(type(data_block) != tuple):
            data_block = tuple([data_block])

        data_length = len(data_block)
        
        command_string = ''
        
        for address in slave_addresses:
            self.address_type_check(address)
            
            command_string += f'{address}@{instruction_byte}W{data_length:02x}$'
            
            data_string = ''

            for data_byte in data_block:
                self.data_type_check(data_byte)
                
                data_string += f'{data_byte}$'
                
            command_string = f'{command_string}{data_string}{read_data_length:02x}P'
        
        self.write(command_string)
        
        
        returned_data = self.generate_data_dictionary(instruction_byte, len(slave_addresses), return_pec)
        
        bus_status = self.get_bus_status()
        
        if(bus_status != 'NO_ERROR'):
            raise SMBStatusError(f'A Bus error occurred: {bus_status}')
            
        # Get rid of the packet length indicator
        for address in slave_addresses:
            returned_data[address] =  returned_data[address][3:]
            
        return returned_data
    
    
class SMBStatusError(Exception):
    pass
            
class DataEntryError(Exception):
    pass

class SMBPECError(Exception):
    pass