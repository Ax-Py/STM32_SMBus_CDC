# STM32_SMBus_CDC
A simple method to communicate with SMBus/PMBus devices over USB CDC via an STM32.

Utilizes TinyUSB to establish a Communications Device Class (CDC) device.

A Python driver is also included that allows easy readings of the most common I2C/SMBus/PMBus command types.

# Getting Started

1. Purchase an ST-Link V2/V3 or a compatible SWD programmer. 
  
2. Download STM32C071FBP6_SMBus_Bridge.elf to your computer.

3. Download STM32CubeProg (https://www.st.com/en/development-tools/stm32cubeprog.html)
   
4. Ensure that the programmer is connected properly to the STM32C0 and that it is plugged into the PC. Ensure that the STM32 is either being powered by the PC or has a separate 3.3V supply.
   
5. Open STM32CubeProg and click on "Open file":
<p align="center">
  <img width="855" height="126" alt="image" src="https://github.com/user-attachments/assets/5fb672d3-f0d7-409f-bdf2-690ba402c586" />
</p>
<br>

6. Select the downloaded ELF file and then click "Download":
<p align="center">
<img width="852" height="92" alt="image" src="https://github.com/user-attachments/assets/68cd6875-6526-47ad-94f3-e8ec56675a4e" />
</p>
<br>

7. The firmware should then be written to the STM32. It is a good idea to then check that the firmware was written successfully by clicking on the arrow next to "Download" and selecting "Verify":
<p align="center">
<img width="203" height="239" alt="image" src="https://github.com/user-attachments/assets/19267a09-fe2a-4522-a2d9-b693b2b106d6" />
</p>
<br>

8. If the verification passes, break all connections from the STM32 to the ST-Link and unplug the STM32 from the PC. Plug it back into the PC and a COM port called "USB Serial Device" with a VID of 0xC0C0 and a PID of 0x0011 should appear. You can then communicate using any serial terminal program or the included Python driver.

# Reading Data

1. After opening a serial terminal connection, commands are sent to the STM32 with the following format:
   
<br>
<p align="center">
<img width="782" height="246" alt="image" src="https://github.com/user-attachments/assets/e478104a-ae8f-42b1-8553-867a3e0e744d" />

</p>
<br>

2. The data returned from the STM32 always starts with the address of the slave device followed by a colon (':') as a delimiter. The characters after the colon are the data that was read back from that slave device. If a PEC byte was requested, the returned data will have another delimiter in the form of a pipe character ('|') that follows the main data packet. The characters after pipe are the PEC byte. A newline character terminates all data returned from the STM32.
   
3. As an example, reading the temperature (two bytes from register 0x00) from a TMP117 Digital Temperature Sensor with an address of 0x48:

  -> Data written to STM32: 48@00R02#

  -> Data returned from STM32: 48:04$32$\n
