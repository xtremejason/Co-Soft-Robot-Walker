//SoftRobotI2C.c

// TODO: double check that NACK is sent to device after last data byte is read, and then follow this with STOP condition
// also, make sure STOP condition is being sent after writing to the device

#include "SoftRobotI2C.h"

#include <util/delay.h>		// delay only used in debug subroutines
#include <stdio.h>

#define I2C_DEBUGMODE 0


// driver specific global variables:

uint8_t I2C_in_use = 0;



uint8_t I2C_read_bytes(uint8_t device_address, uint8_t register_read_address, uint8_t *bytes, uint8_t N)
{
	uint8_t i;
	uint8_t device_write_address = device_address << 1;		// left shift the 7-bit device address for I2C protocol
	uint8_t device_read_address = device_write_address + 1;	// read address is always 1 higher than the write address
	
	// Send START + SLAVE DEVICE ADDRESS + WRITE BIT:
	TWIE.MASTER.ADDR = device_write_address;		//	(start sent automatically, 7bit address with write bit (0) appended)
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm)) //wait for the outbound message to complete
{}

	// Send the internal register number to read from:
	TWIE.MASTER.DATA = register_read_address;
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm)) //wait for the outbound message to complete
{}
	
	// Send repeat START + SLAVE DEVICE ADDRESS + READ BIT:
	TWIE.MASTER.ADDR = device_read_address;		// (start sent automatically, 7bit address with read bit (1) appended)

	for (i=0; i<N; i++)
	{
		// an inbound message (byte) will come in now
		while (!(TWIE.MASTER.STATUS & TWI_MASTER_RIF_bm)) //wait for the inbound message to be received
	{}

		bytes[i] = TWIE.MASTER.DATA;
		if(I2C_DEBUGMODE > 0)
		{
			printf("byte %d: 0x%02x\r\n", i, bytes[i]);
			_delay_ms(1);
		}
		
		if(i == N-1) // last byte
		TWIE.MASTER.CTRLC = 0b00000111; // send NACK followed by STOP condition
		// note: 0b00000111 = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc

		else
		TWIE.MASTER.CTRLC = 0x02; // send an ACK to receive another byte
		
	}

	// an inbound message (byte) will come in now
	TWIE.MASTER.CTRLC = 0b00000111; // send NACK followed by STOP condition	(0b00000111 = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc)
	
	bytes[N-1] = TWIE.MASTER.DATA;
	if(I2C_DEBUGMODE > 0)
	{
		printf("byte %d: 0x%02x\r\n", i, bytes[N-1]);
		_delay_ms(1);
	}
	
	TWIE.MASTER.STATUS |= 0x01;			// what does this do ???
	
	return 0;
}

void I2C_write_byte(uint8_t device_address, uint8_t register_write_address, uint8_t data)
{	
	uint8_t device_write_address = device_address << 1;		// left shift the 7-bit device address for I2C protocol

	//Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIE.MASTER.ADDR = device_write_address;		
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete

	/*
	// check if ACK was received
	if(TWIE.MASTER.STATUS & TWI_MASTER_RXACK_bm)
	{
		// if bit is 1 ACK not received (problem)
		fail = 1;
	}
	*/

	// send the internal register number to write to
	TWIE.MASTER.DATA = register_write_address;		// register to be accessed
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete
	
	// send the data
	TWIE.MASTER.DATA = data;
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete
	
	TWIE.MASTER.CTRLC = 0x03; // issue a STOP condition

	return;
}

void I2C_write_command(uint8_t device_address, uint8_t command)
{	
	uint8_t device_write_address = device_address << 1;		// left shift the 7-bit device address for I2C protocol

	//Send START + SLAVE DEVICE ADDRESS + WRITE BIT: 
	TWIE.MASTER.ADDR = device_write_address;		
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete
	
	// send the data
	TWIE.MASTER.DATA = command;
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete
	
	TWIE.MASTER.CTRLC = 0x03; // issue a STOP condition

	return;
}

uint8_t I2C_check_device_exists(uint8_t device_address)
{
	uint8_t exists;
	
	uint8_t device_write_address = device_address << 1;		// left shift the 7-bit device address for I2C protocol

	//Send START + SLAVE DEVICE ADDRESS + WRITE BIT:
	TWIE.MASTER.ADDR = device_write_address;
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));	//wait for the outbound message to complete

	// check if ACK was received
	if(TWIE.MASTER.STATUS & TWI_MASTER_RXACK_bm)
	{	// note: RXACK bit is SET if there was a PROBLEM  [Ref: manual section 21.9.4]
		// ACK not received (problem)
		exists = 0;
	}

	else
	{
		// ACK received (OK)
		exists = 1;
	}
	
	TWIE.MASTER.CTRLC = 0x03; // issue a STOP condition

	return exists;
}

void check_all_of_I2C(void)
{
	// this may crash the OS via timeout if there are a lot of devices on the bus

	for(uint8_t addr = 0; addr < 128; addr++)
	{
		if(I2C_check_device_exists(addr))
		printf("i2c: %02x found\r\n", addr);
		else if(I2C_DEBUGMODE > 0)
		{
			printf("i2c: %02x\r\n", addr);
			_delay_ms(2);// the ONLY delay in this implementation source file should be HERE
		}
	}
}

// unknown/untested if this is working?
uint8_t check_I2C_busy(void)
{
	uint8_t bus_state = TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_BUSY_gc;
	
	if(bus_state == TWI_MASTER_BUSSTATE_BUSY_gc)
	{
		if(I2C_DEBUGMODE > 0)
			printf("BUSY");
		return 1;
	}

	else if(bus_state == TWI_MASTER_BUSSTATE_OWNER_gc)
	{
		if(I2C_DEBUGMODE > 0)
			printf("OWNR");
		return 1;
	}

	else if(bus_state == TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if(I2C_DEBUGMODE > 0)
			printf("IDLE");
		return 0;
	}

	else
	{
		if(I2C_DEBUGMODE > 0)	// bus_state == 0
			printf("UNK0");
		return 2;
	}
}


// INITIALIZATION:

void init_I2C()
{
	//**** INITIALIZE I2C *********************************************************************************
	
	//	SDA is connected to portCpin0 (Pin 16)	-> this 3.3v signal is then fed through logic level converter to 5v -> SDA5
	//	SCL is connected to portCpin1 (Pin 17)	-> this 3.3v signal is then fed through logic level converter to 5v -> SCL5
	//	SDA3 is connected to portEpin0 (Pin 36)
	//	SCL3 is connected to portEpin1 (Pin 37)
	
	/* comments on I2C protocol **		
										Ref: www.robot-electronics.co.uk/acatalog/I2C_Tutorial.html
										Ref: Fairchild Application Note 794		google:fan794.pdf
	
		Common I2C bus speeds are:			(arbitrarily low clock frequencies are also allowed)
			10 kbit/s "low-speed mode"
			100 kbit/s "standard mode"
			400 kbit/s "fast mode" (recent revision of I2C protocol, slave device may not support this) 
	
		I2C Data is transferred in sequences of 8 bits.
		The bits are placed on the SDA line starting with the MSB (Most Significant Bit).
		For every 8 bits transferred, the device receiving the data sends back an acknowledge (ACK) bit, 
		so there are actually 9 SCL clock pulses to transfer each 8 bit byte of data.
		If the receiving device sends back a low ACK bit (0), then it has received the data and is ready to accept another byte. (active response, SDA line is normally high)
		If the receiving device sends back a high ACK bit (1), then it is indicating it cannot accept any further data 
		and the master should terminate the transfer by sending a stop sequence.  
	
		I2C Device Addressing:
			Virtually all I2C addresses are 7 bits (10 bits rare). 
			It is possible to have up to 128 devices on the I2C bus, since a 7-bit number can be from 0 to 127.
			When sending out the 7-bit address, the protocol is to still always send 8 bits.
			The extra bit is used to inform the slave if the master is writing to it (0) or reading from it (1).
			The 7-bit address is placed in the upper 7 bits of the byte and the Read/Write (R/W) bit is in the LSB (Least Significant Bit).
			
			SDA:	A6		A5		A4		A3		A2		A1		A0		R/W		ACK		(address bits)
			SCL:	1		2		3		4		5		6		7		8		9		(clock pulses)
	
			The placement of the 7 bit address in the upper 7 bits of the byte may be a source of confusion. 
			(e.g. to write to address 21, you must actually send out 42 which is 21 moved over by 1 bit)
			The following alternative description may also be used: 
				I2C bus addresses are 8 bit addresses, with even addresses as write only, 
				and the odd addresses are the read address for the same device.
				
		The I2C Software Protocol:
			The first thing that will happen is that the master will send out a start sequence.
			This will alert all the slave devices on the bus that a transaction is starting and they should listen in case it is for them.
			Next the master will send out the device address.
			The slave that matches this address will continue with the transaction, any others will ignore the rest of this transaction and wait for the next.
			Having addressed the slave device the master must now send out the internal location or register number inside the slave that it wishes to write to or read from.
			This number is obviously dependent on what the slave actually is and how many internal registers it has.
			Some very simple devices do not have any, but most do.
			Having sent the I2C address and the internal register address the master can now send the data byte(s).
			The master can continue to send data bytes to the slave and these will normally be placed in the sequentially increasing registers.
			The slave should automatically increment the internal register address after each byte. 
			When the master has finished writing all data to the slave, it sends a stop sequence which completes the transaction. So to write to a slave device: 

				TO WRITE TO A SLAVE:
				1. Send a start sequence
				2. Send the I2C address of the slave with the R/W bit low (0)
				3. Send the internal register number you want to write to
				4. Send the data byte
				5. [Optionally, send any further data bytes]
				6. Send the stop sequence.	
	
			Before reading data from the slave device, you must tell it which of its internal addresses you want to read.
			So a read of the slave actually starts off by writing to it. 
			This is the same as when you want to write to it: You send the start sequence, 
			the I2C address of the slave with the R/W bit low (0), and the internal register number you want to write to. 
			Now you send another start sequence (sometimes called a restart) and the I2C address again - this time with the read bit high (1). 
			You then read as many data bytes as you wish and terminate the transaction with a stop sequence.
			
				TO READ FROM A SLAVE:
				1. Send a start sequence
				2. Send the I2C address of the slave with the R/W bit low (0)
				3. Send the internal register number you want to read from
				4. Send a start sequence again (repeated start)
				5. Send the I2C address of the slave with the R/W bit high (1)
				6. Read data byte
				7. Send the stop sequence.


	*/ // End comments on I2C protocol
	
	//TWIE.MASTER.CTRLB |= TWI_MASTER_SMEN_bm;   // Enable smart mode (automatic acknowledgement) 
	//TWIE.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;   // Enable quick command
	
	//TWIE.MASTER.BAUD = 155; 				// 100 kHz (Ref: doc8077.pdf, pg 221, [eqn. 2])
	TWIE.MASTER.BAUD = 35;  				// 400 kHz
	
	TWIE.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm; // Master enable
	

	TWIE.MASTER.CTRLB |= 0b00001000;			// Enable inactive bus timeout supervisor (21.9.2 manual)

	TWIE.MASTER.STATUS |= 0x01;                // Set bus idle

	
	
	_delay_ms(10);
	//check_all_of_I2C();
}