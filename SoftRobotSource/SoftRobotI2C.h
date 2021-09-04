//SoftRobotI2C.h

#include <avr/io.h>

#ifdef __cplusplus
extern "C"
{
#endif

// read N consecutive registers (8bit) via I2C
//	device_address: 7-bit I2C address
//	register_read_address: byte representing the memory address where read begins
//	*bytes: (address of) array containing N bytes, to write the values into
//	N: number of consecutive bytes to read, N >= 1
uint8_t I2C_read_bytes(uint8_t device_address, uint8_t register_read_address, uint8_t *bytes, uint8_t N);
// note: this function INCLUDES the write command to the device which moves the device pointer to the memory read address 

// write to a single register (on the slave device) via I2C
//	device_address: 7-bit I2C address
//	register_write_address: byte representing the memory address to write to
//	data: a single byte that is passed to the slave device
void I2C_write_byte(uint8_t device_address, uint8_t register_write_address, uint8_t data);

// write a command byte to an unspecified register (on the slave device) via I2C
//	device_address: 7-bit I2C address
//	command: a single byte sent to the slave device
void I2C_write_command(uint8_t device_address, uint8_t command);

// boolean, check if there is a working I2C device on the bus with the given address
uint8_t I2C_check_device_exists(uint8_t device_address);
// returns 1 if device acknowledged, 0 o/w

// check all valid I2C addresses (brute force), print those that respond
void check_all_of_I2C(void);

// boolean, used to check if there is a device currently using the I2C bus
// initially written when I2C collisions were causing the system to freeze (wait forever for reply)
// unknown, the status of this, best not to call it
uint8_t check_I2C_busy(void);


// INITIALIZATION:

// initialize XMEGA for I2C use (currently only on PORTE for internal components)
void init_I2C(void);

#ifdef __cplusplus
}
#endif