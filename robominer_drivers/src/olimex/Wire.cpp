#include "Wire.h"
#include <sys/ioctl.h>       // For ioctl
#include <fcntl.h>           /* For O_RDWR */
#include <stdlib.h>
#include <cstdio>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include "Olimexino.h"




/** Default constructor **/
TwoWire::TwoWire() {}

/**
Opens the I2C system bus
@param {uint8_t} busAddress: the address of the bus to join
Public
**/
void TwoWire::begin(uint8_t busAddress)
{	
	char _busName[NAME_LENGTH];
	snprintf(_busName, 19, "/dev/i2c-%d", busAddress);
	begin(_busName);	
}

void TwoWire::begin(void)
{	
	debug("TwoWire::begin(): Using pre-defined bus name %s\n",busName);
	begin(busName); // assuming there has been already a bus name set
}

void TwoWire::begin(char * _busName)
{	
	
	if(strcmp(_busName,busName) == 0)
	{
		//debug("TwoWire::begin(): Bus %s is already open\n",busName);
		return;
	}
	
	file = open(_busName, O_RDWR);
	//printf("TwoWire::begin(): Opening I2C bus %s...\n",_busName);
	if (file < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		debug("TwoWire::begin(): Error opening I2C bus %s.\n",_busName);
		exit(1);
	}	
	snprintf(busName, 19, "%s", _busName);
	debug("TwoWire::begin(): I2C bus is now open: %s\n",busName);
}

/**
Changes the address of the I2C slave device
@param {uint8_t} address: the address of the slave
@returns true if slave device was selected successfully, otherwise false 
Private
**/
bool TwoWire::selectSlave(uint8_t address)
{
	if (ioctl(file, I2C_SLAVE, address) < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		debug("TwoWire::selectSlave(): Error opening slave device 0x%02X.\n", address);
		return false;
	}	
	slaveAddress = address;
	//printf("TwoWire::selectSlave(): The slave address is set to %d\n",slaveAddress);
	return true;
}

/**
Starts a new transmission with a slave device
Note: No need to call this when using requestFrom(address, quantity)
@param {uint8_t} address: the address of the slave device
Public
**/
void TwoWire::beginTransmission(uint8_t address)
{
	//printf("TwoWire::beginTransmission(): Existing slave address: %d, New slave address: %d\n",slaveAddress,address);
	bufIndex = 0;	
	bufSize = 0;	
	if(address == slaveAddress)
	{
		return;
	}
	selectSlave(address);	
}

/**
Ends the transmission with a slave device
Writes the actual send buffer out onto the I2C bus
@param {bool} b (unused)
@returns {int}: 0 in case of success, -1 if there was nothing in the send buffer, 1 if the number of bytes written does not match the number of bytes that should have been written
Public
**/
int TwoWire::endTransmission(bool b)
{
	//printf("TwoWire::endTransmission()\n");
	int ret = -1;
	if(b)
	{
		//printf("TwoWire::endTransmission(): Warning: Stop condition is not supported.\n");
	}
	
	if(txBufSize != 0)
	{		
		ret = 0;
		ssize_t bytesToSend = (ssize_t)txBufSize;
		ssize_t bytesWritten = ::write(file, txBuffer, bytesToSend);		
		if (bytesWritten != bytesToSend) {
			debug("TwoWire::endTransmission(): Error writing to slave 0x%02X: Unexpected number of bytes.\n",slaveAddress);					
			debug("  bytesToSend: %ld\n  bytesWritten: %ld\n",bytesToSend,bytesWritten);
			debug("  errno: %s\n",strerror(errno));
			ret = 1;
		}	
		//delay(100); //// TODO: figure out if delay needed here
	}
	txBufSize = 0;
	txBufIndex = 0;
	slaveAddress = 0;
	
	return ret;
}

int TwoWire::endTransmission(void)
{
	return endTransmission(false);
}

/**
Writes data to the I2C send buffer
@param {const uint8_t *} data: the data to be sent
@param {ssize_t} quantity: the number of bytes to be sent
@returns {size_t}: the number of bytes written into the send buffer
Public
**/
size_t TwoWire::write(const uint8_t *data, ssize_t quantity)
{
	//printf("TwoWire::write(): Setting txBuffer content (quantity: %ld):\n  ",quantity);
	int bytesCopied = 0; //snprintf(txBuffer, quantity, "%hhn", data);
	for(int i=0; i<quantity; i++)
	{
		write(data[i]);
		bytesCopied++;
	}
	//printf("Copied %d bytes\n",txBufSize);
	return txBufSize;
}

/**
Writes one byte of data to the I2C send buffer
@param {uint8_t} dataByte: the byte to be sent
@returns {size_t}: 1 (success)
Public
**/
size_t TwoWire::write(uint8_t dataByte)
{
	//printf("TwoWire::write(): Setting txBuffer content\n");
	txBuffer[txBufIndex] = (char)dataByte;
	//printf("  txBufIndex: %d, data: %d\n",txBufIndex,txBuffer[txBufIndex]);
	txBufIndex++;		
	txBufSize = txBufIndex;
	//printf("  txBufSize: %d\n",txBufSize);
	return 1;
}

/**
Reads the next byte of data from the receive buffer
@returns {uint8_t}: the byte in the buffer at the current index
Public
**/
uint8_t TwoWire::read(void)
{
	if(bufIndex >= bufSize)
	{
		debug("TwoWire::read(): Error reading from buffer: index exceeds amount of data currently in buffer.\n");
		bufIndex = 0;
	}	
	bufIndex++;
	return buffer[bufIndex-1];
}

/**
Reads a certain amount of data from the I2C slave device into the receive buffer
@param {uint8_t} address: the address of the slave device
@param {uint8_t} quantity: the number of bytes expected from the slave device
@returns {uint8_t}: the number of bytes read, or 0 if the received number of bytes does not match the expected number
Public
**/
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{	
	// set slave address and reset buffer
	beginTransmission(address);
	// clamp to buffer length
	if(quantity > BUFFER_LENGTH){
		debug("TwoWire::requestFrom(): Error: Too many bytes requested. Please increase BUFFER_LENGTH.\n");
		quantity = BUFFER_LENGTH;
	}
	//delay(10);
	// perform blocking read into buffer
	int r = ::read(file, buffer, quantity);
	if(r != quantity)
	{
	  debug("TwoWire::requestFrom(): Error reading slave device 0x%02X: Unexpected number of bytes.\n",address);
	  debug("  bytes expected: %d\n  bytes read: %d\n",quantity, r);
	  debug("  errno: %s\n",strerror(errno));
	  return 0;
	}
	// set new buffer size value
	bufSize = r;
	
	endTransmission(false); // (seems obsolete)
	return r;	
}

/**
Gets the number of unread bytes in the receive buffer
@returns {int}: the number of unread bytes
Public
**/
int TwoWire::available(void)
{
	return bufSize - bufIndex;	
}

/**
Closes the I2C bus file if it has been opened
Implemented especially for sanity in Linux,
but not sure if it is really required
Public
**/
void TwoWire::end(void)
{
	if(file != 0)
	{
		close(file);
		debug("TwoWire::end(): Bus closed\n");
	}
}

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire = TwoWire();
