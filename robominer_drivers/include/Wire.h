#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>
#include <cstddef>
#include <unistd.h>


#define BUFFER_LENGTH 100
#define NAME_LENGTH 20

class TwoWire
{
	public:
		TwoWire();              // default (empty)
		void begin(void);
		void begin(char *);
		void begin(uint8_t);                   
		void beginTransmission(uint8_t);   // set device address and empty the buffer									   
		int endTransmission(bool);
		int endTransmission(void);
		uint8_t requestFrom(uint8_t, uint8_t);  
		size_t write(uint8_t);
		size_t write(const uint8_t *, ssize_t);
		uint8_t read(void);// read out buffer byte by byte
		int available(void);  	
		void end(void); // added: close the I2C bus file
	
	private:
		int file;
		bool selectSlave(uint8_t);    // set/change slave device address    
		uint8_t slaveAddress;		
		uint8_t buffer[BUFFER_LENGTH];   // RX buffer
		uint8_t bufIndex = 0;
		uint8_t bufSize = 0;
		char txBuffer[BUFFER_LENGTH]; // TX buffer
		int txBufIndex = 0;
		int txBufSize = 0;
		
		char busName[NAME_LENGTH];
};
extern TwoWire Wire; // declares the variable Wire as an object of class TwoWire

#endif
