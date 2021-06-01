/**
 * This cpp file includes common methods from the Arduino environment
 * which are translated to execute in the general C++ environment.
 */ 

#include "Olimexino.h"
#include <chrono>


void delay(uint32_t ms) {
	usleep(1000*ms);
}

void print(unsigned char *buf, int size)
{
	for(int c = 0; c < size; c++)
	{
		printf("%c",buf[c]);
	}
}

using namespace std::chrono_literals;

unsigned long millis(void)
{
    return std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();    
}
