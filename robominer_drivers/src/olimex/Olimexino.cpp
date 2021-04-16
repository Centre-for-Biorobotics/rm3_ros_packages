#include "Olimexino.h"


void delay(uint32_t ms) {
	usleep(1000*ms);
}

void print(string s)
{
}
void println(string s)
{
}
void print(int i)
{
}
void println(int i)
{
}
void print(float f)
{
}
void println(float f)
{
}

void print(unsigned char *buf, int size)
{
	for(int c = 0; c < size; c++)
	{
		printf("%c",buf[c]);
	}
}
