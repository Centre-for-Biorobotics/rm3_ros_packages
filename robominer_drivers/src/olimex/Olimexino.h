#ifndef OLIMEXINO_H
#define OLIMEXINO_H

#include "config.h"


#ifdef DEBUG

	#define debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#warning DEBUG mode is on. Debug messages will be printed to the console. Undefine it in ./olimex/config.h if no debug messages should be printed.

#else

	#define debug(fmt, ...) ((void)0)

#endif

#include <cstdint>
#include <unistd.h>
#include <string>
#include <math.h>

#define PI M_PI

using namespace std;


void delay(uint32_t);
void print(string);
void println(string);
void print(int);
void println(int);
void print(float);
void println(float);
void print(unsigned char *, int);


#endif
