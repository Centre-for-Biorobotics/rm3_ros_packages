#ifndef OLIMEXINO_H
#define OLIMEXINO_H



#include <cstdint>
#include <unistd.h>
#include <string>
#include <math.h>

#define PI M_PI

using namespace std;


void delay(uint32_t);
void print(unsigned char *buf, int size);
unsigned long millis(void);



#endif
