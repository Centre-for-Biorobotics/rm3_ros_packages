#ifndef OLIMEXINO_H
#define OLIMEXINO_H



#include <cstdint>
#include <unistd.h>
#include <string>
#include <math.h>

#define PI M_PI

using namespace std;


void delay(uint16_t);
void print(unsigned char *, int);
bool definePin(std::string);
bool pinMode(int, int);
bool pinMode(std::string, int);
bool digitalWrite(int, int);
bool digitalWrite(std::string, int);
unsigned long millis(void);



#endif
