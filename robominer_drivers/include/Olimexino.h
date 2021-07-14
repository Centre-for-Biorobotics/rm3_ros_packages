#ifndef OLIMEXINO_H
#define OLIMEXINO_H



#include <cstdint>
#include <unistd.h>
#include <string>
#include <math.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>

#define PI M_PI
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1

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
