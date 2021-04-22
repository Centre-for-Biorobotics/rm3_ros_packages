#ifndef __CONFIG_H__
#define __CONFIG_H__

#define DEBUG   // if #define'd, debug messages will be printed to the console, otherwise not
//#define UBUNTU  // if #define'd, the platform to compile for is Linux Ubuntu, otherwise Olimex Linux

#ifdef UBUNTU
#define I2C_BUS_ID 8
#else
#define I2C_BUS_ID 1
#endif                

#endif
