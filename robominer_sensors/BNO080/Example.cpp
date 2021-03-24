// Compile this in example in Linux command line with:
// g++ -I./ -O -o example.o Example.cpp BNO080_i2c.cpp

// Example C / C++ code.
// 2021-03-24 Jaan Rebane - modified Sparkfun BNO080 library example code.

#include <stdio.h>           // For printf
#include <stdlib.h>          // For exit()
#include <unistd.h>          // for usleep

// #include <string>            // for std::string
// #include <stdint.h>          // for uint_8t
// #include <math.h>            // For sqrt

#include "BNO080_i2c.h"

BNO080 myIMU;

void setup()
{
  printf("BNO080 Read Example\n");

  //Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
//  //=================================
//  delay(100); //  Wait for BNO to boot
//  // Start i2c and BNO080
//  Wire.flush();   // Reset I2C
//  IMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
//  Wire.begin(4, 5);
//  Wire.setClockStretchLimit(4000);
//  //=================================

//  This debugging prints out a lot of messages from the library.
//  myIMU.enableDebugging();

  if (myIMU.begin() == false)
  {
    printf("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...\n");
    exit(1);
  }

  // TODO: wrap this to Olimex:
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  printf("Enable rotation vector\n");
  myIMU.enableRotationVector(50); //Send data update every 50ms

  printf("Rotation vector enabled\n");
  // printf("Output in form i, j, k, real, accuracy\n");
  printf("Output: roll, pitch, yaw\n");
}

int main()
{
  setup();

  //Look for reports from the IMU
  //while (myIMU.dataAvailable() != true)
  while (1)
  {
    if (myIMU.dataAvailable() == true) {
 /* float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

/*    printf("%lf", quatI);
    printf(", ");
    printf("%lf", quatJ);
    printf(", ");
    printf("%lf", quatK);
    printf(", ");
    printf("%lf", quatReal);
    printf(", ");
    printf("%lf", quatRadianAccuracy);
    printf(" "); */

    float roll = myIMU.getRoll();
    float pitch = myIMU.getPitch();
    float yaw = myIMU.getYaw();
    printf("%6.2f %6.2f %6.2f",roll,pitch,yaw);

    printf("\n");
    usleep(50000);
    }
  }
}
