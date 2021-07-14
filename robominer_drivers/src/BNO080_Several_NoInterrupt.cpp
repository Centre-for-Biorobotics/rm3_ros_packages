/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>
#include "SparkFun_BNO080.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

BNO080 myIMU;
bool enablePlotter = true;

void setup()
{
  
  Wire.begin();

  myIMU.begin(0x4B);

  //Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.softReset();

  myIMU.enableLinearAccelerometer(100); //Send data update every 100ms
  myIMU.enableRotationVector(100); //Send data update every 100 ms

  if(!enablePlotter) {
    Serial.println(F("Linear Accelerometer and rotation vector enabled"));
  }
  else
  {
    Serial.println("Linear_x:,Linear_y:,Linear_z:,Rotation_qx:,Rotation_qy:,Rotation_qz:,Rotation_qw:");    
  }
}

void loop()
{
  if(!enablePlotter) { Serial.print("."); }
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {    
    float x = myIMU.getLinAccelX();
    float y = myIMU.getLinAccelY();
    float z = myIMU.getLinAccelZ();
    byte linAccuracy = myIMU.getLinAccelAccuracy();

    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float qw = myIMU.getQuatReal();
    byte rotAccuracy = myIMU.getQuatRadianAccuracy();
    byte accAccuracy = myIMU.getQuatAccuracy();

    if(!enablePlotter)
    {
      Serial.println();
      
      Serial.print("Linear: ");
      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.print(F(","));
      Serial.print(z, 2);
      Serial.print(F(","));
      Serial.print(linAccuracy);
  
      Serial.println();
  
      Serial.print("Rotation: ");
      Serial.print(qx, 2);
      Serial.print(F(","));
      Serial.print(qy, 2);
      Serial.print(F(","));
      Serial.print(qz, 2);
      Serial.print(F(","));
      Serial.print(qw, 2);
      Serial.print(F(","));
      Serial.print(rotAccuracy);
      Serial.print(F(","));
      Serial.print(accAccuracy);
  
      Serial.println();
    }
    else
    {
      //Serial.print("Linear x:");
      Serial.print(x);
      Serial.print(",");
      //Serial.print("Linear y:");
      Serial.print(y);
      Serial.print(",");
      //Serial.print("Linear z:");
      Serial.print(z);
      Serial.print(",");

      //Serial.print("Rotation qx:");
      Serial.print(qx);
      Serial.print(",");
      //Serial.print("Rotation qy:");
      Serial.print(qy);
      Serial.print(",");
      //Serial.print("Rotation qz:");
      Serial.print(qz);
      Serial.print(",");
      //Serial.print("Rotation qw:");
      Serial.print(qw);
      
      Serial.println();
    }
  }
}
