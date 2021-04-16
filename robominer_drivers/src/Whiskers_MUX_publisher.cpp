// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Initial node:
// Kilian Ochs, CfB Tallinn, 2021-04-16

/**
 * Kilian Ochs, 16.04.2021
 * Center for Biorobotics, TALTECH, Tallinn
 * Robominers experimental setup
 * 
 * This sketch is used to interface magnetic Hall sensors of type TLV493D
 * using several multiplexers of type TCA9548A. It publishes messages of type "WhiskerArray"
 * in the ROS2 environment.
 * 
 * Note: The number of sensors per multiplexer ("NUM_SENSORS") must be the same for all multiplexers.
 * 
 * Continuously reads the sensors one by one, then prints all sensor readings to Console in one burst.         
 * When reading the Console outputs during manual verification, the format mf = PlainText should be used.
 *
 * The format mf = Compressed packs each sensor float value into a 16bit integer. The result is stored in
 * txString, which can be used for data acquisition purposes in the future (e.g., send to SimuLink via Serial).
 * 
 * To run this code on Ubuntu instead of Olimex, uncomment "#define UBUNTU" in "./olimex/config.h".
 * To view debug messages,                       uncomment "#define DEBUG"  in "./olimex/config.h".
 * 
 * When running on an Ubuntu computer (Desktop/laptop), a USB-to-I2C adapter can be used. Please note the name
 * of the i2c bus using dmesg after plugging in the adapter. To define platform-specific bus IDs,
 * see "./olimex/config.h".
 *  * 
 * To be compiled for a Linux-based system.
 * This is a ported version of the library originally made for the Adafruit Feather M0
 * (see gitHub: kilian2323/3dMagSensorsVisual).
 */
 
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robominer_msgs/msg/whisker_pos_in_grid.hpp"
#include "robominer_msgs/msg/whisker.hpp"
#include "robominer_msgs/msg/whisker_array.hpp"

#include "Whiskers_MUX.h"
#ifndef UBUNTU
#include "./olimex/conio.h"
#else
#include <fcntl.h>
#endif





//////////// Start of user-defined constants /////////////

#define MUX_STARTADDR 0x70    // [0x70] Address of the first multiplexer; the others must be consecutive
#define NUM_MUX 1             // Number of multiplexers
#define NUM_SENSORS 1         // Number of sensors per multiplexer (max. 8)
#define MAXBUF 1000           // Maximum char length of an output message
#define PUBLISH_INTERVAL 40ms // Interval for whisker message publishing

const bool fastMode = true;		       // [true] if false, the sensor readings come more slowly, but might be more accurate

const unsigned char endSignature[2] = {'\r','\n'};     // for Compressed message format: end of message  
const bool sendPolarRadius = false;    // [false] if true, the polar radius will be sent as the third value in Spherical mode using Compressed message format,
                                       //         otherwise it will be omitted (only two values will be sent)
const int multiplier = 100;            // [100] used for Compressed message format. Higher value: better accuracy, but lower range of values to capture

const Type t = Cartesian;              // [Cartesian] type of representation of sensor data: Cartesian or Spherical
const MessageFormat mf = PlainText;    // [PlainText] format for serial messages: PlainText or Compressed

//////////// End of user-defined constants /////////////







using namespace std::chrono_literals;

class WhiskersPublisher : public rclcpp::Node
{
  public:
    WhiskersPublisher() 
    : Node("whiskers_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<robominer_msgs::msg::WhiskerArray>("/whiskers", 10);
      timer_ = this->create_wall_timer(
				PUBLISH_INTERVAL, std::bind(&WhiskersPublisher::timer_callback, this)
      );
    }
	
  private:  
    void timer_callback()
    {
      auto message = robominer_msgs::msg::WhiskerArray();
      
      // Acquire data from sensors
      for(uint8_t m=MUX_STARTADDR; m<MUX_STARTADDR+NUM_MUX; m++)
      {
        // Deselect all channels of the previous multiplexer    
        muxDisablePrevious(m);

        for(uint8_t i=0; i<NUM_SENSORS; i++)
        { 
          // Switch to next multiplexed port  			  
          tcaSelect(m,i);   			  

          // Read sensor with selected type of representation
          readSensor(m-MUX_STARTADDR,i); 		  
        }    
      }
      
      // Publish
      message.header.stamp = this->now();
      message.num_mux = NUM_MUX;
      message.num_sensors = NUM_SENSORS;
      message.whiskers = std::vector<robominer_msgs::msg::Whisker>(NUM_MUX*NUM_SENSORS);
      int sensorNum = 0;
      for(uint8_t m=0; m < NUM_MUX; m++)
      {
		  for(uint8_t s = 0; s < NUM_SENSORS; s++)
		  {     			  
			  
			  /*
			   * TODO: move representation to higher level message
			   * */
			  
			  if(t == Spherical)
			  {
				  message.whiskers[sensorNum].representation = "Spherical";
			  }
			  else // Cartesian
			  {
				  message.whiskers[sensorNum].representation = "Cartesian";
			  }

			  message.whiskers[sensorNum].pos = robominer_msgs::msg::WhiskerPosInGrid();
			  message.whiskers[sensorNum].pos.row_num = m;
			  message.whiskers[sensorNum].pos.col_num = s;
			  if(m % 2 != 0)
			  {
				  message.whiskers[sensorNum].pos.offset_y = -0.5; // offset of sensor row in y direction
			  }
			  else
			  {
				  message.whiskers[sensorNum].pos.offset_y = 0;
			  }			  			  
			  sensorNum++;
		  }		  
		  
	  }      
      publisher_->publish(message);      
      
      // Print readings to console
      printOut();		
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robominer_msgs::msg::WhiskerArray>::SharedPtr publisher_;
    size_t count_;
			
};

Tlv493d *Tlv493dMagnetic3DSensor[NUM_MUX][NUM_SENSORS]; // 2D array of pointers to Tlv493d Object

unsigned char * txString = new unsigned char[MAXBUF];
uint8_t txIndex = 0;
unsigned char encodeResult[2];
float data[NUM_MUX][NUM_SENSORS][3];
bool init = true;
uint32_t loopSeq = 0;


using namespace std;

/**
 * Executable entry point.
 * 
 * @return Error code (TODO: currently not implemented)
 */
int main(int argc, char **argv)
{    
  // ROS SETUP 
  rclcpp::init(argc, argv);  
  
  // SENSORS AND MUX SETUP 
  setup(); 
  
  // MAIN LOOP 
  debug("We are here");
  rclcpp::spin(std::make_shared<WhiskersPublisher>());
  
  // ON EXIT
  rclcpp::shutdown();  
  Wire.end();
  debug("\n>>>> Good-bye!\n\n");
  return 0;     
}


/**
 * Opens the I2C bus, constructs and initializes the sensors.
 */
void setup()
{
  debug(">>>> RESET\n");	  
	
  debug(">>>> Wire.begin()\n");
  Wire.begin((uint8_t)I2C_BUS_ID);
  
  debug(">>>> Constructing objects of TLV493D\n");
  for(uint8_t m=0; m<NUM_MUX; m++)
  { 
    for(uint8_t i=0; i<NUM_SENSORS; i++)
    {
      Tlv493dMagnetic3DSensor[m][i] = new Tlv493d();
    }
  }

  muxDisableAll();
		
  for(uint8_t m=MUX_STARTADDR; m<MUX_STARTADDR+NUM_MUX; m++)
  {
	debug(">>>> Initializing %d sensor(s) TLV493D\n",NUM_SENSORS);
    debug("     on MUX address 0x%02X\n",m);
	muxDisablePrevious(m); 

	for(uint8_t i=0; i<NUM_SENSORS; i++)
	{
	  tcaSelect(m,i);
	  if(!initSensor(m-MUX_STARTADDR,i))
	  {
		  RCLCPP_ERROR(rclcpp::get_logger("Sensor_MUX_main_ROS2"), "Sensor %d.%d not detected at default I2C address. Check the connection.\n",m-MUX_STARTADDR,i);
	  }			
	}
  }	
  testAndReinitialize(); // This is some dirty hack to avoid "badly initialized" (?) sensors in Linux
  init = false;
  debug(">>>> Setup ended. Waiting for some seconds...\n");
#ifdef DEBUG
  delay(5000);
#endif
}

/**
 * After sensor initialization, the sensors are polled several times to see if initialization was successful.
 * In case of failure, a sensor will be reinitialized.
 */
void testAndReinitialize()
{
	for(uint8_t m=MUX_STARTADDR; m<MUX_STARTADDR+NUM_MUX; m++)
	{
		// Deselect all channels of the previous multiplexer    
		muxDisablePrevious(m);
		
		for(uint8_t i=0; i<NUM_SENSORS; i++)
		{ 
		  // Switch to next multiplexed port  			  
		  tcaSelect(m,i);   
		  debug(">>>> Checking sensor %d.%d\n",m-MUX_STARTADDR,i);
		  
		  bool ok = false;	
		  int attempts = 0;	  
		  
		  while(!ok && attempts < 10)
		  {
			  debug("     Attempt: %d/10\n",(attempts+1));
			  int readNum = 0;
			  while(readNum < 5)
			  {
				  debug("       Reading data (%d/5)\n",(readNum+1));
				  readSensor(m-MUX_STARTADDR,i); 
				  readNum++;	
				  if(data[m-MUX_STARTADDR][i][0] != 0 || data[m-MUX_STARTADDR][i][1] != 0 || data[m-MUX_STARTADDR][i][2] != 0)
				  {
					  ok = true;
					  break;
				  }
				  if(readNum == 5)
				  {
					debug("     Invalid data; reinitializing\n");
					initSensor(m-MUX_STARTADDR,i);
				  }		  
			  }
			  attempts++;
		  }		
		  if(!ok)
		  {
			  debug("     Failed to initialize this sensor.\n");
		  }	 
		  else
		  {
			  debug("     Sensor ok.\n");
		  } 
	  		    
		}    
	}
}

/**
 * Selects a channel on a multiplexer.
 * 
 * @params m The selected multiplexer address
 * @params i The selected channel of the multiplexer (1...8)
 */
void tcaSelect(uint8_t m, uint8_t i) 
{
  if(init == false && NUM_MUX == 1 && NUM_SENSORS == 1)
  {
	  return;
  }
  Wire.beginTransmission(m);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

/**
 * Disables all 8 multiplexers starting from address MUX_STARTADDR.
 */
void muxDisableAll()
{
  for(int m=MUX_STARTADDR; m<MUX_STARTADDR+8; m++)
  {
    tcaDisable(m);
  }
}

/**
 * Disables the multiplexer in sequence before the currently selected.
 * If the first multiplexer is selected, it disables the last one.
 * Disabling is done by deselecting all channels.
 * 
 * @params m The address of the currently selected multiplexer
 */
void muxDisablePrevious(uint8_t m)
{
  if(NUM_MUX == 1)
  {
    return;
  }
  uint8_t muxToDisable;
  if(m == MUX_STARTADDR)
  {
    muxToDisable = MUX_STARTADDR + NUM_MUX - 1;
  }
  else
  {
    muxToDisable = m - 1;
  }
  tcaDisable(muxToDisable);
}

/**
 * Disables the multiplexer at the specific address.
 * 
 * @params addr The address of the multiplexer to disable
 */
void tcaDisable(uint8_t addr)
{
  Wire.beginTransmission(addr);
  Wire.write(0);
  Wire.endTransmission();
  delay(5); 
}

/**
 * Initializes a sensor of type Tlv493dMagnetic3DSensor which is connected to one channel of a multiplexer TCA9548a.
 * 
 * @params m The number of the multiplexer to which the sensor is attached (0...NUM_MUX-1)
 * @params i The number of the multiplexer's channel on which the sensor is attached (0...NUM_SENSORS-1)
 * @return Error flag: true in case of success, otherwise (in case of bus error) false.
 */
bool initSensor(uint8_t m, uint8_t i)
{
  bool ret = true;
  Tlv493dMagnetic3DSensor[m][i]->begin();
  if(fastMode)
  {
	  ret = Tlv493dMagnetic3DSensor[m][i]->setAccessMode(Tlv493dMagnetic3DSensor[m][i]->FASTMODE);
	  if(ret == BUS_ERROR)
	  {
		  ret = false;
	  }
  }
  Tlv493dMagnetic3DSensor[m][i]->disableTemp();  
  return ret;
}

/**
 * Polls a sensor of type Tlv493dMagnetic3DSensor which is connected to one channel of a multiplexer TCA9548a.
 * The returned results from each sensor are stored in the global 3D array "data[][][]".
 * The returned values may represent the magnet's position in Cartesian coordinates or in Polar coordinates,
 * depending on the value of global constant "t".
 * 
 * @params m The number of the multiplexer to which the sensor is attached (0...NUM_MUX-1)
 * @params i The number of the multiplexer's channel on which the sensor is attached (0...NUM_SENSORS-1)
 */
void readSensor(uint8_t m, uint8_t i)
{
  int dly = Tlv493dMagnetic3DSensor[m][i]->getMeasurementDelay();
  delay(dly);
  Tlv493d_Error_t err = Tlv493dMagnetic3DSensor[m][i]->updateData();
  if(err != TLV493D_NO_ERROR)
  {
    return;
  }  

  if(t == Spherical)
  {      
    data[m][i][0] = radToDeg(Tlv493dMagnetic3DSensor[m][i]->getAzimuth());                       // angle in xy plane relative to x axis
    data[m][i][1] = radToDeg(Tlv493dMagnetic3DSensor[m][i]->getPolar());                         // inclination angle relative to x axis
    data[m][i][2] = Tlv493dMagnetic3DSensor[m][i]->getAmount();                                  // distance between magnet and sensor
  }
  else if(t == Cartesian)
  {
    data[m][i][0] = Tlv493dMagnetic3DSensor[m][i]->getX();
    data[m][i][1] = Tlv493dMagnetic3DSensor[m][i]->getY();
    data[m][i][2] = Tlv493dMagnetic3DSensor[m][i]->getZ();       
  }   
  
  // results are according to sensor reference frame if the third value is non-negative (not tested for Spherical)
  if(data[m][i][2] < 0) { 
    data[m][i][2] *= -1;
    data[m][i][1] *= -1;
    data[m][i][0] *= -1;
  }   
}

/**
 * Prints the sensor values currently stored in the global 3D array "data[][][]" to console.
 * The type of console output (plain text or compressed) depends on the value of the global constant "mf".
 */
void printOut()
{
  if(mf == PlainText)
  {    
    for(uint8_t m=0;m<NUM_MUX;m++)
    {  
      printf("{");
      for(uint8_t i=0;i<NUM_SENSORS;i++)
      {
        printf("[");
        for(uint8_t j=0;j<3;j++)
        {
          printf("%.2f",data[m][i][j]);
          if(j < 2)
          {
            printf(";");    
          }
        }    
        printf("]");
      }
      printf("}");
    }
    printf("\n");
     
  }
  else if (mf == Compressed)
  {    
	for(uint8_t m=0;m<NUM_MUX;m++)
	{
      for(uint8_t i=0;i<NUM_SENSORS;i++)
      {
		uint8_t numValues = 3;
		if(t == Spherical && !sendPolarRadius)
		{
		  numValues = 2;
		}
		for(int j=0;j<numValues;j++)
		{
		  encode(data[m][i][j]);
		  writeTx(encodeResult[0]);
		  writeTx(encodeResult[1]);
		}		
	  }          
	} 
    writeTx(endSignature[0]);
    writeTx(endSignature[1]);  
    
    print(txString,txIndex);  // Instead of printing txString to console, it can be used in some other way (e.g., sent via Serial interface)
    txIndex = 0;
  }
}





/**
 * Encodes a float into a 16-bit representation as Little Endian.
 * The result is stored in the global array "encodeResult[]".
 * 
 * @params f The float to be encoded
 */
void encode(float f)
{  
  int16_t s = f * multiplier;  // multiplying the float by 100, then dropping the last 16 bits
  
  encodeResult[1] = s >> 8;
  encodeResult[0] = s & 0x00ff;     
}

/**
 * Converts a value in radians to degrees.
 * 
 * @params rad The value to be converted
 * @return The converted value
 */
float radToDeg(float rad)
{
  return rad * 180.0 / PI;
}

/**
 * Concatenates characters into the global string txIndex.
 * This is used to assemble the string printed to console in case message format is Compressed. 
 * 
 * @params c The character to be concatenated
 */
void writeTx(unsigned char c) 
{
  if(txIndex >= MAXBUF) { return; }
  txString[txIndex] = c;
  txIndex++;
}




