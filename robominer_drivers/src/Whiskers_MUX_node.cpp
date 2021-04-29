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
 * Kilian Ochs, 27.04.2021
 * Centre for Biorobotics, TALTECH, Tallinn
 * Robominers experimental setup
 * 
 * This sketch is used to interface magnetic Hall sensors of type TLV493D
 * using several multiplexers of type TCA9548A. It publishes messages of
 * type "WhiskerArray" in the ROS2 environment.
 * 
 * Note: The number of sensors per multiplexer ("NUM_SENSORS") must be
 * the same for all multiplexers.
 * 
 * Continuously reads the sensors one by one, then prints all sensor
 * readings to Console in one burst. When reading the Console outputs
 * during manual verification, the MessageFormat f = PlainText should be used.
 *
 * The MessageFormat f = Compressed packs each sensor float value into a 16bit
 * integer. The result is stored in "txString", which can be used for data
 * acquisition purposes in the future (e.g., send to SimuLink via Serial).
 * 
 * To run this code on Ubuntu instead of Olimex, uncomment "#define UBUNTU"
 * in "./olimex/config.h".
 * To view debug messages, uncomment "#define DEBUG"  in "./olimex/config.h".
 * 
 * When running on an Ubuntu computer (Desktop/laptop), a USB-to-I2C
 * adapter can be used. Please note the name of the i2c bus using "dmesg"
 * after plugging in the adapter. To define platform-specific bus IDs,
 * see "Whiskers_MUX_node.h".
 *  * 
 * To be compiled for a Linux-based system.
 * This is a ported version of the library originally made for the Adafruit Feather M0
 * (see gitHub: kilian2323/3dMagSensorsVisual).
 *
 * Note: Define all constants in Whiskers_MUX_node.h.
 */




#include "Whiskers_MUX_node.h"


   




/*******************************
 * 
 * MAIN METHOD
 * 
 *******************************/

 
 
 
 
 
 

using namespace std;

/**
 * Executable entry point.
 * 
 * @return Error code: 0 if ok, otherwise 1.
 */
int main(int argc, char **argv)
{    
    // GRID OBJECT CONSTRUCTION
    SensorGrid grid(Cartesian, Grid, true, true); // Representation, Message format, Hall sensors in fast mode?, Send polar radius if Spherical/compressed?
    
    // ROS SETUP 
    rclcpp::init(argc, argv);    
    
    // SENSORS AND MUX SETUP 
    int ret = grid.setup(); 
    if(ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"), "Errors encountered during sensor setup.\nCheck NUM_MUX, NUM_SENSORS and hardware connections.\n");
        //return ret;
    }

    // MAIN LOOP 
    rclcpp::spin(std::make_shared<WhiskersPublisher>(&grid)); // This constructs node and then spins it.

    // ON EXIT
    rclcpp::shutdown();  
    Wire.end();
    debug("\n>>>> Good-bye!\n\n");
    
    return ret;     
}

   

   
   

/*******************************
 * 
 * ROS-SPECIFIC FUNCTIONALITY
 * 
 *******************************/

 
 
 
 
 
using namespace std::chrono_literals;

/**
 * ROS node class constructor.
 */
WhiskersPublisher::WhiskersPublisher(SensorGrid * _grid) : rclcpp::Node("whiskers_interface")
{
    publisher_ = this->create_publisher<robominer_msgs::msg::WhiskerArray>("/whiskers", 10);
    timer_ = this->create_wall_timer(PUBLISH_INTERVAL, std::bind(&WhiskersPublisher::timer_callback, this));
    grid = _grid;
#ifdef DEBUG
    lastLoop = 0;
#endif
}

/**
 * Callback function for the timer interrupt in ROS. Publishes at defined intervals (PUBLISH_INTERVAL).
 */
void WhiskersPublisher::timer_callback(void)
{
    // Calculate actual publishing frequency
#ifdef DEBUG
    volatile unsigned long now = millis();
    loopFreq = 1000.0 / float(now - lastLoop);
    lastLoop = now;      
    debug("\nPublishing @ %.2f Hz\n", loopFreq);
#endif
      
    // Acquire data from sensors

    for(uint8_t m=0; m<NUM_MUX; m++)
    {
        // Deselect all channels of the previous multiplexer    
        grid->muxDisablePrevious(m);

        for(uint8_t i=0; i<NUM_SENSORS; i++)
        { 
            // Switch to next multiplexed port   
            grid->multiplexers[m].selectChannel(i);
            // Read sensor with selected type of representation
            grid->sensors[m][i].read();                      
        }    
    }
      
    // Assemble and publish message to ROS

    auto message = robominer_msgs::msg::WhiskerArray();

    message.header.stamp = this->now();
    message.num_mux = NUM_MUX;
    message.num_sensors = NUM_SENSORS;

    if(grid->r == Spherical)
    {
        message.representation = "Spherical";
    }
    else // Cartesian
    {
        message.representation = "Cartesian";
    }
      
    for(uint8_t sNum = 0; sNum < NUM_MUX*NUM_SENSORS; sNum++)
    {
        message.whiskers.push_back(robominer_msgs::msg::Whisker());
    }

    int sensorNum = 0;
    for(uint8_t m=0; m < NUM_MUX; m++)
    {
        for(uint8_t s = 0; s < NUM_SENSORS; s++)
        {                   
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
            message.whiskers[sensorNum].has_error = grid->sensors[m][s].hasError || !(grid->sensors[m][s].initOK); 
            message.whiskers[sensorNum].x = grid->sensors[m][s].data[0];    
            message.whiskers[sensorNum].y = grid->sensors[m][s].data[1];  
            message.whiskers[sensorNum].z = grid->sensors[m][s].data[2];                    
            sensorNum++;
        }               
    }      
    publisher_->publish(message);      
      
      // Print readings to console
#ifdef CONSOLE_PRINT
    grid->printReadingsToConsole();     
#endif
}








/*******************************
 * 
 * HARDWARE-SPECIFIC FUNCTIONALITY
 * 
 *******************************/
 
 
 
 
 
 
 
 

using namespace std;


/**
 * SensorGrid class constructor.
 * 
 * @param _r The tzpe of representation of sensor data (Cartesian or Spherical).
 * @param _f The type of message representation for displaying sensor data
 *           (PlainText, Compressed or Grid). Note: will print only if CONSOLE_PRINT
 *           is #define'd.
 * @param _fastMode [optional; default: true] If true, the sensors are being
 *                  read in a higher frequency than when set to false.
 * @param _sendPolarRadius [optional; default: true] If true, the sensor data 
 *                         printed when using Compressed message format
 *                         and Spherical representation will include the third
 *                         value as polar radius, otherwise not (only two values
 *                         will be printed).
 */
SensorGrid::SensorGrid(Representation _r, MessageFormat _f, bool _fastMode, bool _sendPolarRadius)
{
    init = true;    
    r = _r;
    f = _f;
    sendPolarRadius = _sendPolarRadius;
    fastMode = _fastMode;
    txString = new unsigned char[MAXBUF];
    txIndex = 0; 
}

/**
 * Opens the I2C bus, constructs and initializes the sensors.
 * 
 * @return Error code: 0 if ok, otherwise 1.
 */
int SensorGrid::setup(void)
{
    debug(">>>> RESET\n");     
    
    int ret = 0; 
    
    endSignature[0] = '\r';
    endSignature[1] = '\n';
    
    debug(">>>> Wire.begin()\n");
    Wire.begin((uint8_t)I2C_BUS_ID);
    
    debug(">>>> Preparing sensor objects TLV493D for the grid\n");
    for(uint8_t m=0; m<NUM_MUX; m++)
    { 
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        {
            sensors[m][i].setGridPosition(m,i);
        }
    }
    
    debug(">>>> Constructing multiplexers, initializing sensors TLV493D\n");
    for(uint8_t m=0; m<NUM_MUX; m++)
    { 
        multiplexers.push_back(Multiplexer(MUX_STARTADDR+m));
    }
    for(uint8_t m=0; m<NUM_MUX; m++)
    {
        muxDisablePrevious(m);
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        {
            multiplexers[m].selectChannel(i,true);
            if(sensors[m][i].initialize(fastMode) == BUS_ERROR)
            {
                debug(">>>> Sensor %d.%d not detected at default I2C address. Check the connection.\n",m,i);
            }
        }
    }        
    
    ret = hallTestAndReinitialize(); // This is some dirty hack to avoid "badly initialized" (?) sensors in Linux
    
#ifdef DEBUG
    debug(">>>> Setup ended. Waiting for some seconds...\n");
    delay(5000);
#endif
    return ret;
}

/**
 * After sensor initialization, the sensors are polled several times to
 * see if initialization was successful.
 * In case of failure, a sensor will be reinitialized here.
 * 
 * @return Error code: 0 if ok, otherwise 1.
 */
int SensorGrid::hallTestAndReinitialize(void)
{
    int ret = 0;
    for(uint8_t m=0; m<NUM_MUX; m++)
    {
        // Deselect all channels of the previous multiplexer    
        muxDisablePrevious(m);
        
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        { 
            // Switch to next multiplexed port              
            multiplexers[m].selectChannel(i,true); 

            debug(">>>> Checking sensor %d.%d\n",m,i);

            bool ok = false;  
            int attempts = 0;  
            bool present = true; 

            while(!ok && attempts < 10 && present)
            {
                debug("     Attempt: %d/10\n",(attempts+1));
                int readNum = 0;
                while(readNum < 5)
                {                    
                    debug("       Reading data (%d/5)\n",(readNum+1));
                    if(!sensors[m][i].read(r))
                    {
                        present = false;
                        if(sensors[m][i].initialize(fastMode) == BUS_ERROR)
                        {
                            debug(">>>> Sensor %d.%d not detected at default I2C address. Check the connection.\n",m,i);
                        }
                        break;
                    }
                    readNum++;    
                    //debug("         Got: %.2f, %.2f, %.2f\n",sensors[m][i].data[0],sensors[m][i].data[1],sensors[m][i].data[2]);
                    if(sensors[m][i].data[0] != 0 || sensors[m][i].data[1] != 0 || sensors[m][i].data[2] != 0)
                    {
                        ok = true;
                        break;
                    }
                    if(readNum == 5)
                    {
                        debug("     Invalid data; reinitializing\n");
                        if(sensors[m][i].initialize(fastMode) == BUS_ERROR)
                        {
                            debug(">>>> Sensor %d.%d not detected at default I2C address. Check the connection.\n",m,i);
                        }
                    }       
                }
                attempts++;
            }     
            if(!ok)
            {
                debug("     Failed to initialize sensor %d.%d.\n",m,i);
                ret = 1;
            }  
            else
            {
                debug("     Sensor ok.\n");
            } 
        }    
    }
    init = false;
    return ret;
}

/**
 * Disables the multiplexer in sequence before the currently selected.
 * If the first multiplexer is selected, it disables the last one defined.
 * (Disabling is done by deselecting all channels of the multiplexer.)
 * 
 * @param muxNum The index number of the currently selected multiplexer
 */
void SensorGrid::muxDisablePrevious(uint8_t muxNum)
{
    if(muxNum > 0)
    {
        multiplexers[muxNum-1].disable();
    }
    else
    {
        multiplexers[NUM_MUX-1].disable();
    }    
}

/**
 * Disables each multiplexer starting from the first one up to the number specified.
 * @param totalNum [optional; default: NUM_MUX] The number of multiplexers to disable.
 */
void SensorGrid::muxDisableAll(uint8_t totalNum)
{
    for(int m=0; m<totalNum; m++)
    {
        multiplexers[m].disable();
    }
}

/**
 * Prints the sensor values currently stored all the sensors' field member "data[]"
 * to console. The type of console output (plain text or compressed) depends
 * on the value of "f".
 */
 #ifdef CONSOLE_PRINT
void SensorGrid::printReadingsToConsole(void)
{
    if(f == PlainText)
    {    
        for(uint8_t m=0;m<NUM_MUX;m++)
        {  
            printf("{");
            for(uint8_t i=0;i<NUM_SENSORS;i++)
            {
                printf("[");
                for(uint8_t j=0;j<3;j++)
                {
                    printf("%.2f",sensors[m][i].data[j]);
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
    else if (f == Compressed)
    {    
        for(uint8_t m=0;m<NUM_MUX;m++)
        {
            for(uint8_t i=0;i<NUM_SENSORS;i++)
            {
                uint8_t numValues = 3;
                if(r == Spherical && !sendPolarRadius)
                {
                    numValues = 2;
                }
                for(int j=0;j<numValues;j++)
                {
                    unsigned char * encoded = new unsigned char[2];
                    sensors[m][i].encode(j, encoded);
                    writeTx(encoded[0]);
                    writeTx(encoded[1]);
                }       
            }          
        } 
        writeTx(endSignature[0]);
        writeTx(endSignature[1]);  
        print(txString,txIndex);  // TODO: Instead of printing txString to console, it can be used in some other way later (e.g., sent via Serial interface)
        txIndex = 0;
    }
    else if (f == Grid)
    {
        const static int charsWidthPerSensor = 10; // horizontal space inside a table cell without vertical line
        const static int charsHeightPerRow = 5;    // vertical space inside a table cell without horizontal line  
                
        const static int gridWidth = (charsWidthPerSensor+1)*NUM_SENSORS+2; // includes all verticals and newline character
        
        const static int totalCharSize = NUM_MUX * (gridWidth*(charsHeightPerRow+1))+gridWidth + 2; // includes last horizontal line, final newline character and null terminator
        
        char * gridFull = new char[totalCharSize];
        int indexPos = 0; // keeping track of characters written in total
        
        // Horizontal lines in between rows
        char * horizontalLine = new char[gridWidth+1]; // includes newline character and null terminator
        for(int c=0; c<gridWidth-1; c++)
        {
            horizontalLine[c] = '-';
        }
        horizontalLine[gridWidth-1] = '\n'; 
        horizontalLine[gridWidth] = '\0';       // null-terminating (for strcat)
        strcat(gridFull,horizontalLine);          
        indexPos = gridWidth;    
                
        for(uint8_t m=0;m<NUM_MUX;m++)
        {              
            // One multiline table row including all sensors per multiplexer and all three values per sensor, no horizontals
            char * gridRow = new char[gridWidth*charsHeightPerRow+1]; // includes all newline characters and one final null terminator
            int rowIndexPos = 0;
            for(int row = 0; row<charsHeightPerRow; row++) // one table row consists of four rows
            {
                gridRow[rowIndexPos] = '|'; // start vertical
                rowIndexPos++;
                for(int cell = 0;cell<NUM_SENSORS;cell++)
                {                      
                    if(row == 0 || row == 4)    // empty rows in cell
                    {
                        for(int c = 0; c < charsWidthPerSensor; c++)
                        {
                            gridRow[rowIndexPos] = ' ';
                            rowIndexPos++;
                        }
                    }
                    else                        // valued rows in cell
                    {
                        char * content = new char[charsWidthPerSensor+1]; // content will be null terminated
                        int len;
                        if(!sensors[m][cell].hasError && sensors[m][cell].initOK)
                        {
                            len = sprintf(content,"  %6.2f  ",sensors[m][cell].data[row-1]);
                        }
                        else
                        {
                            len = sprintf(content,"  XXXXXX  ");
                        }
                        gridRow[rowIndexPos] = '\0'; // null-terminating for strcat to work properly
                        strcat(gridRow,content);
                        rowIndexPos += len;
                    }                   
                    gridRow[rowIndexPos] = '|'; // dividing or end vertical
                    rowIndexPos++;                             
                }
                gridRow[rowIndexPos] = '\n'; // end of line
                rowIndexPos++;
            }
            // end of mux row
            gridRow[rowIndexPos] = '\0'; // null-terminating
            strcat(gridFull,gridRow);    // add table row to full table     
            indexPos += rowIndexPos;
            strcat(gridFull,horizontalLine); // add horizontal line to full table
            indexPos += gridWidth;
        }
        gridFull[indexPos] = '\n';
        indexPos++;
        gridFull[indexPos] = '\0'; // null-terminating (for printf)
        printf("%s\n",gridFull);    
        
    }
}
#endif

/**
 * Concatenates characters into the class-global string txIndex.
 * This is used to assemble the string printed to console in case message
 * format is Compressed. 
 * 
 * @params c The character to be concatenated
 */
void SensorGrid::writeTx(unsigned char c) 
{
  if(txIndex >= MAXBUF) { return; }
  txString[txIndex] = c;
  txIndex++;
}

/**
 * SensorGrid::Multiplexer class constructor.
 * 
 * @param addr The multiplexer's I2C address.
 */
SensorGrid::Multiplexer::Multiplexer(uint8_t addr)
{
    //debug(">>>> Constructing Multiplexer object with address 0x%02X\n",addr);
    address = addr;
}

/**
 * Disables the multiplexer by writing 0 to its I2C address.
 */
void SensorGrid::Multiplexer::disable(void)
{
    Wire.beginTransmission(address);
    Wire.write(0);
    Wire.endTransmission();
}

/**
 * Selects a channel on the multiplexer.
 * 
 * @param ch The selected channel of the multiplexer (0...7).
 * qparam init [optional; default: false] If true, the channel selection will
 *             definitely be performed, even if there is only one MUX and one sensor.
 */
void SensorGrid::Multiplexer::selectChannel(uint8_t ch, bool init)
{
    // We have to set a channel definitely during init,
    // and otherwise in case there is more than one sensor
    // or more than one multiplexer.
    if(init == false && NUM_MUX == 1 && NUM_SENSORS == 1) 
    {   
        return; 
    }
    Wire.beginTransmission(address);
    Wire.write(1 << ch);
    Wire.endTransmission();  
}

/**
 * Overridden default constructor for class SensorGrid::HallSensor.
 */
SensorGrid::HallSensor::HallSensor()
{
    //debug(">>>> Constructing HallSensor object using default constructor\n");
    for(int i=0; i<3; i++)
    {
        data[i] = 0.0;
    }
    initOK = false;  
    hasError = false;  
    numReadZeros = 0;
}

/**
 * Allows to set the row and column number inside the HallSensor object.
 * 
 * @param mNum The number of the row (0...NUM_MUX-1).
 * @param sNum The number of the column (0...NUM_SENSORS-1).
 */
void SensorGrid::HallSensor::setGridPosition(uint8_t mNum, uint8_t sNum)
{
    pos[0] = mNum;
    pos[1] = sNum;
}


/**
 * Initializes the sensor of type Tlv493dMagnetic3DSensor.
 * 
 * @param fastMode If true, sets the access mode of the sensor to FASTMODE.
 * @param reinitialize If true, the sensor will be reset before initializing.
 *                     This can be useful if a sensor needs to be reinitialized
 *                     due to failure during operation (repeated zero-readings).
 * @return Error flag: true in case of success, otherwise (in case of bus error) false.
 */
bool SensorGrid::HallSensor::initialize(bool fastMode, bool reinitialize)
{
    bool ret = false;
    initOK = true;
    if(reinitialize)
    {
        // TODO: call Tlv493d::begin with bus identifier, to force reset of the sensor
        sensor.begin(&Wire);
    }
    else
    {
        sensor.begin();
    }
    if(fastMode)
    {
        ret = sensor.setAccessMode(sensor.FASTMODE);
        if(ret == BUS_ERROR)
        {
            debug(">>>> Bus error on access mode = FASTMODE\n");
            initOK = false;
            return ret;
        }
    }
    sensor.disableTemp();  
    return ret;
}

/**
 * Polls the sensor of type Tlv493dMagnetic3DSensor.
 * The returned results from each sensor are stored in the array "data[]".
 * The returned values may represent the magnet's position in Cartesian
 * coordinates or in Polar coordinates, depending on the value of "r".
 * 
 * @param r [optional; default: Cartesian] The desired data representation
 *          (Cartesian or Spherical).
 * @return Success code: true if readable, false if not.
 */
bool SensorGrid::HallSensor::read(Representation r)
{
    int dly = sensor.getMeasurementDelay();
    delay(dly);
    Tlv493d_Error_t err = sensor.updateData();
    if(err != TLV493D_NO_ERROR)
    {
        debug("Error reading sensor %d.%d. Error (%d) was: %s.\n",pos[0],pos[1],errno,strerror(errno));
        hasError = true;
        return false;
    }    
    hasError = false;  

    if(r == Spherical)
    {      
        data[0] = radToDeg(sensor.getAzimuth());                       // angle in xy plane relative to x axis
        data[1] = radToDeg(sensor.getPolar());                         // inclination angle relative to x axis
        data[2] = sensor.getAmount();                                  // distance between magnet and sensor
    }
    else if(r == Cartesian)
    {
        data[0] = sensor.getX();
        data[1] = sensor.getY();
        data[2] = sensor.getZ();       
    }   
    
    // repeatedly reading only zeros indicates a faulty sensor
    if(data[0] == 0 && data[1] == 0 && data[2] == 0)
    {
        numReadZeros++;
    }
    else
    {
        numReadZeros = 0;
    }
    if(numReadZeros >= MAX_READS_ZEROS)
    {
        hasError = true;
        numReadZeros = MAX_READS_ZEROS;
        
        return false;
    }

    // results are according to sensor reference frame if the third value is non-negative (not tested for Spherical)
    if(data[2] < 0)
    { 
        data[2] *= -1;
        data[1] *= -1;
        data[0] *= -1;
    }   
    return true;
}

/**
 * Converts a value from radians to degrees.
 * 
 * @param rad The value in radians to be converted.
 * @return The converted value in degrees.
 */
float SensorGrid::HallSensor::radToDeg(float rad)
{
  return rad * 180.0 / PI;
}

/**
 * Encodes a float in data[] into a 16-bit representation
 * as Little Endian.
 * 
 * @param index The index of the float in data[].
 * @param result A pointer to two values of type unsigned char.
 */
void SensorGrid::HallSensor::encode(uint8_t index, unsigned char * result)
{  
    // multiplying the float by ENCODE_MULTIPLIER, then dropping the last 16 bits  
    int16_t s = data[index] * ENCODE_MULTIPLIER;    
    result[1] = s >> 8;
    result[0] = s & 0x00ff;     
}
