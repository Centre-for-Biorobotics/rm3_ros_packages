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
 * Kilian Ochs, 17.05.2021
 * Centre for Biorobotics, TALTECH, Tallinn
 * Robominers experimental setup
 * 
 * This sketch is used to interface magnetic Hall sensors of type TLV493D
 * using several multiplexers of type TCA9548A. It publishes messages of
 * type "WhiskerArray" under the topic "/whiskers" in the ROS2 environment.
 * 
 * Continuously reads the sensors one by one, then prints all sensor
 * readings to Console in one burst. When reading the Console outputs
 * during manual verification, the MessageFormat f = PlainText or f = Grid
 * should be used.
 * 
 * Define number of multiplexers and number of sensors per multiplexers
 * in "Whiskers_MUX_node.h" (NUM_MUX and NUM_SENSORS).
 * 
 * Note: The number of sensors per multiplexer ("NUM_SENSORS") should be
 * the same for all multiplexers. If not, define the maximum number of
 * sensors on a multiplexer. The sensors which are missing on the other
 * multiplexers won't initialize, but will not break the program.
 * 
 * If a sensor does not get successfully initialized, it will be flagged 
 * and ignored from there on. If a sensor has been successfully initialized
 * but fails during later operation, the program attempts to reinitialize it
 * until it comes back.
 *
 * The MessageFormat f = Compressed packs each sensor float value into a 16bit
 * integer. The result is stored in "txString", which can be used for data
 * acquisition purposes in the future (e.g., send to SimuLink via Serial).
 * 
 * To activate debug messages in the console, the parameter "debug_mode"
 * must be set to true. If sensor data should be printed to the console, the
 * parameter "use_console_print" must be set to true. These two parameters can
 * be set in the launch file of this node.
 * 
 * To run this code on Olimex, uncomment "#define OLIMEX" in "Whiskers_MUX_node.h".
 * 
 * When running on an Ubuntu computer (Desktop/laptop), a USB-to-I2C
 * adapter can be used. Please note the name of the i2c bus using "dmesg"
 * after plugging in the adapter. To define platform-specific bus IDs,
 * see "Whiskers_MUX_node.h".
 *  
 * To be compiled for a Linux-based system.
 * This is a ported version of the library originally made for the Adafruit Feather M0
 * (see gitHub: kilian2323/3dMagSensorsVisual).
 *
 * Note: Define all constants in "Whiskers_MUX_node.h".
 */



#include "Whiskers_MUX_node.h"

   
bool use_debug_mode = true;
bool use_console_print = true;


/*******************************
 * 
 * MAIN METHOD
 * 
 *******************************/

 
 
 
 
 
 

using namespace std;

/**
 * Executable entry point.
 * 
 * @return Error code: see sensorGrid::setup()
 */
int main(int argc, char **argv)
{   
    // ROS SETUP 
    rclcpp::init(argc, argv);    
    RCLCPP_INFO(rclcpp::get_logger("whiskers_interface"), "Initializing...");
    
    // GRID OBJECT CONSTRUCTION
    SensorGrid grid(Cartesian, Grid, true); // Representation, Message format, Hall sensors in fast mode?
    
    // NODE CONSTRUCTION
    WhiskersPublisher *whiskersPub = new WhiskersPublisher(&grid);// Will immediately poll parameters from the parameter server
    shared_ptr<WhiskersPublisher> sharedWhiskersPub(whiskersPub); // Convert raw pointer to shared pointer
    
    // SENSORS AND MUX SETUP 
    int ret = grid.setup(); 
    if(ret == 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"), "Errors encountered during sensor initialization.\nCheck NUM_MUX, NUM_SENSORS and hardware connections.");
    }
    else if(ret == 1)
    {
        RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"), "Errors encountered during sensor read checks.");
    }
    if(::use_debug_mode) { printf("\n<<<< SETUP ended. Waiting for some seconds...\n"); ::delay(5000); }
    grid.init = false;

    // KEEP SPINNING   
    RCLCPP_INFO(rclcpp::get_logger("whiskers_interface"), ">>>>> Starting to publish. <<<<<");  
    rclcpp::spin(sharedWhiskersPub);

    // ON EXIT
    RCLCPP_INFO(rclcpp::get_logger("whiskers_interface"), "Clean exit in progress; return code %d.",ret); 
    rclcpp::shutdown();  
    grid.powerdown();
    Wire.end();
    
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
    grid = _grid;
    publisher_ = this->create_publisher<robominer_msgs::msg::WhiskerArray>("/whiskers", 10);
    timer_ = this->create_wall_timer(PUBLISH_INTERVAL, std::bind(&WhiskersPublisher::timer_callback, this));    

    this->declare_parameter<bool>("debug_mode",::use_debug_mode);
    this->declare_parameter<bool>("console_print",::use_console_print);    
    this->get_parameter("debug_mode",::use_debug_mode);
    this->get_parameter("console_print",::use_console_print);    
    timer_p_ = this->create_wall_timer(PARAM_UPDATE_INTERVAL,std::bind(&WhiskersPublisher::parameters_update, this));
        
    lastLoop = 0;
}

/**
 * Callback function for a timer interrupt in ROS. Publishes at defined intervals (PUBLISH_INTERVAL).
 */
void WhiskersPublisher::timer_callback(void)
{
    if(grid->init)
    {
        return;
    }
    
    // Calculate actual publishing frequency

    if(::use_debug_mode) {
        unsigned long now = ::millis();
        loopFreq = 1000.0 / float(now - lastLoop);
        lastLoop = now;      
        printf("\n>>>> Publishing @ %.2f Hz\n", loopFreq);
    }    
      
    // Acquire data from sensors

    for(uint8_t m=0; m<NUM_MUX; m++)
    {
        // Deselect all channels of the previous multiplexer
        grid->muxDisablePrevious(m);

        for(uint8_t i=0; i<NUM_SENSORS; i++)
        {
            // Switch to next multiplexed port if there is an initialized sensor
            if(!grid->sensors[m][i].initOK)
            {
                continue;
            }
            grid->multiplexers[m].selectChannel(i);
            // Read sensor with selected type of representation
            if(grid->sensors[m][i].read(grid->r) > 0) // If code is -1, it means the sensor has never been initialized (-> ignore now).
                                                      // If code is > 0, it means there was a reading error, but the sensor has been initialized before.
            {
                RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"),"Error reading sensor %d.%d; initializing again.",m,i);
                if(grid->sensors[m][i].initialize(grid->fastMode) == 2)
                {
                    RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"),"Previously initialized sensor %d.%d now disabled.",m,i);
                }
            }
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

    grid->printReadingsToConsole();     
}

/**
 * Callback function for a timer interrupt in ROS. Checks the parameter server and updates parameters
 * at defined intervals (PARAM_UPDATE_INTERVAL).
 */
void WhiskersPublisher::parameters_update(void)
{
    this->get_parameter("debug_mode",::use_debug_mode);
    this->get_parameter("console_print",::use_console_print);
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
 * @param _r The type of representation of sensor data (Cartesian or Spherical).
 * @param _f The type of message representation for displaying sensor data
 *           (PlainText, Compressed or Grid). Note: will print only if use_console_print
 *           is true.
 * @param _fastMode [optional; default: true] If true, the sensors are being
 *                  read in a higher frequency than when set to false. 
 */
SensorGrid::SensorGrid(Representation _r, MessageFormat _f, bool _fastMode)
{
    init = true;    // we are in init mode during setup
    r = _r;
    f = _f;
    fastMode = _fastMode;
    txString = new unsigned char[MAXBUF];
    txIndex = 0; 
}

/**
 * Puts all sensors back into POWERDOWNMODE.
 */
void SensorGrid::powerdown(void)
{
    muxForceDisableAll();
    for(int m=0; m<NUM_MUX; m++)
    {
        muxDisablePrevious(m);
        for(int i=0; i<NUM_SENSORS; i++)
        {
            multiplexers[m].selectChannel(i,true);
            sensors[m][i].sensor.setAccessMode(sensors[m][i].sensor.POWERDOWNMODE);
        }
    }
}    

/**
 * Opens the I2C bus, constructs and initializes the sensors.
 * 
 * @return Success code: 0 if all sensors ok, otherwise the largest error (1 or 2)
 *                       returned during initialization of all sensors.
 *                       1: all sensors initialized, but not all passed the read check;
 *                       2: not all sensors initialized.
 */
int SensorGrid::setup(void)
{
    if(::use_debug_mode) { printf("\n>>>> SETUP\n"); }
    
    int worstError = 0; 
    
    endSignature[0] = '\r';
    endSignature[1] = '\n';
    
    Wire.begin((uint8_t)I2C_BUS_ID);
    
    if(::use_debug_mode) { printf("\n >>> Disabling 8 physical multiplexers\n"); }
    
    muxForceDisableAll();
    
    if(::use_debug_mode) { printf("\n >>> Preparing sensor objects TLV493D for the grid\n"); }
    for(uint8_t m=0; m<NUM_MUX; m++)
    { 
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        {
            sensors[m][i].setGridPosition(m,i);
        }
    }
    
    if(::use_debug_mode) { printf("\n >>> Constructing multiplexers, initializing sensors TLV493D\n"); }
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
            //::delay(50);
            if(::use_debug_mode) { printf("  >> Now initializing sensor %d.%d\n",m,i); }
            int ret = sensors[m][i].initialize(fastMode);            
            if(ret > worstError)
            {
                worstError = ret;
            }
            if(ret == 2)
            {                
                RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"),"Sensor %d.%d initialization failed. Check the connection.",m,i);
            }
            else if(ret == 1)
            {
                RCLCPP_WARN(rclcpp::get_logger("whiskers_interface"),"Sensor %d.%d initialization ok, but read check failed.", m,i);
            }
        }
    }        
    printSetupResult();
    return worstError;
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
    else if(muxNum == 0 && NUM_MUX > 1)
    {
        multiplexers[NUM_MUX-1].disable();
    }
    else
    {
        if(::use_debug_mode) { printf("  >> Skipping disable (only 1 MUX in use).\n"); }
    }    
}

/**
 * Disables each physical multiplexer starting from the first one up to the number specified.
 * The multiplexer objects are not required for this process.
 * @param totalNum [optional; default: 8] The number of multiplexers to disable.
 */
void SensorGrid::muxForceDisableAll(uint8_t totalNum)
{
    for(int m=MUX_STARTADDR; m<MUX_STARTADDR+totalNum; m++)
    {
        if(::use_debug_mode) { printf("  >> Disabling multiplexer at address 0x%02X\n",m); }
        Wire.beginTransmission(m);
        Wire.write((uint8_t)0x00);
        Wire.endTransmission();

//        Wire.requestFrom(m,1);
//        uint8_t sent_data=Wire.read();
//        if (::use_debug_mode) { printf(" Got back: %02X\n",sent_data); }
    }
}

/**
 * Prints the sensor values currently stored all the sensors' field member "data[]"
 * to console. The type of console output (plain text or compressed) depends
 * on the value of "f".
 */
void SensorGrid::printReadingsToConsole(void)
{
    if(!::use_console_print)
    {
        return;
    }
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
#ifdef SEND_POLAR_RADIUS
                if(r == Spherical)
                {        
                    numValues = 2;
                }
#endif
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
                        else if(sensors[m][cell].initOK)  // may be temporarily faulty
                        {
                            len = sprintf(content,"  XXXXXX  ");
                        }
                        else                              // is permanently faulty/not present
                        {
                            len = sprintf(content,"  ------  ");
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

/**
 * Prints the result of the sensor setup routine to the console in the
 * form of a grid.
 */
void SensorGrid::printSetupResult(void)
{
    for(int m=0; m<NUM_MUX; m++)
    {
        for(int i=0; i<NUM_SENSORS; i++)
        {
            if(::use_debug_mode) { printf("  %s  ",sensors[m][i].initOK ? "OK" : "XX"); }
        }
        if(::use_debug_mode) { printf("\n"); }
    }
    if(::use_debug_mode) { printf("\n\n"); }
}


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
    //::debug(">>>> Constructing Multiplexer object with address 0x%02X\n",addr);
    address = addr;
}

/**
 * Disables the multiplexer by writing 0 to its I2C address.
 */
void SensorGrid::Multiplexer::disable(void)
{
    if(::use_debug_mode) { printf("  >> Disabling multiplexer at address 0x%02X\n",address); }
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();

//    Wire.requestFrom(address,1);
//    uint8_t sent_data=Wire.read();
//    if (::use_debug_mode) { printf(" Got back: %02X\n",sent_data); }

}

/**
 * Selects a channel on the multiplexer.
 * 
 * @param ch The selected channel of the multiplexer (0...7).
 * @param init [optional; default: false] If true, the channel selection will
 *             definitely be performed, even if there is only one MUX and one sensor.
 */
void SensorGrid::Multiplexer::selectChannel(uint8_t ch, bool init)
{
    // We have to set a channel definitely during init,
    // and otherwise in case there is more than one sensor
    // or more than one multiplexer.
    if(init == false && NUM_MUX == 1 && NUM_SENSORS == 1) 
    {   
        if(::use_debug_mode) { printf("  >> Skipping channel selection: 0X%02X, ch. %d\n",address,ch); }
        return; 
    }
    if(::use_debug_mode) { printf("  >> Select mux 0X%02X, ch. %d\n",address,ch); }

    uint8_t sent_data=0;

    sent_data = 1 << ch;
//    if (::use_debug_mode) { printf(" Sent to mux: %02X\n",sent_data); }
    Wire.beginTransmission(address);
    Wire.write(sent_data);
    Wire.endTransmission();

//    Wire.requestFrom(address,1);
//    sent_data=Wire.read();
//    if (::use_debug_mode) { printf(" Got back: %02X\n",sent_data); }
//    Wire.endTransmission(); // ???

}

/**
 * Overridden default constructor for class SensorGrid::HallSensor.
 */
SensorGrid::HallSensor::HallSensor()
{
    //::debug(">>>> Constructing HallSensor object using default constructor\n");
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
 * Initializes the sensor of type Tlv493d (see Tlv493d.h).
 * After sensor initialization, the sensors are polled several times to
 * see if initialization was successful.
 * In case of failure during operation, a sensor will be reinitialized here.
 * @param fastMode The mode of reading the sensor.
 * @param reinitialize [optional; default: false] If true, the sensor will be uninitialized
 *                     before reinitialization. This can be useful if a sensor needs to be
 *                     reinitialized during operation (when giving repeated zero-readings).
 * @param r [optional; default: Cartesian] The data representation to use when getting sensor readings.
 * 
 * @return Success code: 0 if ok, 1 if initialization ok, but read check failed, 2 if initialization failed completely.
 */
int SensorGrid::HallSensor::initialize(bool fastMode, Representation r)
{
    const int maxAttempts = 3; // default was 5
    int attemptNum = 0;
    bool checkOK = false;
    initOK = false;    
    
    while(!checkOK && attemptNum < maxAttempts)
    {
        attemptNum++;  
        if(::use_debug_mode) { printf("   > Attempt: %d/%d\n",(attemptNum),maxAttempts); }
        sensor.begin();
//        if(::use_debug_mode) { printf("   > > Survived sensor.begin \n"); }

        if(fastMode)
        {
            bool ret = sensor.setAccessMode(sensor.FASTMODE);
//            if(::use_debug_mode) { printf("   > > Survived sensor.setAccessMode? \n"); }
            if(ret == BUS_ERROR)
            {
                if(::use_debug_mode) { printf("   > Bus error on access mode = FASTMODE\n"); }
                continue;
            }
            else if(ret == BUS_OK)
            {
                initOK = true;
            }
        }
        sensor.disableTemp();  
        if(initOK)
        {
            checkOK = check(r);    
        }          
    }
    if(initOK)
    {
        if(checkOK)
        {
            return 0;
        }
        return 1;
    }
    return 2;
}

/**
 * Checks a sensor after successful initialization. If it returns only
 * zero-readings for MAX_READS_ZEROS times, it is considered badly initialized.
 * 
 * @param r The data representation to use when getting sensor readings.
 * @return Success code: true if sensor returned useful data, otherwise false.
 */
bool SensorGrid::HallSensor::check(Representation r)
{
    bool ret = false;
    uint8_t readNum = 0;
    if(::use_debug_mode) { printf("  >> Checking sensor %d.%d\n",pos[0],pos[1]); }
    while(readNum < MAX_READS_ZEROS)
    {                    
        if(::use_debug_mode) { printf("   > Reading data (%d/%d)\n",(readNum+1),MAX_READS_ZEROS); }
        if(read(r) != 0)
        {
            if(::use_debug_mode) { printf("  << Reading failed\n"); }
            break;
        }
        readNum++;    
        if(data[0] != 0 || data[1] != 0 || data[2] != 0)
        {
            ret = true;
            if(::use_debug_mode) { printf("  << Good data\n"); }
            break;
        }            
    }
    if(!ret)
    {
        if(::use_debug_mode) { printf("  << Check failed\n"); }
    }
    return ret;
}

/**
 * Polls the sensor of type Tlv493dMagnetic3DSensor.
 * The returned results from each sensor are stored in the array "data[]".
 * The returned values may represent the magnet's position in Cartesian
 * coordinates or in Polar coordinates, depending on the value of "r".
 * 
 * @param r The desired data representation (Cartesian or Spherical).
 * @return Error code: sensor error code in case of sensor error;
 *                     -1 if the sensor has never been initialized,
 *                     1 in case of repeated zero-valued readings,
 *                     0 in case of success.
 */
int SensorGrid::HallSensor::read(Representation r)
{
    if(!initOK)
    {
        if(::use_debug_mode) { printf("   > Not reading sensor %d.%d (not initialized)\n",pos[0],pos[1]); }
        return -1;
    }
    uint16_t dly = sensor.getMeasurementDelay();
    ::delay(dly);
    Tlv493d_Error_t err = sensor.updateData();
    if(err != TLV493D_NO_ERROR)
    {
        if(::use_debug_mode) { printf("   < Error reading sensor %d.%d. Error (%d) was: %s\n",pos[0],pos[1],errno,strerror(errno)); }
        hasError = true;
//        i2c_recover_bus((uint8_t)I2C_BUS_ID);
//        Bus recovery is 9x SCL cycles
//        It's done from one of the Adafruit Feather M0s
        return err;
    }

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
        hasError = false;  
    }
    if(numReadZeros >= MAX_READS_ZEROS)
    {
        hasError = true;
        numReadZeros = MAX_READS_ZEROS;        
        return 1;
    }

    // results are according to sensor reference frame if the third value is non-negative (not tested for Spherical)
    if(data[2] < 0)
    { 
        data[2] *= -1;
        data[1] *= -1;
        data[0] *= -1;
    }   
    return 0;
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
