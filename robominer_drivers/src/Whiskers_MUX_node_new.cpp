#include "robominer_msgs/msg/whisker_pos_in_grid.hpp"
#include "robominer_msgs/msg/whisker.hpp"
#include "robominer_msgs/msg/whisker_array.hpp"

#include "Whiskers_MUX_node.h"

// Note: Define constants in Whiskers_MUX_node.h.           


/**
 * 
 * ROS-SPECIFIC FUNCTIONALITY
 * 
 */

using namespace std::chrono_literals;










/**
 * 
 * HARDWARE-SPECIFIC FUNCTIONALITY
 * 
 */

using namespace std;


/**
 * SensorGrid class constructor.
 * 
 * @param _r The tzpe of representation of sensor data (Cartesian or Spherical).
 * @param _f The type of message representation for sending data out to
 *           non-ROS interfaces such as Serial (PlainText or Compressed).
 *           [TODO: Serial not yet implemented.]
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
    setup();
    r = _r;
    f = _f;
    sendPolarRadius = _sendPolarRadius;
    fastMode = _fastMode;
    txString = new unsigned char[MAXBUF];
    txIndex = 0;
}

/**
 * Opens the I2C bus, constructs and initializes the sensors.
 */
void SensorGrid::setup(void)
{
    debug(">>>> RESET\n");      
    
    debug(">>>> Wire.begin()\n");
    Wire.begin((uint8_t)I2C_BUS_ID);
    
    debug(">>>> Constructing multiplexers, initializing sensors TLV493D\n");
    for(uint8_t m=0; m<NUM_MUX; m++)
    { 
        multiplexers.push_back(Multiplexer(MUX_STARTADDR+m));
        muxDisablePrevious(m);
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        {
            multiplexers[m].selectChannel(i);
            if(sensors[m][i].initialize(fastMode) == BUS_ERROR)
            {
                debug(">>>> Sensor %d.%d not detected at default I2C address. Check the connection.\n",m,i);
            }
        }
    }        
    
    hallTestAndReinitialize(); // This is some dirty hack to avoid "badly initialized" (?) sensors in Linux
    init = false;
    
#ifdef DEBUG
    debug(">>>> Setup ended. Waiting for some seconds...\n");
    delay(5000);
#endif
}

/**
 * After sensor initialization, the sensors are polled several times to
 * see if initialization was successful.
 * In case of failure, a sensor will be reinitialized here.
 */
void SensorGrid::hallTestAndReinitialize(void)
{
    for(uint8_t m=0; m<NUM_MUX; m++)
    {
        // Deselect all channels of the previous multiplexer    
        muxDisablePrevious(m);
        
        for(uint8_t i=0; i<NUM_SENSORS; i++)
        { 
            // Switch to next multiplexed port              
            multiplexers[m].selectChannel(i); 

            debug(">>>> Checking sensor %d.%d\n",m,i);

            bool ok = false;  
            int attempts = 0;   

            while(!ok && attempts < 10)
            {
                debug("     Attempt: %d/10\n",(attempts+1));
                int readNum = 0;
                while(readNum < 5)
                {
                    debug("       Reading data (%d/5)\n",(readNum+1));
                    sensors[m][i].read(); 
                    readNum++;    
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
            }  
            else
            {
                debug("     Sensor ok.\n");
            } 
        }    
    }
}

/**
 * Disables the multiplexer in sequence before the currently selected.
 * If the first multiplexer is selected, it disables the last one.
 * Disabling is done by deselecting all channels of the multiplexer.
 * 
 * @param muxNum The index number of the currently selected multiplexer
 */
void SensorGrid::muxDisablePrevious(uint8_t muxNum)
{
    if(m > 0)
    {
        multiplexers[m-1].disable();
    }
    else
    {
        multiplexers[NUM_MUX-1].disable();
    }    
}

/**
 * Assumes the maximum number of multiplexers (8) being installed and
 * disables each of them starting from address MUX_STARTADDR.
 */
void SensorGrid::muxDisableAll(void)
{
    for(int m=0; m<8; m++)
    {
        multiplexers[m].disable();
    }
}

/**
 * Prints the sensor values currently stored all the sensors' field member "data[]"
 * to console. The type of console output (plain text or compressed) depends
 * on the value of "f".
 */
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
                    sensor[m][i].encode(j, encoded);
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
}

/**
 * SensorGrid::Multiplexer class constructor.
 * 
 * @param addr The multiplexer's I2C address.
 */
SensorGrid::Multiplexer::Multiplexer(uint8_t addr)
{
    address = addr;
}

/**
 * Disables the multiplexer by writing 0 to its I2C address.
 */
void SensorGrid::Multiplexer disable(void)
{
    Wire.beginTransmission(address);
    Wire.write(0);
    Wire.endTransmission();
}

/**
 * Selects a channel on the multiplexer.
 * 
 * @param ch The selected channel of the multiplexer (0...7).
 */
void SensorGrid::Multiplexer selectChannel(uint8_t ch)
{
    if(init == false && NUM_MUX == 1 && NUM_SENSORS == 1) // We have to set a channel definitely during init, otherwise in case if there is more than one sensor or more than one multiplexer
    {   
        return; 
    }
    Wire.beginTransmission(address);
    Wire.write(1 << (i+1));
    Wire.endTransmission();  
}

/**
 * Overridden default constructor for class SensorGrid::HallSensor.
 */
void SensorGrid::HallSensor::HallSensor()
{
    for(int i=0; i<3; i++)
    {
        data[i] = 0.0;
    }
}

/**
 * Initializes the sensor of type Tlv493dMagnetic3DSensor.
 * 
 * @param fastMode If true, sets the access mode of the sensor to FASTMODE.
 * @return Error flag: true in case of success, otherwise (in case of bus error) false.
 */
bool SensorGrid::HallSensor::initialize(bool fastMode)
{
    bool ret = false;
    sensor.begin();
    if(fastMode)
    {
        ret = sensor.setAccessMode(sensor.FASTMODE);
        if(ret == BUS_ERROR)
        {
            debug(">>>> Bus error on access mode = FASTMODE");
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
 * @param r The desired data representation (Cartesian or Spherical).
 */
void SensorGrid::HallSensor::read(Representation r)
{
    int dly = sensor.getMeasurementDelay();
    delay(dly);
    Tlv493d_Error_t err = sensor.updateData();
    if(err != TLV493D_NO_ERROR)
    {
        return;
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

    // results are according to sensor reference frame if the third value is non-negative (not tested for Spherical)
    if(data[2] < 0)
    { 
        data[2] *= -1;
        data[1] *= -1;
        data[0] *= -1;
    }   
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
    // multiplying the float by 100, then dropping the last 16 bits  
    int16_t s = data[index] * multiplier;    
    result[1] = s >> 8;
    result[0] = s & 0x00ff;     
}
