#ifndef WHISKERS_MUX_H
#define WHISKERS_MUX_H

#include "config.h"
#include "Tlv493d.h"
#include "RegMask.h"
#include "BusInterface2.h"

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"





//////////// Start of user-defined constants /////////////

#define DEBUG                 // If #define'd, debug messages will be printed to the console, otherwise not
//#define UBUNTU              // If #define'd, the platform to compile for is Linux Ubuntu, otherwise Olimex Linux
#define MUX_STARTADDR 0x70    // [0x70] Address of the first multiplexer; the others must be consecutive
#define NUM_MUX 4             // Number of multiplexers
#define NUM_SENSORS 8         // Number of sensors per multiplexer (max. 8)
#define MAXBUF 1000           // Maximum char length of an output message
#define PUBLISH_INTERVAL 40ms // Interval for whisker message publishing
#define CONSOLE_PRINT 1       // If #define'd, sensor readings will be printed to the local console

//////////// End of user-defined constants /////////////




// Set buss address
#ifdef UBUNTU
#define I2C_BUS_ID 8
#else
#define I2C_BUS_ID 1
#endif    

// Set debug() to printf()
#ifdef DEBUG
	#define debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#warning DEBUG mode is on. Debug messages will be printed to the console. Undefine it in ./olimex/config.h if no debug messages should be printed.
#else
	#define debug(fmt, ...) ((void)0)
#endif 

// Representation of sensor data
enum Representation {  
  Cartesian,
  Spherical
};

// Message format for console print (TODO: for Serial)
enum MessageFormat {  
  PlainText,        // PlainText:  Full string, in Cartesian up to 1424 bytes in total for 8x8
  Compressed        // Compressed: Cartesian: 3 values per sensor,                               3 x 2 bytes per sensor, 388 bytes in total for 8x8
                    //             Spherical: 2 values per sensor (if sendPolarRadius is false), 2 x 2 bytes per sensor, 260 bytes in total for 8x8
                    //                        3 values per sensor (if sendPolarRadius is true),  3 x 2 bytes per sensor, 388 bytes in total for 8x8
};

using namespace std;

class SensorGrid
{
    public:      

        SensorGrid(Representation _r, MessageFormat _f, bool _fastMode = true, bool _sendPolarRadius = true);
       
        class Multiplexer
        {
            public:
                Multiplexer(uint8_t _addr);
            
                uint8_t address;                
                void disable(void);
                void selectChannel(uint8_t ch, bool init = false);        
        };
        
        class HallSensor
        {
            public:
                HallSensor(); // the constructor without arguments is explicitly implemented
                
                Tlv493d sensor;
                float data[3];
                bool init;                
                void read(Representation r);                
                void encode(uint8_t index, unsigned char * result);     
            private:
                float radToDeg(float rad);
        };
        
        HallSensor sensors[NUM_MUX][NUM_SENSORS];        // will call to (explicitly implemented) default constructor for all objects 
        std::vector<Multiplexer> multiplexers (NUM_MUX); // this makes it easier to call to the non-default constructor which requires one argument when filling the vector array
        unsigned char * txString;
        bool sendPolarRadius;
        bool fastMode;
        Representation r;
        MessageFormat f;                 
        
        void setup(void);       
        
        void muxDisablePrevious(uint8_t muxNum);
        
        void muxDisableAll(uint8_t totalNum = NUM_MUX);
#ifdef CONSOLE_PRINT                
        void printReadingsToConsole(void);
#endif    

    private:
    
        uint16_t txIndex;   
        bool init;        
        void hallTestAndReinitialize(void);  
};

using namespace std::chrono_literals;

class WhiskersPublisher : public rclcpp::Node
{
    public:
        
        WhiskersPublisher(SensorGrid _grid);
        SensorGrid grid;
        
    private:
        
        void timer_callback(void);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<robominer_msgs::msg::WhiskerArray>::SharedPtr publisher_;
#ifdef DEBUG
        volatile unsigned long lastLoop;
        float loopFreq;         
#endif
};

#endif
