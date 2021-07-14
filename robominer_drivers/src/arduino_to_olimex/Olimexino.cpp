/**
 * This cpp file includes common methods from the Arduino environment
 * which are translated to execute in the general C++ environment under Linux.
 */ 

#include "Olimexino.h"



void delay(uint16_t ms) {
	usleep(1000*ms);
}

void print(unsigned char *buf, int size)
{
	for(int c = 0; c < size; c++)
	{
		printf("%c",buf[c]);
	}
}

bool definePin(std::string pinName)
{
    std::string fileName = "/sys/class/gpio/export";
    std::string pinDirFileName = "/sys/class/gpio/gpio"+pinName+"/direction";
    int pinNameLen = pinName.length();
    const char * c_fileName = fileName.c_str();
    const char * c_pinDirFileName = pinDirFileName.c_str();
    
    // Check if file exists
    
    int fd;
    
    if((fd = open(c_pinDirFileName, O_RDONLY)) >= 0) {
        //printf("File exists: %s\n",c_pinDirFileName);
        close(fd);
        return true;
    }
    
    // Export the desired pin by writing to /sys/class/gpio/export

    fd = open(c_fileName, O_WRONLY);
    if (fd < 0) {
        printf("Unable to open /sys/class/gpio/export\n");
        return false;
    }

    if (write(fd, pinName.c_str(), pinNameLen) != pinNameLen) {
        printf("Error writing to /sys/class/gpio/export\n");
        close(fd);
        return false;
    }

    close(fd);
    return true;
}

bool pinMode(int pin, int direction)
{
    return pinMode(std::to_string(pin), direction);
}

bool pinMode(std::string pinName, int direction)
{
    if(!definePin(pinName))
    {
        return false;
    }
    std::string fileName = "/sys/class/gpio/gpio"+pinName+"/direction";
    const char * c_fileName = fileName.c_str();
    
    char dir[3] = "in";
    int dirLen = 2;
    if(direction == OUTPUT)
    {
        dir[0] = 'o';
        dir[1] = 'u';
        dir[2] = 't';
        dirLen = 3;
    }
    int fd = open(c_fileName, O_WRONLY);
    if(fd < 0)
    {
        printf(">>>> Unable to open %s\n",c_fileName);
        return false;
    }
    
    if (write(fd, dir, dirLen) != dirLen)
    {
        printf(">>>> Error writing to %s\n",c_fileName);
        close(fd);
        return false;
    }
    
    close(fd);
    return true;    
}

bool digitalWrite(int pin, int value)
{
    if(value != LOW && value != HIGH)
    {
        return false;
    }
    return digitalWrite(std::to_string(pin), value);
}

bool digitalWrite(std::string pinName, int value)
{
    std::string fileName = "/sys/class/gpio/gpio"+pinName+"/value";    
    std::string strVal = std::to_string(value);
    const char * c_fileName = fileName.c_str();
    const char * c_strVal = strVal.c_str();
    
    int fd = open(c_fileName, O_WRONLY);
    if(fd < 0)
    {
        printf(">>>> Unable to open %s\n",c_fileName);
        return false;
    }
    
    if (write(fd, c_strVal, 1) != 1)
    {
        printf(">>>> Error writing to %s\n",c_fileName);
        close(fd);
        return false;
    }
    
    close(fd);
    return true;    
}

using namespace std::chrono_literals;

unsigned long millis(void)
{
    return std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();    
}
