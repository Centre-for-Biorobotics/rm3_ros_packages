#ifndef BNO080_IMU_H
#define BNO080_IMU_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "SparkFun_BNO080.h"

#define I2C_ADDRESS 0x4B
#define POLL_INTERVAL_LIN_ACC 100
#define POLL_INTERVAL_ROT 100
#define BNO080_PUBLISH_INTERVAL 100



class Bno080ImuPublisher : public rclcpp::Node
{
    public:
        Bno080ImuPublisher(BNO080 * _myIMU);
        
    private:
        BNO080 * myIMU;
    
}

#endif
