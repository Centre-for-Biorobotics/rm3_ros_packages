#ifndef BNO080_IMU_H
#define BNO080_IMU_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "SparkFun_BNO080.h"

//////////// Start of user-defined constants /////////////

#define BNO080_OLIMEX                 // If #define'd, the platform to compile for is Olimex. Note: Only the I2C bus address
                                      // depends on this setting.
#define UPDATE_INTERVAL_LIN_ACC 10   // IMU updates its linear acceleration data at this interval.
#define UPDATE_INTERVAL_ROT 10       // IMU updates its rotation vectors and gyroscope data at this interval.
#define BNO080_PUBLISH_INTERVAL 10ms // The IMU is being polled for data at this interval and the imu message is being published.
                                      // If the publish interval is too fast, there will be duplicate data in subsequent messages,
                                      // if it is too slow, some data will be skipped. Should be equal to shortest update interval.

//////////// End of user-defined constants /////////////

// Set I2C bus address
#ifdef BNO080_OLIMEX
    #define BNO080_I2C_BUS_ID 2
#else
    #define BNO080_I2C_BUS_ID 8
#endif  


/**
 * Class defining a ROS publisher node.
 * The callback function "timer_callback()" executes the main code.
 */
class Bno080ImuPublisher : public rclcpp::Node
{
    public:
        Bno080ImuPublisher(BNO080 * _imu);
        
    private:
        BNO080 * imu;
        void timer_callback(void);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;    
};

#endif
