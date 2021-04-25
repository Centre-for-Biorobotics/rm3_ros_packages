#ifndef PI_WMS_NODE_H
#define PI_WMS_NODE_H

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;


class PiWMSNode : public rclcpp::Node 
{
public:

    serial::Serial imu_serial_;
    // rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr read_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pi_wms_publisher_;
    PiWMSNode();

private:
    void init_serial();
    void read_sample();
    void pub_callback(std_msgs::msg::String pi_wms_string);


};

void enumerate_ports();



#endif