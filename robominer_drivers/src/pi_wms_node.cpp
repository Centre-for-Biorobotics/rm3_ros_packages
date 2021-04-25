#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "serial/serial.h"      // get from https://github.com/RoverRobotics-forks/serial-ros2
#include "pi_wms_node.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;

// public constructor
PiWMSNode::PiWMSNode() 
: Node("pi_wms_publisher")
{
    pi_wms_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_wms_raw", 10);

    // use std::bind to pass the publisher (this) as one of the arguments to the callback
    // pub_timer_ = this->create_wall_timer(100ms, std::bind(&PiWMSNode::pub_callback, this));

    read_timer = this->create_wall_timer(100ms, std::bind(&PiWMSNode::read_sample, this));

    wms_port = scan_ports();

    PiWMSNode::init_serial(wms_port); // setup and connect to serial port
}


// private member functions

std::string PiWMSNode::scan_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;

        // find pi-wms device and get port
        std::string device_ = device.description.c_str();
        std::size_t device_found = device_.find("Prolific");        // find unique device description

        // printf( "(port: %s, device description: %s, device hardware_id: %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );

        if(device_found!=std::string::npos )
        {
            printf("pi-wms port: %s\n", device.port.c_str());

            std::string port_ = device.port.c_str();

            return port_;
        }
        // else
        // {
        //     printf("port not found");
        // }
    }
    return "nan";
}

void PiWMSNode::init_serial(std::string port_)
{
    std::int32_t baud_ = 460800;

    // try connecting to serial port
    try
    {
        imu_serial_.setPort(port_);
        imu_serial_.setBaudrate(baud_);
        serial::Timeout to_ = serial::Timeout(200, 200, 0, 200, 0);
        imu_serial_.setTimeout(to_);

        imu_serial_.open();
    }
    catch(serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port %s ", port_.c_str());
    }
}

void PiWMSNode::read_sample()
{
    std::string data = imu_serial_.readline(100, "\n");

    auto pi_wms_msg = std_msgs::msg::String();
    pi_wms_msg.data = data;
    pub_callback(pi_wms_msg);
}

void PiWMSNode::pub_callback(std_msgs::msg::String pi_wms_string)
{

    pi_wms_publisher_->publish(pi_wms_string);

    // RCLCPP_INFO(this->get_logger(), "%s\n", data);
}





int main(int argc, char * argv[])
{
    // initialize rclcpp
    rclcpp::init(argc, argv);

    // spin node
    rclcpp::spin(std::make_shared<PiWMSNode>());

    rclcpp::shutdown();
    return 0;
}