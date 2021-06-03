/**
 * Node that handles PI-WMS serial interface.
 * 
 * Uses serial library by William Woodall (https://github.com/wjwwood/serial), currently a variant for ROS2 is used.
 * 
 * @author: Roza Gkliva
 * @contact: roza.gkliva@ttu.ee
 * @date: 2021-04-25
 */

#include <chrono>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "serial/serial.h"      // get from https://github.com/RoverRobotics-forks/serial-ros2
#include "pi_wms_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

using std::vector;
using std::string;

/**
 * Constructor for ROS node. Creates two publishers, a timer for a callback. Calls for 
 * initialization of the serial port.
 */
PiWMSNode::PiWMSNode() 
: Node("pi_wms_publisher")
{
    pi_wms_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_wms_string", 10);

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("pi_wms_odom", 10);

    // use std::bind to pass the publisher (this) as one of the arguments to the callback
    // pub_timer_ = this->create_wall_timer(100ms, std::bind(&PiWMSNode::pub_callback, this));

    read_timer = this->create_wall_timer(100ms, std::bind(&PiWMSNode::read_sample, this));

    wms_port = scan_ports();                        // scan ports to find where the WMS is connected

    if (wms_port.compare("nan") != 0)               // if the port is not 'nan' then try connecting
    {
        PiWMSNode::init_serial(wms_port);           // setup and connect to serial port
    }
    else
    {
        throw std::runtime_error("WMS port not found!");
    }
}


// private member functions

/**
 * Scans all ports for a device with a given hardware ID. Finds port where device is connected.
 * 
 * @return string(port), or 'nan' if port not found.
 */
std::string PiWMSNode::scan_ports()
{
    std::string port_;

    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;

        // find pi-wms device and get port
        std::string device_ = device.hardware_id.c_str();

        // find the device
        std::size_t device_found = device_.find("067b:2303");   // TODO: find a unique device identifier, this only identifies the USB-Serial adapter

        // printf( "(port: %s, device description: %s, device hardware_id: %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );

        if(device_found!=std::string::npos )
        {
            port_ = device.port.c_str();
            RCLCPP_INFO(this->get_logger(), "pi-wms_port: %s\n", port_.c_str());
        }
        else
        {
            port_ = "nan";
        }
    }
    return port_;
}

/**
 * Initializes serial port.
 * 
 * @param string_port_ String denoting the port where WMS is connected.
 * 
 * @throws serial::IOException Thrown if unable to open port
 */
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

/**
 * Reads one line from the serial buffer, until the '\n' eol charachter is found.
 * Calls to parse the string.
 */
void PiWMSNode::read_sample()
{
    std::string data_string = imu_serial_.readline(100, "\n");

    parse_wms_message(data_string);         // parse line of data

    pub_callback(data_string);              // publishes entire message as string
}

/**
 * Publishes a string type message.
 * 
 * @param data_string String containing the entire message to be published.
 */
void PiWMSNode::pub_callback(std::string data_string)
{
    auto pi_wms_msg = std_msgs::msg::String();
    pi_wms_msg.data = data_string;

    pi_wms_publisher_->publish(pi_wms_msg);
}

/**
 * Parses a string into separate variables, using a delimiter.
 * 
 * @param message_string String containing the entire message.
 */
void PiWMSNode::parse_wms_message(std::string message_string)
{
    /* format = <NMEA type>,<seq nmbr>,<X-coord>,<Y-coord>,<velocity>,<heading>,<temperature>,<bat. voltage>,<checksum?> */
    std::size_t string_start = message_string.find("$WSDTA");

    // RCLCPP_INFO(this->get_logger(), "where is the header: %d ", string_start);

    if(string_start!=std::string::npos)
    {
        std::int8_t iter_;
        std::int64_t seq_;
        // std::vector<float> payload_;
        std::string wms_checksum_;
        float x, y, speed, heading, temperature, voltage;

        std::stringstream stream(message_string);

        iter_= 0;

        while(stream.good())
        {
            std::string substr;
            getline(stream, substr, ',');

            switch(iter_) {
                case 1: 
                    seq_ = std::stoi(substr); 
                    break;
                case 2: 
                    x = std::stof(substr); 
                    break;
                case 3: 
                    y = std::stof(substr); 
                    break;
                case 4: 
                    speed = std::stof(substr); 
                    break;
                case 5: 
                    heading = std::stof(substr); 
                    break;
                case 6: 
                    temperature = std::stof(substr); 
                    break;
                case 7: 
                    voltage = std::stof(substr); 
                    break;
                case 8: 
                    wms_checksum_ = substr; 
                    break;
            }
            publish_odometry(x, y, heading, speed);

            iter_++;
        }
        // RCLCPP_INFO(this->get_logger(), "x, y coord: %f, %f ", x, y);

    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "bad string \n");
    }
}

/**
 * Populates and publishes an 'odometry' type message.
 * 
 * @param x Linear translation along the x axis
 * @param y Linear translation along the y axis
 * @param heading Angular position around the z (vertical) axis
 * @param speed {TODO: not sure what speed this is yet}
 */
void PiWMSNode::publish_odometry(float x, float y, float heading, float speed)
{
    auto odom_msg = nav_msgs::msg::Odometry();

    // convert euler to quaternion
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, heading);
    // RCLCPP_INFO(this->get_logger(), quat_tf.);

    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    

    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link"; // change these to whatever makes sense

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;

    odom_msg.pose.pose.orientation = quat_msg;

    odom_publisher_->publish(odom_msg);

}



/**
 * Main entry point for the node.
 */
int main(int argc, char * argv[])
{
    // initialize rclcpp
    rclcpp::init(argc, argv);

    // spin node
    rclcpp::spin(std::make_shared<PiWMSNode>());

    rclcpp::shutdown();
    return 0;
}