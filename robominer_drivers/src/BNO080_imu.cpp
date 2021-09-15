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
// Initial BNO080 node:
// Jaan Rebane 2021-03-25
// Modified:
// Kilian Ochs 2021-07-15

/**
 * Kilian Ochs, 15.07.2021
 * Centre for Biorobotics, TALTECH, Tallinn
 * Robominers experimental setup
 * 
 * This sketch is used to interface IMU BNO080 on the Adafruit breakout
 * board using a stripped-down Sparkfun library.
 * 
 * Instructs the sensor to acquire linear accelerations, rotation vectors
 * and gyroscope data at defined intervals, then polls the sensor at a
 * defined interval, retrieving the latest data of all sets and publishing
 * messages of type "Imu" under the topic "/imu" in the ROS2 environment.
 * 
 * Define sensor update intervals and polling/publishing frequency in
 * "BNO080_imu.h".
 * 
 * When running on an Ubuntu computer (Desktop/laptop), a USB-to-I2C
 * adapter can be used. Please note the name of the i2c bus using "dmesg"
 * after plugging in the adapter. To define platform-specific bus IDs,
 * see "BNO080_imu.h".
 *  
 * To be compiled for a Linux-based system.
 * This is a ported version of the library originally provided for Arduino
 * (see https://downloads.arduino.cc/libraries/github.com/sparkfun/SparkFun_BNO080_Cortex_Based_IMU-1.1.10.zip).
 *
 * Note: Define all constants in "BNO080_imu.h".
 */

#include "BNO080_imu.h"

uint8_t use_i2c_address = 0x4A;   // The address of the IMU; please set via launch file parameter so it matches the hardware configuration:
                                  // If pin D1 is pulled high, address is 0x4B, otherwise 0x4A.



/*******************************
 * 
 * MAIN METHOD
 * 
 *******************************/
 
 
 
 

using namespace std;

/**
 * Executable entry point.
 * 
 * @return Error code: 0 on clean exit.
 */ 
int main(int argc, char * argv[])
{
    // ROS SETUP
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Initializing...\n");    
    
    // BNO080 OBJECT CONSTRUCTION
    BNO080 imu;
  
    // NODE CONSTRUCTION
    Bno080ImuPublisher *imuPub = new Bno080ImuPublisher(&imu);
    shared_ptr<Bno080ImuPublisher> sharedImuPub(imuPub); // Convert raw pointer to shared pointer
    
    // BNO080 OBJECT INITIALIZATION    
    Wire.begin(BNO080_I2C_BUS_ID);  // Start I²C on chosen bus. This won't have any effect if the bus has already been opened before.
    if (imu.begin(::use_i2c_address,Wire) == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bno080_imu"), "BNO080 not detected at 0x%02X. Check the connection. Shutting down...",::use_i2c_address);
        rclcpp::shutdown();
        return -1;
    }
    imu.enableLinearAccelerometer(UPDATE_INTERVAL_LIN_ACC); 
    RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled linear accelerometer.");
    imu.enableGyroIntegratedRotationVector(UPDATE_INTERVAL_ROT);
    RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled gyro + rotation vector.");
    delay(2);
  
    // KEEP SPINNING   
    RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), ">>>>> Starting to publish. <<<<<");  
    rclcpp::spin(sharedImuPub);
    
    // ON EXIT
    RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Clean exit in progress."); 
    rclcpp::shutdown();
    Wire.end();
    
    return 0;
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
Bno080ImuPublisher::Bno080ImuPublisher(BNO080 * _imu) : rclcpp::Node("bno080_imu")
{
    imu = _imu;
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);    
    timer_ = this->create_wall_timer(
        BNO080_PUBLISH_INTERVAL, std::bind(&Bno080ImuPublisher::timer_callback, this)
    );
    this->declare_parameter<uint8_t>("i2c_address",::use_i2c_address);
    this->get_parameter("i2c_address",::use_i2c_address);
}

/**
 * Callback function for a timer interrupt in ROS. Publishes at defined intervals (BNO080_PUBLISH_INTERVAL).
 */
void Bno080ImuPublisher::timer_callback(void)
{
    auto message = sensor_msgs::msg::Imu();

    if (imu->dataAvailable() == true) {
        float linAccelX, linAccelY, linAccelZ;
        float gyroX, gyroY, gyroZ;
        float qx, qy, qz, qw;
        //uint8_t linAccuracy = 0;
        //uint8_t gyroAccuracy = 0;
        //uint8_t magAccuracy = 0;
        float quatRadianAccuracy = 0;
        uint8_t quatAccuracy = 0;

        //    myIMU.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
        gyroX = imu->getFastGyroX();
        gyroY = imu->getFastGyroY();
        gyroZ = imu->getFastGyroZ();
        message.angular_velocity.x = gyroX;
        message.angular_velocity.y = gyroY;
        message.angular_velocity.z = gyroZ;

        //    myIMU.getLinAccel(linAccelX, linAccelY, linAccelZ, linAccuracy);
        linAccelX = imu->getLinAccelX();
        linAccelY = imu->getLinAccelY();
        linAccelZ = imu->getLinAccelZ();
        message.linear_acceleration.x = linAccelX;
        message.linear_acceleration.y = linAccelY;
        message.linear_acceleration.z = linAccelZ;

        imu->getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
        //    qx = imu->getQuatI();
        //    qy = imu->getQuatJ();
        //    qz = imu->getQuatK();
        //    qw = imu->getQuatReal();

        message.orientation.x = qx;
        message.orientation.y = qy;
        message.orientation.z = qz;
        message.orientation.w = qw;

        message.header.stamp = this->now();

        // printf("%f %f %f %f %f %f %f %f %f %f\n",gyroX,gyroY,gyroZ,linAccelX,linAccelY,linAccelZ,qx,qy,qz,qw);

        publisher_->publish(message);
    }
}


