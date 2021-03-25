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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include <tf2/LinearMath/Quaternion.h>

#include "BNO080_i2c.h"

#include <unistd.h>   // for usleep

BNO080 myIMU;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Bno080ImuPublisher : public rclcpp::Node
{
public:
  Bno080ImuPublisher()
  : Node("bno080_imu"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Bno080ImuPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::Imu();

    if (myIMU.dataAvailable() == true) {
      float linAccelX=0.0, linAccelY=0.0, linAccelZ=0.0;
      float gyroX=0.0, gyroY=0.0, gyroZ=0.0;
      float qx=0.0, qy=0.0, qz=0.0, qw=0.0;
      byte linAccuracy = 0;
      byte gyroAccuracy = 0;
      //byte magAccuracy = 0;
      float quatRadianAccuracy = 0;
      byte quatAccuracy = 0;

//    myIMU.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
      gyroX = myIMU.getFastGyroX();
      gyroY = myIMU.getFastGyroY();
      gyroZ = myIMU.getFastGyroZ();
      message.angular_velocity.x = gyroX;
      message.angular_velocity.y = gyroY;
      message.angular_velocity.z = gyroZ;

//    myIMU.getLinAccel(linAccelX, linAccelY, linAccelZ, linAccuracy);
      message.linear_acceleration.x = linAccelX;
      message.linear_acceleration.y = linAccelY;
      message.linear_acceleration.z = linAccelZ;

      myIMU.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
//    qx = myIMU.getQuatI();
//    qy = myIMU.getQuatJ();
//    qz = myIMU.getQuatK();
//    qw = myIMU.getQuatReal();

      message.orientation.x = qx;
      message.orientation.y = qy;
      message.orientation.z = qz;
      message.orientation.w = qw;

      message.header.stamp = this->now();

      count_++;

      // printf("%f %f %f %f %f %f %f %f %f %f\n",gyroX,gyroY,gyroZ,linAccelX,linAccelY,linAccelZ,qx,qy,qz,qw);

      publisher_->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (myIMU.begin() == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bno080_imu"), "BNO080 not detected at default I2C address. Check the connection.\n");
    rclcpp::shutdown();
    return -1;
  }

//  myIMU.enableLinearAccelerometer(50);
//  RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled linear accelerometer");

//  myIMU.enableGyro(50);
//  RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled gyro");

//  myIMU.enableRotationVector(50);
//  RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled rotation vector");
//  usleep(2000);

  myIMU.enableGyroIntegratedRotationVector(50);
  RCLCPP_INFO(rclcpp::get_logger("bno080_imu"), "Enabled gyro + rotation vector");
  RCLCPP_WARN(rclcpp::get_logger("bno080_imu"), "Enabling linear acceleration simulaneously may break the output.");
  usleep(2000);

  rclcpp::spin(std::make_shared<Bno080ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
