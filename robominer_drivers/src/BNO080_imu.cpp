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
// #include "tf/tf.h"
// #include <tf2/LinearMath/Quaternion.h>

#include "BNO080_i2c.h"

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
    publisher_ = this->create_publisher<std_msgs::msg::String>("/imu", 10);
//  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Bno080ImuPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
//  auto message = sensor_msgs::msg::Imu();

//  float linAccelX=-0.1, linAccelY=0.0, linAccelZ=0.1;
    float linAccelX = myIMU.getLinAccelX();
    float linAccelY = myIMU.getLinAccelY();
    float linAccelZ = myIMU.getLinAccelZ();
/*
    message.linear_acceleration.x = linAccelX;
    message.linear_acceleration.y = linAccelY;
    message.linear_acceleration.z = linAccelZ;
*/
    float gyroX = myIMU.getGyroX();
    float gyroY = myIMU.getGyroY();
    float gyroZ = myIMU.getGyroZ();

/*    message.angular_velocity.x = gyroX;
    message.angular_velocity.y = gyroY;
    message.angular_velocity.z = gyroZ;
*/

    float i, j, k, real, radAccuracy;
    uint8_t accuracy;
    myIMU.getQuat(i, j, k, real, radAccuracy, accuracy);

//    orientation.setRPY(roll, pitch, yaw);

//    RCLCPP_INFO(this->get_logger(), "Publishing...", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (myIMU.begin() == false)
  {
    printf("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...\n");
    while(1);
  }
  myIMU.enableRotationVector(50); // update every 50 ms */
  rclcpp::spin(std::make_shared<Bno080ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
