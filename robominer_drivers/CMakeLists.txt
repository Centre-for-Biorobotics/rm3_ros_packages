cmake_minimum_required(VERSION 3.5)
project(robominer_drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(robominer_msgs)
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include) # This makes it possible to find header files in the include folder

add_executable(bno080_imu
  src/BNO080_imu.cpp
  src/arduino_to_olimex/Olimexino.cpp
  src/arduino_to_olimex/Wire.cpp
  src/bno080_lib/src/SparkFun_BNO080.cpp
  #src/BNO080_i2c.cpp
  )
ament_target_dependencies(bno080_imu rclcpp std_msgs sensor_msgs)

add_executable(tlv493d_tca9548a_whiskers
  src/Whiskers_MUX_node.cpp
  src/arduino_to_olimex/Olimexino.cpp
  src/arduino_to_olimex/Wire.cpp
  src/tlv493d_lib/util/RegMask.cpp
  src/tlv493d_lib/util/BusInterface.cpp
  src/tlv493d_lib/Tlv493d.cpp)
ament_target_dependencies(tlv493d_tca9548a_whiskers rclcpp std_msgs robominer_msgs)

# pi wms odometry sensor
add_executable(pi_wms_interface src/pi_wms_node.cpp)
ament_target_dependencies(pi_wms_interface rclcpp std_msgs serial nav_msgs geometry_msgs)

install(TARGETS
  bno080_imu
  tlv493d_tca9548a_whiskers
  pi_wms_interface
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/motor_serial_interface.py
  scripts/temperature_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
