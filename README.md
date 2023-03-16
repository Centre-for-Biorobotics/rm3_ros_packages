# :warning: **WARNING**: This is the mining feature branch, that differs from the main one, do not merge it to main!
<p align="center"><img width=60% src=../assets/media/logo_rm_horiz.png?raw=true></p>


# ROS 2 Packages for ROBOMINER (RM3)
The following is a list of the packages used with the RM3 small-scale prototype. 


Requirements:
|Ubuntu 20.04 LTS (Focal Fossa)| ROS 2 Foxy Fitzroy |
|:---:|:---:|


## Table of contents

- [robominer_bringup](#robominer_bringup)
- [robominer_drivers](#robominer_drivers)
- [robominer_msgs](#robominer_msgs)
- [robominer_locomotion_control](#robominer_locomotion_control)

---

## robominer_bringup
This is a high-level Python package that contains only launch files. Lower level launch files also exist in other packages.

- *open_loop_steering.launch.py:* launches nodes that interface with all motor modules, a node that handles body velocity commands, and an inverse kinematics node. A desired velocity from a joystick, keyboard, or other source is used to calculate and pass RPM setpoints to all four motor modules.
- *open_loop_steering_w_sensors.launch.py:* all the above, plus interfacing an IMU and a temperature sensor.
- *imu_and_whiskers.launch.py:* launches interface nodes for whisker sensors and a bno080 imu.
- *bag_record.launch.py:* recording a bag file with all topics in the ros2 network. File is saved in the launch location with the start time encoded in the filename.
- *sim_open_loop_steering.launch.py:* **(outdated)** launches open_loop_steering node and teleop_twist_keyboard for vortex studio simulations.
- *sim_open_loop_steering_joy.launch.py:* **(outdated)** launches open_loop_steering node and teleop_twist_joy for gazebo simulations.
---

## robominer_drivers
Contains drivers and other software for interfacing devices, such as motor modules, sensors, etc. This is a mixed C++ and Python package.
Currently included interfaces to:

- motor modules,
- BNO080 IMU,
- MCP9808 temperature sensor,
- Whisker sensor array,
- Pacific Inertial Wheel Motion Sensor **(not used)**,
- Pacific Inertial PI-48 IMU.

### whisker sensors interface
- *whiskers_launch.py* launches the node under the name 'whiskers_publisher' to interface a total of 64 sensors. Set parameters:
    - *debug_mode*: `True` enables debugging messages on the console
    - *console_print*: `True` enables printing of data from the sensors. The format of the printout can be chosen when constructing the SensorGrid object.

- When running the node using `ros2 run robominer_drivers tlv493d_tca9548a_whiskers`, both parameters are initialized as `true`.

### PI WMS (not used)
The C++ node that interfaces the PI WMS device uses the [serial port library](https://github.com/wjwwood/serial). To use it, clone this ROS2 version:

```
# navigate to dev_ws/src/
git clone https://github.com/RoverRobotics-forks/serial-ros2
```

### PI-48 IMU
Python node that interfaces the device using pyserial. Two identical devices appear in the list of serial ports, the second one is used.

### motors interface
The nodes that interface the arduinos in the motor modules identify them by their FTDI's serial number. The launch file creates parameters containing this information for each node. The current configuration is:

| Position | Serial Number |
| --- | --- |
| Front Right | AB0LB3F2 |
| Rear Right | AB0LB3F4 |
| Rear Left | AB0LB3F3 |
| Front Left | AQ00PUPC |

---

### robominer_msgs
C++ package that contains all custom interfaces (messages and actions) of the repository. Needs to be built first for other packages to be able to find the interfaces.
```
# navigate to dev_ws/ and build custom messages
colcon build --packages-select robominer_msgs
```
---

## robominer_state_estimation
Python package that contains state estimation for RM3, including forward and inverse kinematics, a forward dynamic model, data fusion tools, etc.

## robominer_locomotion_control
Python package that contains control software for the RM3 locomotion, including trajectory tracking, closed-loop pose control, etc.

