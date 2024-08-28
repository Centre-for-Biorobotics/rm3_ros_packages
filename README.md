<p align="center"><img width=60% src=../assets/media/logo_rm_horiz.png?raw=true></p>


# ROS 2 Packages for ROBOMINER (RM3)
The following is a list of the packages used with the RM3 small-scale ROBOMINERS prototype. 


Requirements:
|Ubuntu 22.04 LTS (Jammy Jellyfish)| ROS 2 Humble Hawksbill |
|:---:|:---:|


## Table of contents

- [robominer_bringup](#robominer_bringup)
- [robominer_drivers](#robominer_drivers)
- [robominer_msgs](#robominer_msgs)
- [robominer_locomotion_control](#robominer_locomotion_control)
- [robominer_state_estimation](#robominer_state_estimation)

---

## robominer_bringup
This is a high-level Python package that contains only launch files. Lower level launch files also exist in other packages.

---

## robominer_drivers
Contains drivers and other software for interfacing devices, such as motor modules, sensors, etc. This is a mixed C++ and Python package.
Currently included interfaces to:

- motor modules,
- BNO080 IMU,
- MCP9808 temperature sensor,
- Whisker sensor array,
- Pacific Inertial Wheel Motion Sensor **(not used)**,
- Pacific Inertial PI-48 IMU **(not used)**

### whisker sensors interface
- *whiskers_launch.py* launches the node under the name 'whiskers_publisher' to interface a total of 64 sensors. Set parameters:
    - *debug_mode*: `True` enables debugging messages on the console
    - *console_print*: `True` enables printing of data from the sensors. The format of the printout can be chosen when constructing the SensorGrid object.

- When running the node using `ros2 run robominer_drivers tlv493d_tca9548a_whiskers`, both parameters are initialized as `true`.

<!-- ### PI WMS (not used)
The C++ node that interfaces the PI WMS device uses the [serial port library](https://github.com/wjwwood/serial). To use it, clone this ROS2 version:

```
# navigate to dev_ws/src/
git clone https://github.com/RoverRobotics-forks/serial-ros2
```

### PI-48 IMU
Python node that interfaces the device using pyserial. Two identical devices appear in the list of serial ports, the second one is used. -->

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
Python package that contains state estimation for RM3, including forward and inverse kinematics and dynamics, data fusion tools, etc.

---
## robominer_locomotion_control
Python package that contains control software for the RM3 locomotion, including trajectory tracking, path-finding (a-star, theta-star), closed-loop pose control, etc.

---
# References
The following publications contain work from this repository:

| Title | accompanying video |
| -- | -- |
| Gkliva, R., Remmas, W., Godon, S., Rebane, J., Ochs, K., Kruusmaa, M., & Ristolainen, A. (2024). [A Multi-Terrain Robot Prototype with Archimedean Screw Actuators: Design, Realisation, Modelling, and Control](https://doi.org/10.1109/ACCESS.2024.3426105). IEEE Access. | [![CfB on YouTube](http://img.youtube.com/vi/etZ-jDItyA0/0.jpg)](https://www.youtube.com/watch?v=etZ-jDItyA0 "A multi-terrain robot prototype with archimedean screw actuators") |
| Kossas, T., Remmas, W., Gkliva, R., Ristolainen, A., & Kruusmaa, M. (2024, May). [Whisker-based tactile navigation algorithm for underground robots](https://doi.org/10.1109/ICRA57147.2024.10610762). In 2024 IEEE International Conference on Robotics and Automation (ICRA) (pp. 13164-13170). IEEE. | [![CfB on YouTube](http://img.youtube.com/vi/-EIxSckOMZ0/0.jpg)](https://www.youtube.com/watch?v=-EIxSckOMZ0 "Whisker-based tactile navigation algorithm for underground robots") |

Find more information in our [webpage](https://taltech.ee/en/biorobotics)
