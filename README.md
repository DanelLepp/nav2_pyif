# ROS CPP and Python Integration

## Setup
* Ubuntu 22.04.1 LTS for running ROS Humble Distribution
* Clone the respository into to your colcon_ws and build it.

## Startup

open a new terminal: <br/>
```source /opt/ros/humble/setup.bash```<br/>
```source install/setup.bash```<br/>
```ros2 run my_package my_node```<br/>

## Using custom Python planner

{Create a Python package}(https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) <br/>
```cd ~/ros2_ws/src```<br/>
```ros2 pkg create --build-type ament_python <package_name>```<br/>
```ros2 pkg create --build-type ament_python --node-name my_node my_package```<br/>
```cd ~/ros2_ws```<br/>
```colcon build```<br/>
