# ROS CPP and Python Integration

## Setup
* Ubuntu 22.04.1 LTS for running ROS Humble Distribution
* Clone the respository into to your ros2_ws and build it. <br/>
```cd ~/ros2_ws```<br/>
```colcon build```<br/>

## Running the example
```source install/setup.bash```<br/>
```ros2 run my_package my_node```<br/>

## Creating your own custom Python planner
### [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
```cd ~/ros2_ws/src```<br/>
```ros2 pkg create --build-type ament_python <package_name>```<br/>
```ros2 pkg create --build-type ament_python --node-name my_node my_package```<br/>
```cd ~/ros2_ws```<br/>
```colcon build```<br/>

### Add mandatory overrides for controller plugin
```
from geometry_msgs.msg import TwistStamped

def computeVelocityCommands(occupancy_grid, pose, twist):
    cmd_vel = TwistStamped()
    return cmd_vel

def setPath(global_plan):
    return

def setSpeedLimit(speed_limit, is_percentage):
    return
```
### [Create a wheel package](https://datacadamia.com/lang/python/shipping/wheel)
```cd ~/ros2_ws/src/my_package/```<br/>
```python3 setup.py bdist_wheel```<br/>

### Add the package to YAML
```
python_module: "my_package.my_node"
python_delegates:
  set_plan: "setPath"
  set_speed_limit: "setSpeedLimit"
  compute_velocity_commands: "computeVelocityCommands"
```
