# ROS CPP and Python Integration

## Setup
* Ubuntu 22.04.1 LTS for running ROS Humble Distribution
* Clone the respository into to your ros2_ws and build it. <br/>
```cd ~/ros2_ws```<br/>
```colcon build```<br/>

## Running the example
```
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=<path_to_ros2_ws>/src/params.yaml 
```

## Creating your own custom Python controller
### [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
```source /opt/ros/humble/setup.bash```<br/>
```cd ~/ros2_ws/src```<br/>
```ros2 pkg create --build-type ament_python <package_name>```<br/>
Add your <python_controller>.py into ```~/ros2_ws/src/<package_name>/<package_name>/```<br/>

### Add mandatory overrides for controller plugin in <python_controller>.py
```
from geometry_msgs.msg import TwistStamped

def <compute_velocity_commands_override>(occupancy_grid, pose, twist):
    cmd_vel = TwistStamped()
    return cmd_vel

def <set_plan_override>(global_plan):
    return

def <set_speed_limit_override>(speed_limit, is_percentage):
    return
```

### Build the package (NB! changes is <python_controller>.py take effect only after building)
```cd ~/ros2_ws```<br/>
```colcon build```<br/>

### [Create a wheel package](https://datacadamia.com/lang/python/shipping/wheel)
```cd ~/ros2_ws/src/<package_name>/```<br/>
```python3 setup.py bdist_wheel```<br/>

### Add the package to YAML
```
python_module: "<package_name>.<python_controller>" # NB! ".py" is not added
python_delegates:
  set_plan: "<set_plan_override>"
  set_speed_limit: "<set_speed_limit_override>"
  compute_velocity_commands: "<func_computeVelocityCommands_override>"
```
