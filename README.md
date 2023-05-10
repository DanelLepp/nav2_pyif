# ROS 2 Nav 2 and Python Integration

## Setup
* Ubuntu 22.04.1 LTS for running ROS Humble Distribution
* Clone the respository into to your ros2_ws and build it. <br/>
```source /opt/ros/humble/setup.bash```<br/>
```cd nav2_pyif```<br/>
```colcon build```<br/>

## Running the example
```
pip install -e src/python_controller/
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=<path_to_nav2_pyif>/src/params.yaml 
```

## Creating your own custom Python controller
### [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
```source /opt/ros/humble/setup.bash```<br/>
```cd nav2_pyif/src```<br/>
```ros2 pkg create --build-type ament_python <package_name>```<br/>
Add your <python_controller>.py into ```nav2_pyif/src/<package_name>/<package_name>/```<br/>

### Add mandatory overrides for controller plugin in <python_controller>.py
```
from geometry_msgs.msg import TwistStamped

def <compute_velocity_commands_override>(occupancy_grid, pose, twist):
    cmd_vel = TwistStamped()
    # integration with your own code here
    return cmd_vel

def <set_plan_override>(global_plan):
    # integration with your own code here
    return

def <set_speed_limit_override>(speed_limit, is_percentage):
    # integration with your own code here
    return
```

### Build the package (NB! changes is <python_controller>.py take effect only after building)
Go back into the root folder.<br/>
```cd ..```<br/>
```colcon build```<br/>

### Install the local package
```pip install -e src/<package_name>/```<br/>

### [Add the package to src/params.yaml](https://github.com/DanelLepp/ros_cppy/blob/main/src/params.yaml)
```
python_module: "<package_name>.<python_controller>" # NB! ".py" is not added
python_delegates:
  set_plan: "<set_plan_override>"
  set_speed_limit: "<set_speed_limit_override>"
  compute_velocity_commands: "<compute_velocity_commands_override>"
```
