# ROS 2 Nav 2 Python Interface (ROS 2 Nav 2 PYIF)
Python facilitates rapid prototyping and offers a wide array of libraries that are utilized in motion planning research. 
A powerful tool for testing, benchmarking and deploying motion planners,the Nav 2, is however implemented in C++ and does not support python-based motion planners.

Porting Python code to C++ is often complicated and separating the planner into a different application is cumbersome.  

The Nav 2 PYIF provides tools for non-invasive integration of Python-based motion planners as native Nav 2 C++ plugins. 

[See more info.](https://ims.ut.ee/www-public2/at/2023/bsc/atprog-bakalaureuset55-loti.05.029-danel-leppenen-text-20230520.pdf)

## Requirements
* Ubuntu 22.04.1 LTS
* ROS Humble Distribution
* Nav 2

## Setup
``` bash
cd colcon_ws/src
git clone <todo>
source /opt/ros/humble/setup.bash
cd ..
colcon build
```

# APF Python Example
The APF example demonstrates the functionality of the Nav 2 PYIF. 
The example includes a params.yaml config file to run the example.

## Setup 
``` bash
pip install -e src/python_controller/
```

## Running the example
``` bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=<path_to_nav2_pyif>/src/params.yaml 
```

## Creating your own custom Python controller
### [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

``` bash
source /opt/ros/humble/setup.bash
cd nav2_pyif/src
ros2 pkg create --build-type ament_python <package_name>
```
Add your <python_controller>.py into ```nav2_pyif/src/```<package_name>/<package_name>/```<br/>

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
