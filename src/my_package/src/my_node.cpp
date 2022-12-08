#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


/* From C-Pyhton API:

Note

For all # variants of formats (s#, y#, etc.), 
the macro PY_SSIZE_T_CLEAN must be defined before including Python.h. 
On Python 3.9 and older, the type of the length argument is Py_ssize_t 
if the PY_SSIZE_T_CLEAN macro is defined, or int otherwise. 

https://docs.python.org/3/c-api/arg.html#arg-parsing
*/
#define PY_SSIZE_T_CLEAN
#include "python3.10/Python.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "py_wrapper_geometry_msgs.hpp"
#include "py_wrapper_nav2_plugins.hpp"

geometry_msgs::msg::PoseStamped AssemblePoseStamped() {
  std_msgs::msg::Header header = std_msgs::msg::Header();
  // header.stamp = 
  header.frame_id = "kanahakkliha";

  geometry_msgs::msg::Point point = geometry_msgs::msg::Point();
  point.x = 10.0;
  point.y = 23.0;
  point.z = 3.0;

  geometry_msgs::msg::Quaternion orientation = geometry_msgs::msg::Quaternion();
  orientation.x = 1.0;
  orientation.y = 2.0;
  orientation.z = 3.0;
  orientation.w = 0.0;

  geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
  pose.orientation = orientation;
  pose.position = point;

  geometry_msgs::msg::PoseStamped poseStamped = geometry_msgs::msg::PoseStamped();
  poseStamped.header = header;
  poseStamped.pose = pose;

  return poseStamped;
}

geometry_msgs::msg::Twist AssembleTwist() {
  geometry_msgs::msg::Vector3 vectorA = geometry_msgs::msg::Vector3();
  vectorA.x = 11.0;
  vectorA.y = 22.0;
  vectorA.z = 33.0;

  geometry_msgs::msg::Vector3 vectorB = geometry_msgs::msg::Vector3();
  vectorB.x = 0.11;
  vectorB.y = 0.22;
  vectorB.z = 0.33;

  geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
  twist.linear = vectorA;
  twist.angular = vectorB;

  return twist;
}

geometry_msgs::msg::TwistStamped ComputeVelocity() {
  geometry_msgs::msg::PoseStamped poseStamped = AssemblePoseStamped();
  geometry_msgs::msg::Twist twist = AssembleTwist();
  PyWrapper::Nav2Plugins plugins = PyWrapper::Nav2Plugins();

  auto start = std::chrono::system_clock::now();
  geometry_msgs::msg::TwistStamped twistStamped = plugins.ComputeVelocity(poseStamped, twist);
  auto end = std::chrono::system_clock::now();

  std::cout << "Wrapper function call time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << '\n';

  return twistStamped;
}

int main() {
  Py_Initialize();
  ComputeVelocity();
  Py_Finalize();
  return 0;
}
// ros2 run my_package my_node
