#include <iostream>

#include "python3.10/Python.h"
#include "python_wrappers/apf_py_wrapper.hpp"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

void ImportTest() {
  ArtificialPotentialField* apfModule = ArtificialPotentialField::GetInstance();

  nav_msgs::msg::OccupancyGrid costmap_msg = nav_msgs::msg::OccupancyGrid();
  costmap_msg.info.resolution = 1;
  costmap_msg.info.width = 4;
  costmap_msg.info.height = 4;
  costmap_msg.data = std::vector<signed char>{10, 20, 30, 40, 11, 21, 31, 41, 12, 22, 32, 42, 13, 23, 33, 43};
  geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
  nav_msgs::msg::Path globalPath = nav_msgs::msg::Path();
  geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.pose.position.x = 2;
  goal_pose.pose.position.y = 2;
  globalPath.poses = std::vector<geometry_msgs::msg::PoseStamped>{goal_pose};
  std::cout << "size " <<  globalPath.poses.size() << std::endl;

  geometry_msgs::msg::TwistStamped twistStamped = apfModule->getVelocity(costmap_msg, pose, globalPath);
}

int main() {
  Py_Initialize();
  //ComputeVelocity();
  ImportTest();
  std::cout << "deinit" << std::endl;
  Py_Finalize();
  return 0;
}