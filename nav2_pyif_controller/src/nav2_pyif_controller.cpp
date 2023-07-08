#include "nav2_pyif_controller/nav2_pyif_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_pyif/nav2_pyif_utils.hpp"

namespace nav2_pyif_controller {

PYIFController::PYIFController() {}

PYIFController::~PYIFController() {}

void PYIFController::configure( 
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) 
{
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();
  tf_ = tf;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ = costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_module", rclcpp::ParameterValue("artificial_potential_field.artificial_potential_field"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.set_plan", rclcpp::ParameterValue("setPath"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.set_speed_limit", rclcpp::ParameterValue("setSpeedLimit"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.compute_velocity_commands", rclcpp::ParameterValue("computeVelocityCommands"));

  node->get_parameter(plugin_name_ + ".python_module", python_module_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_plan", set_plan_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_speed_limit", set_speed_limit_);
  node->get_parameter(plugin_name_ + ".python_delegates.compute_velocity_commands", compute_velocity_commands_);

  pyif::PyMap::Init({
    {python_module_, set_plan_},
    {python_module_, set_speed_limit_},
    {python_module_, compute_velocity_commands_}
  });

  RCLCPP_INFO(
  logger_,
  "Configuring controller: %s using:\n"
  "python_module: %s\npython_delegates:\n"
  "set_plan: %s\n"
  "set_speed_limit: %s\n"
  "compute_velocity_commands: %s\n",
  plugin_name_.c_str(),
  python_module_.c_str(),
  set_plan_.c_str(),
  set_speed_limit_.c_str(),
  compute_velocity_commands_.c_str());

}

void PYIFController::activate() 
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_pyif_controller::PYIFController",
    plugin_name_.c_str());
}

void PYIFController::deactivate() 
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_pyif_controller::PYIFController",
    plugin_name_.c_str());
}

void PYIFController::cleanup() 
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "nav2_pyif_controller::PYIFController",
    plugin_name_.c_str());

  pyif::PyMap::DeInit();
}

void PYIFController::setPlan(const nav_msgs::msg::Path & path) 
{
  static PyObject* py_path;
  
  if (py_path == NULL) {
      py_path = pyif::NavMsgs::PyPath_FromPath(path, NULL);
  }
  else
      pyif::NavMsgs::PyPath_FromPath(path, py_path);

  // std::assertm(py_path != NULL, "py_path is NULL");

  static PyObject* py_args;

  if (py_args == NULL) {
      py_args = PyTuple_New(1);
      PyTuple_SetItem(py_args, 0, py_path);
  }

  // std::assertm(py_args != NULL, "py_args is NULL");
  
  static PyObject* py_func;

  if (py_func == NULL) {
      py_func = pyif::PyMap::GetFunction(python_module_, set_plan_);
  }

  // std::assertm(py_func != NULL, "py_func is NULL");
  
  PyObject_CallObject(py_func, py_args);
}

void PYIFController::setSpeedLimit(const double& speed_limit, const bool & is_percentage) 
{
  static PyObject* py_args;
  if (py_args == NULL) {
    py_args = PyTuple_New(2);
    PyTuple_SetItem(py_args, 0, PyFloat_FromDouble(speed_limit));
    PyTuple_SetItem(py_args, 0, PyBool_FromLong(is_percentage));
  }

  static PyObject* py_func;
  if (py_func == NULL)
    py_func = pyif::PyMap::GetFunction(python_module_, set_speed_limit_);

  PyObject_CallObject(py_func, py_args);
}

// TODO: Costmap should not copy the data, but instead use the data pointer
// from the costmap object. This is a workaround for now.
nav_msgs::msg::OccupancyGrid PYIFController::getOccupancyGridMsg() 
{
  nav_msgs::msg::OccupancyGrid occupancy_grid = nav_msgs::msg::OccupancyGrid();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  occupancy_grid.info.resolution           = costmap_->getResolution();
  occupancy_grid.info.width                = costmap_->getSizeInCellsX();
  occupancy_grid.info.height               = costmap_->getSizeInCellsY();
  occupancy_grid.info.origin.position.x    = costmap_->getOriginX();
  occupancy_grid.info.origin.position.y    = costmap_->getOriginY();

  signed char* cost_array_ptr = (signed char*) costmap_->getCharMap();
  unsigned int cost_array_len = occupancy_grid.info.width * occupancy_grid.info.height;

  occupancy_grid.data = std::vector<signed char>(cost_array_ptr, cost_array_ptr + cost_array_len);

  return occupancy_grid;
}

geometry_msgs::msg::TwistStamped PYIFController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& twist,
  nav2_core::GoalChecker*) 
{
  nav_msgs::msg::OccupancyGrid occupancy_grid = PYIFController::getOccupancyGridMsg();

  static PyObject* py_occupancy_grid;

  if (py_occupancy_grid == NULL) {
    py_occupancy_grid = pyif::NavMsgs::PyOccupancyGrid_FromOccupancyGrid(occupancy_grid, NULL);
  }
  else 
    pyif::NavMsgs::PyOccupancyGrid_FromOccupancyGrid(occupancy_grid, py_occupancy_grid);
  
  // std::assertm(py_occupancy_grid != NULL, "py_occupancy_grid is NULL");

  static PyObject* py_pose;

  if (py_pose == NULL) {
    py_pose = pyif::GeoMsgs::PyPoseStamped_FromPoseStamped(pose, NULL);
  }
  else 
    pyif::GeoMsgs::PyPoseStamped_FromPoseStamped(pose, py_pose);

  // std::assertm(py_pose != NULL, "py_pose is NULL");
  static PyObject* py_twist;

  if (py_twist == NULL) {
    py_twist = pyif::GeoMsgs::PyTwist_FromTwist(twist, NULL);
  }
  else 
    pyif::GeoMsgs::PyTwist_FromTwist(twist, py_twist);

  // std::assertm(py_twist != NULL, "py_twist is NULL");
  static PyObject* py_args;

  if (py_args == NULL) {
    py_args = PyTuple_New(3);
    PyTuple_SetItem(py_args, 0, py_occupancy_grid);
    PyTuple_SetItem(py_args, 1, py_pose);
    PyTuple_SetItem(py_args, 2, py_twist);
  }

  static PyObject* py_py_func;

  if (py_py_func == NULL)
    py_py_func = pyif::PyMap::GetFunction(python_module_, compute_velocity_commands_);

  PyObject* py_twistStamped = PyObject_CallObject(py_py_func, py_args);

  auto twistStamped = pyif::GeoMsgs::PyTwistStamped_AsTwistStamped(py_twistStamped);

  return twistStamped;
}

} // namespace nav2_pyif_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_pyif_controller::PYIFController, nav2_core::Controller)