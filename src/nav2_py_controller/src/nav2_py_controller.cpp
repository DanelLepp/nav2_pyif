#include "nav2_py_controller/nav2_py_controller.hpp"
#include "python_wrappers/py_wrapper.hpp"
#include "python_wrappers/nav_msgs.hpp"
#include "python_wrappers/geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include <dlfcn.h>

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

namespace nav2_py_controller {

PyController::PyController()
{
  Py_Initialize();
  dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
}

PyController::~PyController()
{
  Py_Finalize();
}

void PyController::configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                              std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos) 
{
    
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();
  tf_ = tf;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap = costmapRos->getCostmap();;

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_module", rclcpp::ParameterValue("artificial_potential_field.artificial_potential_field"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.set_plan", rclcpp::ParameterValue("setPath"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.set_speed_limit", rclcpp::ParameterValue("setSpeedLimit"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.compute_velocity_commands", rclcpp::ParameterValue("computeVelocityCommands"));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".python_delegates.update_costmap", rclcpp::ParameterValue(""));

  node->get_parameter(plugin_name_ + ".python_module", python_module_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_plan", set_plan_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_speed_limit", set_speed_limit_);
  node->get_parameter(plugin_name_ + ".python_delegates.compute_velocity_commands", compute_velocity_commands_);
  node->get_parameter(plugin_name_ + ".python_delegates.update_costmap", update_costmap_);

  RCLCPP_INFO(
  logger_,
  "Configuring controller: %s of type "
  "nav2_py_controller::PyController",
  plugin_name_.c_str());

  RCLCPP_INFO(
  logger_,
  "python_module: %s\n"
  "set_plan: %s\n"
  "set_speed_limit: %s\n"
  "compute_velocity_commands: %s\n"
  "update_costmap: %s\n",
  python_module_.c_str(),
  set_plan_.c_str(),
  set_speed_limit_.c_str(),
  compute_velocity_commands_.c_str(),
  update_costmap_.c_str());

}

void PyController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_py_controller::PyController",
    plugin_name_.c_str());
}

void PyController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_py_controller::PyController",
    plugin_name_.c_str());
}

void PyController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "nav2_py_controller::PyController",
    plugin_name_.c_str());
}

void PyController::setPlan(const nav_msgs::msg::Path & path) 
{
    RCLCPP_INFO(logger_, "setPlan");
    globalPath = path;
    PyObject* pyGlobalPath = PyPath_FromPath(path);
    PyObject* arguments = PyTuple_New(1);
    PyTuple_SetItem(arguments, 0, pyGlobalPath);

    PyObject* func_setPath= PyWrapper::GetFunction(python_module_, set_plan_);
    PyObject_CallObject(func_setPath, arguments);
}

void PyController::setSpeedLimit(const double & , const bool & ) 
{
  PyObject* arguments = PyTuple_New(1);
  PyTuple_SetItem(arguments, 0, NULL);

  PyObject* func_getVelocity = PyWrapper::GetFunction(python_module_, set_speed_limit_);
  PyObject_CallObject(func_getVelocity, arguments);
}

nav_msgs::msg::OccupancyGrid PyController::getOccupancyGridMsg()
{
    nav_msgs::msg::OccupancyGrid occupancyGrid = nav_msgs::msg::OccupancyGrid();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    occupancyGrid.info.resolution           = costmap->getResolution();
    occupancyGrid.info.width                = costmap->getSizeInCellsX();
    occupancyGrid.info.height               = costmap->getSizeInCellsY();
    occupancyGrid.info.origin.position.x    = costmap->getOriginX();
    occupancyGrid.info.origin.position.y    = costmap->getOriginY();

    signed char* costArrayPtr = (signed char*) costmap->getCharMap();
    unsigned int costArrayLen = occupancyGrid.info.width * occupancyGrid.info.height;

    std::vector<signed char> costArray(costArrayPtr, costArrayPtr + costArrayLen);

    occupancyGrid.data = costArray;

    return occupancyGrid;
}

geometry_msgs::msg::TwistStamped PyController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist&,
    nav2_core::GoalChecker*) 
{
    RCLCPP_INFO(logger_, "computeVelocityCommands");
    nav_msgs::msg::OccupancyGrid occupancyGrid = PyController::getOccupancyGridMsg();

    PyObject* pyOccupancyGrid = PyOccupancyGrid_FromOccupancyGrid(occupancyGrid);
    PyObject* pyPose = PyPoseStamped_FromPoseStamped(pose);
    // PyObject* pyTwist = PyTwist_FromTwist(twist);

    PyObject* arguments = PyTuple_New(2);
    PyTuple_SetItem(arguments, 0, pyOccupancyGrid);
    PyTuple_SetItem(arguments, 1, pyPose);
    // PyTuple_SetItem(arguments, 2, pyTwist);

    

    PyObject* func_getVelocity = PyWrapper::GetFunction(python_module_, compute_velocity_commands_);
    
    PyObject* pyTwistStamped = PyObject_CallObject(func_getVelocity, arguments);
    

    if (pyTwistStamped == NULL) {
        std::cout << "value == NULL" << std::endl;
    }

    auto twistStamped = PyTwistStamped_AsTwistStamped(pyTwistStamped);

    // Py_XDECREF(pyOccupancyGrid);
    // Py_XDECREF(pyPose);
    // Py_XDECREF(pyGlobalPath);
    // Py_XDECREF(arguments);
    // Py_XDECREF(func_getVelocity);
    // Py_XDECREF(value);

    // geometry_msgs::msg::TwistStamped twistStamped = apf_controller->getVelocity(costmap_msg, pose, globalPath);
    RCLCPP_INFO(
    logger_,
    "TwistStamped: %f %f %f\n",
    twistStamped.twist.linear.x, twistStamped.twist.linear.y, twistStamped.twist.linear.z);
    return twistStamped;
}

} // namespace nav2_py_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_py_controller::PyController, nav2_core::Controller)