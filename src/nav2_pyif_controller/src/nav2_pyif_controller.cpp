#include "nav2_pyif_controller/nav2_pyif_controller.hpp"
#include "python_wrappers/pyif.hpp"
#include "python_wrappers/nav_msgs.hpp"
#include "python_wrappers/geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include <dlfcn.h>

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

namespace nav2_pyif_controller {

PYIFController::PYIFController()
{
  // Py_Initialize();
  dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
}

PYIFController::~PYIFController()
{
}

void PYIFController::configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
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

  node->get_parameter(plugin_name_ + ".python_module", python_module_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_plan", set_plan_);
  node->get_parameter(plugin_name_ + ".python_delegates.set_speed_limit", set_speed_limit_);
  node->get_parameter(plugin_name_ + ".python_delegates.compute_velocity_commands", compute_velocity_commands_);

  RCLCPP_INFO(
  logger_,
  "Configuring controller: %s of type "
  "nav2_pyif_controller::PYIFController",
  plugin_name_.c_str());

  PYIF::Init({
    {python_module_, set_plan_},
    {python_module_, set_speed_limit_},
    {python_module_, compute_velocity_commands_}
  });

  RCLCPP_INFO(
  logger_,
  "python_module: %s\n"
  "set_plan: %s\n"
  "set_speed_limit: %s\n"
  "compute_velocity_commands: %s\n",
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

  PYIF::DeInit();
}

void PYIFController::setPlan(const nav_msgs::msg::Path & path) 
{
    // RCLCPP_INFO(logger_, "setPlan");
    globalPath = path;

    static PyObject* pyGlobalPath;
    
    if (pyGlobalPath == NULL) {
        std::cout << "pyGlobalPath is NULL" << std::endl;
        pyGlobalPath = PyPath_FromPath(path);
    }
    else
        PyPath_FromPath(path, pyGlobalPath);

    if (pyGlobalPath == NULL) {
        std::cout << "pyGlobalPath is NULL" << std::endl;
    }

    static PyObject* arguments;

    if (arguments == NULL) {
        std::cout << "arguments is NULL" << std::endl;
        arguments = PyTuple_New(1);
        PyTuple_SetItem(arguments, 0, pyGlobalPath);
    }

    static PyObject* func_setPath;

    if (func_setPath == NULL) {
        std::cout << "func_setPath is NULL" << std::endl;
        func_setPath = PYIF::GetFunction(python_module_, set_plan_);
    }

    PyObject_CallObject(func_setPath, arguments);
}

void PYIFController::setSpeedLimit(const double& speed_limit, const bool & is_percentage) 
{
  static PyObject* arguments;
  if (arguments == NULL) {
    arguments = PyTuple_New(2);
    PyTuple_SetItem(arguments, 0, PyFloat_FromDouble(speed_limit));
    PyTuple_SetItem(arguments, 0, PyBool_FromLong(is_percentage));
  }

  static PyObject* func_getVelocity;
  if (func_getVelocity == NULL)
    func_getVelocity = PYIF::GetFunction(python_module_, set_speed_limit_);

  PyObject_CallObject(func_getVelocity, arguments);
}

nav_msgs::msg::OccupancyGrid PYIFController::getOccupancyGridMsg()
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

geometry_msgs::msg::TwistStamped PYIFController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& twist,
    nav2_core::GoalChecker*) 
{
    nav_msgs::msg::OccupancyGrid occupancyGrid = PYIFController::getOccupancyGridMsg();

    static PyObject* pyOccupancyGrid;

    if (pyOccupancyGrid == NULL) {
      pyOccupancyGrid = PyOccupancyGrid_FromOccupancyGrid(occupancyGrid);
    }
    else 
      PyOccupancyGrid_FromOccupancyGrid(occupancyGrid, pyOccupancyGrid);
    
    static PyObject* pyPose;

    if (pyPose == NULL) {
      pyPose = PyPoseStamped_FromPoseStamped(pose);
    }
    else 
      PyPoseStamped_FromPoseStamped(pose, pyPose);

    static PyObject* pyTwist;

    if (pyTwist == NULL) {
      pyTwist = PyTwist_FromTwist(twist);
    }
    else 
      PyTwist_FromTwist(twist, pyTwist);

    static PyObject* pyArgs;

    if (pyArgs == NULL) {
      pyArgs = PyTuple_New(3);
      PyTuple_SetItem(pyArgs, 0, pyOccupancyGrid);
      PyTuple_SetItem(pyArgs, 1, pyPose);
      PyTuple_SetItem(pyArgs, 2, pyTwist);
    }

    static PyObject* pyFunc;

    if (pyFunc == NULL)
      pyFunc = PYIF::GetFunction(python_module_, compute_velocity_commands_);

    PyObject* pyTwistStamped = PyObject_CallObject(pyFunc, pyArgs);

    auto twistStamped = PyTwistStamped_AsTwistStamped(pyTwistStamped);

    return twistStamped;
}

} // namespace nav2_pyif_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_pyif_controller::PYIFController, nav2_core::Controller)