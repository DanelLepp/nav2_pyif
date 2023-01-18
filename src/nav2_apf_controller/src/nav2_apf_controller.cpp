#include "nav2_apf_controller/nav2_apf_controller.hpp"

#include <dlfcn.h>

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

namespace nav2_apf_controller {

ArtificialPotentialFieldController::ArtificialPotentialFieldController()
{
  Py_Initialize();
  dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
}

ArtificialPotentialFieldController::~ArtificialPotentialFieldController()
{
  Py_Finalize();
}

void ArtificialPotentialFieldController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos) {
    
    plugin_name_ = name;
    node_ = parent;
    auto node = parent.lock();
    tf_ = tf;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    costmap = costmapRos->getCostmap();;

    RCLCPP_INFO(
    logger_,
    "Configuring controller: %s of type "
    "nav2_apf_controller::ArtificialPotentialFieldController",
    plugin_name_.c_str());

    apf_controller = ArtificialPotentialField::GetInstance();

}

void ArtificialPotentialFieldController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_apf_controller::ArtificialPotentialFieldController",
    plugin_name_.c_str());
}

void ArtificialPotentialFieldController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_apf_controller::ArtificialPotentialFieldController",
    plugin_name_.c_str());
}

void ArtificialPotentialFieldController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "nav2_apf_controller::ArtificialPotentialFieldController",
    plugin_name_.c_str());
}

void ArtificialPotentialFieldController::setPlan(const nav_msgs::msg::Path & path) {
    RCLCPP_INFO(logger_, "setPlan");
    globalPath = path;
}

void ArtificialPotentialFieldController::setSpeedLimit(const double & , const bool & )
{
}

nav_msgs::msg::OccupancyGrid ArtificialPotentialFieldController::getOccupancyGridMsg()
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

geometry_msgs::msg::TwistStamped ArtificialPotentialFieldController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist&,
    nav2_core::GoalChecker*) {

    nav_msgs::msg::OccupancyGrid costmap_msg = ArtificialPotentialFieldController::getOccupancyGridMsg();

    geometry_msgs::msg::TwistStamped twistStamped = apf_controller->getVelocity(costmap_msg, pose, globalPath);
    RCLCPP_INFO(
    logger_,
    "TwistStamped: %f %f %f\n",
    twistStamped.twist.linear.x, twistStamped.twist.linear.y, twistStamped.twist.linear.z);
    return twistStamped;
} // namespace nav2_apf_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_apf_controller::ArtificialPotentialFieldController, nav2_core::Controller)