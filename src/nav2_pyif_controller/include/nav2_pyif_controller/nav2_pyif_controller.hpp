#ifndef NAV2_APF_CONTROLLER_HPP
#define NAV2_APF_CONTROLLER_HPP

#include "nav2_core/controller.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"

// #include "python3.10/Python.h"

namespace nav2_pyif_controller {

class PYIFController : public nav2_core::Controller
{
    public:
        PYIFController();
        ~PYIFController() override;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos) override;


        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * goal_checker) override;

        void setPlan(const nav_msgs::msg::Path & path) override;

        void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    protected:
        nav_msgs::msg::Path globalPath;

        std::string python_module_;
        std::string set_plan_;
        std::string set_speed_limit_;
        std::string compute_velocity_commands_;
        std::string update_costmap_;

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        rclcpp::Logger logger_ {rclcpp::get_logger("PYIFController")};
        rclcpp::Clock::SharedPtr clock_;
        nav2_costmap_2d::Costmap2D* costmap;

        nav_msgs::msg::OccupancyGrid getOccupancyGridMsg();
};

} // namespace nav2_pyif_controller

#endif // NAV2_APF_CONTROLLER_HPP