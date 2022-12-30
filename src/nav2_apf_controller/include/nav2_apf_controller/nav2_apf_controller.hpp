// #ifndef NAV2_APF_CONTROLLER_HPP
// #define NAV2_APF_CONTROLLER_HPP

// #include "nav2_core/controller.hpp"
// // #include "rclcpp/rclcpp.hpp"
// // #include "pluginlib/class_loader.hpp"
// // #include "pluginlib/class_list_macros.hpp"

// class ArtificialPotentialFieldController : public nav2_core::Controller
// {
//     public:
//     ArtificialPotentialFieldController() = default;
//     ~ArtificialPotentialFieldController() override = default;

//     void configure(
//         const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//         std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
//         const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);


//     void cleanup() override;
//     void activate() override;
//     void deactivate() override;

//     geometry_msgs::msg::TwistStamped computeVelocityCommands(
//         const geometry_msgs::msg::PoseStamped & pose,
//         const geometry_msgs::msg::Twist & velocity) override;

//     void setPlan(const nav_msgs::msg::Path & path) override;

//     protected:
//         nav_msgs::msg::Path global_plan_;
// };

// #endif // NAV2_APF_CONTROLLER_HPP