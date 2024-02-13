#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "waypoints_interfaces/action/waypoint_action.hpp"
#include "math.h"
#include <functional>
#include <memory>
#include <thread>


class WaypointActionServer : public rclcpp::Node
{
public:

    using TortoisebotWaypoints = waypoints_interfaces::action::WaypointAction;
    using GoalHandleTortoisebotWaypoints = rclcpp_action::ServerGoalHandle<TortoisebotWaypoints>;

    explicit WaypointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("tortoisebot_as", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<waypoints_interfaces::action::WaypointAction>(
            this, 
            "tortoisebot_as", 
            std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
            std::bind(&WaypointActionServer::handle_cancel, this, _1), 
            std::bind(&WaypointActionServer::handle_accepted, this, _1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));
    }

private:

    rclcpp_action::Server<TortoisebotWaypoints>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TortoisebotWaypoints::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Goal received");
        (void)uuid;

        // Your goal handling logic here

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTortoisebotWaypoints> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        (void)goal_handle;

        // Your cancellation logic here

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTortoisebotWaypoints> goal_handle)
    {
        using namespace std::placeholders;

        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&WaypointActionServer::execute, this, _1), goal_handle}.detach();
  
        // Your goal acceptance logic here
    }

    void execute(const std::shared_ptr<GoalHandleTortoisebotWaypoints> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Getting robot's current position

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Variables 
    geometry_msgs::msg::Point current_position;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<WaypointActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
