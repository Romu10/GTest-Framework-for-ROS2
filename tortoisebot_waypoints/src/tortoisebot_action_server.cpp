#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
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

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TortoisebotWaypoints::Goal> goal)
    {
        (void)uuid;

        des_pos.x = goal->position.x;
        des_pos.y = goal->position.y;
        RCLCPP_INFO(this->get_logger(), "Goal received: \nx: %f \ny: %f", des_pos.x, des_pos.y);

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

        // Define desired position and erros
        desired_yaw = std::atan2(des_pos.y - cur_pos.y, des_pos.x - cur_pos.x);
        err_pos = std::sqrt(std::pow(des_pos.y - cur_pos.y, 2) + std::pow(des_pos.x - cur_pos.x, 2));
        err_yaw = desired_yaw - current_yaw; 

        // Printing 
        RCLCPP_INFO_ONCE(this->get_logger(), "Goal information: \nDesired yaw: %f \nError yaw: %f \nError position: %f", desired_yaw, err_yaw, err_pos);

    }

    long double calculateYaw(long double qx, long double qy, long double qz, long double qw) {

        long double yaw = std::atan2(2 * ((qw * qz) + (qx * qy)), 1 - 2 * ((qy * qy) + (qz * qz)));
        return yaw;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Getting robot's current position
        cur_pos = msg->pose.pose.position; 

        // Calculate Yaw
        quaternion_x = msg->pose.pose.orientation.x;
        quaternion_y = msg->pose.pose.orientation.y;
        quaternion_z = msg->pose.pose.orientation.z;
        quaternion_w = msg->pose.pose.orientation.w;

        quaternion.x = quaternion_x;
        quaternion.y = quaternion_y;
        quaternion.z = quaternion_z;
        quaternion.w = quaternion_w;

        calculate_yaw = calculateYaw(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
        current_yaw = (calculate_yaw * (180.0 / M_PI));

        if (!msg){
            RCLCPP_ERROR(this->get_logger(), "No data received from topic: /odom");
        }
        else{
            RCLCPP_INFO_ONCE(this->get_logger(), "Data received from topic /odom: \nYaw: %f", current_yaw);
        }

    }


    rclcpp_action::Server<TortoisebotWaypoints>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Pos related 
    geometry_msgs::msg::Point cur_pos;
    geometry_msgs::msg::Point des_pos; 

    // Yaw related
    double calculate_yaw; 
    double current_yaw; 
    double desired_yaw; 

    // Calculate Yaw related
    geometry_msgs::msg::Quaternion quaternion;
    long double quaternion_x, quaternion_y, quaternion_z, quaternion_w = 0.0;  

    // Error related
    double err_pos; 
    double err_yaw;

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
