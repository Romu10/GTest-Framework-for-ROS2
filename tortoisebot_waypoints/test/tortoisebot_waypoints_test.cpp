#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "waypoints_interfaces/action/waypoint_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <ostream>

// using std::placeholders::_1;
using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionClient : public rclcpp::Node, public ::testing::Test {
public:

    using TortoisebotWaypoints = waypoints_interfaces::action::WaypointAction;
    using GoalHandleTortoisebotWaypoints = rclcpp_action::ClientGoalHandle<TortoisebotWaypoints>;

    explicit WaypointActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("tortoisebot_ac_position_test", node_options), goal_done_(false)
    {
        using namespace std::placeholders;

        // Create the action client
        this->client_ptr_ = rclcpp_action::create_client<TortoisebotWaypoints>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "tortoisebot_as");

        // Define timer
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WaypointActionClient::send_goal, this)

        );

        // Odom subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointActionClient::odom_callback, this, _1));
    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal(){
        using namespace std::placeholders;
        this->timer_->cancel();
        this->goal_done_ = false; 

        if (!this->client_ptr_){
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        // Define goal message
        auto goal_msg = TortoisebotWaypoints::Goal();
        goal_msg.position.x = 0.1;
        goal_msg.position.y = 0.1;
        goal_pos_.x = goal_msg.position.x; 
        goal_pos_.y = goal_msg.position.y;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<TortoisebotWaypoints>::SendGoalOptions();

        send_goal_options.goal_response_callback = 
            std::bind(&WaypointActionClient::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
            std::bind(&WaypointActionClient::feedback_callback, this, _1, _2);
        
        send_goal_options.result_callback = 
            std::bind(&WaypointActionClient::result_callback, this, _1);
        
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    };

    // Test methods
    double ActionClientTestPosition(){
        // Define var
        double err_pos;
        // Calculate error position
        err_pos = std::sqrt(std::pow(goal_pos_.y - cur_pos_.y, 2) + std::pow(goal_pos_.x - cur_pos_.x, 2));
        RCLCPP_INFO(this->get_logger(), "Error pos calculated");
        return err_pos; 
    }

    double ActionClientTestRotation(){
        // Define var
        double desired_yaw;
        double err_yaw;
        // Calculate error yaw
        desired_yaw = std::atan2(goal_pos_.y - cur_pos_.y, goal_pos_.x - cur_pos_.x);
        RCLCPP_INFO(this->get_logger(), "Error yaw calculated");
        err_yaw = desired_yaw - calculate_yaw; 
        
        return std::abs(err_yaw); 
    }

    void TestBody() override {}

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<TortoisebotWaypoints>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;
    bool goal_success_; 
    geometry_msgs::msg::Point cur_pos_;
    geometry_msgs::msg::Point goal_pos_;

    // Calculate Yaw related
    geometry_msgs::msg::Quaternion quaternion;
    long double quaternion_x, quaternion_y, quaternion_z, quaternion_w = 0.0;  

    // Yaw related
    double calculate_yaw;

    void goal_response_callback(std::shared_future<GoalHandleTortoisebotWaypoints::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    rclcpp_action::ClientGoalHandle<TortoisebotWaypoints>::SharedPtr goal_handle,
    const std::shared_ptr<const TortoisebotWaypoints::Feedback> feedback)
    {
        cur_pos_.x = feedback->position.x;
        cur_pos_.y = feedback->position.y;
        RCLCPP_INFO(this->get_logger(), "Feedback position received: \nx: %f \ny: %f", cur_pos_.x, cur_pos_.y);
        RCLCPP_INFO(this->get_logger(), "Feedback state received: %s", feedback->state.c_str());
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<TortoisebotWaypoints>::WrappedResult& result)
    {
        this->goal_done_ = true;
        
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        if (result.result->success){
            RCLCPP_INFO(this->get_logger(), "Operation: successed");
        }else{
            RCLCPP_INFO(this->get_logger(), "Operation: failed");
        }
    }

    long double calculateYaw(long double qx, long double qy, long double qz, long double qw) {

        long double yaw = std::atan2(2 * ((qw * qz) + (qx * qy)), 1 - 2 * ((qy * qy) + (qz * qz)));
        return yaw;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // Getting robot's current position
        cur_pos_ = msg->pose.pose.position; 

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
    }

};

TEST_F(WaypointActionClient, PositionRotationTest) {

    auto action_client = std::make_shared<WaypointActionClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_client);
    float pos_err;

    // Parameters
    float dist_precision = 0.05;

    while (!action_client->is_goal_done()) {
        executor.spin_some();
    }
    
    pos_err = action_client->ActionClientTestPosition();
    
    EXPECT_LE(pos_err, dist_precision);
}

TEST_F(WaypointActionClient, RotationTest) {
    auto action_client = std::make_shared<WaypointActionClient>();
    float yaw_err;

    // Parameters
    double yaw_precision = 3.1415 / 90; 

    yaw_err = action_client->ActionClientTestRotation();

    EXPECT_LE(yaw_err, yaw_precision);

    rclcpp::shutdown();

}


