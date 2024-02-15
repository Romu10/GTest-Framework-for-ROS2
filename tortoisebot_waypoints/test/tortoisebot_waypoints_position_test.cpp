#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "waypoints_interfaces/action/waypoint_action.hpp"

#include "gtest/gtest.h"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionClient : public ::testing::Test {
public:

    using TortoisebotWaypoints = waypoints_interfaces::action::WaypointAction;
    using GoalHandleTortoisebotWaypoints = rclcpp_action::ServerGoalHandle<TortoisebotWaypoints>;

    explicit WaypointActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    {
        
        // Initialize the ROS node 
        node = rclcpp::Node::make_shared("tortoisebot_ac", node_options);
        
        // Create the action client
        action_client_ = rclcpp_action::create_client<waypoints_interfaces::action::WaypointAction>(
            node->get_node_base_interface(),
            node->get_node_graph_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            "tortoisebot_as");

        // Define timer
        timer_ = node->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WaypointActionClient::send_goal, this)

        );


    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal(){
        using namespace std::placeholders;
        this->timer_->cancel();
        this->goal_done_ = false; 

        if (!this->action_client_){
            RCLCPP_ERROR(node->get_logger(), "Action client not initialized");
        }

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))){
            RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        // Define goal message
        auto goal_msg = TortoisebotWaypoints::Goal();
        goal_msg.position.x = 0.1;
        goal_msg.position.y = 0.1;

        RCLCPP_INFO(node->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<TortoisebotWaypoints>::SendGoalOptions();

    

    };

protected: 
    std::shared_ptr<rclcpp::Node> node;
    rclcpp_action::Client<waypoints_interfaces::action::WaypointAction>::SharedPtr action_client_;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(const GoalHandleTortoisebotWaypoints & goal_handle){
    

    }

    void feedback_callback(
        GoalHandleTortoisebotWaypoints,
        const std::shared_ptr<const TortoisebotWaypoints::Feedback> feedback){
        
    }

    void result_callback(const GoalHandleTortoisebotWaypoints & result){
    
    }

};

TEST_F(WaypointActionClient, PositionTest) {
  EXPECT_TRUE(true);
}

// Using ROS2 BASICS PART2 as guide. 
