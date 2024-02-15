#include "geometry_msgs/msg/detail/point__struct.hpp"
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

// using std::placeholders::_1;
// using namespace std::chrono_literals;

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
    : Node("tortoisebot_ac", node_options), goal_done_(false)
    {
        
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

      double ActionClientTest(float test_data);

private:
    rclcpp_action::Client<TortoisebotWaypoints>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;
    bool goal_success_; 
    geometry_msgs::msg::Point position_;

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
        position_.x = feedback->position.x;
        position_.y = feedback->position.y;
        RCLCPP_INFO(this->get_logger(), "Feedback position received: \nx: %f \ny: %f", position_.x, position_.y);
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

};

double WaypointActionClient::ActionClientTest(float test_data){
    

}

TEST_F(WaypointActionClient, PositionTest) {
  EXPECT_TRUE(true);
}

// Using ROS2 BASICS PART2 as guide. 
