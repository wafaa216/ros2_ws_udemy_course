#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rob_interfaces/action/count_until.hpp"
#include "time.h"

using CountUntil = rob_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUnitlClient : public rclcpp::Node {
public:
  CountUnitlClient() : Node("count_until_client") {
    //initialize cb group 
    count_until_client_ =
        rclcpp_action::create_client<CountUntil>(this, "count_until");
  }

  void send_goal(int target_number, double period) {
    // wait for server
    count_until_client_->wait_for_action_server();

    // add goal
    auto goal = CountUntil::Goal();
    goal.target_number = target_number;
    goal.period = period;

    // add callbacks
    auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
    options.result_callback =
        std::bind(&CountUnitlClient::goal_result_callback, this, _1);
    options.goal_response_callback =
        std::bind(&CountUnitlClient::goal_response_callback, this, _1);
    options.feedback_callback =
        std::bind(&CountUnitlClient::goal_feedback_callback, this, _1, _2);

    // send goal
    RCLCPP_INFO(this->get_logger(), "Sending a goal");
    count_until_client_->async_send_goal(goal, options);

    // Cance the Goal Test
    // time.sleep(200);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(
            &CountUnitlClient::timer_callback,
            this)); // this means timer_callback will be called every 2 sec
  }

private:
  rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  CountUntilGoalHandle::SharedPtr goal_handle_;

  // this timer to test cancel request only
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Cancel the goal request");
    count_until_client_->async_cancel_goal(goal_handle_);
    timer_->cancel();
  }
  // callback to know if the goal is accepted or rejected
  void
  goal_response_callback(std::shared_future<CountUntilGoalHandle::SharedPtr>
                             future) // in foxy is future in humbe is different
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
  }

  // callback to recieve the result once the goal is done
  void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result) {
    auto status = result.code;
    if (status == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    } else if (status == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(this->get_logger(), "Goal Aborted");
    } else if (status == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Goal Canceled");
    }

    int reached_number = result.result->reached_number;
    RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
  }

  // callback to send feedback while goal is executing
  void goal_feedback_callback(
      const CountUntilGoalHandle::SharedPtr &goal_handle,
      const std::shared_ptr<const CountUntil::Feedback> feedback) {
    (void)goal_handle;
    int number = feedback->current_number;
    RCLCPP_INFO(this->get_logger(), "Got feedback: %d", number);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUnitlClient>();
  node->send_goal(6, 1.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}