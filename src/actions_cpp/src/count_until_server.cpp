#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rob_interfaces/action/count_until.hpp"

using CountUntil = rob_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node {
public:
  CountUntilServerNode() : Node("count_until_server") {
    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    count_until_server_ = rclcpp_action::create_server<CountUntil>(
        this, "count_until",
        std::bind(&CountUntilServerNode::goal_callback, this, _1, _2),
        std::bind(&CountUntilServerNode::cancel_callback, this, _1),
        std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(), // this is mendatory to pass
                                                 // cb_group to the create
                                                 // server
        cb_group_);
    RCLCPP_INFO(this->get_logger(), "Action Server has been started ");
  };

private:
  rclcpp_action::GoalResponse
  goal_callback(const rclcpp_action::GoalUUID &uuid,
                const std::shared_ptr<const CountUntil::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Recieved a goal ");
    if (goal->target_number < 0.0) {
      RCLCPP_INFO(this->get_logger(), "Rejecting the goal ");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepting the goal ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancel_callback(const std::shared_ptr<CountUntilGoalHandle> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received a cencel request");
    return rclcpp_action::CancelResponse::ACCEPT; // or reject
  }

  void handle_accepted_callback(
      const std::shared_ptr<CountUntilGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing the goal ");
    execute_goal(goal_handle);
  }

  void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle) {
    // Get request grom Goal
    int target_number = goal_handle->get_goal()->target_number;
    int period = goal_handle->get_goal()->period;

    // Execute the action
    int counter = 0;
    auto result = std::make_shared<CountUntil::Result>();
    auto feedback = std::make_shared<CountUntil::Feedback>();
    rclcpp::Rate loop_rate(1.0 / period);
    for (int i = 0; i < target_number; i++) {
      if (goal_handle->is_canceling()) {
        result->reached_number = counter;
        goal_handle->canceled(result);
        return;
      }
      counter++;
      feedback->current_number = counter;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "%d", counter);
      loop_rate.sleep();
    }
    // Set final state and return result
    result->reached_number = counter;
    RCLCPP_INFO(this->get_logger(), "Result: %d", counter);
    goal_handle->succeed(result);
    // goal_handle->abort(result);
  }

  rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUntilServerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
