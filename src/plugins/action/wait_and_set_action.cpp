#include "my_bt_pkg/plugins/action/wait_and_set_action.hpp"

namespace nav2_behavior_tree
{

WaitAndSetAction::WaitAndSetAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  getInput("topic", topic_); // Get the topic input port value
  node_ = rclcpp::Node::make_shared("_"); // Create a node
  service_ = node_->create_service<obo_nav_msgs::srv::SetDockAction>(topic_, std::bind(&WaitAndSetAction::handle_service_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Get the output key from the configuration
  if (!getInput("output_key", output_key_)) {
    throw BT::RuntimeError("output_key not set for WaitAndSetAction");
  }
}

BT::NodeStatus WaitAndSetAction::tick()
{
  if (dock_action_ == "None") {
    return BT::NodeStatus::RUNNING;
  }

  // Set the value in the blackboard
  config().blackboard->set(output_key_, dock_action_);

  if (config().blackboard->get<std::string>(output_key_) == dock_action_) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void WaitAndSetAction::handle_service_request(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Request> request,
    std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Response> response)
{
  (void)request_header; 
  dock_action_ = request->data;
  response->success = true;
  response->message = "Set dock action success";
}

}  // namespace nav2_behavior_tree
