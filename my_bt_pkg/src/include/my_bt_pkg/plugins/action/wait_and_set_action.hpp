#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_SET_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_SET_ACTION_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "obo_nav_msgs/srv/set_dock_action.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class WaitAndSetAction : public BT::SyncActionNode
{
public:
  WaitAndSetAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

static BT::PortsList providedPorts()
{
  return{
    BT::InputPort<std::string>("topic", "The topic to subscribe to for the action"),
    BT::InputPort<std::string>("output_key", "The output key to set the value in the blackboard")
  };
}

  BT::NodeStatus tick() override;

private:
  void handle_service_request(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Request> request,
    std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Response> response);

  rclcpp::Service<obo_nav_msgs::srv::SetDockAction>::SharedPtr service_;
  rclcpp::Node::SharedPtr node_; // เพิ่มบรรทัดนี้
  std::string dock_action_ = "None";
  std::string topic_;
  std::string output_key_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_SET_ACTION_HPP_
