#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

class WaitAndSetAction : public BT::CoroActionNode
{
public:
  WaitAndSetAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  std::string getDockAction() const { return dock_action_; } // Getter method added

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("topic"),
             BT::InputPort<std::string>("output_key") };
  }
  void printDockAction() const;

private:
  void handle_service_request(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Request> request,
    std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Response> response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<obo_nav_msgs::srv::SetDockAction>::SharedPtr service_;
  std::string topic_;
  std::string dock_action_ = "None";
  std::string output_key_;
};
void WaitAndSetAction::printDockAction() const
{
  std::cout << "Dock action: " << dock_action_ << std::endl;
}

WaitAndSetAction::WaitAndSetAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  getInput("topic", topic_);
  node_ = rclcpp::Node::make_shared("service");
  service_ = node_->create_service<obo_nav_msgs::srv::SetDockAction>(topic_, std::bind(&WaitAndSetAction::handle_service_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  if (!getInput("output_key", output_key_)) {
    throw BT::RuntimeError("output_key not set for WaitAndSetAction");
  }
}

BT::NodeStatus WaitAndSetAction::tick()
{
  if (dock_action_ == "None") {
    return BT::NodeStatus::RUNNING;
  }

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
    
    
  std::cout << "Received service request with data: " << request->data << std::endl;

  dock_action_ = request->data;
  response->success = true;
  response->message = "Set dock action success";
}
class TickCount : public BT::CoroActionNode
{
  public:
    TickCount(const std::string& name, const BT::NodeConfiguration& config)
        : BT::CoroActionNode(name, config), tick_count_(0)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("ticks") };
    }

    BT::NodeStatus tick() override
    {
        int ticks;
        if (!getInput("ticks", ticks))
        {
            throw BT::RuntimeError("missing required input [ticks]: ", 
                                    this->name());
        }

        if (tick_count_ < ticks)
        {
            tick_count_++;
            std::cout << this->name() << " is RUNNING. Tick count: " << tick_count_ << std::endl;
            setStatus(BT::NodeStatus::RUNNING);
            return BT::NodeStatus::RUNNING;
        }
        else if (tick_count_ >= ticks)
        {
            std::cout << this->name() << " is SUCCESS. Tick count: " << tick_count_ << std::endl;
            tick_count_ = 0;
            setStatus(BT::NodeStatus::SUCCESS);
            return BT::NodeStatus::SUCCESS;
        }
        std::cout << this->name() << " is FAILURE. Tick count: " << tick_count_ << std::endl;
        setStatus(BT::NodeStatus::FAILURE);
        return BT::NodeStatus::FAILURE;
    }

  private:
    int tick_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto main_node = rclcpp::Node::make_shared("main_BT");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitAndSetAction>("WaitAndSetAction");
  factory.registerNodeType<TickCount>("TickCount");  

  auto tree = factory.createTreeFromFile("/home/obodroid/Desktop/ros2_playground/src/experiments/ros2_learners_pun/my_bt_pkg/trees/Charge_worker.xml");

  BT::PublisherZMQ publisher_zmq(tree);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(main_node);
  

  // Start spinning the node in a separate thread
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // Execute the tree every 1 second and log the status
  while (rclcpp::ok())
  {
    auto status = tree.rootNode()->executeTick();
    switch(status)
    {
        case BT::NodeStatus::RUNNING:
            RCLCPP_INFO(main_node->get_logger(), "Tree is RUNNING");
            break;
        case BT::NodeStatus::SUCCESS:
            RCLCPP_INFO(main_node->get_logger(), "Tree is SUCCESS");
            break;
        case BT::NodeStatus::FAILURE:
            RCLCPP_INFO(main_node->get_logger(), "Tree is FAILURE");
            break;
        default:
            break;

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // When done, stop the spin thread and join it
  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();

  return 0;
}
