#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<TickCount>("TickCount");

    auto tree = factory.createTreeFromFile("/home/obodroid/Desktop/Bt_test/src/my_bt_pkg/trees/my_tree.xml");

    // Monitor the execution using a ZMQ Publisher
    BT::PublisherZMQ publisher_zmq(tree);

    while (rclcpp::ok())
    {
        auto status = tree.rootNode()->executeTick();
        switch(status)
        {
            case BT::NodeStatus::RUNNING:
                std::cout << "Tree is RUNNING" << std::endl;
                break;
            case BT::NodeStatus::SUCCESS:
                std::cout << "Tree is SUCCESS" << std::endl;
                break;
            case BT::NodeStatus::FAILURE:
                std::cout << "Tree is FAILURE" << std::endl;
                break;
            default:
                break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();

    return 0;
}

