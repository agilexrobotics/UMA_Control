// uma_ros/src/uma_ros_node.cpp

#include "uma_ros/uma_ros_messenger.hpp"
#include "uma_sdk/base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cstdint>
#include <string>
#include <chrono>

using namespace umacore;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto uma = std::make_shared<UmaBase>();
    auto node = std::make_shared<umacore::UmaRosMessenger>(uma.get(), "uma_ros_node");

    std::string port_name = "can0";
    double pub_rate = 200.0;
    node->GetRosParameter("port_name", port_name);
    node->GetRosParameter("pub_rate", pub_rate);

    RCLCPP_INFO(node->get_logger(), "can_port: %s", port_name.c_str());

    uma->Connect(port_name);
    if (!uma->IsConnected())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open CAN interface: %s", port_name.c_str());
    }
    node->SetupSubscription();
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    rclcpp::Rate publish_rate(pub_rate > 0.0 ? pub_rate : 200.0);
    while (rclcpp::ok())
    {
        node->PublishStateToROS();
        rclcpp::spin_some(node);
        publish_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
