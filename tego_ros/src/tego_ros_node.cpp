// tego_ros/src/tego_ros_node.cpp

#include "tego_ros/tego_ros_messenger.hpp"
#include "tego_sdk/base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cstdint>
#include <string>
#include <chrono>

using namespace tegocore;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto tego = std::make_shared<TegoBase>();
    auto node = std::make_shared<tegocore::TegoRosMessenger>(tego.get(), "tego_ros_node");

    std::string port_name = "can0";
    double pub_rate = 200.0;
    node->GetRosParameter("port_name", port_name);
    node->GetRosParameter("pub_rate", pub_rate);

    RCLCPP_INFO(node->get_logger(), "can_port: %s", port_name.c_str());

    tego->Connect(port_name);
    if (!tego->IsConnected())
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
