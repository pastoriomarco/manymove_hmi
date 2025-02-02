#ifndef ROS2_WORKER_HPP
#define ROS2_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <memory>

// Forward declaration of HmiGui so that we can update the GUI from ROS2 callbacks.
class HmiGui;

class Ros2Worker : public rclcpp::Node {
public:
    /// Construct the node and set up subscriptions and service clients.
    Ros2Worker(const std::string &node_name, HmiGui *gui);

    // Methods to call the three services.
    void callStartExecution();
    void callStopExecution();
    void callResetProgram();

private:
    /// Callback for the blackboard_status topic.
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);

    HmiGui *gui_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
};

#endif // ROS2_WORKER_HPP
