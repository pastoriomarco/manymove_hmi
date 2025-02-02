#include "manymove_hmi/ros2_worker.hpp"
#include "manymove_hmi/hmi_gui.hpp"
#include <sstream>
#include <QMetaObject>

using namespace std::chrono_literals;

Ros2Worker::Ros2Worker(const std::string &node_name, HmiGui *gui)
    : Node(node_name), gui_(gui)
{
    // Subscribe to the topic that publishes the JSON status.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "blackboard_status", 10,
        std::bind(&Ros2Worker::statusCallback, this, std::placeholders::_1));

    // Create the three service clients.
    start_client_ = this->create_client<std_srvs::srv::Empty>("start_execution");
    stop_client_ = this->create_client<std_srvs::srv::Empty>("stop_execution");
    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset_program");

    RCLCPP_INFO(this->get_logger(), "Ros2Worker node started.");
}

void Ros2Worker::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // Here we do a simple (naive) parsing of the JSON text.
    // In a production system you might use a JSON library.
    std::string data = msg->data;
    bool execution_resumed = (data.find("\"execution_resumed\": true") != std::string::npos);
    bool stop_execution = (data.find("\"stop_execution\": true") != std::string::npos);
    bool abort_mission = (data.find("\"abort_mission\": true") != std::string::npos);

    // Invoke the GUI slot updateStatus() in a thread‐safe manner.
    QMetaObject::invokeMethod(gui_, "updateStatus", Qt::QueuedConnection,
                              Q_ARG(bool, execution_resumed),
                              Q_ARG(bool, stop_execution),
                              Q_ARG(bool, abort_mission));
}

void Ros2Worker::callStartExecution()
{
    if (!start_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "start_execution service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    start_client_->async_send_request(request);
}

void Ros2Worker::callStopExecution()
{
    if (!stop_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "stop_execution service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_client_->async_send_request(request);
}

void Ros2Worker::callResetProgram()
{
    if (!reset_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "reset_program service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_client_->async_send_request(request);
}
