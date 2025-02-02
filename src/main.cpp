#include <QApplication>
#include "manymove_hmi/hmi_gui.hpp"
#include "manymove_hmi/ros2_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

int main(int argc, char *argv[]) {
    // Initialize ROS2.
    rclcpp::init(argc, argv);

    // Initialize the Qt application.
    QApplication app(argc, argv);

    // Create the HMI GUI window.
    HmiGui gui;
    gui.show();

    // Create the ROS2 worker node and pass the pointer to the GUI.
    auto ros2_worker = std::make_shared<Ros2Worker>("hmi_ros2_worker", &gui);

    // Connect GUI button signals to the corresponding service calls in the ROS2 node.
    QObject::connect(&gui, &HmiGui::startExecutionRequested, [ros2_worker]() {
        ros2_worker->callStartExecution();
    });
    QObject::connect(&gui, &HmiGui::stopExecutionRequested, [ros2_worker]() {
        ros2_worker->callStopExecution();
    });
    QObject::connect(&gui, &HmiGui::resetProgramRequested, [ros2_worker]() {
        ros2_worker->callResetProgram();
    });

    // Run the ROS2 event loop in a separate thread.
    std::thread ros2_thread([ros2_worker]() {
        rclcpp::spin(ros2_worker);
    });

    // Run the Qt main loop.
    int ret = app.exec();

    // On exit, shutdown ROS2 and join the ROS2 thread.
    rclcpp::shutdown();
    if (ros2_thread.joinable()) {
        ros2_thread.join();
    }
    return ret;
}
