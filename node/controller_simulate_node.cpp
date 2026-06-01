/**
 * @file  controller_simulate_node.cpp
 * @brief ROS 节点入口：创建 ControllerSimulate → 后台 spin
 */
#include <ros/ros.h>

#include "indooruav_controller/controller_simulate.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_simulate_node");
    ros::NodeHandle node_handle;

    try {
        indooruav_controller::ControllerSimulate controller_simulate(node_handle);

        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL("controller_simulate_node exception: %s", e.what());
        return 1;
    }

    ROS_INFO("controller_simulate_node exiting.");
    return 0;
}
