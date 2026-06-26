/**
 * @file  controller_hardware_with_state_node.cpp
 * @brief Combined node: bridge + PSDK state adapter in one PSDK process.
 */
#include <ros/ros.h>

#include "dependences/application.hpp"
#include "indooruav_controller/controller_hardware.hpp"
#include "indooruav_controller/psdk_state_adapter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_hardware_with_state_node");
    ros::NodeHandle node_handle;

    try {
        Application application(argc, argv);
        indooruav_controller::ControllerHardware controller_hardware(node_handle);
        indooruav_controller::PsdkStateAdapter psdk_state_adapter(node_handle);

        ros::AsyncSpinner spinner(3);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL("controller_hardware_with_state_node exception: %s", e.what());
        return 1;
    }

    return 0;
}
