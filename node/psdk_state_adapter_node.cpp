/**
 * @file  psdk_state_adapter_node.cpp
 * @brief Standalone node for reading UAV state from PSDK.
 */
#include <ros/ros.h>

#include "dependences/application.hpp"
#include "indooruav_controller/psdk_state_adapter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "psdk_state_adapter_node");
    ros::NodeHandle node_handle;

    try {
        Application application(argc, argv);
        indooruav_controller::PsdkStateAdapter adapter(node_handle);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL("psdk_state_adapter_node exception: %s", e.what());
        return 1;
    }

    return 0;
}
