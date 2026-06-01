/**
 * @file  controller_hardware_node.cpp
 * @brief ROS 节点入口：初始化 PSDK 平台 → 创建 ControllerHardware → 后台 spin
 */
#include <ros/ros.h>

#include "dependences/application.hpp"
#include "indooruav_controller/controller_hardware.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_hardware_node");
    ros::NodeHandle node_handle;

    try {
        // 必须先于 ControllerHardware 完成 PSDK 平台初始化，否则注册低速通道
        // 接收回调时 PSDK 尚未就绪。
        Application application(argc, argv);

        indooruav_controller::ControllerHardware controller_hardware(node_handle);

        // AsyncSpinner(2)：服务回调 + cmd_vel 订阅/Timer 可并发，
        // 类内部已用互斥锁保证 PSDK SendData 串行化和共享状态一致性。
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL("controller_hardware_node exception: %s", e.what());
        return 1;
    }

    ROS_INFO("controller_hardware_node exiting.");
    return 0;
}
