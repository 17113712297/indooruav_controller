/**
 * @file  controller_simulate.hpp
 * @brief 仿真速度控制器（GPS 定位版）
 *
 * 位置 setpoint 统一使用 mavros_msgs::PositionTarget（setpoint_raw/local），
 * type_mask 中设置 IGNORE_YAW，PX4 不接受任何 yaw 指令，起飞/悬停不转头。
 *
 * 速度控制：OnCmdVel 以原始机体 FLU 坐标系存储速度；
 * OnVelSendTimer 通过 GetYaw() 将机体速度旋转到 ENU 全局坐标系后
 * 以 frame_id="map" 发布给 MAVROS（LOCAL_NED 模式），
 * 确保 cmd_vel 的 +X 始终沿机头方向运动，与全局坐标系无关。
 *
 * PX4 SITL 需设置 COM_RCL_EXCEPT=4（OFFBOARD 模式豁免 RC 丢失检查）。
 */
#pragma once

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>

namespace indooruav_controller {

class ControllerSimulate {
public:
    explicit ControllerSimulate(ros::NodeHandle& node_handle);
    ~ControllerSimulate();

    ControllerSimulate(const ControllerSimulate&)            = delete;
    ControllerSimulate& operator=(const ControllerSimulate&) = delete;

private:
    enum class FlightState { IDLE, TAKING_OFF, FLYING, LANDING };

    void LoadParameters();
    void AdvertiseServiceServers();
    void CreateServiceClients();
    void CreateSubscribersPublishersAndTimers();

    void StateCallback(const mavros_msgs::State::ConstPtr& msg);
    void LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    bool TakeoffCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool LandCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool HoverCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void OnCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
    void OnVelSendTimer(const ros::TimerEvent& event);

    /**
     * 构造位置 setpoint（ENU 坐标输入，MAVROS 内部转 NED）。
     * type_mask 包含 IGNORE_YAW：PX4 保持当前 yaw，起飞/悬停不转头。
     */
    static mavros_msgs::PositionTarget MakePositionTarget(double x_enu,
                                                           double y_enu,
                                                           double z_enu);

    /**
     * 从 ENU 四元数中提取 yaw 角（弧度，绕 +Z 轴，CCW 为正，
     * 零点指向全局 +X 即正东方向）。
     */
    static double GetYaw(const geometry_msgs::Quaternion& q);

    static bool IsZeroVelocity(const geometry_msgs::Twist& vel);

    bool SendModeRequest(const std::string& mode_name);
    bool SendArmingRequest(bool arm);

    ros::NodeHandle& node_handle_;

    std::string takeoff_service_name_;
    std::string land_service_name_;
    std::string hover_service_name_;
    std::string cmd_vel_sub_topic_;
    std::string cmd_vel_pub_topic_;
    std::string setpoint_raw_topic_;      // setpoint_raw/local（PositionTarget）
    std::string mavros_state_topic_;
    std::string local_pose_topic_;
    std::string mavros_arming_service_;
    std::string mavros_set_mode_service_;
    double      vel_send_rate_hz_      = 30.0;
    double      takeoff_altitude_      = 2.5;
    int         warmup_setpoint_count_ = 100;
    double      takeoff_timeout_sec_   = 20.0;
    double      request_retry_sec_     = 1.0;

    ros::ServiceServer takeoff_service_server_;
    ros::ServiceServer land_service_server_;
    ros::ServiceServer hover_service_server_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Subscriber mavros_state_sub_;
    ros::Subscriber local_pose_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher  cmd_vel_pub_;         // TwistStamped → setpoint_velocity/cmd_vel（ENU，frame_id=map）
    ros::Publisher  setpoint_raw_pub_;    // PositionTarget → setpoint_raw/local
    ros::Timer      vel_send_timer_;

    mutable std::mutex        mutex_;
    FlightState                flight_state_           = FlightState::IDLE;
    mavros_msgs::State         current_state_;

    // 当前本地位姿（ENU，由 local_position/pose 持续更新）
    geometry_msgs::PoseStamped current_local_pose_;
    bool                       has_local_pose_         = false;

    // 悬停锚点（速度为零时发位置 setpoint 到此处）
    geometry_msgs::Point       hover_position_;        // ENU
    bool                       hover_pose_set_         = false;

    // latest_vel_ 以机体 FLU 坐标系存储（+x 前, +y 左, +z 上）
    geometry_msgs::Twist       latest_vel_;
    bool                       vel_forwarding_enabled_ = false;
    bool                       prev_vel_zero_          = true;
};

}  // namespace indooruav_controller

