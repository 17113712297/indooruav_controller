/**
 * @file  controller_simulate.cpp
 * @brief ControllerSimulate 实现（GPS 定位版）
 *
 * 位置 setpoint 全部改用 setpoint_raw/local（PositionTarget），
 * type_mask 设置 IGNORE_YAW，PX4 起飞和悬停时完全不改变 yaw，根治机头转向问题。
 *
 * Bug 修复历史：
 *   v1  移除 RC Override 定时器（rc.channels[2]=1000 与速度环对抗导致晃动）
 *   v2  起飞读当前 yaw 保持机头；移除 OnCmdVel 手动 FLU→FRD 取反
 *   v3  悬停改用位置 setpoint（零速度 setpoint 无位置积分，会漂移晃动）
 *   v4  位置 setpoint 改用 PositionTarget + IGNORE_YAW，彻底解决机头转向问题；
 *       移除 lidar_transfer.py 依赖（已换 GPS 定位）
 *   v5  修复速度控制沿全局坐标轴运动的问题：
 *       OnCmdVel 直接存储机体 FLU 速度；OnVelSendTimer 用当前 yaw 将机体速度
 *       旋转到 ENU 全局坐标系后发布（frame_id="map"，mav_frame=LOCAL_NED），
 *       保证 +vx 始终沿机头方向飞行，与机头朝向无关。
 */
#include "indooruav_controller/controller_simulate.hpp"

#include <cmath>

namespace indooruav_controller {

// =============================================================================
// 构造 / 析构
// =============================================================================

ControllerSimulate::ControllerSimulate(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
    LoadParameters();
    AdvertiseServiceServers();
    CreateServiceClients();
    CreateSubscribersPublishersAndTimers();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_vel_             = geometry_msgs::Twist{};
        vel_forwarding_enabled_ = false;
        hover_pose_set_         = false;
        prev_vel_zero_          = true;
    }

    vel_send_timer_.start();

    ROS_INFO("[ControllerSimulate] initialized (GPS mode, body-frame velocity control).");
    ROS_INFO("[ControllerSimulate] NOTE: ensure PX4 param COM_RCL_EXCEPT=4 is set.");
}

ControllerSimulate::~ControllerSimulate() {
    vel_send_timer_.stop();
}

// =============================================================================
// 初始化
// =============================================================================

void ControllerSimulate::LoadParameters() {
    node_handle_.param<std::string>("/indooruav_controller/simulate/services/takeoff",
                                    takeoff_service_name_, "indooruav_controller/controller_simulate/takeoff");
    node_handle_.param<std::string>("/indooruav_controller/simulate/services/land",
                                    land_service_name_, "indooruav_controller/controller_simulate/land");
    node_handle_.param<std::string>("/indooruav_controller/simulate/services/hover",
                                    hover_service_name_, "indooruav_controller/controller_simulate/hover");

    node_handle_.param<std::string>("/indooruav_controller/simulate/topics/cmd_vel_sub",
                                    cmd_vel_sub_topic_, "indooruav_controller/waypoint_tracker/cmd_vel");
    node_handle_.param<std::string>("/indooruav_controller/simulate/topics/cmd_vel_pub",
                                    cmd_vel_pub_topic_,
                                    "/solo_0/mavros/setpoint_velocity/cmd_vel");
    node_handle_.param<std::string>("/indooruav_controller/simulate/topics/setpoint_raw_pub",
                                    setpoint_raw_topic_,
                                    "/solo_0/mavros/setpoint_raw/local");
    node_handle_.param<std::string>("/indooruav_controller/simulate/topics/mavros_state_sub",
                                    mavros_state_topic_,
                                    "/solo_0/mavros/state");
    node_handle_.param<std::string>("/indooruav_controller/simulate/topics/local_pose_sub",
                                    local_pose_topic_,
                                    "/solo_0/mavros/local_position/pose");

    node_handle_.param<std::string>("/indooruav_controller/simulate/mavros/arming",
                                    mavros_arming_service_, "/solo_0/mavros/cmd/arming");
    node_handle_.param<std::string>("/indooruav_controller/simulate/mavros/set_mode",
                                    mavros_set_mode_service_, "/solo_0/mavros/set_mode");

    node_handle_.param<double>("/indooruav_controller/simulate/parameters/vel_send_rate_hz",
                               vel_send_rate_hz_, 30.0);
    node_handle_.param<double>("/indooruav_controller/simulate/parameters/takeoff_altitude",
                               takeoff_altitude_, 2.5);
    node_handle_.param<int>("/indooruav_controller/simulate/parameters/warmup_setpoint_count",
                            warmup_setpoint_count_, 100);
    node_handle_.param<double>("/indooruav_controller/simulate/parameters/takeoff_timeout_sec",
                               takeoff_timeout_sec_, 20.0);
    node_handle_.param<double>("/indooruav_controller/simulate/parameters/request_retry_sec",
                               request_retry_sec_, 1.0);
}

void ControllerSimulate::AdvertiseServiceServers() {
    takeoff_service_server_ = node_handle_.advertiseService(
        takeoff_service_name_, &ControllerSimulate::TakeoffCallback, this);
    land_service_server_ = node_handle_.advertiseService(
        land_service_name_, &ControllerSimulate::LandCallback, this);
    hover_service_server_ = node_handle_.advertiseService(
        hover_service_name_, &ControllerSimulate::HoverCallback, this);

    ROS_INFO_STREAM("[ControllerSimulate] takeoff:          " << takeoff_service_name_);
    ROS_INFO_STREAM("[ControllerSimulate] land:             " << land_service_name_);
    ROS_INFO_STREAM("[ControllerSimulate] hover:            " << hover_service_name_);
    ROS_INFO_STREAM("[ControllerSimulate] cmd_vel sub:      " << cmd_vel_sub_topic_);
    ROS_INFO_STREAM("[ControllerSimulate] cmd_vel pub:      " << cmd_vel_pub_topic_);
    ROS_INFO_STREAM("[ControllerSimulate] setpoint raw pub: " << setpoint_raw_topic_);
    ROS_INFO_STREAM("[ControllerSimulate] local pose sub:   " << local_pose_topic_);
}

void ControllerSimulate::CreateServiceClients() {
    arming_client_   = node_handle_.serviceClient<mavros_msgs::CommandBool>(mavros_arming_service_);
    set_mode_client_ = node_handle_.serviceClient<mavros_msgs::SetMode>(mavros_set_mode_service_);
}

void ControllerSimulate::CreateSubscribersPublishersAndTimers() {
    mavros_state_sub_ = node_handle_.subscribe(
        mavros_state_topic_, 10, &ControllerSimulate::StateCallback, this);
    local_pose_sub_ = node_handle_.subscribe(
        local_pose_topic_, 10, &ControllerSimulate::LocalPoseCallback, this);
    cmd_vel_sub_ = node_handle_.subscribe<geometry_msgs::Twist>(
        cmd_vel_sub_topic_, 1, &ControllerSimulate::OnCmdVel, this);

    cmd_vel_pub_      = node_handle_.advertise<geometry_msgs::TwistStamped>(cmd_vel_pub_topic_, 1);
    setpoint_raw_pub_ = node_handle_.advertise<mavros_msgs::PositionTarget>(setpoint_raw_topic_, 20);

    vel_send_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / vel_send_rate_hz_),
        &ControllerSimulate::OnVelSendTimer, this,
        /*oneshot=*/false, /*autostart=*/false);
}

// =============================================================================
// 回调
// =============================================================================

void ControllerSimulate::StateCallback(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_ = *msg;
}

void ControllerSimulate::LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_local_pose_ = *msg;
    has_local_pose_     = true;
}

// =============================================================================
// 服务回调
// =============================================================================

bool ControllerSimulate::TakeoffCallback(std_srvs::Empty::Request& /*req*/,
                                         std_srvs::Empty::Response& /*res*/) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flight_state_ == FlightState::TAKING_OFF ||
            flight_state_ == FlightState::FLYING) {
            ROS_WARN("[ControllerSimulate] Takeoff ignored: already taking off or flying.");
            return true;
        }
        flight_state_ = FlightState::TAKING_OFF;
    }

    vel_send_timer_.stop();

    // 等待 EKF 输出有效位置（Gazebo 真值 bridge 也需要几秒收敛）
    ros::Time wait_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - wait_start).toSec() < 10.0) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_local_pose_) break;
        }
        ros::Duration(0.1).sleep();
    }

    // local_position/pose 已经是 ENU (x东 y北 z上)，直接读
    double x0 = 0.0, y0 = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_local_pose_) {
            x0 = current_local_pose_.pose.position.x;
            y0 = current_local_pose_.pose.position.y;
        }
    }
    ROS_INFO("[ControllerSimulate] Takeoff origin: (%.2f, %.2f)", x0, y0);

    ros::Rate rate(vel_send_rate_hz_);

    // 1. 预发 PositionTarget（IGNORE_YAW），满足 PX4 OFFBOARD 预流要求
    //    不命令任何 yaw，PX4 保持当前机头朝向
    ROS_INFO("[ControllerSimulate] Sending %d warmup setpoints (IGNORE_YAW)...",
             warmup_setpoint_count_);
    for (int i = 0; ros::ok() && i < warmup_setpoint_count_; ++i) {
        auto target        = MakePositionTarget(x0, y0, takeoff_altitude_);
        target.header.stamp = ros::Time::now();
        setpoint_raw_pub_.publish(target);
        rate.sleep();
    }

    // 2. 请求 OFFBOARD + arm
    ROS_INFO("[ControllerSimulate] Requesting OFFBOARD + arm...");
    ros::Time last_mode_request;
    ros::Time last_arm_request;
    const ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        mavros_msgs::State state;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = current_state_;
        }

        if (state.mode == "OFFBOARD" && state.armed) break;

        if ((ros::Time::now() - start_time).toSec() > takeoff_timeout_sec_) {
            ROS_ERROR("[ControllerSimulate] Takeoff timed out.");
            std::lock_guard<std::mutex> lock(mutex_);
            flight_state_ = FlightState::IDLE;
            vel_send_timer_.start();
            return true;
        }

        if (state.mode != "OFFBOARD" &&
            (last_mode_request.isZero() ||
             (ros::Time::now() - last_mode_request).toSec() >= request_retry_sec_)) {
            if (SendModeRequest("OFFBOARD"))
                ROS_INFO("[ControllerSimulate] OFFBOARD mode request sent.");
            last_mode_request = ros::Time::now();
        } else if (!state.armed &&
                   (last_arm_request.isZero() ||
                    (ros::Time::now() - last_arm_request).toSec() >= request_retry_sec_)) {
            if (SendArmingRequest(true))
                ROS_INFO("[ControllerSimulate] Arm request sent.");
            last_arm_request = ros::Time::now();
        }

        auto target         = MakePositionTarget(x0, y0, takeoff_altitude_);
        target.header.stamp = ros::Time::now();
        setpoint_raw_pub_.publish(target);
        rate.sleep();
    }

    // 3. 爬升至目标高度
    ROS_INFO("[ControllerSimulate] OFFBOARD+armed, climbing...");
    const ros::Time climb_start    = ros::Time::now();
    const double    climb_duration = takeoff_altitude_ / 0.5;

    while (ros::ok() &&
           (ros::Time::now() - climb_start).toSec() < climb_duration) {
        auto target         = MakePositionTarget(x0, y0, takeoff_altitude_);
        target.header.stamp = ros::Time::now();
        setpoint_raw_pub_.publish(target);
        rate.sleep();
    }

    // 4. 设置悬停锚点，启动定时器
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_vel_             = geometry_msgs::Twist{};
        vel_forwarding_enabled_ = true;
        prev_vel_zero_          = true;
        flight_state_           = FlightState::FLYING;

        if (has_local_pose_) {
            hover_position_ = current_local_pose_.pose.position;
            // 强制使用目标高度（刚完成爬升，高度最准确）
            hover_position_.z = takeoff_altitude_;
        } else {
            hover_position_.x = x0;
            hover_position_.y = y0;
            hover_position_.z = takeoff_altitude_;
        }
        hover_pose_set_ = true;
    }
    vel_send_timer_.start();

    ROS_INFO("[ControllerSimulate] Takeoff complete, hovering at (%.2f, %.2f, %.2f).",
             hover_position_.x, hover_position_.y, hover_position_.z);
    return true;
}

bool ControllerSimulate::LandCallback(std_srvs::Empty::Request& /*req*/,
                                      std_srvs::Empty::Response& /*res*/) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flight_state_ != FlightState::FLYING) {
            ROS_WARN("[ControllerSimulate] Land ignored: not flying.");
            return true;
        }
        flight_state_           = FlightState::LANDING;
        vel_forwarding_enabled_ = false;
        latest_vel_             = geometry_msgs::Twist{};
    }

    if (SendModeRequest("AUTO.LAND"))
        ROS_INFO("[ControllerSimulate] AUTO.LAND mode request sent.");
    else
        ROS_WARN("[ControllerSimulate] AUTO.LAND mode request failed.");
    return true;
}

bool ControllerSimulate::HoverCallback(std_srvs::Empty::Request& /*req*/,
                                       std_srvs::Empty::Response& /*res*/) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flight_state_ != FlightState::FLYING) {
            ROS_WARN("[ControllerSimulate] Hover ignored: not flying.");
            return true;
        }
        vel_forwarding_enabled_ = false;
        latest_vel_             = geometry_msgs::Twist{};
        prev_vel_zero_          = true;

        if (has_local_pose_) {
            hover_position_ = current_local_pose_.pose.position;
            hover_pose_set_ = true;
            ROS_INFO("[ControllerSimulate] Hover: anchoring at (%.2f, %.2f, %.2f).",
                     hover_position_.x, hover_position_.y, hover_position_.z);
        }
    }
    return true;
}

// =============================================================================
// cmd_vel + 定时发布
// =============================================================================

void ControllerSimulate::OnCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool incoming_zero = IsZeroVelocity(*msg);

    // 悬停关门后，非零速度到达时自动重新开门
    if (!vel_forwarding_enabled_ && flight_state_ == FlightState::FLYING && !incoming_zero) {
        vel_forwarding_enabled_ = true;
        ROS_INFO("[ControllerSimulate] Gate re-opened by incoming velocity.");
    }

    if (!vel_forwarding_enabled_) return;

    if (!prev_vel_zero_ && incoming_zero && has_local_pose_) {
        hover_position_ = current_local_pose_.pose.position;
        hover_pose_set_ = true;
        ROS_INFO("[ControllerSimulate] Velocity stopped, anchoring hover at (%.2f, %.2f, %.2f).",
                 hover_position_.x, hover_position_.y, hover_position_.z);
    }

    // 直接存储原始机体 FLU 坐标系速度（+x 前, +y 左, +z 上, angular.z CCW）
    // 坐标系旋转在 OnVelSendTimer 中通过当前 yaw 完成，此处不做手动变换
    latest_vel_ = *msg;
    prev_vel_zero_ = incoming_zero;
}

void ControllerSimulate::OnVelSendTimer(const ros::TimerEvent& /*event*/) {
    geometry_msgs::Twist vel;
    geometry_msgs::Point hover;
    bool                 use_position = false;
    double               current_yaw  = 0.0;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        vel          = latest_vel_;
        hover        = hover_position_;
        use_position = (flight_state_ == FlightState::FLYING)
                       && hover_pose_set_
                       && IsZeroVelocity(vel);
        if (has_local_pose_) {
            current_yaw = GetYaw(current_local_pose_.pose.orientation);
        }
    }

    if (use_position) {
        // 悬停：PositionTarget + IGNORE_YAW，PX4 位置控制环，稳定无漂移，不转头
        auto target         = MakePositionTarget(hover.x, hover.y, hover.z);
        target.header.stamp = ros::Time::now();
        setpoint_raw_pub_.publish(target);
    } else {
        // 飞行：将机体 FLU 速度旋转到 ENU 全局坐标系后发布
        //
        // latest_vel_ 约定：
        //   linear.x  = 前向速度 (m/s，机体 +X，FLU)
        //   linear.y  = 左向速度 (m/s，机体 +Y，FLU)
        //   linear.z  = 上升速度 (m/s，机体 +Z，FLU)
        //   angular.z = 偏航角速度 (rad/s，CCW 为正)
        //
        // ENU 旋转（yaw = ψ，绕 +Z 轴 CCW）：
        //   v_east  =  vx_body * cos(ψ) - vy_body * sin(ψ)
        //   v_north =  vx_body * sin(ψ) + vy_body * cos(ψ)
        //
        // MAVROS setpoint_velocity/cmd_vel 在 mav_frame=LOCAL_NED 时
        // 期望 ENU 输入并自动转换为 NED 发送给 PX4。
        const double cy = std::cos(current_yaw);
        const double sy = std::sin(current_yaw);

        geometry_msgs::TwistStamped vs;
        vs.header.stamp    = ros::Time::now();
        vs.header.frame_id = "map";                                   // ENU 全局坐标系
        vs.twist.linear.x  =  vel.linear.x * cy - vel.linear.y * sy; // ENU East
        vs.twist.linear.y  =  vel.linear.x * sy + vel.linear.y * cy; // ENU North
        vs.twist.linear.z  =  vel.linear.z;                           // ENU Up（直接透传）
        vs.twist.angular.z =  vel.angular.z;                          // 偏航角速度（CCW 保持）
        cmd_vel_pub_.publish(vs);
    }
}

// =============================================================================
// 辅助
// =============================================================================

mavros_msgs::PositionTarget ControllerSimulate::MakePositionTarget(double x_enu,
                                                                    double y_enu,
                                                                    double z_enu) {
    mavros_msgs::PositionTarget t;
    t.header.frame_id  = "map";
    // MAVROS 对 setpoint_raw/local 做 ENU→NED 转换后再发给 PX4
    t.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // 只控制 XYZ 位置；速度、加速度、YAW 全部忽略
    t.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX       |
        mavros_msgs::PositionTarget::IGNORE_VY       |
        mavros_msgs::PositionTarget::IGNORE_VZ       |
        mavros_msgs::PositionTarget::IGNORE_AFX      |
        mavros_msgs::PositionTarget::IGNORE_AFY      |
        mavros_msgs::PositionTarget::IGNORE_AFZ      |
        mavros_msgs::PositionTarget::IGNORE_YAW;     // ← 核心：不命令 yaw，不转头
    t.position.x = x_enu;  // ENU East
    t.position.y = y_enu;  // ENU North
    t.position.z = z_enu;  // ENU Up（正值 = 向上）
    return t;
}

double ControllerSimulate::GetYaw(const geometry_msgs::Quaternion& q) {
    // 从 ENU 四元数提取偏航角（弧度，绕 +Z 轴，CCW 为正）
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool ControllerSimulate::IsZeroVelocity(const geometry_msgs::Twist& vel) {
    constexpr double kEps = 1e-6;
    return std::abs(vel.linear.x)  < kEps &&
           std::abs(vel.linear.y)  < kEps &&
           std::abs(vel.linear.z)  < kEps &&
           std::abs(vel.angular.z) < kEps;
}

bool ControllerSimulate::SendModeRequest(const std::string& mode_name) {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode_name;
    if (!set_mode_client_.call(srv)) {
        ROS_WARN("[ControllerSimulate] set_mode service call failed.");
        return false;
    }
    return srv.response.mode_sent;
}

bool ControllerSimulate::SendArmingRequest(bool arm) {
    mavros_msgs::CommandBool srv;
    srv.request.value = arm;
    if (!arming_client_.call(srv)) {
        ROS_WARN("[ControllerSimulate] arming service call failed.");
        return false;
    }
    return srv.response.success;
}

}  // namespace indooruav_controller

