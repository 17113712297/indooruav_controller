/**
 * @file  controller_hardware.cpp
 * @brief ControllerHardware 实现
 */
#include "indooruav_controller/controller_hardware.hpp"

#include <cmath>
#include <stdexcept>

namespace indooruav_controller {

namespace {

constexpr double kRadToDeg = 57.29577951308232;

// ── 默认服务名 (仅当 yaml 未提供时使用) ────────────────────
constexpr char kDefaultTakeoffService[] =
    "indooruav_controller/controller_hardware/takeoff";
constexpr char kDefaultLandService[] =
    "indooruav_controller/controller_hardware/land";
constexpr char kDefaultHoverService[] =
    "indooruav_controller/controller_hardware/hover";

constexpr char kDefaultNotifyUavOpenLightService[] =
    "indooruav_controller/controller_hardware/light_open";
constexpr char kDefaultNotifyUavCloseLightService[] =
    "indooruav_controller/controller_hardware/light_close";
constexpr char kDefaultNotifyUavAutoLightService[] =
    "indooruav_controller/controller_hardware/light_auto";

constexpr char kDefaultNotifyUavVideoRecordingStartService[] =
    "indooruav_controller/controller_hardware/camera_video_start";
constexpr char kDefaultNotifyUavVideoRecordingStopService[] =
    "indooruav_controller/controller_hardware/camera_video_stop";

constexpr char kDefaultCameraModePhotoService[] =
    "indooruav_controller/controller_hardware/camera_mode_photo";
constexpr char kDefaultCameraModeVideoService[] =
    "indooruav_controller/controller_hardware/camera_mode_video";
constexpr char kDefaultCameraShootService[] =
    "indooruav_controller/controller_hardware/camera_photo_shoot";
constexpr char kDefaultCameraVideoConfigService[] =
    "indooruav_controller/controller_hardware/camera_video_config";
constexpr char kDefaultCameraZoomService[] =
    "indooruav_controller/controller_hardware/camera_zoom";

constexpr char kDefaultGimbalYawFollowService[] =
    "indooruav_controller/controller_hardware/gimbal_yaw_follow";
constexpr char kDefaultGimbalAngleService[] =
    "indooruav_controller/controller_hardware/gimbal_angle";

constexpr char kDefaultTakeoffCompleteService[] =
    "indooruav_core/state_machine_event/takeoff_complete";
constexpr char kDefaultLandCompleteService[] =
    "indooruav_core/state_machine_event/land_complete";
constexpr char kDefaultHoverCompleteService[] =
    "indooruav_core/state_machine_event/hover_complete";

constexpr char kDefaultWaypointRecordService[] =
    "indooruav_controller/waypoint_recorder/record";
constexpr char kDefaultWaypointSaveService[] =
    "indooruav_controller/waypoint_recorder/save";
constexpr char kDefaultWaypointClearService[] =
    "indooruav_controller/waypoint_recorder/clear";

constexpr char kDefaultCmdVelTopic[] = "indooruav_controller/waypoint_tracker/cmd_vel";

}  // namespace

// ── 静态成员 ──────────────────────────────────────────────
ControllerHardware* ControllerHardware::instance_ = nullptr;

// =============================================================================
// 构造 / 析构
// =============================================================================

ControllerHardware::ControllerHardware(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
    if (instance_ != nullptr) {
        throw std::runtime_error(
            "ControllerHardware is a singleton; only one instance is allowed.");
    }
    instance_ = this;

    LoadParameters();

    // 初始化 PSDK 通道前注册必须先于此处的 Application 完成 PSDK 平台初始化，
    // 由 main() 保证调用顺序。
    InitializePsdkChannel();

    AdvertiseServiceServers();
    CreateServiceClients();
    CreateSubscribersAndTimers();

    ROS_INFO("[ControllerHardware] initialized.");
}

ControllerHardware::~ControllerHardware() {
    // 注意：PSDK 没有公开"反注册接收回调"接口，析构后若 PSDK 线程仍调用
    // StaticOnRecvFromMsdk，instance_ == nullptr 时会安全返回 SUCCESS。
    instance_ = nullptr;
}

// =============================================================================
// 初始化
// =============================================================================

void ControllerHardware::LoadParameters() {
    // 入站服务名
    node_handle_.param<std::string>("/indooruav_controller/services/takeoff",
                                    takeoff_service_name_, kDefaultTakeoffService);
    node_handle_.param<std::string>("/indooruav_controller/services/land",
                                    land_service_name_, kDefaultLandService);
    node_handle_.param<std::string>("/indooruav_controller/services/hover",
                                    hover_service_name_, kDefaultHoverService);

    node_handle_.param<std::string>("/indooruav_controller/services/notify_uav_open_light",
                                    notify_uav_open_light_service_name_,
                                    kDefaultNotifyUavOpenLightService);
    node_handle_.param<std::string>("/indooruav_controller/services/notify_uav_close_light",
                                    notify_uav_close_light_service_name_,
                                    kDefaultNotifyUavCloseLightService);
    node_handle_.param<std::string>("/indooruav_controller/services/notify_uav_auto_light",
                                    notify_uav_auto_light_service_name_,
                                    kDefaultNotifyUavAutoLightService);

    node_handle_.param<std::string>("/indooruav_controller/services/notify_uav_video_recording_start",
                                    notify_uav_video_recording_start_service_name_,
                                    kDefaultNotifyUavVideoRecordingStartService);
    node_handle_.param<std::string>("/indooruav_controller/services/notify_uav_video_recording_stop",
                                    notify_uav_video_recording_stop_service_name_,
                                    kDefaultNotifyUavVideoRecordingStopService);

    node_handle_.param<std::string>("/indooruav_controller/services/camera_mode_photo",
                                    camera_mode_photo_service_name_,
                                    kDefaultCameraModePhotoService);
    node_handle_.param<std::string>("/indooruav_controller/services/camera_mode_video",
                                    camera_mode_video_service_name_,
                                    kDefaultCameraModeVideoService);
    node_handle_.param<std::string>("/indooruav_controller/services/camera_shoot",
                                    camera_shoot_service_name_,
                                    kDefaultCameraShootService);
    node_handle_.param<std::string>("/indooruav_controller/services/camera_video_config",
                                    camera_video_config_service_name_,
                                    kDefaultCameraVideoConfigService);
    node_handle_.param<std::string>("/indooruav_controller/services/camera_zoom",
                                    camera_zoom_service_name_,
                                    kDefaultCameraZoomService);

    node_handle_.param<std::string>("/indooruav_controller/services/gimbal_yaw_follow",
                                    gimbal_yaw_follow_service_name_,
                                    kDefaultGimbalYawFollowService);
    node_handle_.param<std::string>("/indooruav_controller/services/gimbal_angle",
                                    gimbal_angle_service_name_,
                                    kDefaultGimbalAngleService);

    // 出站完成事件服务名
    node_handle_.param<std::string>("/indooruav_controller/services/takeoff_complete",
                                    takeoff_complete_service_name_,
                                    kDefaultTakeoffCompleteService);
    node_handle_.param<std::string>("/indooruav_controller/services/land_complete",
                                    land_complete_service_name_,
                                    kDefaultLandCompleteService);
    node_handle_.param<std::string>("/indooruav_controller/services/hover_complete",
                                    hover_complete_service_name_,
                                    kDefaultHoverCompleteService);

    node_handle_.param<std::string>("/indooruav_controller/services/waypoint_record",
                                    waypoint_record_service_name_,
                                    kDefaultWaypointRecordService);
    node_handle_.param<std::string>("/indooruav_controller/services/waypoint_save",
                                    waypoint_save_service_name_,
                                    kDefaultWaypointSaveService);
    node_handle_.param<std::string>("/indooruav_controller/services/waypoint_clear",
                                    waypoint_clear_service_name_,
                                    kDefaultWaypointClearService);

    // 话题 + 频率
    node_handle_.param<std::string>("/indooruav_controller/topics/cmd_vel",
                                    cmd_vel_topic_, kDefaultCmdVelTopic);
    node_handle_.param<double>("/indooruav_controller/parameters/vel_send_rate_hz",
                               vel_send_rate_hz_, 10.0);
    if (vel_send_rate_hz_ < 1.0) {
        vel_send_rate_hz_ = 1.0;
    }
}

void ControllerHardware::InitializePsdkChannel() {
    T_DjiReturnCode ret = DjiLowSpeedDataChannel_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_FATAL("[ControllerHardware] DjiLowSpeedDataChannel_Init FAILED: 0x%08llX",
                  static_cast<unsigned long long>(ret));
        throw std::runtime_error("DjiLowSpeedDataChannel_Init failed");
    }
    ret = DjiLowSpeedDataChannel_RegRecvDataCallback(
        DJI_CHANNEL_ADDRESS_MASTER_RC_APP, &ControllerHardware::StaticOnRecvFromMsdk);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_FATAL("[ControllerHardware] RegRecvDataCallback FAILED: 0x%08llX",
                  static_cast<unsigned long long>(ret));
        throw std::runtime_error("DjiLowSpeedDataChannel_RegRecvDataCallback failed");
    }
}

void ControllerHardware::AdvertiseServiceServers() {
    // 飞控
    takeoff_service_server_ = node_handle_.advertiseService(
        takeoff_service_name_, &ControllerHardware::TakeoffCallback, this);
    land_service_server_ = node_handle_.advertiseService(
        land_service_name_, &ControllerHardware::LandCallback, this);
    hover_service_server_ = node_handle_.advertiseService(
        hover_service_name_, &ControllerHardware::HoverCallback, this);

    // 补光灯
    notify_uav_open_light_service_server_ = node_handle_.advertiseService(
        notify_uav_open_light_service_name_,
        &ControllerHardware::NotifyUavOpenLightCallback, this);
    notify_uav_close_light_service_server_ = node_handle_.advertiseService(
        notify_uav_close_light_service_name_,
        &ControllerHardware::NotifyUavCloseLightCallback, this);
    notify_uav_auto_light_service_server_ = node_handle_.advertiseService(
        notify_uav_auto_light_service_name_,
        &ControllerHardware::NotifyUavAutoLightCallback, this);

    // 相机
    notify_uav_video_recording_start_service_server_ = node_handle_.advertiseService(
        notify_uav_video_recording_start_service_name_,
        &ControllerHardware::NotifyUavVideoRecordingStartCallback, this);
    notify_uav_video_recording_stop_service_server_ = node_handle_.advertiseService(
        notify_uav_video_recording_stop_service_name_,
        &ControllerHardware::NotifyUavVideoRecordingStopCallback, this);
    camera_mode_photo_service_server_ = node_handle_.advertiseService(
        camera_mode_photo_service_name_,
        &ControllerHardware::CameraModePhotoCallback, this);
    camera_mode_video_service_server_ = node_handle_.advertiseService(
        camera_mode_video_service_name_,
        &ControllerHardware::CameraModeVideoCallback, this);
    camera_shoot_service_server_ = node_handle_.advertiseService(
        camera_shoot_service_name_,
        &ControllerHardware::CameraShootCallback, this);
    camera_video_config_service_server_ = node_handle_.advertiseService(
        camera_video_config_service_name_,
        &ControllerHardware::CameraVideoConfigCallback, this);
    camera_zoom_service_server_ = node_handle_.advertiseService(
        camera_zoom_service_name_,
        &ControllerHardware::CameraZoomCallback, this);

    // 云台
    gimbal_yaw_follow_service_server_ = node_handle_.advertiseService(
        gimbal_yaw_follow_service_name_,
        &ControllerHardware::GimbalYawFollowCallback, this);
    gimbal_angle_service_server_ = node_handle_.advertiseService(
        gimbal_angle_service_name_,
        &ControllerHardware::GimbalAngleCallback, this);

    ROS_INFO_STREAM("[ControllerHardware] takeoff:    " << takeoff_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] land:       " << land_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] hover:      " << hover_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] light on:   " << notify_uav_open_light_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] light off:  " << notify_uav_close_light_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] light auto: " << notify_uav_auto_light_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] rec start:  " << notify_uav_video_recording_start_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] rec stop:   " << notify_uav_video_recording_stop_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam photo:  " << camera_mode_photo_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam video:  " << camera_mode_video_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam shoot:  " << camera_shoot_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam vcfg:   " << camera_video_config_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam zoom:   " << camera_zoom_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] gimbal yf:  " << gimbal_yaw_follow_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] gimbal ang: " << gimbal_angle_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cmd_vel topic: " << cmd_vel_topic_);
}

void ControllerHardware::CreateServiceClients() {
    takeoff_complete_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(takeoff_complete_service_name_);
    land_complete_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(land_complete_service_name_);
    hover_complete_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(hover_complete_service_name_);

    waypoint_record_client_ =
        node_handle_.serviceClient<std_srvs::Trigger>(waypoint_record_service_name_);
    waypoint_save_client_ =
        node_handle_.serviceClient<std_srvs::Trigger>(waypoint_save_service_name_);
    waypoint_clear_client_ =
        node_handle_.serviceClient<std_srvs::Trigger>(waypoint_clear_service_name_);

    ROS_INFO_STREAM("[ControllerHardware] takeoff_complete: " << takeoff_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] land_complete:    " << land_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] hover_complete:   " << hover_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp record: " << waypoint_record_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp save:   " << waypoint_save_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp clear:  " << waypoint_clear_service_name_);
}

void ControllerHardware::CreateSubscribersAndTimers() {
    cmd_vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>(
        cmd_vel_topic_, 1, &ControllerHardware::OnCmdVel, this);

    vel_send_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / vel_send_rate_hz_),
        &ControllerHardware::OnVelSendTimer, this);
}

// =============================================================================
// PSDK 接收回调
// =============================================================================

T_DjiReturnCode ControllerHardware::StaticOnRecvFromMsdk(const uint8_t* data,
                                                         uint16_t len) {
    if (instance_ != nullptr) {
        instance_->OnRecvFromMsdk(data, len);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void ControllerHardware::OnRecvFromMsdk(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len == 0) return;

    drone_comm::Frame frame = drone_comm::decode(data, len);
    if (!frame.valid) {
        ROS_WARN("[ControllerHardware] RX: invalid frame (len=%u)", len);
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK) {
        const uint8_t acked_cmd = (frame.len >= 1) ? frame.payload[0] : 0xFF;
        const uint8_t status    = (frame.len >= 2) ? frame.payload[1]
                                                   : drone_comm::ACK_UNKNOWN;
        const char* status_str  = (status == drone_comm::ACK_OK) ? "OK" : "FAIL";
        const uint32_t count = ack_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        ROS_INFO_THROTTLE(10.0, "[ControllerHardware] ACK #%u  cmd=0x%02X  status=%s",
                         count, acked_cmd, status_str);
    } else if (frame.cmd == drone_comm::CMD_ACK_TAKEOFF_COMPLETE) {
        ROS_INFO("[ControllerHardware] NOTIFY: TAKEOFF COMPLETE");
        // 起飞完成 → 打开 cmd_vel 门 → 通知上游
        SetVelForwardingEnabled(true);
        NotifyTakeoffComplete();
    } else if (frame.cmd == drone_comm::CMD_ACK_LAND_COMPLETE) {
        ROS_INFO("[ControllerHardware] NOTIFY: LAND COMPLETE");
        // 降落完成 → 门保持关闭 → 通知上游
        SetVelForwardingEnabled(false);
        NotifyLandComplete();
    } else if (frame.cmd == drone_comm::CMD_ACK_HOVER_COMPLETE) {
        ROS_INFO("[ControllerHardware] NOTIFY: HOVER COMPLETE");
        // 悬停完成 → 重新打开 cmd_vel 门 → 通知上游
        SetVelForwardingEnabled(true);
        NotifyHoverComplete();
    } else if (frame.cmd == drone_comm::CMD_RECORD_WAYPOINT) {
        ROS_INFO("[ControllerHardware] CMD: RECORD WAYPOINT");
        std_srvs::Trigger srv;
        bool ok = waypoint_record_client_.call(srv);
        uint8_t status = (ok && srv.response.success)
                         ? drone_comm::ACK_OK : drone_comm::ACK_FAIL;
        if (ok && srv.response.success) {
            ROS_INFO("[ControllerHardware] Waypoint recorded: %s", srv.response.message.c_str());
        } else {
            ROS_WARN("[ControllerHardware] Waypoint record failed: %s",
                     ok ? srv.response.message.c_str() : "service call error");
        }
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_RECORD_WAYPOINT;
        ack_frame[4] = status;
        ack_frame[5] = ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
    } else if (frame.cmd == drone_comm::CMD_SAVE_WAYPOINTS) {
        ROS_INFO("[ControllerHardware] CMD: SAVE WAYPOINTS");
        std_srvs::Trigger srv;
        bool ok = waypoint_save_client_.call(srv);
        uint8_t status = (ok && srv.response.success)
                         ? drone_comm::ACK_OK : drone_comm::ACK_FAIL;
        if (ok && srv.response.success) {
            ROS_INFO("[ControllerHardware] Waypoints saved: %s", srv.response.message.c_str());
        } else {
            ROS_WARN("[ControllerHardware] Waypoints save failed: %s",
                     ok ? srv.response.message.c_str() : "service call error");
        }
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_SAVE_WAYPOINTS;
        ack_frame[4] = status;
        ack_frame[5] = ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
    } else if (frame.cmd == drone_comm::CMD_CLEAR_WAYPOINTS) {
        ROS_INFO("[ControllerHardware] CMD: CLEAR WAYPOINTS");
        std_srvs::Trigger srv;
        bool ok = waypoint_clear_client_.call(srv);
        uint8_t status = (ok && srv.response.success)
                         ? drone_comm::ACK_OK : drone_comm::ACK_FAIL;
        if (ok && srv.response.success) {
            ROS_INFO("[ControllerHardware] Waypoints cleared: %s", srv.response.message.c_str());
        } else {
            ROS_WARN("[ControllerHardware] Waypoints clear failed: %s",
                     ok ? srv.response.message.c_str() : "service call error");
        }
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_CLEAR_WAYPOINTS;
        ack_frame[4] = status;
        ack_frame[5] = ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
    } else {
        ROS_INFO("[ControllerHardware] RX unknown cmd=0x%02X len=%u",
                 frame.cmd, frame.len);
    }
}

// =============================================================================
// 上行通知 (本节点 → 上游)
// =============================================================================

void ControllerHardware::NotifyTakeoffComplete() {
    CallEmptyService(takeoff_complete_client_,
                     takeoff_complete_service_name_,
                     "takeoff_complete");
}

void ControllerHardware::NotifyLandComplete() {
    CallEmptyService(land_complete_client_,
                     land_complete_service_name_,
                     "land_complete");
}

void ControllerHardware::NotifyHoverComplete() {
    CallEmptyService(hover_complete_client_,
                     hover_complete_service_name_,
                     "hover_complete");
}

bool ControllerHardware::CallEmptyService(ros::ServiceClient& client,
                                          const std::string& service_name,
                                          const char* service_label) {
    std_srvs::Empty service;
    if (!client.call(service)) {
        ROS_WARN_STREAM("[ControllerHardware] Failed to call "
                        << service_label << " service: " << service_name);
        return false;
    }
    ROS_INFO_STREAM("[ControllerHardware] Called "
                    << service_label << " service: " << service_name);
    return true;
}

// =============================================================================
// cmd_vel 订阅 + 定时发送
// =============================================================================

void ControllerHardware::OnCmdVel(const geometry_msgs::Twist::ConstPtr& message) {
    // ── 坐标系转换：ROS Body Frame → 协议 VelPayload (机体系) ─────────
    //   ROS REP-103：linear.x=前 (正)、linear.y=左 (正)、linear.z=上 (正)、angular.z=逆时针(rad/s)
    //   协议 VelPayload：vx=前 (正)、vy=右 (正)、vz=上 (正)、yaw_rate=右转 (deg/s)
    //   故 vy 与 yaw_rate 取反；yaw_rate 由 rad/s → deg/s。
    drone_comm::VelPayload vel;
    vel.vx       = static_cast<float>(message->linear.x);
    vel.vy       = static_cast<float>(-message->linear.y);
    vel.vz       = static_cast<float>(message->linear.z);
    vel.yaw_rate = static_cast<float>(-message->angular.z * kRadToDeg);

    std::lock_guard<std::mutex> lock(vel_mutex_);
    if (!vel_forwarding_enabled_) {
        // 门关闭期间直接丢弃，不缓存
        return;
    }
    latest_vel_   = vel;
    vel_updated_  = true;
}

void ControllerHardware::OnVelSendTimer(const ros::TimerEvent& /*event*/) {
    drone_comm::VelPayload vel_to_send;
    bool should_send = false;
    {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        if (vel_forwarding_enabled_ && vel_updated_) {
            vel_to_send  = latest_vel_;
            should_send  = true;
            // ── 一次 cmd_vel → 一次 VEL 帧 ──
            // MSDK 端有 feedTimer 负责喂 SDK，本节点不重复发同一帧；MSDK 端 1 秒
            // VEL 看门狗会在我们停止发送后让飞机自动悬停，形成安全闭环。
            vel_updated_ = false;
        }
    }

    if (!should_send) return;

    uint8_t buf[32];
    const uint8_t len = drone_comm::encode_vel(vel_to_send, buf, sizeof(buf));
    if (len == 0) {
        ROS_WARN_THROTTLE(2.0, "[ControllerHardware] encode_vel failed");
        return;
    }
    if (!SendFrame(buf, len)) {
        ROS_WARN_THROTTLE(2.0, "[ControllerHardware] VEL TX failed");
    }
}

// =============================================================================
// 服务回调：飞控
// =============================================================================

bool ControllerHardware::TakeoffCallback(std_srvs::Empty::Request& /*request*/,
                                         std_srvs::Empty::Response& /*response*/) {
    // 关闭 cmd_vel 门，避免起飞过程中外部 cmd_vel 与自动起飞抢 VirtualStick
    SetVelForwardingEnabled(false);

    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_TAKEOFF, buf, sizeof(buf));
    if (len == 0) {
        ROS_ERROR("[ControllerHardware] encode_simple(takeoff) failed");
        return true;
    }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX takeoff");
    } else {
        ROS_WARN("[ControllerHardware] TX takeoff FAILED");
    }
    return true;
}

bool ControllerHardware::LandCallback(std_srvs::Empty::Request& /*request*/,
                                      std_srvs::Empty::Response& /*response*/) {
    SetVelForwardingEnabled(false);

    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_LAND, buf, sizeof(buf));
    if (len == 0) {
        ROS_ERROR("[ControllerHardware] encode_simple(land) failed");
        return true;
    }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX land");
    } else {
        ROS_WARN("[ControllerHardware] TX land FAILED");
    }
    return true;
}

bool ControllerHardware::HoverCallback(std_srvs::Empty::Request& /*request*/,
                                       std_srvs::Empty::Response& /*response*/) {
    // hover 期间也关门，待 MSDK 上报 HOVER COMPLETE 后再开 (与 takeoff 对称)
    SetVelForwardingEnabled(false);

    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_HOVER, buf, sizeof(buf));
    if (len == 0) {
        ROS_ERROR("[ControllerHardware] encode_simple(hover) failed");
        return true;
    }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX hover");
    } else {
        ROS_WARN("[ControllerHardware] TX hover FAILED");
    }
    return true;
}

// =============================================================================
// 服务回调：补光灯
// =============================================================================

bool ControllerHardware::NotifyUavOpenLightCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_ON;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_aux_light(on) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX AUX_LIGHT on");
    else                     ROS_WARN("[ControllerHardware] TX AUX_LIGHT on FAILED");
    return true;
}

bool ControllerHardware::NotifyUavCloseLightCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_OFF;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_aux_light(off) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX AUX_LIGHT off");
    else                     ROS_WARN("[ControllerHardware] TX AUX_LIGHT off FAILED");
    return true;
}

bool ControllerHardware::NotifyUavAutoLightCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_AUTO;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_aux_light(auto) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX AUX_LIGHT auto");
    else                     ROS_WARN("[ControllerHardware] TX AUX_LIGHT auto FAILED");
    return true;
}

// =============================================================================
// 服务回调：相机
// =============================================================================

bool ControllerHardware::NotifyUavVideoRecordingStartCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamRecordPayload payload{};
    payload.action = drone_comm::CAM_RECORD_START;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_record(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_record(start) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX CAM_RECORD start");
    else                     ROS_WARN("[ControllerHardware] TX CAM_RECORD start FAILED");
    return true;
}

bool ControllerHardware::NotifyUavVideoRecordingStopCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamRecordPayload payload{};
    payload.action = drone_comm::CAM_RECORD_STOP;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_record(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_record(stop) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX CAM_RECORD stop");
    else                     ROS_WARN("[ControllerHardware] TX CAM_RECORD stop FAILED");
    return true;
}

bool ControllerHardware::CameraModePhotoCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamModePayload payload{};
    payload.mode = drone_comm::CAM_MODE_PHOTO;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_mode(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_mode(photo) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX CAM_MODE photo");
    else                     ROS_WARN("[ControllerHardware] TX CAM_MODE photo FAILED");
    return true;
}

bool ControllerHardware::CameraModeVideoCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamModePayload payload{};
    payload.mode = drone_comm::CAM_MODE_VIDEO;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_mode(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_mode(video) failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX CAM_MODE video");
    else                     ROS_WARN("[ControllerHardware] TX CAM_MODE video FAILED");
    return true;
}

bool ControllerHardware::CameraShootCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_cam_shoot(buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_shoot failed"); return true; }
    if (SendFrame(buf, len)) ROS_INFO("[ControllerHardware] TX CAM_SHOOT");
    else                     ROS_WARN("[ControllerHardware] TX CAM_SHOOT FAILED");
    return true;
}

bool ControllerHardware::CameraVideoConfigCallback(
    indooruav_msgs::CameraVideoConfig::Request& request,
    indooruav_msgs::CameraVideoConfig::Response& /*response*/) {
    drone_comm::CamVideoCfgPayload payload{};
    payload.resolution = request.resolution;
    payload.frame_rate = request.frame_rate;

    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_video_cfg(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_video_cfg failed"); return true; }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX CAM_VIDEO_CFG res=0x%02X fps=0x%02X",
                 payload.resolution, payload.frame_rate);
    } else {
        ROS_WARN("[ControllerHardware] TX CAM_VIDEO_CFG FAILED");
    }
    return true;
}

bool ControllerHardware::CameraZoomCallback(
    indooruav_msgs::CameraZoom::Request& request,
    indooruav_msgs::CameraZoom::Response& /*response*/) {
    drone_comm::CamZoomPayload payload{};
    payload.lens      = request.lens;
    payload.action    = request.action;
    payload.reserved0 = 0;
    payload.reserved1 = 0;
    payload.ratio     = (request.ratio > 0.0f) ? request.ratio : 1.0f;

    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_zoom(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_cam_zoom failed"); return true; }
    if (SendFrame(buf, len)) {
        if (payload.action == drone_comm::CAM_ZOOM_SWITCH_AND_SET) {
            ROS_INFO("[ControllerHardware] TX CAM_ZOOM lens=0x%02X ratio=%.2f",
                     payload.lens, payload.ratio);
        } else {
            ROS_INFO("[ControllerHardware] TX CAM_ZOOM lens=0x%02X (switch only)",
                     payload.lens);
        }
    } else {
        ROS_WARN("[ControllerHardware] TX CAM_ZOOM FAILED");
    }
    return true;
}

// =============================================================================
// 服务回调：云台
// =============================================================================

bool ControllerHardware::GimbalYawFollowCallback(
    indooruav_msgs::GimbalYawFollow::Request& request,
    indooruav_msgs::GimbalYawFollow::Response& /*response*/) {
    drone_comm::GimbalYawFollowPayload payload{};
    payload.pitch = request.pitch;
    payload.roll  = request.roll;

    uint8_t buf[32];
    const uint8_t len = drone_comm::encode_gimbal_yaw_follow(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_gimbal_yaw_follow failed"); return true; }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX GIMBAL_YAW_FOLLOW pitch=%.1f roll=%.1f",
                 payload.pitch, payload.roll);
    } else {
        ROS_WARN("[ControllerHardware] TX GIMBAL_YAW_FOLLOW FAILED");
    }
    return true;
}

bool ControllerHardware::GimbalAngleCallback(
    indooruav_msgs::GimbalAngle::Request& request,
    indooruav_msgs::GimbalAngle::Response& /*response*/) {
    drone_comm::GimbalAnglePayload payload{};
    if (request.mode == indooruav_msgs::GimbalAngle::Request::MODE_RELATIVE) {
        payload.mode = drone_comm::GIMBAL_MODE_RELATIVE;
    } else {
        payload.mode = drone_comm::GIMBAL_MODE_ABSOLUTE;
    }
    payload.reserved = 0;
    payload.pitch    = request.pitch;
    payload.roll     = request.roll;
    payload.yaw      = request.yaw;
    payload.duration = (request.duration > 0.0f) ? request.duration : 1.0f;

    uint8_t buf[32];
    const uint8_t len = drone_comm::encode_gimbal_angle(payload, buf, sizeof(buf));
    if (len == 0) { ROS_ERROR("[ControllerHardware] encode_gimbal_angle failed"); return true; }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX GIMBAL_ANGLE mode=%s pitch=%.1f roll=%.1f yaw=%.1f dur=%.2fs",
                 (payload.mode == drone_comm::GIMBAL_MODE_RELATIVE) ? "relative" : "absolute",
                 payload.pitch, payload.roll, payload.yaw, payload.duration);
    } else {
        ROS_WARN("[ControllerHardware] TX GIMBAL_ANGLE FAILED");
    }
    return true;
}

// =============================================================================
// 工具方法
// =============================================================================

bool ControllerHardware::SendFrame(const uint8_t* buf, uint8_t len) {
    // 串行化所有 PSDK 低速通道写操作 —— AsyncSpinner(2) 下可能有两个回调
    // (例如 OnVelSendTimer 与服务回调) 同时进入此处
    std::lock_guard<std::mutex> lock(tx_mutex_);
    const T_DjiReturnCode ret = DjiLowSpeedDataChannel_SendData(
        DJI_CHANNEL_ADDRESS_MASTER_RC_APP, buf, len);
    return ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void ControllerHardware::SetVelForwardingEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vel_forwarding_enabled_ = enabled;
    if (!enabled) {
        // 门关闭时清掉任何待发送的 vel，避免门重新打开后发出 stale 数据
        vel_updated_ = false;
    }
}

}  // namespace indooruav_controller
