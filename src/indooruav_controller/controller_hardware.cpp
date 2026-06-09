/**
 * @file  controller_hardware.cpp
 * @brief ControllerHardware 实现
 */
#include "indooruav_controller/controller_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
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
constexpr char kDefaultUploadMissionPhotosFromSdService[] =
    "indooruav_controller/controller_hardware/upload_mission_photos_from_sd";
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
constexpr char kDefaultHttpUploadImageBytesService[] =
    "/indooruav_http/upload_image_bytes";

constexpr char kDefaultVisionCheckService[] =
    "indooruav_controller/controller_hardware/vision_check";
constexpr char kDefaultVisionLandingService[] =
    "indooruav_controller/controller_hardware/vision_landing";
constexpr char kDefaultCheckPassedService[] =
    "indooruav_core/state_machine_event/check_passed";
constexpr char kDefaultCheckFailedService[] =
    "indooruav_core/state_machine_event/check_failed";

constexpr char kDefaultCheckPassedAuxService[] =
    "indooruav_mission/preflight_check/check_passed";
constexpr char kDefaultCheckFailedAuxService[] =
    "indooruav_mission/preflight_check/check_failed";
constexpr char kDefaultLandCompleteAuxService[] =
    "indooruav_mission/landing/land_complete";

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
    InitializeCameraManager();

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
    node_handle_.param<std::string>("/indooruav_controller/services/upload_mission_photos_from_sd",
                                    upload_mission_photos_from_sd_service_name_,
                                    kDefaultUploadMissionPhotosFromSdService);
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
    node_handle_.param<std::string>("/indooruav_controller/services/http_upload_image_bytes",
                                    http_upload_image_bytes_service_name_,
                                    kDefaultHttpUploadImageBytesService);

    node_handle_.param<std::string>("/indooruav_controller/services/vision_check",
                                    vision_check_service_name_,
                                    kDefaultVisionCheckService);
    node_handle_.param<std::string>("/indooruav_controller/services/vision_landing",
                                    vision_landing_service_name_,
                                    kDefaultVisionLandingService);
    node_handle_.param<std::string>("/indooruav_controller/services/check_passed",
                                    check_passed_service_name_,
                                    kDefaultCheckPassedService);
    node_handle_.param<std::string>("/indooruav_controller/services/check_failed",
                                    check_failed_service_name_,
                                    kDefaultCheckFailedService);

    node_handle_.param<std::string>("/indooruav_controller/services/check_passed_aux",
                                    check_passed_aux_service_name_,
                                    kDefaultCheckPassedAuxService);
    node_handle_.param<std::string>("/indooruav_controller/services/check_failed_aux",
                                    check_failed_aux_service_name_,
                                    kDefaultCheckFailedAuxService);
    node_handle_.param<std::string>("/indooruav_controller/services/land_complete_aux",
                                    land_complete_aux_service_name_,
                                    kDefaultLandCompleteAuxService);

    // 话题 + 频率
    node_handle_.param<std::string>("/indooruav_controller/topics/cmd_vel",
                                    cmd_vel_topic_, kDefaultCmdVelTopic);
    node_handle_.param<double>("/indooruav_controller/parameters/vel_send_rate_hz",
                               vel_send_rate_hz_, 10.0);
    node_handle_.param<int>("/indooruav_controller/parameters/media_camera_mount_position",
                            media_camera_mount_position_, -1);
    node_handle_.param<double>("/indooruav_controller/parameters/media_time_tolerance_sec",
                               media_time_tolerance_sec_, 5.0);
    node_handle_.param<double>("/indooruav_controller/parameters/media_file_wait_timeout_sec",
                               media_file_wait_timeout_sec_, 60.0);
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

bool ControllerHardware::InitializeCameraManager() {
    const T_DjiReturnCode ret = DjiCameraManager_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_ERROR("[ControllerHardware] DjiCameraManager_Init failed: 0x%08llX",
                  static_cast<unsigned long long>(ret));
        camera_manager_initialized_.store(false);
        return false;
    }

    camera_manager_initialized_.store(true);
    ROS_INFO("[ControllerHardware] camera manager initialized");
    return true;
}

bool ControllerHardware::EnsureCameraManagerReady() {
    if (camera_manager_initialized_.load()) {
        return true;
    }
    return InitializeCameraManager();
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
    upload_mission_photos_from_sd_service_server_ = node_handle_.advertiseService(
        upload_mission_photos_from_sd_service_name_,
        &ControllerHardware::UploadMissionPhotosFromSdCallback, this);
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

    // 任务/自动化 (0x5x 段)
    vision_check_service_server_ = node_handle_.advertiseService(
        vision_check_service_name_,
        &ControllerHardware::VisionCheckCallback, this);
    vision_landing_service_server_ = node_handle_.advertiseService(
        vision_landing_service_name_,
        &ControllerHardware::VisionLandingCallback, this);

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
    ROS_INFO_STREAM("[ControllerHardware] media upload: " << upload_mission_photos_from_sd_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam vcfg:   " << camera_video_config_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] cam zoom:   " << camera_zoom_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] gimbal yf:  " << gimbal_yaw_follow_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] gimbal ang: " << gimbal_angle_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] vision check:  " << vision_check_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] vision land:  " << vision_landing_service_name_);
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
    upload_image_bytes_client_ =
        node_handle_.serviceClient<indooruav_msgs::UploadImageBytes>(http_upload_image_bytes_service_name_);

    check_passed_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(check_passed_service_name_);
    check_failed_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(check_failed_service_name_);

    check_passed_aux_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(check_passed_aux_service_name_);
    check_failed_aux_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(check_failed_aux_service_name_);
    land_complete_aux_client_ =
        node_handle_.serviceClient<std_srvs::Empty>(land_complete_aux_service_name_);

    ROS_INFO_STREAM("[ControllerHardware] takeoff_complete: " << takeoff_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] land_complete:    " << land_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] hover_complete:   " << hover_complete_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp record: " << waypoint_record_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp save:   " << waypoint_save_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] wp clear:  " << waypoint_clear_service_name_);
    ROS_INFO_STREAM("[ControllerHardware] http upload image bytes: " << http_upload_image_bytes_service_name_);
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
    } else if (frame.cmd == drone_comm::CMD_ACK_CHECK_PASSED) {
        ROS_INFO("[ControllerHardware] NOTIFY: CHECK PASSED");
        NotifyCheckPassed();
    } else if (frame.cmd == drone_comm::CMD_ACK_CHECK_FAILED) {
        const uint8_t reason = (frame.len >= 1) ? frame.payload[0]
                                                : drone_comm::CHECK_FAIL_REASON_UNKNOWN;
        ROS_WARN("[ControllerHardware] NOTIFY: CHECK FAILED (reason=0x%02X)", reason);
        NotifyCheckFailed();
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
    // 并行通知 mission 节点
    CallEmptyService(land_complete_aux_client_,
                     land_complete_aux_service_name_,
                     "land_complete_aux");
}

void ControllerHardware::NotifyHoverComplete() {
    CallEmptyService(hover_complete_client_,
                     hover_complete_service_name_,
                     "hover_complete");
}

void ControllerHardware::NotifyCheckPassed() {
    CallEmptyService(check_passed_client_,
                     check_passed_service_name_,
                     "check_passed");
    // 并行通知 mission 节点（mission 不在时静默失败）
    CallEmptyService(check_passed_aux_client_,
                     check_passed_aux_service_name_,
                     "check_passed_aux");
}

void ControllerHardware::NotifyCheckFailed() {
    CallEmptyService(check_failed_client_,
                     check_failed_service_name_,
                     "check_failed");
    // 并行通知 mission 节点
    CallEmptyService(check_failed_aux_client_,
                     check_failed_aux_service_name_,
                     "check_failed_aux");
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

bool ControllerHardware::UploadMissionPhotosFromSdCallback(
    indooruav_msgs::TransferMissionMedia::Request& request,
    indooruav_msgs::TransferMissionMedia::Response& response) {
    response.result_code = 1;
    response.matched_count = 0;
    response.uploaded_count = 0;
    response.failed_count = 0;

    if (request.airline_key.empty() || request.detect_time_cur.empty()) {
        ROS_WARN("[ControllerHardware] SD transfer skipped because airline_key or detect_time_cur is empty");
        response.result_code = 3;
        return true;
    }

    if (media_camera_mount_position_ < 0 ||
        media_camera_mount_position_ == DJI_MOUNT_POSITION_UNKNOWN) {
        ROS_WARN("[ControllerHardware] SD transfer skipped because media_camera_mount_position is not configured");
        response.result_code = 3;
        return true;
    }

    if (sd_transfer_running_.exchange(true)) {
        ROS_WARN("[ControllerHardware] SD transfer ignored because another transfer is still running");
        response.result_code = 3;
        return true;
    }

    struct RunningGuard {
        std::atomic<bool>& flag;
        ~RunningGuard() {
            flag.store(false);
        }
    } running_guard{sd_transfer_running_};

    if (!EnsureCameraManagerReady()) {
        response.result_code = 2;
        return true;
    }

    const E_DjiMountPosition mount_position =
        static_cast<E_DjiMountPosition>(media_camera_mount_position_);
    const T_DjiReturnCode reg_ret =
        DjiCameraManager_RegDownloadFileDataCallback(mount_position,
                                                     &ControllerHardware::StaticDownloadFileDataCallback);
    if (reg_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_WARN("[ControllerHardware] Failed to register media download callback: 0x%08llX",
                 static_cast<unsigned long long>(reg_ret));
        response.result_code = 2;
        return true;
    }

    const T_DjiReturnCode rights_ret = DjiCameraManager_ObtainDownloaderRights(mount_position);
    if (rights_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_WARN("[ControllerHardware] Failed to obtain downloader rights: 0x%08llX",
                 static_cast<unsigned long long>(rights_ret));
        response.result_code = 2;
        return true;
    }

    struct RightsGuard {
        E_DjiMountPosition mount_position;
        ~RightsGuard() {
            const T_DjiReturnCode ret = DjiCameraManager_ReleaseDownloaderRights(mount_position);
            if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                ROS_WARN("[ControllerHardware] Failed to release downloader rights: 0x%08llX",
                         static_cast<unsigned long long>(ret));
            }
        }
    } rights_guard{mount_position};

    T_DjiCameraManagerFileList file_list{};
    const T_DjiReturnCode list_ret = DjiCameraManager_DownloadFileList(mount_position, &file_list);
    if (list_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_WARN("[ControllerHardware] Failed to download camera file list: 0x%08llX",
                 static_cast<unsigned long long>(list_ret));
        response.result_code = 2;
        return true;
    }

    const std::time_t mission_start_unix = ParseDetectTimeCur(request.detect_time_cur);
    if (mission_start_unix <= 0) {
        ROS_WARN("[ControllerHardware] Failed to parse detectTimeCur for SD transfer: %s",
                 request.detect_time_cur.c_str());
        response.result_code = 3;
        return true;
    }
    const std::time_t workflow_end_unix =
        std::time(nullptr) + static_cast<std::time_t>(std::ceil(std::max(0.0, media_time_tolerance_sec_)));

    std::vector<T_DjiCameraManagerFileListInfo> matched_files;
    matched_files.reserve(file_list.totalCount);
    for (uint16_t i = 0; i < file_list.totalCount; ++i) {
        const T_DjiCameraManagerFileListInfo& file_info = file_list.fileListInfo[i];
        if (!IsSupportedMediaType(file_info.type)) {
            continue;
        }
        if (!IsMediaInMissionWindow(file_info, mission_start_unix, workflow_end_unix)) {
            continue;
        }
        matched_files.push_back(file_info);
    }

    std::sort(matched_files.begin(), matched_files.end(),
              [this](const T_DjiCameraManagerFileListInfo& left,
                     const T_DjiCameraManagerFileListInfo& right) {
                  const std::time_t left_time = FileCreateTimeToUnix(left.createTime);
                  const std::time_t right_time = FileCreateTimeToUnix(right.createTime);
                  if (left_time != right_time) {
                      return left_time < right_time;
                  }
                  return left.fileIndex < right.fileIndex;
              });

    response.matched_count = static_cast<int32_t>(matched_files.size());
    if (matched_files.empty()) {
        ROS_INFO("[ControllerHardware] SD transfer found no matching photos for detectTimeCur=%s",
                 request.detect_time_cur.c_str());
        return true;
    }

    for (const T_DjiCameraManagerFileListInfo& file_info : matched_files) {
        if (UploadMediaFile(file_info, request.airline_key, request.detect_time_cur)) {
            ++response.uploaded_count;
        } else {
            ++response.failed_count;
        }
    }

    if (response.failed_count > 0) {
        response.result_code = 2;
    }

    ROS_INFO("[ControllerHardware] SD transfer finished for detectTimeCur=%s matched=%d uploaded=%d failed=%d",
             request.detect_time_cur.c_str(),
             response.matched_count,
             response.uploaded_count,
             response.failed_count);
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
// 服务回调：任务/自动化 (0x5x 段)
// =============================================================================

bool ControllerHardware::VisionCheckCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(
        drone_comm::CMD_CHECK_BEFORE_TAKEOFF, buf, sizeof(buf));
    if (len == 0) {
        ROS_ERROR("[ControllerHardware] encode_simple(check_before_takeoff) failed");
        return true;
    }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX check_before_takeoff");
    } else {
        ROS_WARN("[ControllerHardware] TX check_before_takeoff FAILED");
    }
    return true;
}

bool ControllerHardware::VisionLandingCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(
        drone_comm::CMD_VISION_LANDING, buf, sizeof(buf));
    if (len == 0) {
        ROS_ERROR("[ControllerHardware] encode_simple(vision_landing) failed");
        return true;
    }
    if (SendFrame(buf, len)) {
        ROS_INFO("[ControllerHardware] TX vision_landing");
    } else {
        ROS_WARN("[ControllerHardware] TX vision_landing FAILED");
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

T_DjiReturnCode ControllerHardware::StaticDownloadFileDataCallback(T_DjiDownloadFilePacketInfo packet_info,
                                                                   const uint8_t* data,
                                                                   uint16_t data_len) {
    if (instance_ != nullptr) {
        return instance_->OnDownloadFileData(packet_info, data, data_len);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode ControllerHardware::OnDownloadFileData(T_DjiDownloadFilePacketInfo packet_info,
                                                       const uint8_t* data,
                                                       uint16_t data_len) {
    std::lock_guard<std::mutex> lock(download_mutex_);
    if (active_download_file_indices_.count(packet_info.fileIndex) == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    switch (packet_info.downloadFileEvent) {
        case DJI_DOWNLOAD_FILE_EVENT_START:
            download_buffer_.clear();
            download_error_message_.clear();
            download_finished_ = false;
            download_success_ = false;
            downloading_file_index_ = packet_info.fileIndex;
            break;
        case DJI_DOWNLOAD_FILE_EVENT_TRANSFER:
            if (data != nullptr && data_len > 0) {
                download_buffer_.insert(download_buffer_.end(), data, data + data_len);
            }
            break;
        case DJI_DOWNLOAD_FILE_EVENT_END:
            download_finished_ = true;
            download_success_ = true;
            active_download_file_indices_.erase(packet_info.fileIndex);
            download_cv_.notify_all();
            break;
        case DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END:
            download_finished_ = true;
            download_success_ = true;
            active_download_file_indices_.erase(packet_info.fileIndex);
            download_cv_.notify_all();
            break;
        default:
            break;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool ControllerHardware::IsSupportedMediaType(E_DjiCameraMediaFileType media_type) const {
    return media_type == DJI_CAMERA_FILE_TYPE_JPEG ||
           media_type == DJI_CAMERA_FILE_TYPE_DNG ||
           media_type == DJI_CAMERA_FILE_TYPE_TIFF;
}

bool ControllerHardware::UploadMediaFile(const T_DjiCameraManagerFileListInfo& file_info,
                                         const std::string& airline_key,
                                         const std::string& detect_time_cur) {
    std::vector<uint8_t> file_bytes;
    std::string error_message;
    if (!DownloadFileToBuffer(file_info.fileIndex, &file_bytes, &error_message)) {
        ROS_WARN("[ControllerHardware] Failed to download media file index=%u name=%s: %s",
                 file_info.fileIndex,
                 file_info.fileName,
                 error_message.c_str());
        return false;
    }

    return UploadDownloadedBytes(file_info, file_bytes, airline_key, detect_time_cur);
}

bool ControllerHardware::UploadDownloadedBytes(const T_DjiCameraManagerFileListInfo& file_info,
                                               const std::vector<uint8_t>& file_bytes,
                                               const std::string& airline_key,
                                               const std::string& detect_time_cur) {
    if (file_bytes.empty()) {
        ROS_WARN("[ControllerHardware] Skip uploading empty media bytes for file index=%u name=%s",
                 file_info.fileIndex,
                 file_info.fileName);
        return false;
    }

    indooruav_msgs::UploadImageBytes service;
    service.request.airline_key = airline_key;
    service.request.detect_time_cur = detect_time_cur;
    service.request.source_name = file_info.fileName;
    service.request.image_extension = DetectMediaExtension(file_info);
    service.request.image_bytes = file_bytes;

    if (!upload_image_bytes_client_.call(service)) {
        ROS_WARN("[ControllerHardware] Failed to call http upload image bytes service for file index=%u",
                 file_info.fileIndex);
        return false;
    }

    if (service.response.result_code != 1) {
        ROS_WARN("[ControllerHardware] HTTP upload image bytes returned resultCode=%d for file index=%u",
                 service.response.result_code,
                 file_info.fileIndex);
        return false;
    }

    ROS_INFO("[ControllerHardware] Uploaded SD-card media file index=%u name=%s",
             file_info.fileIndex,
             file_info.fileName);
    return true;
}

bool ControllerHardware::DownloadFileToBuffer(uint32_t file_index,
                                              std::vector<uint8_t>* buffer,
                                              std::string* error_message) {
    if (buffer == nullptr || error_message == nullptr) {
        return false;
    }

    const E_DjiMountPosition mount_position =
        static_cast<E_DjiMountPosition>(media_camera_mount_position_);

    {
        std::lock_guard<std::mutex> lock(download_mutex_);
        download_buffer_.clear();
        download_error_message_.clear();
        downloading_file_index_ = file_index;
        download_in_progress_ = true;
        download_finished_ = false;
        download_success_ = false;
        active_download_file_indices_.insert(file_index);
    }

    const T_DjiReturnCode ret = DjiCameraManager_DownloadFileByIndex(mount_position, file_index);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::lock_guard<std::mutex> lock(download_mutex_);
        active_download_file_indices_.erase(file_index);
        download_in_progress_ = false;
        *error_message = "DjiCameraManager_DownloadFileByIndex failed";
        return false;
    }

    if (!WaitForDownloadResult(error_message)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(download_mutex_);
        *buffer = download_buffer_;
        download_in_progress_ = false;
    }
    return true;
}

bool ControllerHardware::WaitForDownloadResult(std::string* error_message) {
    if (error_message == nullptr) {
        return false;
    }

    std::unique_lock<std::mutex> lock(download_mutex_);
    const bool finished = download_cv_.wait_for(
        lock,
        std::chrono::milliseconds(static_cast<int>(std::max(1.0, media_file_wait_timeout_sec_ * 1000.0))),
        [this]() { return download_finished_; });

    if (!finished) {
        active_download_file_indices_.erase(downloading_file_index_);
        download_in_progress_ = false;
        *error_message = "download timed out";
        return false;
    }

    if (!download_success_) {
        download_in_progress_ = false;
        *error_message = download_error_message_.empty() ? "download failed" : download_error_message_;
        return false;
    }

    *error_message = "";
    return true;
}

bool ControllerHardware::IsMediaInMissionWindow(const T_DjiCameraManagerFileListInfo& file_info,
                                                std::time_t mission_start_unix,
                                                std::time_t workflow_end_unix) const {
    const std::time_t file_time = FileCreateTimeToUnix(file_info.createTime);
    if (file_time <= 0) {
        return false;
    }

    const std::time_t lower_bound =
        mission_start_unix - static_cast<std::time_t>(std::ceil(std::max(0.0, media_time_tolerance_sec_)));
    const std::time_t upper_bound =
        workflow_end_unix + static_cast<std::time_t>(std::ceil(std::max(0.0, media_time_tolerance_sec_)));
    return file_time >= lower_bound && file_time <= upper_bound;
}

std::time_t ControllerHardware::ParseDetectTimeCur(const std::string& detect_time_cur) const {
    std::tm tm_value{};
    std::istringstream ss(detect_time_cur);
    ss >> std::get_time(&tm_value, "%Y%m%d%H%M%S");
    if (ss.fail()) {
        return static_cast<std::time_t>(0);
    }
    tm_value.tm_isdst = -1;
    return std::mktime(&tm_value);
}

std::time_t ControllerHardware::FileCreateTimeToUnix(const T_DjiCameraManagerFileCreateTime& create_time) const {
    std::tm tm_value{};
    tm_value.tm_year = static_cast<int>(create_time.year) - 1900;
    tm_value.tm_mon = static_cast<int>(create_time.month) - 1;
    tm_value.tm_mday = static_cast<int>(create_time.day);
    tm_value.tm_hour = static_cast<int>(create_time.hour);
    tm_value.tm_min = static_cast<int>(create_time.minute);
    tm_value.tm_sec = static_cast<int>(create_time.second);
    tm_value.tm_isdst = -1;
    return std::mktime(&tm_value);
}

std::string ControllerHardware::DetectMediaExtension(const T_DjiCameraManagerFileListInfo& file_info) const {
    const std::string file_name(file_info.fileName);
    const std::size_t dot_pos = file_name.find_last_of('.');
    if (dot_pos != std::string::npos) {
        return file_name.substr(dot_pos);
    }

    switch (file_info.type) {
        case DJI_CAMERA_FILE_TYPE_JPEG:
            return ".jpg";
        case DJI_CAMERA_FILE_TYPE_DNG:
            return ".dng";
        case DJI_CAMERA_FILE_TYPE_TIFF:
            return ".tiff";
        default:
            return ".bin";
    }
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
