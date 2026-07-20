/**
 * @file  controller_hardware.cpp
 * @brief PSDK <-> MSDK bridge implementation.
 */
#include "indooruav_controller/controller_hardware.hpp"

#include <cmath>
#include <stdexcept>

namespace indooruav_controller {

namespace {

constexpr double kRadToDeg = 57.29577951308232;

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

constexpr char kDefaultModeCommandService[] =
    "indooruav_core/mode_manager/command";

constexpr char kDefaultCmdVelTopic[] =
    "indooruav_controller/waypoint_tracker/cmd_vel";

}  // namespace

ControllerHardware* ControllerHardware::instance_ = nullptr;

ControllerHardware::ControllerHardware(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
    if (instance_ != nullptr) {
        throw std::runtime_error(
            "ControllerHardware is a singleton; only one instance is allowed.");
    }
    instance_ = this;

    LoadParameters();
    InitializePsdkChannel();
    AdvertiseServiceServers();
    CreateServiceClients();
    CreateSubscribersAndTimers();

    ROS_INFO("[ControllerHardware] initialized.");
}

ControllerHardware::~ControllerHardware() {
    instance_ = nullptr;
}

void ControllerHardware::LoadParameters() {
    node_handle_.param<std::string>("/indooruav_controller/services/takeoff",
                                    takeoff_service_name_,
                                    kDefaultTakeoffService);
    node_handle_.param<std::string>("/indooruav_controller/services/land",
                                    land_service_name_,
                                    kDefaultLandService);
    node_handle_.param<std::string>("/indooruav_controller/services/hover",
                                    hover_service_name_,
                                    kDefaultHoverService);

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

    node_handle_.param<std::string>("/indooruav_controller/services/mode_command",
                                    mode_command_service_name_,
                                    kDefaultModeCommandService);

    node_handle_.param<std::string>("/indooruav_controller/topics/cmd_vel",
                                    cmd_vel_topic_,
                                    kDefaultCmdVelTopic);
    node_handle_.param<double>("/indooruav_controller/parameters/vel_send_rate_hz",
                               vel_send_rate_hz_,
                               10.0);
    if (vel_send_rate_hz_ < 1.0) {
        vel_send_rate_hz_ = 1.0;
    }
}

void ControllerHardware::InitializePsdkChannel() {
    constexpr int kMaxRetries = 10;
    constexpr int kRetryIntervalMs = 1000;

    T_DjiReturnCode ret = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
        ret = DjiLowSpeedDataChannel_Init();
        if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            break;
        }
        ROS_WARN("[ControllerHardware] DjiLowSpeedDataChannel_Init attempt %d/%d FAILED: 0x%08llX",
                 attempt, kMaxRetries, static_cast<unsigned long long>(ret));
        if (attempt < kMaxRetries) {
            ros::Duration(kRetryIntervalMs / 1000.0).sleep();
        }
    }
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_FATAL("[ControllerHardware] DjiLowSpeedDataChannel_Init FAILED after %d attempts: 0x%08llX",
                  kMaxRetries, static_cast<unsigned long long>(ret));
        throw std::runtime_error("DjiLowSpeedDataChannel_Init failed");
    }

    ret = DjiLowSpeedDataChannel_RegRecvDataCallback(
        DJI_CHANNEL_ADDRESS_MASTER_RC_APP,
        &ControllerHardware::StaticOnRecvFromMsdk);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_FATAL("[ControllerHardware] RegRecvDataCallback FAILED: 0x%08llX",
                  static_cast<unsigned long long>(ret));
        throw std::runtime_error("DjiLowSpeedDataChannel_RegRecvDataCallback failed");
    }
}

void ControllerHardware::AdvertiseServiceServers() {
    takeoff_service_server_ = node_handle_.advertiseService(
        takeoff_service_name_, &ControllerHardware::TakeoffCallback, this);
    land_service_server_ = node_handle_.advertiseService(
        land_service_name_, &ControllerHardware::LandCallback, this);
    hover_service_server_ = node_handle_.advertiseService(
        hover_service_name_, &ControllerHardware::HoverCallback, this);

    notify_uav_open_light_service_server_ = node_handle_.advertiseService(
        notify_uav_open_light_service_name_,
        &ControllerHardware::NotifyUavOpenLightCallback,
        this);
    notify_uav_close_light_service_server_ = node_handle_.advertiseService(
        notify_uav_close_light_service_name_,
        &ControllerHardware::NotifyUavCloseLightCallback,
        this);
    notify_uav_auto_light_service_server_ = node_handle_.advertiseService(
        notify_uav_auto_light_service_name_,
        &ControllerHardware::NotifyUavAutoLightCallback,
        this);

    notify_uav_video_recording_start_service_server_ = node_handle_.advertiseService(
        notify_uav_video_recording_start_service_name_,
        &ControllerHardware::NotifyUavVideoRecordingStartCallback,
        this);
    notify_uav_video_recording_stop_service_server_ = node_handle_.advertiseService(
        notify_uav_video_recording_stop_service_name_,
        &ControllerHardware::NotifyUavVideoRecordingStopCallback,
        this);
    camera_mode_photo_service_server_ = node_handle_.advertiseService(
        camera_mode_photo_service_name_,
        &ControllerHardware::CameraModePhotoCallback,
        this);
    camera_mode_video_service_server_ = node_handle_.advertiseService(
        camera_mode_video_service_name_,
        &ControllerHardware::CameraModeVideoCallback,
        this);
    camera_shoot_service_server_ = node_handle_.advertiseService(
        camera_shoot_service_name_,
        &ControllerHardware::CameraShootCallback,
        this);
    camera_video_config_service_server_ = node_handle_.advertiseService(
        camera_video_config_service_name_,
        &ControllerHardware::CameraVideoConfigCallback,
        this);
    camera_zoom_service_server_ = node_handle_.advertiseService(
        camera_zoom_service_name_,
        &ControllerHardware::CameraZoomCallback,
        this);

    gimbal_yaw_follow_service_server_ = node_handle_.advertiseService(
        gimbal_yaw_follow_service_name_,
        &ControllerHardware::GimbalYawFollowCallback,
        this);
    gimbal_angle_service_server_ = node_handle_.advertiseService(
        gimbal_angle_service_name_,
        &ControllerHardware::GimbalAngleCallback,
        this);

    vision_check_service_server_ = node_handle_.advertiseService(
        vision_check_service_name_,
        &ControllerHardware::VisionCheckCallback,
        this);
    vision_landing_service_server_ = node_handle_.advertiseService(
        vision_landing_service_name_,
        &ControllerHardware::VisionLandingCallback,
        this);
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

    mode_command_client_ =
        node_handle_.serviceClient<indooruav_msgs::ModeCommand>(mode_command_service_name_);
}

void ControllerHardware::CreateSubscribersAndTimers() {
    cmd_vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>(
        cmd_vel_topic_, 1, &ControllerHardware::OnCmdVel, this);

    vel_send_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / vel_send_rate_hz_),
        &ControllerHardware::OnVelSendTimer,
        this);
}

T_DjiReturnCode ControllerHardware::StaticOnRecvFromMsdk(const uint8_t* data, uint16_t len) {
    if (instance_ != nullptr) {
        instance_->OnRecvFromMsdk(data, len);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void ControllerHardware::OnRecvFromMsdk(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len == 0) {
        return;
    }

    const drone_comm::Frame frame = drone_comm::decode(data, len);
    if (!frame.valid) {
        ROS_WARN("[ControllerHardware] RX: invalid frame (len=%u)", len);
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK) {
        const uint8_t acked_cmd = (frame.len >= 1) ? frame.payload[0] : 0xFF;
        const uint8_t status = (frame.len >= 2) ? frame.payload[1] : drone_comm::ACK_UNKNOWN;
        const char* status_str = (status == drone_comm::ACK_OK) ? "OK" : "FAIL";
        const uint32_t count = ack_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        ROS_INFO_THROTTLE(10.0,
                          "[ControllerHardware] ACK #%u  cmd=0x%02X  status=%s",
                          count,
                          acked_cmd,
                          status_str);
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK_TAKEOFF_COMPLETE) {
        SetVelForwardingEnabled(true);
        NotifyTakeoffComplete();
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK_LAND_COMPLETE) {
        SetVelForwardingEnabled(false);
        NotifyLandComplete();
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK_HOVER_COMPLETE) {
        SetVelForwardingEnabled(true);
        NotifyHoverComplete();
        return;
    }

    if (frame.cmd == drone_comm::CMD_RECORD_WAYPOINT) {
        std_srvs::Trigger service;
        const bool ok = waypoint_record_client_.call(service);
        const uint8_t status = (ok && service.response.success) ? drone_comm::ACK_OK
                                                                : drone_comm::ACK_FAIL;
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_RECORD_WAYPOINT;
        ack_frame[4] = status;
        ack_frame[5] =
            ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
        return;
    }

    if (frame.cmd == drone_comm::CMD_SAVE_WAYPOINTS) {
        std_srvs::Trigger service;
        const bool ok = waypoint_save_client_.call(service);
        const uint8_t status = (ok && service.response.success) ? drone_comm::ACK_OK
                                                                : drone_comm::ACK_FAIL;
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_SAVE_WAYPOINTS;
        ack_frame[4] = status;
        ack_frame[5] =
            ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
        return;
    }

    if (frame.cmd == drone_comm::CMD_CLEAR_WAYPOINTS) {
        std_srvs::Trigger service;
        const bool ok = waypoint_clear_client_.call(service);
        const uint8_t status = (ok && service.response.success) ? drone_comm::ACK_OK
                                                                : drone_comm::ACK_FAIL;
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = drone_comm::CMD_CLEAR_WAYPOINTS;
        ack_frame[4] = status;
        ack_frame[5] =
            ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK_CHECK_PASSED) {
        NotifyCheckPassed();
        return;
    }

    if (frame.cmd == drone_comm::CMD_ACK_CHECK_FAILED) {
        NotifyCheckFailed();
    }

    // ── 模式指令（建图/采点/设置 0x60 段）──────────────────────────
    if (frame.cmd >= drone_comm::CMD_MAPPING_SET_NAME &&
        frame.cmd <= drone_comm::CMD_SETTINGS_RESTART_HTTP) {
        indooruav_msgs::ModeCommand srv;
        switch (frame.cmd) {
            case drone_comm::CMD_MAPPING_SET_NAME:
                srv.request.command = "mapping_set_name";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_MAPPING_START:
                srv.request.command = "mapping_start";
                break;
            case drone_comm::CMD_MAPPING_SAVE_MAP:
                srv.request.command = "mapping_save_map";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_MAPPING_STOP:
                srv.request.command = "mapping_stop";
                break;
            case drone_comm::CMD_LIST_MAPS:
                srv.request.command = "list_maps";
                break;
            // ── 采点模式指令 ────────────────────────────────
            case drone_comm::CMD_COLLECT_SET_MAP:
                srv.request.command = "collect_set_map";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_COLLECT_SET_WP_NAME:
                srv.request.command = "collect_set_wp_name";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_COLLECT_START:
                srv.request.command = "collect_start";
                break;
            case drone_comm::CMD_COLLECT_GEN_2D:
                srv.request.command = "collect_gen_2d";
                break;
            case drone_comm::CMD_COLLECT_GEN_PIXEL:
                srv.request.command = "collect_gen_pixel";
                break;
            case drone_comm::CMD_COLLECT_STOP:
                srv.request.command = "collect_stop";
                break;
            // ── 巡航模式指令 ────────────────────────────────
            case drone_comm::CMD_CRUISE_SET_MAP:
                srv.request.command = "cruise_set_map";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_CRUISE_SET_WP:
                srv.request.command = "cruise_set_wp";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_CRUISE_START:
                srv.request.command = "cruise_start";
                break;
            case drone_comm::CMD_LIST_WAYPOINTS:
                srv.request.command = "list_waypoints";
                break;
            case drone_comm::CMD_CRUISE_SELECT_WP:
                srv.request.command = "cruise_select_wp";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_CRUISE_SET_SERVER:
                srv.request.command = "cruise_set_server";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_CRUISE_SET_GIMBAL_PITCH:
                srv.request.command = "cruise_set_gimbal_pitch";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            // ── 设置模式指令 ────────────────────────────────
            case drone_comm::CMD_SETTINGS_UPDATE:
                srv.request.command = "settings_update";
                srv.request.payload = std::string(reinterpret_cast<const char*>(frame.payload), frame.len);
                break;
            case drone_comm::CMD_SETTINGS_GET:
                srv.request.command = "settings_get";
                break;
            case drone_comm::CMD_SETTINGS_RESTART_HTTP:
                srv.request.command = "settings_restart_http";
                break;
        }

        const bool ok = mode_command_client_.call(srv);
        if (!ok) {
            ROS_WARN("[ControllerHardware] mode command '%s' service call failed",
                     srv.request.command.c_str());
            uint8_t ack_frame[6];
            ack_frame[0] = drone_comm::FRAME_HEADER;
            ack_frame[1] = drone_comm::CMD_ACK;
            ack_frame[2] = 2;
            ack_frame[3] = frame.cmd;
            ack_frame[4] = drone_comm::ACK_FAIL;
            ack_frame[5] = ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
            SendFrame(ack_frame, sizeof(ack_frame));
            return;
        }

        // ★ list_maps / list_waypoints / settings_get 等查询指令：回传响应内容
        if (srv.request.command == "list_maps" ||
            srv.request.command == "list_waypoints" ||
            srv.request.command == "settings_get") {
            const std::string& resp_msg = srv.response.message;
            uint8_t resp_len = static_cast<uint8_t>(std::min<size_t>(resp_msg.size(), 240));
            uint8_t resp_buf[4 + 240];
            resp_buf[0] = drone_comm::FRAME_HEADER;
            if (srv.request.command == "list_maps") {
                resp_buf[1] = drone_comm::CMD_FILE_LIST_RESPONSE;
            } else if (srv.request.command == "list_waypoints") {
                resp_buf[1] = drone_comm::CMD_FILE_LIST_RESPONSE_WP;
            } else {
                resp_buf[1] = drone_comm::CMD_SETTINGS_RESPONSE;
            }
            resp_buf[2] = resp_len;
            std::memcpy(resp_buf + 3, resp_msg.data(), resp_len);
            uint8_t xor_val = 0;
            for (uint8_t i = 0; i < static_cast<uint8_t>(3 + resp_len); ++i) {
                xor_val ^= resp_buf[i];
            }
            resp_buf[3 + resp_len] = xor_val;
            if (resp_len > 0) {
                SendFrame(resp_buf, 4 + resp_len);
            }
        }

        // 发送标准 ACK
        const uint8_t status = srv.response.success ? drone_comm::ACK_OK : drone_comm::ACK_FAIL;
        uint8_t ack_frame[6];
        ack_frame[0] = drone_comm::FRAME_HEADER;
        ack_frame[1] = drone_comm::CMD_ACK;
        ack_frame[2] = 2;
        ack_frame[3] = frame.cmd;
        ack_frame[4] = status;
        ack_frame[5] =
            ack_frame[0] ^ ack_frame[1] ^ ack_frame[2] ^ ack_frame[3] ^ ack_frame[4];
        SendFrame(ack_frame, sizeof(ack_frame));
        return;
    }
}

void ControllerHardware::NotifyTakeoffComplete() {
    CallEmptyService(takeoff_complete_client_, takeoff_complete_service_name_, "takeoff_complete");
}

void ControllerHardware::NotifyLandComplete() {
    CallEmptyService(land_complete_client_, land_complete_service_name_, "land_complete");
    CallEmptyService(land_complete_aux_client_, land_complete_aux_service_name_, "land_complete_aux");
}

void ControllerHardware::NotifyHoverComplete() {
    CallEmptyService(hover_complete_client_, hover_complete_service_name_, "hover_complete");
}

void ControllerHardware::NotifyCheckPassed() {
    CallEmptyService(check_passed_client_, check_passed_service_name_, "check_passed");
    CallEmptyService(check_passed_aux_client_, check_passed_aux_service_name_, "check_passed_aux");
}

void ControllerHardware::NotifyCheckFailed() {
    CallEmptyService(check_failed_client_, check_failed_service_name_, "check_failed");
    CallEmptyService(check_failed_aux_client_, check_failed_aux_service_name_, "check_failed_aux");
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
    return true;
}

void ControllerHardware::OnCmdVel(const geometry_msgs::Twist::ConstPtr& message) {
    drone_comm::VelPayload vel;
    vel.vx = static_cast<float>(message->linear.x);
    vel.vy = static_cast<float>(-message->linear.y);
    vel.vz = static_cast<float>(message->linear.z);
    vel.yaw_rate = static_cast<float>(-message->angular.z * kRadToDeg);

    std::lock_guard<std::mutex> lock(vel_mutex_);
    if (!vel_forwarding_enabled_) {
        return;
    }
    latest_vel_ = vel;
    vel_updated_ = true;
}

void ControllerHardware::OnVelSendTimer(const ros::TimerEvent& /*event*/) {
    drone_comm::VelPayload vel_to_send;
    bool should_send = false;
    {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        if (vel_forwarding_enabled_ && vel_updated_) {
            vel_to_send = latest_vel_;
            should_send = true;
            vel_updated_ = false;
        }
    }

    if (!should_send) {
        return;
    }

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

bool ControllerHardware::TakeoffCallback(std_srvs::Empty::Request& /*request*/,
                                         std_srvs::Empty::Response& /*response*/) {
    SetVelForwardingEnabled(false);
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_TAKEOFF, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::LandCallback(std_srvs::Empty::Request& /*request*/,
                                      std_srvs::Empty::Response& /*response*/) {
    SetVelForwardingEnabled(false);
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_LAND, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::HoverCallback(std_srvs::Empty::Request& /*request*/,
                                       std_srvs::Empty::Response& /*response*/) {
    SetVelForwardingEnabled(false);
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_simple(drone_comm::CMD_HOVER, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::NotifyUavOpenLightCallback(std_srvs::Empty::Request& /*request*/,
                                                    std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_ON;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::NotifyUavCloseLightCallback(std_srvs::Empty::Request& /*request*/,
                                                     std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_OFF;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::NotifyUavAutoLightCallback(std_srvs::Empty::Request& /*request*/,
                                                    std_srvs::Empty::Response& /*response*/) {
    drone_comm::AuxLightPayload payload{};
    payload.mode = drone_comm::AUX_LIGHT_AUTO;
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_aux_light(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::NotifyUavVideoRecordingStartCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamRecordPayload payload{};
    payload.action = drone_comm::CAM_RECORD_START;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_record(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::NotifyUavVideoRecordingStopCallback(
    std_srvs::Empty::Request& /*request*/,
    std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamRecordPayload payload{};
    payload.action = drone_comm::CAM_RECORD_STOP;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_record(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::CameraModePhotoCallback(std_srvs::Empty::Request& /*request*/,
                                                 std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamModePayload payload{};
    payload.mode = drone_comm::CAM_MODE_PHOTO;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_mode(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::CameraModeVideoCallback(std_srvs::Empty::Request& /*request*/,
                                                 std_srvs::Empty::Response& /*response*/) {
    drone_comm::CamModePayload payload{};
    payload.mode = drone_comm::CAM_MODE_VIDEO;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_mode(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::CameraShootCallback(std_srvs::Empty::Request& /*request*/,
                                             std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len = drone_comm::encode_cam_shoot(buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
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
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::CameraZoomCallback(indooruav_msgs::CameraZoom::Request& request,
                                            indooruav_msgs::CameraZoom::Response& /*response*/) {
    drone_comm::CamZoomPayload payload{};
    payload.lens = request.lens;
    payload.action = request.action;
    payload.reserved0 = 0;
    payload.reserved1 = 0;
    payload.ratio = (request.ratio > 0.0f) ? request.ratio : 1.0f;
    uint8_t buf[16];
    const uint8_t len = drone_comm::encode_cam_zoom(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::GimbalYawFollowCallback(
    indooruav_msgs::GimbalYawFollow::Request& request,
    indooruav_msgs::GimbalYawFollow::Response& /*response*/) {
    drone_comm::GimbalYawFollowPayload payload{};
    payload.pitch = request.pitch;
    payload.roll = request.roll;
    uint8_t buf[32];
    const uint8_t len = drone_comm::encode_gimbal_yaw_follow(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::GimbalAngleCallback(indooruav_msgs::GimbalAngle::Request& request,
                                             indooruav_msgs::GimbalAngle::Response& /*response*/) {
    drone_comm::GimbalAnglePayload payload{};
    payload.mode = (request.mode == indooruav_msgs::GimbalAngle::Request::MODE_RELATIVE)
                       ? drone_comm::GIMBAL_MODE_RELATIVE
                       : drone_comm::GIMBAL_MODE_ABSOLUTE;
    payload.reserved = 0;
    payload.pitch = request.pitch;
    payload.roll = request.roll;
    payload.yaw = request.yaw;
    payload.duration = (request.duration > 0.0f) ? request.duration : 1.0f;
    uint8_t buf[32];
    const uint8_t len = drone_comm::encode_gimbal_angle(payload, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::VisionCheckCallback(std_srvs::Empty::Request& /*request*/,
                                             std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len =
        drone_comm::encode_simple(drone_comm::CMD_CHECK_BEFORE_TAKEOFF, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::VisionLandingCallback(std_srvs::Empty::Request& /*request*/,
                                               std_srvs::Empty::Response& /*response*/) {
    uint8_t buf[8];
    const uint8_t len =
        drone_comm::encode_simple(drone_comm::CMD_VISION_LANDING, buf, sizeof(buf));
    if (len != 0) {
        SendFrame(buf, len);
    }
    return true;
}

bool ControllerHardware::SendFrame(const uint8_t* buf, uint8_t len) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    const T_DjiReturnCode ret =
        DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP, buf, len);
    return ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void ControllerHardware::SetVelForwardingEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vel_forwarding_enabled_ = enabled;
    if (!enabled) {
        vel_updated_ = false;
    }
}

bool ControllerHardware::SendModeCommand(const std::string& command, const std::string& payload) {
    indooruav_msgs::ModeCommand srv;
    srv.request.command = command;
    srv.request.payload = payload;
    if (!mode_command_client_.call(srv)) {
        ROS_WARN("[ControllerHardware] SendModeCommand('%s') failed to call service",
                 command.c_str());
        return false;
    }
    return srv.response.success;
}

}  // namespace indooruav_controller
