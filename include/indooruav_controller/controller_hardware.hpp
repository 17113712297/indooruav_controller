/**
 * @file  controller_hardware.hpp
 * @brief PSDK <-> MSDK bridge node.1
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <dji_low_speed_data_channel.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <indooruav_msgs/CameraVideoConfig.h>
#include <indooruav_msgs/CameraZoom.h>
#include <indooruav_msgs/GimbalAngle.h>
#include <indooruav_msgs/GimbalYawFollow.h>
#include <indooruav_msgs/ModeCommand.h>

namespace drone_comm {

constexpr uint8_t FRAME_HEADER    = 0xAA;
constexpr uint8_t MAX_PAYLOAD_LEN = 251;

constexpr uint8_t CMD_TAKEOFF           = 0x01;
constexpr uint8_t CMD_LAND              = 0x02;
constexpr uint8_t CMD_HOVER             = 0x03;
constexpr uint8_t CMD_VEL               = 0x04;
constexpr uint8_t CMD_GIMBAL_YAW_FOLLOW = 0x11;
constexpr uint8_t CMD_GIMBAL_ANGLE      = 0x12;
constexpr uint8_t CMD_CAM_MODE          = 0x21;
constexpr uint8_t CMD_CAM_SHOOT         = 0x22;
constexpr uint8_t CMD_CAM_RECORD        = 0x23;
constexpr uint8_t CMD_CAM_VIDEO_CFG     = 0x24;
constexpr uint8_t CMD_CAM_ZOOM          = 0x25;
constexpr uint8_t CMD_AUX_LIGHT         = 0x31;
constexpr uint8_t CMD_RECORD_WAYPOINT   = 0x41;
constexpr uint8_t CMD_SAVE_WAYPOINTS    = 0x42;
constexpr uint8_t CMD_CLEAR_WAYPOINTS   = 0x43;
constexpr uint8_t CMD_CHECK_BEFORE_TAKEOFF = 0x50;
constexpr uint8_t CMD_VISION_LANDING       = 0x51;

// ★ 建图模式指令 (0x60-0x64)
constexpr uint8_t CMD_MAPPING_SET_NAME = 0x60;  // Android→ROS: 设置地图名称
constexpr uint8_t CMD_MAPPING_START    = 0x61;  // Android→ROS: 启动雷达+建图
constexpr uint8_t CMD_MAPPING_SAVE_MAP = 0x62;  // Android→ROS: 保存地图
constexpr uint8_t CMD_MAPPING_STOP     = 0x63;  // Android→ROS: 停止雷达+建图
constexpr uint8_t CMD_LIST_MAPS        = 0x64;  // Android→ROS: 获取地图文件列表

// ★ 采点模式指令 (0x65-0x6A)
constexpr uint8_t CMD_COLLECT_SET_MAP     = 0x65;  // Android→ROS: 设置定位地图
constexpr uint8_t CMD_COLLECT_SET_WP_NAME = 0x66;  // Android→ROS: 设置航点文件名
constexpr uint8_t CMD_COLLECT_START       = 0x67;  // Android→ROS: 启动雷达+定位+记录器
constexpr uint8_t CMD_COLLECT_GEN_2D      = 0x68;  // Android→ROS: 生成2D地图
constexpr uint8_t CMD_COLLECT_GEN_PIXEL   = 0x69;  // Android→ROS: 生成像素坐标
constexpr uint8_t CMD_COLLECT_STOP        = 0x6A;  // Android→ROS: 停止雷达+定位+记录器

// ★ 巡航模式指令 (0x6B-0x6D)
constexpr uint8_t CMD_CRUISE_SET_MAP = 0x6B;  // Android→ROS: 设置定位地图
constexpr uint8_t CMD_CRUISE_SET_WP  = 0x6C;  // Android→ROS: 设置航线文件
constexpr uint8_t CMD_CRUISE_START   = 0x6D;  // Android→ROS: 触发 HTTP 起飞指令

// ★ 通用查询 (0x6E-0x6F)
constexpr uint8_t CMD_LIST_WAYPOINTS = 0x6E;  // Android→ROS: 获取航线文件列表
constexpr uint8_t CMD_CRUISE_SELECT_WP = 0x6F; // Android→ROS: 选择航线（调用 airlineInfo API）
constexpr uint8_t CMD_CRUISE_SET_SERVER = 0x70; // Android→ROS: 设置巡航服务器地址
constexpr uint8_t CMD_CRUISE_SET_GIMBAL_PITCH = 0x71; // Android→ROS: 设置云台俯仰角

// ★ 设置模式指令 (0x72-0x74)
constexpr uint8_t CMD_SETTINGS_UPDATE = 0x72;  // Android→ROS: 修改 HTTP 配置参数
constexpr uint8_t CMD_SETTINGS_GET = 0x73;     // Android→ROS: 获取 HTTP 配置参数
constexpr uint8_t CMD_SETTINGS_RESTART_HTTP = 0x74; // Android→ROS: 重启 HTTP 服务

// ★ 响应指令 (0x90-0x9F)
constexpr uint8_t CMD_FILE_LIST_RESPONSE   = 0x90; // ROS→Android: 地图文件列表响应
constexpr uint8_t CMD_FILE_LIST_RESPONSE_WP = 0x91; // ROS→Android: 航线文件列表响应
constexpr uint8_t CMD_SETTINGS_RESPONSE     = 0x92; // ROS→Android: 设置参数响应
constexpr uint8_t CMD_ACK                    = 0x80;
constexpr uint8_t CMD_ACK_TAKEOFF_COMPLETE   = 0x81;
constexpr uint8_t CMD_ACK_LAND_COMPLETE      = 0x82;
constexpr uint8_t CMD_ACK_HOVER_COMPLETE     = 0x83;
constexpr uint8_t CMD_ACK_CHECK_PASSED       = 0x84;
constexpr uint8_t CMD_ACK_CHECK_FAILED       = 0x85;

constexpr uint8_t ACK_OK      = 0x00;
constexpr uint8_t ACK_FAIL    = 0x01;
constexpr uint8_t ACK_UNKNOWN = 0xFF;

constexpr uint8_t CHECK_FAIL_REASON_UNKNOWN = 0xFF;

constexpr uint8_t GIMBAL_MODE_ABSOLUTE = 0x00;
constexpr uint8_t GIMBAL_MODE_RELATIVE = 0x01;

constexpr uint8_t CAM_MODE_PHOTO = 0x00;
constexpr uint8_t CAM_MODE_VIDEO = 0x01;

constexpr uint8_t CAM_RECORD_STOP  = 0x00;
constexpr uint8_t CAM_RECORD_START = 0x01;

constexpr uint8_t CAM_LENS_WIDE     = 0x00;
constexpr uint8_t CAM_LENS_ZOOM     = 0x01;
constexpr uint8_t CAM_LENS_INFRARED = 0x02;

constexpr uint8_t CAM_ZOOM_SWITCH_ONLY    = 0x00;
constexpr uint8_t CAM_ZOOM_SWITCH_AND_SET = 0x01;

constexpr uint8_t AUX_LIGHT_OFF  = 0x00;
constexpr uint8_t AUX_LIGHT_ON   = 0x01;
constexpr uint8_t AUX_LIGHT_AUTO = 0x02;

#pragma pack(push, 1)
struct VelPayload {
    float vx;
    float vy;
    float vz;
    float yaw_rate;
};

struct GimbalYawFollowPayload {
    float pitch;
    float roll;
};

struct GimbalAnglePayload {
    uint8_t mode;
    uint8_t reserved;
    float   pitch;
    float   roll;
    float   yaw;
    float   duration;
};

struct CamModePayload {
    uint8_t mode;
};

struct CamRecordPayload {
    uint8_t action;
};

struct CamVideoCfgPayload {
    uint8_t resolution;
    uint8_t frame_rate;
};

struct CamZoomPayload {
    uint8_t lens;
    uint8_t action;
    uint8_t reserved0;
    uint8_t reserved1;
    float   ratio;
};

struct AuxLightPayload {
    uint8_t mode;
};
#pragma pack(pop)

static_assert(sizeof(VelPayload) == 16, "VelPayload must be 16 bytes");
static_assert(sizeof(GimbalYawFollowPayload) == 8, "GimbalYawFollowPayload must be 8 bytes");
static_assert(sizeof(GimbalAnglePayload) == 18, "GimbalAnglePayload must be 18 bytes");
static_assert(sizeof(CamModePayload) == 1, "CamModePayload must be 1 byte");
static_assert(sizeof(CamRecordPayload) == 1, "CamRecordPayload must be 1 byte");
static_assert(sizeof(CamVideoCfgPayload) == 2, "CamVideoCfgPayload must be 2 bytes");
static_assert(sizeof(CamZoomPayload) == 8, "CamZoomPayload must be 8 bytes");
static_assert(sizeof(AuxLightPayload) == 1, "AuxLightPayload must be 1 byte");

inline uint8_t encode_simple(uint8_t cmd, uint8_t* buf, uint8_t buf_len) {
    if (buf_len < 4) {
        return 0;
    }
    buf[0] = FRAME_HEADER;
    buf[1] = cmd;
    buf[2] = 0;
    buf[3] = buf[0] ^ buf[1] ^ buf[2];
    return 4;
}

inline uint8_t encode_payload(uint8_t cmd,
                              const void* data,
                              uint8_t data_len,
                              uint8_t* buf,
                              uint8_t buf_len) {
    if (buf == nullptr || data == nullptr) {
        return 0;
    }
    if (buf_len < static_cast<uint16_t>(4 + data_len)) {
        return 0;
    }
    buf[0] = FRAME_HEADER;
    buf[1] = cmd;
    buf[2] = data_len;
    std::memcpy(buf + 3, data, data_len);
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(3 + data_len); ++i) {
        xor_val ^= buf[i];
    }
    buf[3 + data_len] = xor_val;
    return static_cast<uint8_t>(4 + data_len);
}

inline uint8_t encode_vel(const VelPayload& vel, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_VEL, &vel, sizeof(VelPayload), buf, buf_len);
}

inline uint8_t encode_gimbal_yaw_follow(const GimbalYawFollowPayload& payload,
                                        uint8_t* buf,
                                        uint8_t buf_len) {
    return encode_payload(CMD_GIMBAL_YAW_FOLLOW,
                          &payload,
                          sizeof(GimbalYawFollowPayload),
                          buf,
                          buf_len);
}

inline uint8_t encode_gimbal_angle(const GimbalAnglePayload& payload,
                                   uint8_t* buf,
                                   uint8_t buf_len) {
    return encode_payload(CMD_GIMBAL_ANGLE,
                          &payload,
                          sizeof(GimbalAnglePayload),
                          buf,
                          buf_len);
}

inline uint8_t encode_cam_mode(const CamModePayload& payload, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_MODE, &payload, sizeof(CamModePayload), buf, buf_len);
}

inline uint8_t encode_cam_shoot(uint8_t* buf, uint8_t buf_len) {
    return encode_simple(CMD_CAM_SHOOT, buf, buf_len);
}

inline uint8_t encode_cam_record(const CamRecordPayload& payload, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_RECORD, &payload, sizeof(CamRecordPayload), buf, buf_len);
}

inline uint8_t encode_cam_video_cfg(const CamVideoCfgPayload& payload,
                                    uint8_t* buf,
                                    uint8_t buf_len) {
    return encode_payload(CMD_CAM_VIDEO_CFG,
                          &payload,
                          sizeof(CamVideoCfgPayload),
                          buf,
                          buf_len);
}

inline uint8_t encode_cam_zoom(const CamZoomPayload& payload, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_ZOOM, &payload, sizeof(CamZoomPayload), buf, buf_len);
}

inline uint8_t encode_aux_light(const AuxLightPayload& payload, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_AUX_LIGHT, &payload, sizeof(AuxLightPayload), buf, buf_len);
}

struct Frame {
    bool    valid = false;
    uint8_t cmd = 0;
    uint8_t len = 0;
    uint8_t payload[MAX_PAYLOAD_LEN] = {};
};

inline Frame decode(const uint8_t* data, uint16_t data_len) {
    Frame frame;
    if (data == nullptr || data_len < 4) {
        return frame;
    }
    if (data[0] != FRAME_HEADER) {
        return frame;
    }
    const uint8_t len = data[2];
    if (data_len < static_cast<uint16_t>(4 + len)) {
        return frame;
    }
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(3 + len); ++i) {
        xor_val ^= data[i];
    }
    if (xor_val != data[3 + len]) {
        return frame;
    }
    frame.valid = true;
    frame.cmd = data[1];
    frame.len = len;
    if (len > 0) {
        std::memcpy(frame.payload, data + 3, len);
    }
    return frame;
}

}  // namespace drone_comm

namespace indooruav_controller {

class ControllerHardware {
public:
    explicit ControllerHardware(ros::NodeHandle& node_handle);
    ~ControllerHardware();

    ControllerHardware(const ControllerHardware&) = delete;
    ControllerHardware& operator=(const ControllerHardware&) = delete;

private:
    void LoadParameters();
    void InitializePsdkChannel();
    void AdvertiseServiceServers();
    void CreateServiceClients();
    void CreateSubscribersAndTimers();

    static T_DjiReturnCode StaticOnRecvFromMsdk(const uint8_t* data, uint16_t len);
    void OnRecvFromMsdk(const uint8_t* data, uint16_t len);

    void NotifyTakeoffComplete();
    void NotifyLandComplete();
    void NotifyHoverComplete();
    void NotifyCheckPassed();
    void NotifyCheckFailed();

    void OnCmdVel(const geometry_msgs::Twist::ConstPtr& message);
    void OnVelSendTimer(const ros::TimerEvent& event);

    bool TakeoffCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool LandCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HoverCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool NotifyUavOpenLightCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);
    bool NotifyUavCloseLightCallback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);
    bool NotifyUavAutoLightCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);

    bool NotifyUavVideoRecordingStartCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response);
    bool NotifyUavVideoRecordingStopCallback(std_srvs::Empty::Request& request,
                                             std_srvs::Empty::Response& response);
    bool CameraModePhotoCallback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& response);
    bool CameraModeVideoCallback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& response);
    bool CameraShootCallback(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response);
    bool CameraVideoConfigCallback(indooruav_msgs::CameraVideoConfig::Request& request,
                                   indooruav_msgs::CameraVideoConfig::Response& response);
    bool CameraZoomCallback(indooruav_msgs::CameraZoom::Request& request,
                            indooruav_msgs::CameraZoom::Response& response);

    bool GimbalYawFollowCallback(indooruav_msgs::GimbalYawFollow::Request& request,
                                 indooruav_msgs::GimbalYawFollow::Response& response);
    bool GimbalAngleCallback(indooruav_msgs::GimbalAngle::Request& request,
                             indooruav_msgs::GimbalAngle::Response& response);

    bool VisionCheckCallback(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response);
    bool VisionLandingCallback(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response);

    bool SendFrame(const uint8_t* buf, uint8_t len);
    bool CallEmptyService(ros::ServiceClient& client,
                          const std::string& service_name,
                          const char* service_label);
    void SetVelForwardingEnabled(bool enabled);

private:
    static ControllerHardware* instance_;

    ros::NodeHandle& node_handle_;

    std::string takeoff_service_name_;
    std::string land_service_name_;
    std::string hover_service_name_;

    std::string notify_uav_open_light_service_name_;
    std::string notify_uav_close_light_service_name_;
    std::string notify_uav_auto_light_service_name_;

    std::string notify_uav_video_recording_start_service_name_;
    std::string notify_uav_video_recording_stop_service_name_;

    std::string camera_mode_photo_service_name_;
    std::string camera_mode_video_service_name_;
    std::string camera_shoot_service_name_;
    std::string camera_video_config_service_name_;
    std::string camera_zoom_service_name_;

    std::string gimbal_yaw_follow_service_name_;
    std::string gimbal_angle_service_name_;

    std::string takeoff_complete_service_name_;
    std::string land_complete_service_name_;
    std::string hover_complete_service_name_;

    std::string waypoint_record_service_name_;
    std::string waypoint_save_service_name_;
    std::string waypoint_clear_service_name_;

    std::string vision_check_service_name_;
    std::string vision_landing_service_name_;
    std::string check_passed_service_name_;
    std::string check_failed_service_name_;
    std::string check_passed_aux_service_name_;
    std::string check_failed_aux_service_name_;
    std::string land_complete_aux_service_name_;

    // ★ 建图模式
    std::string mode_command_service_name_;
    bool SendModeCommand(const std::string& command, const std::string& payload);

    std::string cmd_vel_topic_;
    double      vel_send_rate_hz_ = 10.0;

    ros::ServiceServer takeoff_service_server_;
    ros::ServiceServer land_service_server_;
    ros::ServiceServer hover_service_server_;
    ros::ServiceServer notify_uav_open_light_service_server_;
    ros::ServiceServer notify_uav_close_light_service_server_;
    ros::ServiceServer notify_uav_auto_light_service_server_;
    ros::ServiceServer notify_uav_video_recording_start_service_server_;
    ros::ServiceServer notify_uav_video_recording_stop_service_server_;
    ros::ServiceServer camera_mode_photo_service_server_;
    ros::ServiceServer camera_mode_video_service_server_;
    ros::ServiceServer camera_shoot_service_server_;
    ros::ServiceServer camera_video_config_service_server_;
    ros::ServiceServer camera_zoom_service_server_;
    ros::ServiceServer gimbal_yaw_follow_service_server_;
    ros::ServiceServer gimbal_angle_service_server_;
    ros::ServiceServer vision_check_service_server_;
    ros::ServiceServer vision_landing_service_server_;

    ros::ServiceClient takeoff_complete_client_;
    ros::ServiceClient land_complete_client_;
    ros::ServiceClient hover_complete_client_;
    ros::ServiceClient waypoint_record_client_;
    ros::ServiceClient waypoint_save_client_;
    ros::ServiceClient waypoint_clear_client_;
    ros::ServiceClient check_passed_client_;
    ros::ServiceClient check_failed_client_;
    ros::ServiceClient check_passed_aux_client_;
    ros::ServiceClient check_failed_aux_client_;
    ros::ServiceClient land_complete_aux_client_;
    ros::ServiceClient mode_command_client_;

    ros::Subscriber cmd_vel_subscriber_;
    ros::Timer      vel_send_timer_;

    std::mutex             vel_mutex_;
    drone_comm::VelPayload latest_vel_{0.0f, 0.0f, 0.0f, 0.0f};
    bool                   vel_updated_ = false;
    bool                   vel_forwarding_enabled_ = false;

    std::mutex tx_mutex_;
    std::atomic<uint32_t> ack_count_{0};
};

}  // namespace indooruav_controller
