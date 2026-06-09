/**
 * @file  controller_hardware.hpp
 * @brief 室内 UAV 硬件控制器
 *
 * 该类整合了原 indooruav_psdk_bridge 中的两个组件：
 *   1) 协议层 (原 drone_comm_protocol.hpp)：PSDK ↔ MSDK 二进制帧编解码
 *      保留为 drone_comm 命名空间下的内部协议工具
 *   2) ROS 接口层 (原 ros_interface + psdk_drone_controller)：服务/订阅/PSDK 通道
 *      整合到 indooruav_controller::ControllerHardware 类
 *
 * 所有飞控/云台/相机/补光灯指令统一通过 ROS 服务暴露：
 *   - 起飞、降落、悬停、单拍、录像 start/stop、相机模式 photo/video、补光灯 on/off/auto
 *     使用 std_srvs/Empty
 *   - 云台 Yaw Follow / 角度、视频参数、变焦使用本包自定义 .srv
 *
 * /drone/cmd_vel 仍为话题订阅 (高频速度指令，由其它节点发布)，并受 takeoff/land/hover 门控。
 *
 * 状态机门控：
 *   gate=false 初始；
 *   takeoff 服务 → gate=false → CMD_TAKEOFF → MSDK CMD_ACK_TAKEOFF_COMPLETE → gate=true → 通知上游
 *   hover   服务 → gate=false → CMD_HOVER   → MSDK CMD_ACK_HOVER_COMPLETE   → gate=true → 通知上游
 *   land    服务 → gate=false → CMD_LAND    → MSDK CMD_ACK_LAND_COMPLETE    → gate=false (保持) → 通知上游
 *
 * cmd_vel 处理策略 (drop)：gate 关闭期间收到的 cmd_vel 直接丢弃，不缓存。
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <dji_camera_manager.h>
#include <dji_low_speed_data_channel.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <indooruav_msgs/CameraVideoConfig.h>
#include <indooruav_msgs/CameraZoom.h>
#include <indooruav_msgs/GimbalAngle.h>
#include <indooruav_msgs/GimbalYawFollow.h>
#include <indooruav_msgs/TransferMissionMedia.h>
#include <indooruav_msgs/UploadImageBytes.h>

// =============================================================================
// 协议层 (与 MSDK Kotlin 端共用同一套编解码规则)
//
// ┌────────┬──────┬────────┬──────────────────┬──────────┐
// │ 0xAA   │ CMD  │  LEN   │   PAYLOAD (N B)  │ XOR校验  │
// │  1 B   │  1 B │  1 B   │      N B         │   1 B    │
// └────────┴──────┴────────┴──────────────────┴──────────┘
//   固定开销 4 B，最大总帧长 = 4 + 251 = 255 B (MSDK单包上限)
//
// 指令分段：
//   0x0x  飞控    0x1x  云台    0x2x  相机    0x3x  配件    0x8x  应答
// =============================================================================
namespace drone_comm {

// ── 帧常量 ────────────────────────────────────────────────
constexpr uint8_t FRAME_HEADER    = 0xAA;
constexpr uint8_t MAX_PAYLOAD_LEN = 251;

// ── 指令类型 ──────────────────────────────────────────────
// 飞控
constexpr uint8_t CMD_TAKEOFF            = 0x01;
constexpr uint8_t CMD_LAND               = 0x02;
constexpr uint8_t CMD_HOVER              = 0x03;
constexpr uint8_t CMD_VEL                = 0x04;
// 云台
constexpr uint8_t CMD_GIMBAL_YAW_FOLLOW  = 0x11;
constexpr uint8_t CMD_GIMBAL_ANGLE       = 0x12;
// 相机
constexpr uint8_t CMD_CAM_MODE           = 0x21;  // 工作模式：照片/视频
constexpr uint8_t CMD_CAM_SHOOT          = 0x22;  // 单拍 (无载荷)
constexpr uint8_t CMD_CAM_RECORD         = 0x23;  // 开始/停止录像
constexpr uint8_t CMD_CAM_VIDEO_CFG      = 0x24;  // 录像分辨率+帧率
constexpr uint8_t CMD_CAM_ZOOM           = 0x25;  // 广角/变焦/红外切换 + 变焦倍数
// 配件
constexpr uint8_t CMD_AUX_LIGHT          = 0x31;  // 下视补光灯控制
// 航点 (Android → Jetson，无载荷)
constexpr uint8_t CMD_RECORD_WAYPOINT  = 0x41;
constexpr uint8_t CMD_SAVE_WAYPOINTS   = 0x42;
constexpr uint8_t CMD_CLEAR_WAYPOINTS  = 0x43;
// 应答
constexpr uint8_t CMD_ACK                    = 0x80;
constexpr uint8_t CMD_ACK_TAKEOFF_COMPLETE   = 0x81;  // 起飞完成通知 (无载荷)
constexpr uint8_t CMD_ACK_LAND_COMPLETE      = 0x82;  // 降落完成通知 (无载荷)
constexpr uint8_t CMD_ACK_HOVER_COMPLETE     = 0x83;  // 悬停完成通知 (无载荷)

// ── ACK 状态码 ────────────────────────────────────────────
constexpr uint8_t ACK_OK      = 0x00;
constexpr uint8_t ACK_FAIL    = 0x01;
constexpr uint8_t ACK_UNKNOWN = 0xFF;

// ── 云台角度模式标志位 ────────────────────────────────────
constexpr uint8_t GIMBAL_MODE_ABSOLUTE = 0x00;
constexpr uint8_t GIMBAL_MODE_RELATIVE = 0x01;

// ── 相机枚举 ──────────────────────────────────────────────
constexpr uint8_t CAM_MODE_PHOTO = 0x00;
constexpr uint8_t CAM_MODE_VIDEO = 0x01;

constexpr uint8_t CAM_RECORD_STOP  = 0x00;
constexpr uint8_t CAM_RECORD_START = 0x01;

constexpr uint8_t CAM_RES_1920X1080 = 0x01;
constexpr uint8_t CAM_RES_3840X2160 = 0x02;
constexpr uint8_t CAM_RES_2720X1530 = 0x03;

constexpr uint8_t CAM_FPS_24 = 0x01;
constexpr uint8_t CAM_FPS_25 = 0x02;
constexpr uint8_t CAM_FPS_30 = 0x03;
constexpr uint8_t CAM_FPS_48 = 0x04;
constexpr uint8_t CAM_FPS_50 = 0x05;
constexpr uint8_t CAM_FPS_60 = 0x06;

constexpr uint8_t CAM_LENS_WIDE     = 0x00;
constexpr uint8_t CAM_LENS_ZOOM     = 0x01;
constexpr uint8_t CAM_LENS_INFRARED = 0x02;

constexpr uint8_t CAM_ZOOM_SWITCH_ONLY    = 0x00;
constexpr uint8_t CAM_ZOOM_SWITCH_AND_SET = 0x01;

// ── 补光灯枚举 ────────────────────────────────────────────
constexpr uint8_t AUX_LIGHT_OFF  = 0x00;
constexpr uint8_t AUX_LIGHT_ON   = 0x01;
constexpr uint8_t AUX_LIGHT_AUTO = 0x02;

// ── VEL_CMD 载荷 (16 B，DJI 机体系) ───────────────────────
//   vx       : 前向 m/s        (正 = 前)
//   vy       : 右向 m/s        (正 = 右)
//   vz       : 上向 m/s        (正 = 上)
//   yaw_rate : 偏航角速度 deg/s (正 = 顺时针俯视 / 右转)
#pragma pack(push, 1)
struct VelPayload {
    float vx;
    float vy;
    float vz;
    float yaw_rate;
};
#pragma pack(pop)
static_assert(sizeof(VelPayload) == 16, "VelPayload must be 16 bytes");

// ── GIMBAL_YAW_FOLLOW 载荷 (8 B) ──────────────────────────
#pragma pack(push, 1)
struct GimbalYawFollowPayload {
    float pitch;  // deg
    float roll;   // deg
};
#pragma pack(pop)
static_assert(sizeof(GimbalYawFollowPayload) == 8, "GimbalYawFollowPayload must be 8 bytes");

// ── GIMBAL_ANGLE 载荷 (18 B) ──────────────────────────────
#pragma pack(push, 1)
struct GimbalAnglePayload {
    uint8_t mode;
    uint8_t reserved;
    float   pitch;
    float   roll;
    float   yaw;
    float   duration;
};
#pragma pack(pop)
static_assert(sizeof(GimbalAnglePayload) == 18, "GimbalAnglePayload must be 18 bytes");

// ── 相机载荷 ──────────────────────────────────────────────
#pragma pack(push, 1)
struct CamModePayload     { uint8_t mode; };
struct CamRecordPayload   { uint8_t action; };
struct CamVideoCfgPayload { uint8_t resolution; uint8_t frame_rate; };
struct CamZoomPayload {
    uint8_t lens;
    uint8_t action;
    uint8_t reserved0;
    uint8_t reserved1;
    float   ratio;
};
#pragma pack(pop)
static_assert(sizeof(CamModePayload)     == 1, "CamModePayload must be 1 byte");
static_assert(sizeof(CamRecordPayload)   == 1, "CamRecordPayload must be 1 byte");
static_assert(sizeof(CamVideoCfgPayload) == 2, "CamVideoCfgPayload must be 2 bytes");
static_assert(sizeof(CamZoomPayload)     == 8, "CamZoomPayload must be 8 bytes");

// ── 补光灯载荷 (1 B) ──────────────────────────────────────
#pragma pack(push, 1)
struct AuxLightPayload { uint8_t mode; };
#pragma pack(pop)
static_assert(sizeof(AuxLightPayload) == 1, "AuxLightPayload must be 1 byte");

// ── 帧编码 ────────────────────────────────────────────────
inline uint8_t encode_simple(uint8_t cmd, uint8_t* buf, uint8_t buf_len) {
    if (buf_len < 4) return 0;
    buf[0] = FRAME_HEADER;
    buf[1] = cmd;
    buf[2] = 0;
    buf[3] = buf[0] ^ buf[1] ^ buf[2];
    return 4;
}

inline uint8_t encode_payload(uint8_t cmd,
                              const void* data, uint8_t data_len,
                              uint8_t* buf, uint8_t buf_len) {
    if (buf == nullptr || data == nullptr) return 0;
    if (buf_len < static_cast<uint16_t>(4 + data_len)) return 0;
    buf[0] = FRAME_HEADER;
    buf[1] = cmd;
    buf[2] = data_len;
    std::memcpy(buf + 3, data, data_len);
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(3 + data_len); ++i) xor_val ^= buf[i];
    buf[3 + data_len] = xor_val;
    return static_cast<uint8_t>(4 + data_len);
}

inline uint8_t encode_vel(const VelPayload& vel, uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_VEL, &vel, sizeof(VelPayload), buf, buf_len);
}

inline uint8_t encode_gimbal_yaw_follow(const GimbalYawFollowPayload& p,
                                        uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_GIMBAL_YAW_FOLLOW, &p,
                          sizeof(GimbalYawFollowPayload), buf, buf_len);
}

inline uint8_t encode_gimbal_angle(const GimbalAnglePayload& p,
                                   uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_GIMBAL_ANGLE, &p,
                          sizeof(GimbalAnglePayload), buf, buf_len);
}

inline uint8_t encode_cam_mode(const CamModePayload& p,
                               uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_MODE, &p, sizeof(CamModePayload), buf, buf_len);
}

inline uint8_t encode_cam_shoot(uint8_t* buf, uint8_t buf_len) {
    return encode_simple(CMD_CAM_SHOOT, buf, buf_len);
}

inline uint8_t encode_cam_record(const CamRecordPayload& p,
                                 uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_RECORD, &p, sizeof(CamRecordPayload), buf, buf_len);
}

inline uint8_t encode_cam_video_cfg(const CamVideoCfgPayload& p,
                                    uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_VIDEO_CFG, &p, sizeof(CamVideoCfgPayload), buf, buf_len);
}

inline uint8_t encode_cam_zoom(const CamZoomPayload& p,
                               uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_CAM_ZOOM, &p, sizeof(CamZoomPayload), buf, buf_len);
}

inline uint8_t encode_aux_light(const AuxLightPayload& p,
                                uint8_t* buf, uint8_t buf_len) {
    return encode_payload(CMD_AUX_LIGHT, &p, sizeof(AuxLightPayload), buf, buf_len);
}

// ── 帧解码 ────────────────────────────────────────────────
struct Frame {
    bool    valid   = false;
    uint8_t cmd     = 0;
    uint8_t len     = 0;
    uint8_t payload[MAX_PAYLOAD_LEN] = {};
};

inline Frame decode(const uint8_t* data, uint16_t data_len) {
    Frame f;
    if (data == nullptr || data_len < 4) return f;
    if (data[0] != FRAME_HEADER) return f;
    const uint8_t len = data[2];
    if (data_len < static_cast<uint16_t>(4 + len)) return f;
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(3 + len); ++i) xor_val ^= data[i];
    if (xor_val != data[3 + len]) return f;
    f.valid = true;
    f.cmd   = data[1];
    f.len   = len;
    if (len > 0) std::memcpy(f.payload, data + 3, len);
    return f;
}

}  // namespace drone_comm


// =============================================================================
// ControllerHardware 类
// =============================================================================
namespace indooruav_controller {

class ControllerHardware {
public:
    explicit ControllerHardware(ros::NodeHandle& node_handle);
    ~ControllerHardware();

    ControllerHardware(const ControllerHardware&)            = delete;
    ControllerHardware& operator=(const ControllerHardware&) = delete;

private:
    // ── 初始化 ─────────────────────────────────────────────
    void LoadParameters();
    void InitializePsdkChannel();
    void AdvertiseServiceServers();
    void CreateServiceClients();
    void CreateSubscribersAndTimers();

    // ── PSDK 接收回调：C 风格静态分发 + 实例方法 ──────────
    static T_DjiReturnCode StaticOnRecvFromMsdk(const uint8_t* data, uint16_t len);
    void OnRecvFromMsdk(const uint8_t* data, uint16_t len);

    // ── 上行通知 (本节点 → 上游 Empty 服务) ───────────────
    void NotifyTakeoffComplete();
    void NotifyLandComplete();
    void NotifyHoverComplete();

    // ── cmd_vel 订阅 + 10Hz 发送定时器 ────────────────────
    void OnCmdVel(const geometry_msgs::Twist::ConstPtr& message);
    void OnVelSendTimer(const ros::TimerEvent& event);

    // ── 服务回调：飞控 ────────────────────────────────────
    bool TakeoffCallback(std_srvs::Empty::Request& request,
                         std_srvs::Empty::Response& response);
    bool LandCallback(std_srvs::Empty::Request& request,
                      std_srvs::Empty::Response& response);
    bool HoverCallback(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response);

    // ── 服务回调：补光灯 ──────────────────────────────────
    bool NotifyUavOpenLightCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);
    bool NotifyUavCloseLightCallback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);
    bool NotifyUavAutoLightCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);

    // ── 服务回调：相机 ────────────────────────────────────
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
    bool UploadMissionPhotosFromSdCallback(indooruav_msgs::TransferMissionMedia::Request& request,
                                           indooruav_msgs::TransferMissionMedia::Response& response);
    bool CameraVideoConfigCallback(indooruav_msgs::CameraVideoConfig::Request& request,
                                   indooruav_msgs::CameraVideoConfig::Response& response);
    bool CameraZoomCallback(indooruav_msgs::CameraZoom::Request& request,
                            indooruav_msgs::CameraZoom::Response& response);

    // ── 服务回调：云台 ────────────────────────────────────
    bool GimbalYawFollowCallback(indooruav_msgs::GimbalYawFollow::Request& request,
                                 indooruav_msgs::GimbalYawFollow::Response& response);
    bool GimbalAngleCallback(indooruav_msgs::GimbalAngle::Request& request,
                             indooruav_msgs::GimbalAngle::Response& response);

    // ── 工具方法 ──────────────────────────────────────────
    bool SendFrame(const uint8_t* buf, uint8_t len);
    bool CallEmptyService(ros::ServiceClient& client,
                          const std::string& service_name,
                          const char* service_label);
    bool InitializeCameraManager();
    bool EnsureCameraManagerReady();
    bool IsSupportedMediaType(E_DjiCameraMediaFileType media_type) const;
    bool UploadMediaFile(const T_DjiCameraManagerFileListInfo& file_info,
                         const std::string& airline_key,
                         const std::string& detect_time_cur);
    bool UploadDownloadedBytes(const T_DjiCameraManagerFileListInfo& file_info,
                               const std::vector<uint8_t>& file_bytes,
                               const std::string& airline_key,
                               const std::string& detect_time_cur);
    bool DownloadFileToBuffer(uint32_t file_index,
                              std::vector<uint8_t>* buffer,
                              std::string* error_message);
    bool WaitForDownloadResult(std::string* error_message);
    bool IsMediaInMissionWindow(const T_DjiCameraManagerFileListInfo& file_info,
                                std::time_t mission_start_unix,
                                std::time_t workflow_end_unix) const;
    std::time_t ParseDetectTimeCur(const std::string& detect_time_cur) const;
    std::time_t FileCreateTimeToUnix(const T_DjiCameraManagerFileCreateTime& create_time) const;
    std::string DetectMediaExtension(const T_DjiCameraManagerFileListInfo& file_info) const;
    void SetVelForwardingEnabled(bool enabled);

    static T_DjiReturnCode StaticDownloadFileDataCallback(T_DjiDownloadFilePacketInfo packet_info,
                                                          const uint8_t* data,
                                                          uint16_t data_len);
    T_DjiReturnCode OnDownloadFileData(T_DjiDownloadFilePacketInfo packet_info,
                                       const uint8_t* data,
                                       uint16_t data_len);

private:
    // PSDK 回调单例指针 (DjiLowSpeedDataChannel 回调签名为 C 风格无 user_data，
    // 因此必须用静态指针分发到实例)
    static ControllerHardware* instance_;

    ros::NodeHandle& node_handle_;

    // ── 参数 ──────────────────────────────────────────────
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
    std::string upload_mission_photos_from_sd_service_name_;
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
    std::string http_upload_image_bytes_service_name_;

    std::string cmd_vel_topic_;
    double      vel_send_rate_hz_ = 10.0;
    int         media_camera_mount_position_ = -1;
    double      media_time_tolerance_sec_ = 5.0;
    double      media_file_wait_timeout_sec_ = 60.0;

    // ── ROS 通信对象 ──────────────────────────────────────
    // 服务端
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
    ros::ServiceServer upload_mission_photos_from_sd_service_server_;
    ros::ServiceServer camera_video_config_service_server_;
    ros::ServiceServer camera_zoom_service_server_;
    ros::ServiceServer gimbal_yaw_follow_service_server_;
    ros::ServiceServer gimbal_angle_service_server_;

    // 客户端
    ros::ServiceClient takeoff_complete_client_;
    ros::ServiceClient land_complete_client_;
    ros::ServiceClient hover_complete_client_;

    ros::ServiceClient waypoint_record_client_;
    ros::ServiceClient waypoint_save_client_;
    ros::ServiceClient waypoint_clear_client_;
    ros::ServiceClient upload_image_bytes_client_;

    // 订阅 + 定时器
    ros::Subscriber cmd_vel_subscriber_;
    ros::Timer      vel_send_timer_;

    // ── 共享状态 ──────────────────────────────────────────
    // vel_mutex_ 守护 latest_vel_ / vel_updated_ / vel_forwarding_enabled_
    // (一把锁同时管理速度数据和门控开关，避免门关闭时竞态导致 stale vel 漏出)
    std::mutex             vel_mutex_;
    drone_comm::VelPayload latest_vel_{0.0f, 0.0f, 0.0f, 0.0f};
    bool                   vel_updated_              = false;
    bool                   vel_forwarding_enabled_   = false;

    // tx_mutex_ 串行化所有 PSDK 低速通道的 SendData 调用
    std::mutex tx_mutex_;

    // ACK 计数 (仅日志用)
    std::atomic<uint32_t> ack_count_{0};
    std::atomic<bool> camera_manager_initialized_{false};
    std::atomic<bool> sd_transfer_running_{false};
    std::mutex download_mutex_;
    std::condition_variable download_cv_;
    uint32_t downloading_file_index_ = 0;
    bool download_in_progress_ = false;
    bool download_finished_ = false;
    bool download_success_ = false;
    std::vector<uint8_t> download_buffer_;
    std::string download_error_message_;
    std::set<uint32_t> active_download_file_indices_;
};

}  // namespace indooruav_controller
