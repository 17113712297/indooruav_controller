/**
 * @file  psdk_state_adapter.hpp
 * @brief PSDK state reader and optional media uploader.
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <dji_camera_manager.h>
#include <dji_fc_subscription.h>
#include <dji_typedef.h>

#include <indooruav_http/SendErrorData.h>
#include <indooruav_msgs/TransferMissionMedia.h>
#include <indooruav_msgs/UploadImageBytes.h>

namespace indooruav_controller {

class PsdkStateAdapter {
public:
    explicit PsdkStateAdapter(ros::NodeHandle& node_handle);
    ~PsdkStateAdapter();

    PsdkStateAdapter(const PsdkStateAdapter&) = delete;
    PsdkStateAdapter& operator=(const PsdkStateAdapter&) = delete;

private:
    void LoadParameters();
    void InitializeFcSubscription();
    void ShutdownFcSubscription();
    bool InitializeCameraManager();
    void ShutdownCameraManager();
    void CreateRosInterfaces();
    void PublishHttpDeviceState();
    void TelemetrySyncTimerCallback(const ros::TimerEvent& event);

    bool UploadMissionPhotosFromSdCallback(indooruav_msgs::TransferMissionMedia::Request& request,
                                           indooruav_msgs::TransferMissionMedia::Response& response);
    bool ReportHttpError(int error_type, const std::string& error_info);
    bool EnsureCameraManagerReady();
    bool IsSupportedMediaType(E_DjiCameraMediaFileType media_type) const;
    bool UploadMediaFile(const T_DjiCameraManagerFileListInfo& file_info,gg
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

    static T_DjiReturnCode StaticDownloadFileDataCallback(T_DjiDownloadFilePacketInfo packet_info,
                                                          const uint8_t* data,
                                                          uint16_t data_len);
    T_DjiReturnCode OnDownloadFileData(T_DjiDownloadFilePacketInfo packet_info,
                                       const uint8_t* data,
                                       uint16_t data_len);

private:
    static PsdkStateAdapter* instance_;

    ros::NodeHandle& node_handle_;

    std::string http_device_state_topic_;
    std::string http_upload_image_bytes_service_name_;
    std::string http_send_error_data_service_name_;
    std::string upload_mission_photos_from_sd_service_name_;

    double media_time_tolerance_sec_ = 5.0;
    double media_file_wait_timeout_sec_ = 60.0;
    int media_camera_mount_position_ = -1;
    bool enable_media_upload_ = false;

    ros::Publisher http_device_state_publisher_;
    ros::Timer telemetry_sync_timer_;
    ros::ServiceClient upload_image_bytes_client_;
    ros::ServiceClient send_error_data_client_;
    ros::ServiceServer upload_mission_photos_from_sd_service_server_;

    std::mutex telemetry_mutex_;
    bool fc_subscription_initialized_ = false;
    bool fc_topics_subscribed_ = false;
    bool fc_telemetry_received_ = true;
    bool rc_logic_connected_ = false;
    bool battery_info_valid_ = false;
    double battery_temperature_c_ = 0.0;
    double battery_soc_percent_ = 0.0;
    double battery_voltage_v_ = 0.0;
    ros::Time last_fc_data_time_;

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
