/**
 * @file  psdk_state_adapter.cpp
 * @brief PSDK state reader and optional media uploader implementation.
 */
#include "indooruav_controller/psdk_state_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace indooruav_controller {

namespace {

constexpr char kDefaultHttpDeviceStateTopic[] =
    "/indooruav_controller/http/device_state";
constexpr char kDefaultHttpUploadImageBytesService[] =
    "/indooruav_http/upload_image_bytes";
constexpr char kDefaultHttpSendErrorDataService[] =
    "/indooruav_http/send_error_data";
constexpr char kDefaultUploadMissionPhotosFromSdService[] =
    "indooruav_controller/psdk_state_adapter/upload_mission_photos_from_sd";

}  // namespace

PsdkStateAdapter* PsdkStateAdapter::instance_ = nullptr;

PsdkStateAdapter::PsdkStateAdapter(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
    if (instance_ != nullptr) {
        throw std::runtime_error(
            "PsdkStateAdapter is a singleton; only one instance is allowed.");
    }
    instance_ = this;

    LoadParameters();
    InitializeFcSubscription();
    if (enable_media_upload_) {
        InitializeCameraManager();
    }
    CreateRosInterfaces();
    PublishHttpDeviceState();

    ROS_INFO("[PsdkStateAdapter] initialized.");
}

PsdkStateAdapter::~PsdkStateAdapter() {
    ShutdownCameraManager();
    ShutdownFcSubscription();
    instance_ = nullptr;
}

void PsdkStateAdapter::LoadParameters() {
    node_handle_.param<std::string>("/indooruav_controller/topics/http_device_state",
                                    http_device_state_topic_,
                                    kDefaultHttpDeviceStateTopic);
    node_handle_.param<std::string>("/indooruav_controller/services/http_upload_image_bytes",
                                    http_upload_image_bytes_service_name_,
                                    kDefaultHttpUploadImageBytesService);
    node_handle_.param<std::string>("/indooruav_controller/services/http_send_error_data",
                                    http_send_error_data_service_name_,
                                    kDefaultHttpSendErrorDataService);
    node_handle_.param<std::string>("/indooruav_controller/services/upload_mission_photos_from_sd",
                                    upload_mission_photos_from_sd_service_name_,
                                    kDefaultUploadMissionPhotosFromSdService);

    node_handle_.param<int>("/indooruav_controller/parameters/media_camera_mount_position",
                            media_camera_mount_position_,
                            -1);
    node_handle_.param<double>("/indooruav_controller/parameters/media_time_tolerance_sec",
                               media_time_tolerance_sec_,
                               5.0);
    node_handle_.param<double>("/indooruav_controller/parameters/media_file_wait_timeout_sec",
                               media_file_wait_timeout_sec_,
                               60.0);
    node_handle_.param<bool>("/indooruav_controller/parameters/enable_media_upload",
                             enable_media_upload_,
                             false);
}

void PsdkStateAdapter::InitializeFcSubscription() {
    const T_DjiReturnCode init_ret = DjiFcSubscription_Init();
    if (init_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_WARN("[PsdkStateAdapter] DjiFcSubscription_Init failed: 0x%08llX",
                 static_cast<unsigned long long>(init_ret));
        return;
    }
    fc_subscription_initialized_ = true;

    const struct {
        E_DjiFcSubscriptionTopic topic;
        E_DjiDataSubscriptionTopicFreq frequency;
        const char* name;
    } topics[] = {
        {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "status_flight"},
        {DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "rc_with_flag_data"},
        {DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "battery_info"},
        {DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "battery_single_info_1"}
    };

    bool all_ok = true;
    for (const auto& item : topics) {
        const T_DjiReturnCode ret =
            DjiFcSubscription_SubscribeTopic(item.topic, item.frequency, nullptr);
        if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            ROS_WARN("[PsdkStateAdapter] Failed to subscribe FC topic %s: 0x%08llX",
                     item.name,
                     static_cast<unsigned long long>(ret));
            all_ok = false;
        }
    }

    fc_topics_subscribed_ = all_ok;
}

void PsdkStateAdapter::ShutdownFcSubscription() {
    if (!fc_subscription_initialized_) {
        return;
    }

    const E_DjiFcSubscriptionTopic topics[] = {
        DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
        DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
        DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT
    };
    for (const E_DjiFcSubscriptionTopic topic : topics) {
        DjiFcSubscription_UnSubscribeTopic(topic);
    }
    DjiFcSubscription_DeInit();
    fc_subscription_initialized_ = false;
    fc_topics_subscribed_ = false;
}

bool PsdkStateAdapter::InitializeCameraManager() {
    const T_DjiReturnCode ret = DjiCameraManager_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_WARN("[PsdkStateAdapter] DjiCameraManager_Init failed: 0x%08llX",
                 static_cast<unsigned long long>(ret));
        camera_manager_initialized_.store(false);
        return false;
    }

    camera_manager_initialized_.store(true);
    return true;
}

void PsdkStateAdapter::ShutdownCameraManager() {
    if (!camera_manager_initialized_.load()) {
        return;
    }
    DjiCameraManager_DeInit();
    camera_manager_initialized_.store(false);
}

void PsdkStateAdapter::CreateRosInterfaces() {
    http_device_state_publisher_ =
        node_handle_.advertise<std_msgs::String>(http_device_state_topic_, 10, true);
    telemetry_sync_timer_ = node_handle_.createTimer(
        ros::Duration(1.0), &PsdkStateAdapter::TelemetrySyncTimerCallback, this);

    if (enable_media_upload_) {
        upload_image_bytes_client_ =
            node_handle_.serviceClient<indooruav_msgs::UploadImageBytes>(
                http_upload_image_bytes_service_name_);
        send_error_data_client_ =
            node_handle_.serviceClient<indooruav_http::SendErrorData>(
                http_send_error_data_service_name_);
        upload_mission_photos_from_sd_service_server_ = node_handle_.advertiseService(
            upload_mission_photos_from_sd_service_name_,
            &PsdkStateAdapter::UploadMissionPhotosFromSdCallback,
            this);
    }
}

void PsdkStateAdapter::PublishHttpDeviceState() {
    if (!http_device_state_publisher_) {
        return;
    }

    int uav_state = 0;
    int control_state = 0;
    double battery_temp = 0.0;
    double battery_soc = 0.0;
    double battery_volt = 0.0;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        uav_state = fc_telemetry_received_ ? 1 : 0;
        control_state = rc_logic_connected_ ? 1 : 0;
        battery_temp = battery_temperature_c_;
        battery_soc = battery_soc_percent_;
        battery_volt = battery_voltage_v_;
    }

    nlohmann::json payload = {
        {"uavState", uav_state},
        {"controlState", control_state},
        {"controlSoc", 0.0},
        {"controlRssi", 0.0},
        {"batteryTemp", battery_temp},
        {"batterySoc", battery_soc},
        {"batteryRssi", 0.0},
        {"batteryVolt", battery_volt},
        {"batteryCycleNum", 0},
    };

    std_msgs::String msg;
    msg.data = payload.dump();
    http_device_state_publisher_.publish(msg);
}

void PsdkStateAdapter::TelemetrySyncTimerCallback(const ros::TimerEvent& /*event*/) {
    if (!fc_subscription_initialized_ || !fc_topics_subscribed_) {
        return;
    }

    T_DjiDataTimestamp timestamp{};
    T_DjiFcSubscriptionFlightStatus flight_status = 0;
    T_DjiFcSubscriptionRCWithFlagData rc_data{};
    T_DjiFcSubscriptionWholeBatteryInfo battery_info{};
    T_DjiFcSubscriptionSingleBatteryInfo battery_single_info{};

    bool should_publish = false;
    bool any_data_received = false;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);

        if (DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                reinterpret_cast<uint8_t*>(&flight_status),
                sizeof(flight_status),
                &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            any_data_received = true;
            should_publish = true;
        }

        if (DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
                reinterpret_cast<uint8_t*>(&rc_data),
                sizeof(rc_data),
                &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            any_data_received = true;
            rc_logic_connected_ = rc_data.flag.logicConnected != 0;
            should_publish = true;
        }

        if (DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                reinterpret_cast<uint8_t*>(&battery_info),
                sizeof(battery_info),
                &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            any_data_received = true;
            battery_soc_percent_ = static_cast<double>(battery_info.percentage);
            battery_voltage_v_ = static_cast<double>(battery_info.voltage) / 1000.0;
            battery_info_valid_ = true;
            should_publish = true;
        }

        if (DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                reinterpret_cast<uint8_t*>(&battery_single_info),
                sizeof(battery_single_info),
                &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            any_data_received = true;
            battery_temperature_c_ =
                static_cast<double>(battery_single_info.batteryTemperature) / 10.0;
            if (!battery_info_valid_) {
                battery_soc_percent_ =
                    static_cast<double>(battery_single_info.batteryCapacityPercent);
                battery_voltage_v_ =
                    static_cast<double>(battery_single_info.currentVoltage) / 1000.0;
            }
            should_publish = true;
        }

        if (any_data_received) {
            last_fc_data_time_ = ros::Time::now();
            fc_telemetry_received_ = true;
        } else if (!last_fc_data_time_.isZero() &&
                   (ros::Time::now() - last_fc_data_time_).toSec() > 3.0) {
            if (fc_telemetry_received_) {
                fc_telemetry_received_ = false;
                should_publish = true;
            }
        }
    }

    if (should_publish) {
        PublishHttpDeviceState();
    }
}

bool PsdkStateAdapter::UploadMissionPhotosFromSdCallback(
    indooruav_msgs::TransferMissionMedia::Request& request,
    indooruav_msgs::TransferMissionMedia::Response& response) {
    response.result_code = 1;
    response.matched_count = 0;
    response.uploaded_count = 0;
    response.failed_count = 0;

    if (!enable_media_upload_) {
        response.result_code = 3;
        return true;
    }

    if (request.airline_key.empty() || request.detect_time_cur.empty()) {
        response.result_code = 3;
        return true;
    }

    if (media_camera_mount_position_ < 0 ||
        media_camera_mount_position_ == DJI_MOUNT_POSITION_UNKNOWN) {
        response.result_code = 3;
        return true;
    }

    if (sd_transfer_running_.exchange(true)) {
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
        ReportHttpError(4, "camera_manager_init_failed");
        response.result_code = 2;
        return true;
    }

    const E_DjiMountPosition mount_position =
        static_cast<E_DjiMountPosition>(media_camera_mount_position_);
    const T_DjiReturnCode reg_ret =
        DjiCameraManager_RegDownloadFileDataCallback(
            mount_position, &PsdkStateAdapter::StaticDownloadFileDataCallback);
    if (reg_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ReportHttpError(4, "register_media_download_callback_failed");
        response.result_code = 2;
        return true;
    }

    const T_DjiReturnCode rights_ret = DjiCameraManager_ObtainDownloaderRights(mount_position);
    if (rights_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ReportHttpError(4, "obtain_downloader_rights_failed");
        response.result_code = 2;
        return true;
    }

    struct RightsGuard {
        E_DjiMountPosition mount_position;
        ~RightsGuard() {
            DjiCameraManager_ReleaseDownloaderRights(mount_position);
        }
    } rights_guard{mount_position};

    T_DjiCameraManagerFileList file_list{};
    const T_DjiReturnCode list_ret = DjiCameraManager_DownloadFileList(mount_position, &file_list);
    if (list_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ReportHttpError(4, "download_camera_file_list_failed");
        response.result_code = 2;
        return true;
    }

    const std::time_t mission_start_unix = ParseDetectTimeCur(request.detect_time_cur);
    if (mission_start_unix <= 0) {
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
    return true;
}

bool PsdkStateAdapter::ReportHttpError(int error_type, const std::string& error_info) {
    if (!enable_media_upload_) {
        return false;
    }

    indooruav_http::SendErrorData service;
    service.request.error_type = error_type;
    service.request.error_info = error_info;
    if (!send_error_data_client_.call(service)) {
        return false;
    }
    return service.response.result_code == 1;
}

bool PsdkStateAdapter::EnsureCameraManagerReady() {
    if (camera_manager_initialized_.load()) {
        return true;
    }
    return InitializeCameraManager();
}

bool PsdkStateAdapter::IsSupportedMediaType(E_DjiCameraMediaFileType media_type) const {
    return media_type == DJI_CAMERA_FILE_TYPE_JPEG ||
           media_type == DJI_CAMERA_FILE_TYPE_DNG ||
           media_type == DJI_CAMERA_FILE_TYPE_TIFF;
}

bool PsdkStateAdapter::UploadMediaFile(const T_DjiCameraManagerFileListInfo& file_info,
                                       const std::string& airline_key,
                                       const std::string& detect_time_cur) {
    std::vector<uint8_t> file_bytes;
    std::string error_message;
    if (!DownloadFileToBuffer(file_info.fileIndex, &file_bytes, &error_message)) {
        return false;
    }
    return UploadDownloadedBytes(file_info, file_bytes, airline_key, detect_time_cur);
}

bool PsdkStateAdapter::UploadDownloadedBytes(const T_DjiCameraManagerFileListInfo& file_info,
                                             const std::vector<uint8_t>& file_bytes,
                                             const std::string& airline_key,
                                             const std::string& detect_time_cur) {
    if (file_bytes.empty()) {
        return false;
    }

    indooruav_msgs::UploadImageBytes service;
    service.request.airline_key = airline_key;
    service.request.detect_time_cur = detect_time_cur;
    service.request.source_name = file_info.fileName;
    service.request.image_extension = DetectMediaExtension(file_info);
    service.request.image_bytes = file_bytes;
    if (!upload_image_bytes_client_.call(service)) {
        return false;
    }
    return service.response.result_code == 1;
}

bool PsdkStateAdapter::DownloadFileToBuffer(uint32_t file_index,
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

bool PsdkStateAdapter::WaitForDownloadResult(std::string* error_message) {
    if (error_message == nullptr) {
        return false;
    }

    std::unique_lock<std::mutex> lock(download_mutex_);
    const bool finished = download_cv_.wait_for(
        lock,
        std::chrono::milliseconds(
            static_cast<int>(std::max(1.0, media_file_wait_timeout_sec_ * 1000.0))),
        [this]() { return download_finished_; });

    if (!finished) {
        active_download_file_indices_.erase(downloading_file_index_);
        download_in_progress_ = false;
        *error_message = "download timed out";
        return false;
    }

    if (!download_success_) {
        download_in_progress_ = false;
        *error_message = download_error_message_.empty() ? "download failed"
                                                         : download_error_message_;
        return false;
    }

    *error_message = "";
    return true;
}

bool PsdkStateAdapter::IsMediaInMissionWindow(const T_DjiCameraManagerFileListInfo& file_info,
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

std::time_t PsdkStateAdapter::ParseDetectTimeCur(const std::string& detect_time_cur) const {
    std::tm tm_value{};
    std::istringstream ss(detect_time_cur);
    ss >> std::get_time(&tm_value, "%Y%m%d%H%M%S");
    if (ss.fail()) {
        return static_cast<std::time_t>(0);
    }
    tm_value.tm_isdst = -1;
    return std::mktime(&tm_value);
}

std::time_t PsdkStateAdapter::FileCreateTimeToUnix(
    const T_DjiCameraManagerFileCreateTime& create_time) const {
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

std::string PsdkStateAdapter::DetectMediaExtension(
    const T_DjiCameraManagerFileListInfo& file_info) const {
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

T_DjiReturnCode PsdkStateAdapter::StaticDownloadFileDataCallback(
    T_DjiDownloadFilePacketInfo packet_info,
    const uint8_t* data,
    uint16_t data_len) {
    if (instance_ != nullptr) {
        return instance_->OnDownloadFileData(packet_info, data, data_len);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode PsdkStateAdapter::OnDownloadFileData(T_DjiDownloadFilePacketInfo packet_info,
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

}  // namespace indooruav_controller
