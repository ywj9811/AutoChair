/**
 * @file DogStatus.cpp
 * @author gusrlLee (gusrlLee@github.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
// -------------------------------------------------------------------------------
#include "DogStatus.h"

/**
 * @brief Construct a new Dog Status:: Dog Status object default system status : false
 * 
 */
DogStatus::DogStatus() {
    is_working_ = false;
}

/**
 * @brief get current frame in dog_status.
 * 
 * @return cv::Mat : current_frame in dog_status 
 */
cv::Mat DogStatus::getCurrentFrame() {
    std::lock_guard<std::mutex> lock(current_frame_mutex_);
    return curr_frame_.clone();
}

cv::Mat DogStatus::getCurrentFrame(int a) {
    std::lock_guard<std::mutex> lock(current_frame_mutex_);
    return curr_frame_.clone();
}

/**
 * @brief set current frame in dog status. 
 * @param current_frame : current_frame received from camera or video.
 */
void DogStatus::setCurrentFrame(cv::Mat current_frame) {    
    std::lock_guard<std::mutex> lock(current_frame_mutex_);
    curr_frame_ = current_frame;
}

void DogStatus::pushFrameBuffer(cv::Mat current_frame) {
    std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
    frame_buffer_.push(current_frame);
}

cv::Mat DogStatus::popFrameBuffer() {
    std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
    cv::Mat current_frame;
    if (!frame_buffer_.empty()) {
        current_frame = frame_buffer_.front();
        frame_buffer_.pop();
    }
    return current_frame;
}

/**
 * @brief get status or mode in dog_status. 
 * 
 * @return true : starting.
 * @return false : waiting or exit.
 */
bool DogStatus::getSystemStatus() {
    std::lock_guard<std::mutex> lock(is_working_mutex_);
    return is_working_;
}

/**
 * @brief set current status in dog status 
 * 
 * @param status : flag to set system status
 */
void DogStatus::setSystemStatus(bool status) {
    std::lock_guard<std::mutex> lock(is_working_mutex_);
    is_working_ = status;
}

/**
 * @brief set current trajectory data in dog status 
 * 
 * @param current_traj_data : data that saved current trajactory data vector  
 */
void DogStatus::setTrajData(std::vector<cv::Point2f> current_traj_data) { 
    std::lock_guard<std::mutex> lock(current_traj_mutex_);
    this->current_traj_ = current_traj_data;
}


void DogStatus::setCurrentLocation(cv::Point2d current_location) {
    std::lock_guard<std::mutex> lock(current_location_mutex_);
    current_location_ = current_location;
}

cv::Point2d DogStatus::getCurrentLocation() {
    std::lock_guard<std::mutex> lock(current_location_mutex_);
    return current_location_;
}

/**
 * @brief get current trajectory data in dog status 
 * 
 * @return std::vector<cv::Point2f> : trajectory data vector 
 */
std::vector<cv::Point2f> DogStatus::getTrajData() {
    std::lock_guard<std::mutex> lock(current_traj_mutex_);
    return current_traj_;
}

/**
 * @brief set current LiDAR data in dog status. 
 * 
 * @param current_lidar_scan_data : scaned LiDAR data 
 * @param count : size of current_scan_data 
 * @remark Before you use this function, you need to trasform array to vector.
 */
void DogStatus::setScanData(std::vector<sl_lidar_response_measurement_node_hq_t> current_lidar_scan_data, size_t count) {
    std::lock_guard<std::mutex> lock(current_scan_data_mutex_);
    this->current_scan_data_ = current_lidar_scan_data;

}

/**
 * @brief get current LiDAR data in dog status
 * 
 * @return std::vector<sl_lidar_response_measurement_node_hq_t> : LiDAR data 
 */
std::vector<sl_lidar_response_measurement_node_hq_t> DogStatus::getScanData() {
    std::lock_guard<std::mutex> lock(current_scan_data_mutex_);
    return current_scan_data_; 
}

