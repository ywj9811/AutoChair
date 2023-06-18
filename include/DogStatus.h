#ifndef DOG_STATUS_H
#define DOG_STATUS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// for lidar 
#include "rplidar.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

class DogStatus {
  public:
    DogStatus();

    // for camera  
    cv::Mat getCurrentFrame();
    void setCurrentFrame(cv::Mat current_frame);

    cv::Mat getCurrentFrame(int a);

    void pushFrameBuffer(cv::Mat current_frame);
    cv::Mat popFrameBuffer();

    // set system status 
    bool getSystemStatus();
    void setSystemStatus(bool flag);
    
    // Trajectory data
    void setTrajData(std::vector<cv::Point2f> current_traj_data);
    std::vector<cv::Point2f> getTrajData();

    void setCurrentLocation(cv::Point2d current_location);
    cv::Point2d getCurrentLocation();

    // LiDAR data
    void setScanData(std::vector<sl_lidar_response_measurement_node_hq_t> current_lidar_scan_data, size_t count);
    std::vector<sl_lidar_response_measurement_node_hq_t> getScanData();

  private:
    cv::Mat curr_frame_;
    cv::Mat prev_frame_;
    std::mutex current_frame_mutex_;

    // for VO 
    std::queue<cv::Mat> frame_buffer_;
    std::mutex frame_buffer_mutex_;
    
    bool is_working_;
    std::mutex is_working_mutex_;

    std::mutex current_traj_mutex_;
    std::vector<cv::Point2f> current_traj_;

    cv::Point2d current_location_;
    std::mutex current_location_mutex_;

    std::mutex current_scan_data_mutex_;
    std::vector<sl_lidar_response_measurement_node_hq_t> current_scan_data_;
};

#endif // DOG_STATUS_H
