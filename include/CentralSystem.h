#ifndef CENTRAL_SYSTEM_H
#define CENTRAL_SYSTEM_H

#include "common_header.h"

#include "opencv2/opencv.hpp"
#include "DogStatus.h"
#include "Camera.h"
#include "config/config.h"
#include "Lidar.h"
#include "VisualOdometry.h"
#include "MotorControlSystem.h"
#include "Node.h"

class CentralSystem {
  public:
    CentralSystem(bool mode, bool is_use_lidar);
    void startProgram();
    void printfSystemInformation(bool mode);

  private:
    // Trajectory Map 
    cv::Mat traj_display;

    // for Camera 
    std::shared_ptr<Camera> camera_; // camera sensor 

    // for LiDAR 
    std::shared_ptr<Lidar> lidar_; // lidar sensor 
    void transformTheta(const float theta, double* output_array);

    // for DogStatus
    std::shared_ptr<DogStatus> dog_status_; // status save space 

    // for Visual odometry
    std::shared_ptr<VisualOdometry> vo_;

    // for Legs 
    std::shared_ptr<MotorControlSystem> motor_control_system_;
    
    // for System 
    bool system_status_ = false;
    bool use_lidar_ = false;

    void pathConstruction( cv::Point starting_coord, cv::Point dst_coord );

    // thread 
    // std::thread camera_capture_thread_;
    std::thread compute_traj_thread_;
    std::thread scan_lidar_thread_;
    std::thread communication_system_thread_;

    static void cameraCaptureThread(std::shared_ptr<Camera> camera, std::shared_ptr<DogStatus> dog_status);
    static void computeTrajectoryThread(std::shared_ptr<Camera> camera, std::shared_ptr<VisualOdometry> vo, std::shared_ptr<DogStatus> dog_status);
    static void scanLidarThread(std::shared_ptr<Lidar> lidar, std::shared_ptr<DogStatus> dog_status);
    static void communicationSystemThread(std::shared_ptr<MotorControlSystem> motor_control_system, std::shared_ptr<DogStatus> dog_status);    
};

#endif
