#include "Camera.h"

Camera::Camera(std::string camera_path, double focal_length, cv::Point2d principal_point, bool system_mode) {
    // init
    camera_current_location_ = cv::Point2d(0, 0);
    camera_path_ = camera_path;
    system_mode_ = system_mode;

    // calibration data
    principal_point_ = principal_point;
    focal_length_ = focal_length;
}