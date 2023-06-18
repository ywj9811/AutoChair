#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_header.h"
#include "Camera.h"

class VisualOdometry 
{
    public:
        VisualOdometry(std::shared_ptr<Camera> camera);

        void addFrame(cv::Mat frame);
        cv::Point2d getCurrentLocation();
        
    private:
        // use Optical Flow 
        std::vector<cv::Point2f> prev_points, curr_points;
        cv::Mat err;
        std::vector<uchar> status;

        cv::Mat curr_gray_frame;
        cv::Mat prev_gray_frame;

        cv::Point2d current_location;

        // this value, check ready prev_frame 
        bool is_ready = false;

        // temp calibration data 
        double focal_length;
        cv::Point2d principal_point;

        cv::Mat display = cv::Mat::zeros(1000, 1000, CV_8UC3);
        cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64F);

        int min_inlier_num = 100;

        // calibration Matrix 
        cv::Mat K; 
        cv::Mat inlier_mask;
        int count=0;

        void extractKeyPoints();
        void computeDescriptors();
        void poseEstimationPnP();
    
};

#endif // VISUAL_ODOMETRY_H
