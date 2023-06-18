/**
 * @file VisualOdometry.cpp
 * @author gusrlLee (gusrlLee@github.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "VisualOdometry.h"

/**
 * @brief Construct a new Visual Odometry:: Visual Odometry object
 * 
 */
VisualOdometry::VisualOdometry(std::shared_ptr<Camera> camera) {
    focal_length = camera->focalLength();
    principal_point = camera->principalPoints();
    
    // intrincis camera data  
    K = (cv::Mat_<double>(3, 3) << focal_length  , 0,             principal_point.x, 
                                   0,              focal_length,  principal_point.y,
                                   0,              0,             1);
    
}

/**
 * @brief This function computes frame that input argument, on our Visual odoemetry algorithm  
 * 
 * @param frame : current Frame  
 */
void VisualOdometry::addFrame(cv::Mat frame) {
    cv::Mat frame_buf;
    cv::cvtColor(frame, frame_buf, cv::COLOR_BGR2GRAY);
    if ( is_ready ) {
        // issue not good matches so connot compute traj 
        curr_gray_frame = frame_buf.clone();

        // compute Visual operation 
        extractKeyPoints();
        computeDescriptors();
        poseEstimationPnP();

        // save prev result information 
        prev_gray_frame = curr_gray_frame.clone();
    }

    else {
        // make prev_information
        prev_gray_frame = frame_buf.clone();
        // And transform ready_status 
        is_ready = true;
    }
}

/**
 * @brief Visual odoemtry에서 진행 된 location을 획득 하는 method
 * 
 * @return * cv::Point2d : current_dog_location 
 */
cv::Point2d VisualOdometry::getCurrentLocation() {
    return current_location;
}

/**
 * @brief feature detection between prev_frame and current_frame 
 * 
 * @return * void 
 */
void VisualOdometry::extractKeyPoints() {
    cv::goodFeaturesToTrack(prev_gray_frame, prev_points, 2000, 0.01, 10);
}

/**
 * @brief 찾아진 prev_keypoints 와 current_keypoints를 가지고 descriptor를 계산. 
 * 
 * @return * void 
 */
void VisualOdometry::computeDescriptors() {
    cv::calcOpticalFlowPyrLK(prev_gray_frame, curr_gray_frame, prev_points, curr_points, status, err );
}

/**
 * @brief 찾아진 descriptor를 가지고 Essential Matrix, 그리고 그것에 대한 recoverPose를 구하여 location 계산.
 * 
 */
void VisualOdometry::poseEstimationPnP() {
    cv::Mat E;
    // Essential matrix 
    E = cv::findEssentialMat(prev_points, curr_points, focal_length, principal_point, cv::RANSAC, 0.999, 1, inlier_mask);

    // Rotation matrix and Translation matrix 
    cv::Mat R, t;
    // SVD and R, T extract
    int inlier_num = cv::recoverPose(E, prev_points, curr_points, R, t, focal_length, principal_point, inlier_mask);

    if(inlier_num > min_inlier_num) {
        cv::Mat T = cv::Mat::eye(4, 4, R.type());
        T(cv::Rect(0, 0, 3, 3)) = R * 1.0;
        T.col(3).rowRange(0, 3) = t * 1.0;
        camera_pose = camera_pose * T.inv();
    }

    this->current_location = cv::Point2d((int)camera_pose.at<double>(0, 3), (int)camera_pose.at<double>(2, 3));
}
