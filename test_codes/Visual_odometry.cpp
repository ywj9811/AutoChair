#include "opencv2/opencv.hpp"

int main(){
    const char* input = "../data/07/image_0/%06d.png";
    double f = 707.0912;
    cv::Point2d c(601.8873, 183.1104);
    bool use_5pt = true;
    int min_inlier_num = 100;

    // Open a video and get the initial image
    cv::VideoCapture video;
    if(!video.open(input)) return -1;

    cv::Mat gray_prev;
    video >> gray_prev;
    if(gray_prev.empty()){
        video.release();
        return -1;
    }

    if(gray_prev.channels() > 1) cv::cvtColor(gray_prev, gray_prev, cv::COLOR_BGR2GRAY);

    cv::Mat display = cv::Mat::zeros(1000, 1000, CV_8UC3);

    cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64F);
    while(true){
        // Grab an image from the video
        cv::Mat image, gray;
        video >> image;
        
        if(image.empty()) break;
        
        // make gray image
        if(image.channels() > 1) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else gray = image.clone();

        // Etract optical flow 
        std::vector<cv::Point2f> point_prev, point;
        // extract feature for good tracking 
        cv::goodFeaturesToTrack(gray_prev, point_prev, 2000, 0.01, 10);
        std::vector<uchar> status;
        cv::Mat err;
        // Calculate Optical Flow
        cv::calcOpticalFlowPyrLK(gray_prev, gray, point_prev, point, status, err);
        gray_prev = gray;

        // Calculate relative pose
        cv::Mat E, inlier_mask;
        cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, c.x, 0, f, c.y, 0, 0, 1);
        // extract Essential matrix
        if(use_5pt){
            E = cv::findEssentialMat(point_prev, point, f, c, cv::RANSAC, 0.99, 1, inlier_mask);
        } else {
            cv::Mat F = cv::findFundamentalMat(point_prev, point, cv::FM_RANSAC, 1, 0.99, inlier_mask);
            E = K.t() * F * K;
        }

        // Rotation matrix and Translation matrix
        cv::Mat R, t;
        // SVD and R, T extract
        int inlier_num = cv::recoverPose(E, point_prev, point, R, t, f, c, inlier_mask);

        // Accumulate relative pose if result is reliable
        if(inlier_num > min_inlier_num) {
            cv::Mat T = cv::Mat::eye(4, 4, R.type());
            T(cv::Rect(0, 0, 3, 3)) = R * 1.0;
            T.col(3).rowRange(0, 3) = t * 1.0;
            camera_pose = camera_pose * T.inv();
        }
        
        // show the image and write camera pose
        if(image.channels() < 3) cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        for (int i = 0; i < point_prev.size(); i++) {
            if (inlier_mask.at<uchar>(i) > 0) cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 0, 255));
            else cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 127, 0));
        }

        // draw camera pose bird view
        cv::Point2d camera_point((int)camera_pose.at<double>(0, 3), (int)camera_pose.at<double>(2, 3));
        cv::drawMarker(display, cv::Point(camera_point.x + 500, camera_point.y + 500), cv::Scalar(0, 0, 255), cv::MARKER_SQUARE, 5, 2);

        // reconstruct 3D points (triangulation)
        cv::Mat P0 = K * cv::Mat::eye(3, 4, CV_64F);
        cv::Mat Rt, X;
        // merge R + t = [3 x 4]
        cv::hconcat(R, t, Rt);

        cv::Mat P1 = K * Rt;
        cv::triangulatePoints(P0, P1, point_prev, point, X);
        // X = [x, y, z, 1]
        X.row(0) = X.row(0) / X.row(3);
        X.row(1) = X.row(1) / X.row(3);
        X.row(2) = X.row(2) / X.row(3);
        X.row(3) = 1;
        

        cv::imshow("Traj", display);
        cv::imshow("VO tutorial", image);
        if(cv::waitKey(30) == 27) break;
    }

    video.release();
    return 0;
}

