#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv){
    (void)argc;
    (void)argv;

    std::vector<cv::String> fileNames;
    cv::glob("../checkimage_dataset1/*.png", fileNames, false);
    cv::Size patternSize(10 - 1, 7 - 1);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 10 x 7
    // field with a size of 4cm x 4cm (40 x 40mm) 
    // 35mm 35mm

    int checker_board[2] = {10, 7};
    // Define the world coordinate for 3D points
    std::vector<cv::Point3f> objp;
    for(int i=1; i<checker_board[1]; i++){
        for(int j=1; j<checker_board[0]; j++){
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    std::vector<cv::Point2f> imgPoints;
    // Detect feature points
    std::size_t i = 0;
    for(auto const &f : fileNames){
        std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = cv::imread(fileNames[i]);
        std::cout << "image size " << img.size() << std::endl;
        cv::Mat gray;
        
        // make gray image
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], 
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            std::cout << "FOUND!!" << std::endl;
            cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }

        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("detection", img);
        cv::waitKey(5);

        i++;
    }
    cv::Matx33f K(cv::Matx33f::eye()); // instrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    // frame size = 1280 720 
    cv::Size frameSize(1280, 720);

    std::cout << "Calibrating . . . " << std::endl;
    std::cout << "Q is size = " << Q.size() << " q size is = " << q.size() << std::endl;
    

    // 4. Call "float error = cv::calibrateCamera()" with the input coordinate
    // and output parameters as declared above ...
    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK = \n" << K << "\nk = \n" << k << std::endl;

    // Precompute lens correction interplation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

    for(auto const &f : fileNames){
        std::cout << std::string(f) << std::endl;
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

        cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0);
    }

    return 0;
}
