#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include "opencv2/opencv.hpp"

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main()
{
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1226 ;
    int display_height = 370 ;
    int framerate = 30 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
    printf("Start ImageCapture Program!\n");

    // camera 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if ( !cap.isOpened()) {
        std::cout << "[ERROR] check your camera path!!" << std::endl;
    }

    int counter = 0;
    char buf[1024];
    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        cv::imshow("Display", frame);
        int key = cv::waitKey(27);
        if (key == 27) {
            break;
        }
        if (key == 115) {
            sprintf(buf, "../dataset/%05d.png", counter);
            cv::imwrite(buf, frame);
            std::cout << "save image path = " << buf << std::endl;
            counter++;
        }
    }


    cv::destroyAllWindows();
    cap.release();
    return 0;
}
