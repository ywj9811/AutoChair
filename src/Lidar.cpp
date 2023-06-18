#include "Lidar.h"

using namespace rp::standalone::rplidar;

Lidar::Lidar(std::string lidar_path, int baud_rate) {
    drv_ = *createLidarDriver();
    bool connect_success = false;

    if (!drv_) {
        fprintf(stderr, "[ERROR]: insufficent memory, exit\n");
        exit(-2);
    }

    channel_ = (*createSerialPortChannel(lidar_path, baud_rate));

    if (SL_IS_OK((drv_)->connect(channel_))) {
        op_result_ = drv_->getDeviceInfo(devinfo_);

        if (SL_IS_OK(op_result_)) {
            connect_success = true;
        }
        else {
            delete drv_;
            drv_ = NULL;
        }   
    }

    if(!connect_success){
        std::cout << "\n[ERROR]: connect status: " << connect_success << std::endl;
        std::cerr << "[ERROR]: CANNOT CONNECT PORT!!\n" << std::endl;
        exit(2);
    }
    if (checkSLAMTECLIDARHealth(drv_)){
        drv_->setMotorSpeed();   
        drv_->startScan(0, 1);
    }
}

Lidar::~Lidar(){
    drv_->stop();
    drv_->setMotorSpeed(0);

    if(drv_)
        delete drv_;
    drv_ = NULL;

}

size_t Lidar::grabScanedLidarData(
    sl_lidar_response_measurement_node_hq_t* nodes,
    size_t counts
) {
    sl_lidar_response_measurement_node_hq_t nodes_buf[8192];
    size_t result_counts = _countof(nodes);
    op_result_ = drv_->grabScanDataHq(nodes, counts);

    if (SL_IS_OK(op_result_)) {
        drv_->ascendScanData(nodes, counts);
    }

    size_t temp = counts;
    return temp;
}

void Lidar::transformTheta(const float theta, double* output_array) {
    double x_theta = 0;
    double y_theta = 0;

    if(0 <= theta && theta < 90){
        x_theta = sin(theta*(PI/180));
        y_theta = -1 * (cos(theta*(PI/180)));

    } else if(90 <= theta && theta < 180){
        x_theta = cos((theta - 90) * (PI/180));
        y_theta = sin((theta - 90) * (PI/180));

    } else if(180 <= theta && theta < 270) {
        x_theta = -1 * sin((theta - 180) * (PI/180));
        y_theta = cos((theta - 180.f) * (PI/180));

    } else {
        x_theta = -1 * cos((theta - 270) * (PI/180));
        y_theta = -1 * sin((theta - 270) * (PI/180));
    }

    output_array[0] = x_theta;
    output_array[1] = y_theta;

    return;
}

bool Lidar::checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        // printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "[ERROR], slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "[ERROR], cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}