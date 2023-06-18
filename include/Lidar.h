#ifndef LIDAR_H
#define LIDAR_H

#include "common_header.h"
#include "config/config.h"

// LIDAR SDK
#include "rplidar.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace rp::standalone::rplidar;

class Lidar {
  public:
    Lidar(std::string lidar_path, int baud_rate);
    ~Lidar();
    size_t grabScanedLidarData(
        sl_lidar_response_measurement_node_hq_t* nodes,
        size_t counts);
    
  private:
    bool ctrl_c_pressed_ = false;
    IChannel* channel_;
    ILidarDriver* drv_;
    sl_result op_result_;
    sl_lidar_response_device_info_t devinfo_;

    bool checkSLAMTECLIDARHealth(ILidarDriver * drv);
    void transformTheta(const float theta, double* output_array);
};

#endif