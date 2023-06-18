// VERSION 1.0
/**
 * @file main.cpp
 * @author hyeon ki (gusrlLee@github.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "common_header.h"

#include "DogStatus.h"
#include "CentralSystem.h"

const char* keys = {" \\ 
    {help h ?            |       | this is help. } \\ 
    {real-mode           |       | this option is Real Mode. } \\
    {simulation-mode     |       | this option is Simulation Mode. } \\
    {use-lidar           |       | this option is use LiDAR}"};


// MAIN SYSTEM =============================================================================================================== 
int main(int argc, char* argv[]) {
    cv::CommandLineParser parser(argc, argv, std::string(keys));
    parser.about("AutoDog System v1.0.0");

    if ( parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    else if (argc < 2){
        parser.printMessage();
        return 0;
    }

    bool system_mode = false;
    bool real_mode = parser.has("real-mode");
    bool simulation_mode = parser.has("simulation-mode");
    bool use_lidar = parser.has("use-lidar");

    // if system mode true, system mode is RealMode 
    // else system mode is simulationMode      
    if ( real_mode ) system_mode = true; 
    if ( simulation_mode ) system_mode = false; 

    CentralSystem* central_system = new CentralSystem(system_mode, use_lidar);
    central_system->startProgram();

    return 0;
}
