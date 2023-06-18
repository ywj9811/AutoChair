- MAIN
  - CentralSystem Class
    - Major Variable
        - camera
        - lidar
        - vo
        - dog_status
        - motor_control_system

    - Major Method
        - startProgram()
        - computeTrajectoryThread()
        - scanLidarThread()
        - communicationSystemThread()

  - DogStatus Class
    - variable
        - current frame, previous frame data
        - current trajectory data
        - is working flag
        - current LiDAR scan Data 
         
    - Major Method
        - set/getcurrentFrame()
        - set/getcurrentLidarData()
        - set/getcurrentTrajectoryData()
        - set/getSystemStatus()
    
  - Lidar Class
    - variable
        - channel
        - driver

    - Major Method
      - checkSLAMTECLIDARHealth()
      - transformTheta()
      - grabSacnedLidarData()

  - MotorControlSystem Class
    - variable
        - file descriptor to Serial Communication.
        - port options
  
    - Major Method
      - sendToCommand()

  - VisualOdometry Class
    - variable
        - current keypoints, previous keypoints
        - K (intrinsic Matrix)

    - Major Method
        - addFrame*()
        - extractKeyPoints()
        - computeDescritpors()
        - poseEstimationPnP()
        - getCurrentLocation()

  - Camera Class
    - variable
        - focal length
        - principer point
        - system mode
        - camera path

    - Major Method
        - cameraPath()
        - focalLength()
        - principalPoints()
        - systemMode()
    