#ifndef MOTOR_CONTROL_SYSTEM_H
#define MOTOR_CONTROL_SYSTEM_H

#include "common_header.h"

class MotorControlSystem {
  public:
    MotorControlSystem(const char* port_path, speed_t baud_rate);
    ~MotorControlSystem() { close(fid_); };
    // TRANSMISSION
    bool sendToCommand(char command);

  private:
    int fid_;
    struct termios port_options_;
};


#endif