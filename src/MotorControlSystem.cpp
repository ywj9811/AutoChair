#include "MotorControlSystem.h"

MotorControlSystem::MotorControlSystem(const char* port_path, speed_t baud_rate) {
    int ii, jj, kk;

    // for init check 
    fid_ = -1;
    tcgetattr(fid_, &port_options_);
    fid_ = open(port_path, O_RDWR | O_NOCTTY );

    tcflush(fid_, TCIFLUSH);
 	tcflush(fid_, TCIOFLUSH);
    usleep(1000000);
    
    // this here 
    if (fid_ == -1) {
		printf("\n[ERROR]: Unable to open UART.  Ensure it is not in use by another application\n");
        exit(2);
	}

    port_options_.c_cflag &= ~PARENB;                         // Disables the Parity Enable bit(PARENB),So No Parity
    port_options_.c_cflag &= ~CSTOPB;                         // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options_.c_cflag &= ~CSIZE;                          // Clears the mask for setting the data size
    port_options_.c_cflag |= CS8;                             // Set the data bits = 8
    port_options_.c_cflag &= ~CRTSCTS;                        // No Hardware flow Control
    port_options_.c_cflag |= CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options_.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable XON/XOFF flow control both input & output
    port_options_.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non Cannonical mode
    port_options_.c_oflag &= ~OPOST;

    port_options_.c_lflag =  0;

    port_options_.c_cc[VMIN] = VMINX;       
    port_options_.c_cc[VTIME] = 0;

    cfsetispeed(&port_options_, baud_rate);    // Set Read  Speed 
    cfsetospeed(&port_options_, baud_rate);

    int att = tcsetattr(fid_, TCSANOW, &port_options_);
    if (att != 0 ) {
        printf("\n[ERROR]: in Setting port attributes\n");
    }

    // Flush Buffers
    tcflush(fid_, TCIFLUSH);
    tcflush(fid_, TCIOFLUSH);

    // 0.5 sec delay
    usleep(500000); 

    bool init_flag = sendToCommand(START_FLAG);
    if (init_flag == false) {
        printf("\n[ERROR]: CANNOT init our dogs, Try Again!\n");
        exit(1);
    }
}

bool MotorControlSystem::sendToCommand(char command) {
    // this is not connected Arduino
    if (fid_ == -1) return false;
    
    unsigned char tx_buffer[2];
    unsigned char *p_tx_buffer;
    p_tx_buffer = &tx_buffer[0];
    
    *p_tx_buffer++ = command;

    int count = write(fid_, &tx_buffer, (p_tx_buffer - & tx_buffer[0]));
    // UART TX error status 
    if (count < 0) return false; 

    return true;
}
