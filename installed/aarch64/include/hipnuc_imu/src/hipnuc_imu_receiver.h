#ifndef _HIPNUC_IMU_RECEIVER_H_
#define _HIPNUC_IMU_RECEIVER_H_

#include "hipnuc.h"

#include <iostream>
#include <stdexcept>
#include <string>

#include <chrono>
#include <iostream>
#include <stdio.h>

#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h> 
#include <fcntl.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <poll.h>
#include "hipnuc.h"
#include <Eigen/Dense>
#include <queue>
#include <mutex>

namespace HIPNUC_IMU
{

    extern std::mutex imu_data_mutex;
    extern std::queue<hipnuc_raw_t> imu_data_queue;
    extern Eigen::Vector3d euler_offset;

    #define GRA_ACC      (9.8)
    #define DEG_TO_RAD   (0.01745329)
    // #define BUF_SIZE     1024

    #define ACC_FACTOR   (0.0048828)
    #define GYR_FACTOR	 (0.001)
    #define MAG_FACTOR   (0.030517)
    #define EUL_FACTOR	 (0.001)
    #define QUA_FACTOR   (0.0001)

    using std::chrono::milliseconds;

    extern std::string imu_port;
    extern int serial_baud;
    extern std::string frame_id;
    extern std::string imu_topic;
    extern int fd;
    extern struct pollfd p;
    static hipnuc_raw_t raw;

    int imu_init();
    void imu_stop();
    bool getImuDataFrame(Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &quat);
    int open_port(std::string port_device, int baud);
    int read_hipnuc_imu(int fd, struct pollfd *p);
    bool get_imu_running_flag();
}
#endif // _imu_receiver_h_
