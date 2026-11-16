
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include "xdacallback.h"
#include <xstypes/xsportinfo.h>
#include "chrono"
#include <Eigen/Dense>
#include <vector>
#include "AutoPopSPSCQueue.h"

struct XsControl;
struct XsDevice;
using logger_callback_vector = std::function<void(const std::string &name, const Eigen::VectorXd &vec)>;
void set_logger_callback(logger_callback_vector callback);

#define IMU_ACC_INDEX 0
#define IMU_GYROHR_INDEX 1
#define IMU_QUAT_INDEX 2

#define IMU_SPIN_QUEUE_LENGTH 5

typedef struct
{
	std::vector<std::pair<struct timespec, Eigen::Vector3d>> acc_raw_vector;
	std::vector<std::pair<struct timespec, Eigen::Vector3d>> acc_vector;
	std::vector<std::pair<struct timespec, Eigen::Vector3d>> gyro_vector;
	std::vector<std::pair<struct timespec, Eigen::Quaterniond>> quat_vector;
	struct timespec last_live_data_time_stamp;
	bool is_new_data = false;
} IMUData;

class PacketCallback;

class XdaInterface
{
public:
    XdaInterface();
    ~XdaInterface();

    void spinFor(std::chrono::milliseconds timeout);
    void registerPublishers();

    bool connectDevice();
    bool prepare();
    void close();

	bool getNewestData(IMUData &data);
    bool getData(IMUData &data);
    bool imu_start_flag;

private:
    void registerCallback(PacketCallback *cb);
    bool handleError(std::string error);

    XsControl *m_control;

    AutoPopSPSCQueue<Eigen::Quaterniond, IMU_SPIN_QUEUE_LENGTH> imuQuatCachedQueue;
    AutoPopSPSCQueue<Eigen::Quaterniond, 1> imuReceivedQuatQueue;

    Eigen::Quaterniond last_used_quat = Eigen::Quaterniond::Identity();
    int reuse_quat_because_no_new_data_cnt = 0;
	int reuse_new_data_cnt = 0;

	XsDevice *m_device;
	XsPortInfo m_port;
	XdaCallback m_xdaCallback;
	std::list<PacketCallback *> m_callbacks;

	IMUData mdata_;
	IMUData last_data_;

	int sync_reset_count = 0;

	int read_cnt_ = 0;
	int prev_acc_cnt = 0;
	int prev_gyro_cnt = 0;
    std::mutex index_mtx;
	std::mutex last_data_mtx;
	int index[3] = {0,0,0};
	int acc_cnt = 0;
	double acc_dt = 0;
	double acc_time= 0;

    double gyro_quat_time = 0;
    double now_time = 0;
    double dynamic_adjust_time_offset = 0.0;

	int quat_cnt = 0;
	struct timespec data_time[3];
	struct timespec init_time[3], init_imu_time[3];
	std::mutex mtx_imu;
	bool acc_read_sequence_flag;
	std::pair<timespec, Eigen::Quaterniond> last_quat_;
	std::mutex acc_time_mutex;
};

#endif
