#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <vector>
#include <iostream>
#include "jodell_tool.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#define MAX_SIZE 256

namespace ClawController
{
    enum ClawType
    {
        LeftClaw = 1,
        RightClaw = 2,
    };
    struct ClawParameters
    {
        ClawType claw_type;
        int pos;
        int speed;
        int tau;
    };
    class ClawController
    {
    public:
        ClawController();
        void init(const std::string &port_name = "/dev/claw_serial");
        void controlClaw(ClawType claw_type, int pos, int speed = 100, int tau = 100);

        void getClawStatus(ClawType claw_type);

        // std::vector<double> get_positions();

        int getClawPos(ClawType claw_type);
        void setPositions(const std::vector<double> &target_position);
        std::vector<double> getPositions();
        void control_thread();

    private:
        int claw_id_l_;
        int claw_id_r_;
        int com_num_; // hardware binding
        int baud_rate_;
        std::thread controlThread_;
        std::atomic<bool> stopThread_;
        std::mutex r_mtx_;
        std::mutex w_mtx_;
        bool updated{true};
        std::vector<ClawParameters> claw_data_est;
        std::vector<ClawParameters> claw_data_des, old_claw_data_des;
        void enableClaw(ClawType claw_type, bool enable);
    };
}
