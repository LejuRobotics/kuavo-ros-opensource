#include <iostream>
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <atomic>
#include "lejuclaw_controller.h"
#include "claw_types.h"

using namespace eef_controller;

LejuClawControllerPtr actuator = nullptr;
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    actuator->close();
    exit(signum);
}

void publishClawState(const LejuClawControllerPtr &controller_ptr)
{
    auto state = GetClawState(controller_ptr);
    std::cout << "Claw State - Position: " 
              << state.data.position[0] << ", " << state.data.position[1]
              << " Velocity: " << state.data.velocity[0] << ", " << state.data.velocity[1]
              << " Effort: " << state.data.effort[0] << ", " << state.data.effort[1]
              << " State: " << static_cast<int>(state.state[0]) << ", " << static_cast<int>(state.state[1])
              << std::endl;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    actuator = LejuClawController::Create();
    bool init_bmapilib = true;
    if(!actuator->initialize(init_bmapilib)) {
        std::cout << "claw initialize failed!" << std::endl;
        return -1;
    }

    /* ELSE */
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>> Leju Claw Test <<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::vector<double> positions = {20.0, 20.0}; // 单个电机位置
    std::vector<double> velocity = {50.0, 50.0};  // 夹取速度
    std::vector<double> torque = {0.5, 0.5};      // 夹取力度
    
    auto print_gripper_state = [](const std::string &name, LejuClawController::State state) {
        std::cout << "[Gripper State] " << name << ": " ;
        switch (state) {
        case LejuClawController::State::kUnknown:
            std::cout << "UNKNOWN" << std::endl;
            break;
        case LejuClawController::State::kGrabbed:
            std::cout << "GRABBED" << std::endl;
            break;
        case LejuClawController::State::kReached:
            std::cout << "REACHED" << std::endl;
            break;
        default:
            std::cout << "UNKNOWN" << std::endl;
            break;    
        }
    };

    usleep(1000000); // 延时1秒
    while (1)
    {
        // 夹爪运动到20位置
        std::cout << "Moved to position -------------------------------------20" << std::endl;
        positions = {20.0, 20.0};
        auto result = actuator->move_paw(positions, velocity, torque);
        print_gripper_state("LEFT", result[0]);
        print_gripper_state("RIGHT", result[1]);
        usleep(1000000); // 延时1秒

        // 获取当前位置并打印
        auto current_positions = actuator->get_positions();
        std::cout << "[Current Positions]: ";
        for (const auto &pos : current_positions)
        {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        // 夹爪运动到95位置
        std::cout << "Moved to position -------------------------------------95" << std::endl;
        positions = {95.0, 95.0};
        result = actuator->move_paw(positions, velocity, torque);
        std::cout << "[move_paw Result]: ";
        print_gripper_state("LEFT", result[0]);
        print_gripper_state("RIGHT", result[1]);

        usleep(1000000); // 延时1秒

        // 获取当前位置并打印
        current_positions = actuator->get_positions();
        std::cout << "[Current Positions]: ";
        for (const auto &pos : current_positions)
        {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }

    actuator->close();
    return 0;
}