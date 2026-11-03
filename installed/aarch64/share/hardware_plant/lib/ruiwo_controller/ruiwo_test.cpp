#include <ruiwo_actuator.h>
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <experimental/filesystem>
#include <csignal>
namespace fs = std::experimental::filesystem;
fs::path source_path = fs::canonical(__FILE__);
// 获取当前源文件所在的目录
fs::path source_directory = source_path.parent_path();
RuiWoActuator actuator(source_directory);
std::vector<std::vector<double>> stateList;
std::vector<double> poseList;
std::vector<double> torList;
std::vector<double> vellist;

void close_ruiwo()
{
    stateList = actuator.get_joint_state();
    poseList = actuator.get_positions();
    torList = actuator.get_torque();
    vellist = actuator.get_velocity();
    actuator.close();
    std::cout << "-------------------------------"<< std::endl;
    std::cout << "[RUIWO motor]:Joint state: "<< std::endl;
    for (const auto &row : stateList)
    {
        for (const auto &element : row)
        {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------"<< std::endl;
    std::cout << "[RUIWO motor]:Joint positions: ";
    for (const auto &element : poseList)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    std::cout << "[RUIWO motor]:Joint torque: ";
    for (const auto &element : torList)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    std::cout << "[RUIWO motor]:Joint velocity: ";
    std::cout << std::endl;
    for (const auto &element : vellist)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}
void signalHandler(int signum) {
    // 终止程序
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    close_ruiwo();
    exit(signum);
}
int main()
{
    signal(SIGINT, signalHandler);
    actuator.initialize();
    auto motors = actuator.get_motor_state();
    for(auto &motor: motors) {
        std::cout << "[Ruiwo motor]: Motor:0x" << std::hex << static_cast<int>(motor.id) << std::dec;
        if(motor.state == RuiWoActuator::State::Disabled)
            std::cout << ", State: Disabled \n";
        else     
            std::cout << ", State: Enable \n";
    }

    auto i = 0;
    while (true)
    {
        i++;
        double positions;
        double torque;
        double velocity;
        positions = 2.5 * sin(i * 3.14 / 180); // 10度的正弦波
        // positions = i * 0.05; // 0.05度正方向递增
        torque = 0.5 * cos(i * 3.14 / 180); 
        velocity = 0 * sin(i * 3.14 / 180); 
        actuator.set_positions({0, 1, 2, 3, 4, 5, 
                                6, 7, 8, 9, 10, 11,
                                12, 13},
                               {positions, positions, positions, positions, positions, positions,
                                -positions, -positions, -positions, -positions, -positions, -positions,
                                positions, positions},
                                {torque, torque, torque, torque, torque, torque,
                                -torque, -torque, -torque, -torque, -torque, -torque,
                                torque, torque},
                                {velocity, velocity, velocity, velocity, velocity, velocity,
                                velocity, velocity, velocity, velocity, velocity, velocity,
                                velocity, velocity});
        // actuator.set_torque({0, 1, 2, 3, 4, 5,
        //                      6, 7, 8, 9, 10, 11,
        //                      12, 13},
        //                        {torque, torque, torque, torque, torque, torque,
        //                         torque, torque, torque, torque, torque, torque,
        //                         torque, torque});
        // actuator.set_velocity({ 0, 1, 2, 3, 4, 5,
        //                         6, 7, 8, 9, 10, 11,
        //                         12, 13},
        //                        {velocity, velocity, velocity, velocity, velocity, velocity,
        //                         velocity, velocity, velocity, velocity, velocity, velocity,
        //                         velocity, velocity});
        usleep(10000);
        actuator.get_joint_state();
        actuator.get_positions();
        actuator.get_torque();
        actuator.get_velocity();
    }
    return 0;
}
