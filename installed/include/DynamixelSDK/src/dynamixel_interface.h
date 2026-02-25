#pragma once

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <ostream>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <map>
#include <array>
#include <cstdint>
#include <string>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <cmath> // For M_PI

#include "dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "group_bulk_write.h"
#include "group_bulk_read.h"

#define L_HAND_UP 17
#define L_HAND_MID 18
#define L_HAND_LOW 19
#define R_HAND_UP 24
#define R_HAND_MID 25
#define R_HAND_LOW 26

// Protocol version
#define PROTOCOL_VERSION 10 // See which protocol version is used in the Dynamixel

// Code specific to protocol version 10
// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36

#define LEN_PRO_PRESENT_POSITION 2

// Default setting
#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

// Data Byte Length
#define LEN_LED_RED 1
#define LEN_GOAL_CURRENT 2
#define LEN_GOAL_POSITION 4
#define LEN_GOAL_Velocity 4
#define LEN_PRESENT_CURRENT 2
#define LEN_PRESENT_POSITION 4
#define LEN_PRESENT_Velocity 4

// Default setting
#define BAUDRATE 1000000
#define DEVICENAME "/dev/usb_servo" // Check which port is being used on your controller
                                    // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE 1                 // Value for enabling the torque
#define TORQUE_DISABLE 0                // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 0    // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 4095 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 20  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

class DynamixelServo
{
public:
    DynamixelServo(bool iscalibrating);
    ~DynamixelServo();

    // Initialize GroupBulkWrite instance
    dynamixel::GroupBulkWrite groupBulkWrite;

    // Initialize GroupBulkRead instance
    dynamixel::GroupBulkRead groupBulkRead;

    bool calibrating;

    std::vector<uint8_t> xh540Ids = {};                                                                     // xh540-w270 舵机id
    std::vector<uint8_t> xc330Ids = {L_HAND_UP, L_HAND_MID, L_HAND_LOW, R_HAND_UP, R_HAND_MID, R_HAND_LOW}; // xc330-t288 舵机id

    std::map<int, std::string> serverNamesMap = {
        {17, "L_HAND_UP"},
        {18, "L_HAND_MID"},
        {19, "L_HAND_LOW"},
        {24, "R_HAND_UP"},
        {25, "R_HAND_MID"},
        {26, "R_HAND_LOW"}};

    std::map<uint8_t, std::array<uint16_t, 3>> init_positions = {
        {L_HAND_UP, {1460, 65535, 0}},
        {L_HAND_MID, {1330, 65535, 0}},
        {L_HAND_LOW, {2342, 65535, 0}},
        {R_HAND_UP, {1011, 423, 3876}},
        {R_HAND_MID, {2054, 1647, 2743}},
        {R_HAND_LOW, {2559, 1636, 3056}},
    };
    std::map<uint8_t, double> motor_directions =
        {
            {L_HAND_UP, -1.0},
            {L_HAND_MID, -1.0},
            {L_HAND_LOW, 1.0},
            {R_HAND_UP, -1.0},
            {R_HAND_MID, 1.0},
            {R_HAND_LOW, -1.0},

    };

    std::map<uint8_t, uint16_t> target_positions = {
        {L_HAND_UP, 0},
        {L_HAND_MID, 0},
        {L_HAND_LOW, 0},
        {R_HAND_UP, 0},
        {R_HAND_MID, 0},
        {R_HAND_LOW, 0}};

    int init(const char *device_name, int baud_rate);
    void initTargetPositions(const std::map<uint8_t, std::array<uint16_t, 3>> &servoPositions, std::map<uint8_t, std::uint16_t> &targetPositions);
    void setPosition(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions);
    void setPosition(uint8_t id, uint16_t position);
    uint16_t getPosition(uint8_t id);
    std::vector<int> getPosition(const std::vector<uint8_t> &ids);
    std::vector<uint8_t> getServoIds();

    void setPositionByRadian(uint8_t id, double radian);
    void setPositionsByRadian(const std::vector<uint8_t> &ids, const std::vector<double> &raidans);
    void disable_torue();

    std::vector<double> getRadians(const std::vector<uint8_t> &ids);
    double getRadian(uint8_t id);

    void convertGoalPosition(uint32_t dxl_goal_position, uint8_t param_goal_position[]);

    struct ServoInfo
    {
        uint32_t position;
    };

    void scanServos();
    void controlThread();
    void startThreads();
    void stopThreads();
    std::string initPositionArrayToString(const std::array<uint16_t, 3> &arr);
    std::array<uint16_t, 3> initPositionStringToArray(const std::string &str);
    void saveInitPositionsToIni(const std::map<uint8_t, std::array<uint16_t, 3>> &init_positions, const std::string &filename);
    std::map<uint8_t, std::array<uint16_t, 3>> loadInitPositionsFromIni(const std::string &filename);
    std::string getHomePathofWhoRunTheCommand();
    std::unordered_map<uint8_t, ServoInfo> servos;

private:
    dynamixel::PortHandler *port_handler;
    dynamixel::PacketHandler *packet_handler;
    dynamixel::GroupBulkRead *groupXh540Reader;
    dynamixel::GroupBulkRead *groupXc330Reader;
    dynamixel::GroupBulkWrite *groupPositionWriter;
    dynamixel::GroupBulkWrite *groupCurrentWriter;

    std::thread reading_thread, control_thread;
    std::atomic<bool> stop_thread{false};
    std::mutex read_mutex;
    std::mutex write_mutex;
};
