#pragma once
#include <iostream>

struct RobotVersion
{
    u_int8_t Major;
    u_int8_t Minor;

    RobotVersion(u_int8_t major, u_int8_t minor) : Major(major), Minor(minor) {}
    std::string versionName() { return std::to_string(Major) + "." + std::to_string(Minor); }
    int versionInt() { return Major * 10 + Minor; }
};
