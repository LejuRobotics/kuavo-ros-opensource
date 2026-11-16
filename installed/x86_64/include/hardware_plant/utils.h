#pragma once
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <filesystem>
#include <csignal>
#include <experimental/filesystem>
#include <cstdio>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

namespace fs = std::experimental::filesystem;

static std::string hardware_plant_dir = "";
void setHardwarePlantPath(const std::string &path);
std::string GetAbsolutePathHW(const std::string &path);
