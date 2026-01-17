#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cstdint>

namespace humanoid_controller
{
namespace vmp
{

/**
 * @brief 从二进制文件加载任务数据
 *
 * @param file_path 数据文件路径
 * @param in_c 每帧特征数量
 * @param data 输出的数据向量
 * @param total_frames 输出的实际总帧数
 * @return true 加载成功
 * @return false 加载失败
 */
inline bool loadTaskData(const std::string& file_path,
                         int in_c,
                         std::vector<float>& data,
                         int& total_frames)
{
    data.clear();
    total_frames = 0;

    if (!std::filesystem::exists(file_path)) {
        std::cerr << "[VMP] Data file not found: " << file_path << std::endl;
        return false;
    }

    size_t file_size = std::filesystem::file_size(file_path);
    total_frames = static_cast<int>(file_size / (in_c * sizeof(float)));

    if (total_frames == 0) {
        std::cerr << "[VMP] No frames to load (file_size=" << file_size
                  << ", in_c=" << in_c << ")" << std::endl;
        return false;
    }

    // 读取数据
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        std::cerr << "[VMP] Cannot open: " << file_path << std::endl;
        return false;
    }

    size_t data_count = total_frames * in_c;
    data.resize(data_count);
    if (!file.read(reinterpret_cast<char*>(data.data()), data_count * sizeof(float))) {
        std::cerr << "[VMP] Read error: " << file_path << std::endl;
        data.clear();
        return false;
    }

    std::cout << "[VMP] Loaded " << total_frames << " frames from " << file_path << std::endl;
    return true;
}

} // namespace vmp
} // namespace humanoid_controller
