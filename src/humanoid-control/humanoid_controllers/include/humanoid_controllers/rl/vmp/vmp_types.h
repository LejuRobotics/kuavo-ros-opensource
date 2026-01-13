#pragma once

#include <string>
#include <vector>
#include <ocs2_core/misc/LoadData.h>

namespace humanoid_controller
{
namespace vmp
{

/**
 * @brief 轨迹数据结构
 * @details 包含轨迹配置信息和预加载的轨迹数据
 */
struct TrajectoryData {
  std::string name;                       ///< 轨迹名称（用于日志和调试）
  std::string data_file;                  ///< 轨迹数据文件
  std::vector<float> task_data;           ///< 原始轨迹数据
  std::vector<float> processed_data;      ///< 处理后数据（包含静止帧）
  bool is_loaded{false};                  ///< 是否已加载
  bool is_processed{false};               ///< 是否已处理
  size_t original_frames{0};              ///< 原始帧数
  size_t total_frames{0};                 ///< 总帧数（含静止帧）

  /**
   * @brief 重置轨迹数据到初始状态（保留 name 和 data_file）
   */
  void reset() {
    task_data.clear();
    processed_data.clear();
    is_loaded = false;
    is_processed = false;
    original_frames = 0;
    total_frames = 0;
  }
};

} // namespace vmp

/**
 * @brief VMP (Variable Movement Primitive) 配置参数
 *
 * 包含 VAE 模型配置和参考运动索引配置
 */
struct VMPConfig {
    //==========================================================================
    // VAE 模型参数 (vaeModel)
    //==========================================================================
    int in_c = 77;                      ///< 输入通道数
    int window_l = 30;                  ///< 窗口长度 (2*window_r)
    int window_r = 15;                  ///< 右窗口长度
    int base_c = 128;                   ///< 基础通道数
    int latent_d = 512;                 ///< 潜在维度
    int scalors = 3;                    ///< 缩放器数量 [0.8, 0.6, 0.4]
    bool encode_atten = true;           ///< 是否使用注意力编码
    double a = 0.01;                    ///< 参数 a
    double d = 2;                       ///< 参数 d (1 <= d <= 3)
    double beta = 0.7;                  ///< beta 参数
    std::string encode_method = "TCN";  ///< 编码方法 (TCN 或 classic)

    //==========================================================================
    // 参考运动索引配置 (refTask)
    // 定义参考运动数据中各字段的起止索引
    //==========================================================================
    int h_start_id = 0;                 ///< 高度开始索引
    int h_end_id = 1;                   ///< 高度结束索引
    int theta_start_id = 1;             ///< 方向(旋转矩阵)开始索引
    int theta_end_id = 7;               ///< 方向(旋转矩阵)结束索引
    int v_start_id = 7;                 ///< 速度开始索引
    int v_end_id = 13;                  ///< 速度结束索引
    int q_start_id = 13;                ///< 关节位置开始索引
    int q_end_id = 39;                  ///< 关节位置结束索引
    int q_dot_start_id = 39;            ///< 关节速度开始索引
    int q_dot_end_id = 65;              ///< 关节速度结束索引
    int p_start_id = 65;                ///< 位置开始索引
    int p_end_id = 77;                  ///< 位置结束索引
    int num_tracklink = 4;              ///< 追踪链接数量

    /**
     * @brief 从配置文件加载 VMP 配置参数
     * @param config_file 配置文件路径
     */
    void loadConfig(const std::string& config_file) {
        using ocs2::loadData::loadCppDataType;

        // VAE 模型参数
        loadCppDataType(config_file, "vaeModel.in_c", this->in_c);
        loadCppDataType(config_file, "vaeModel.window_l", this->window_l);
        loadCppDataType(config_file, "vaeModel.window_r", this->window_r);
        loadCppDataType(config_file, "vaeModel.base_c", this->base_c);
        loadCppDataType(config_file, "vaeModel.latent_d", this->latent_d);
        loadCppDataType(config_file, "vaeModel.scalors", this->scalors);
        loadCppDataType(config_file, "vaeModel.encode_atten", this->encode_atten);
        loadCppDataType(config_file, "vaeModel.a", this->a);
        loadCppDataType(config_file, "vaeModel.d", this->d);
        loadCppDataType(config_file, "vaeModel.beta", this->beta);

        // refTask 配置
        loadCppDataType(config_file, "refTask.h_start_id", this->h_start_id);
        loadCppDataType(config_file, "refTask.h_end_id", this->h_end_id);
        loadCppDataType(config_file, "refTask.theta_start_id", this->theta_start_id);
        loadCppDataType(config_file, "refTask.theta_end_id", this->theta_end_id);
        loadCppDataType(config_file, "refTask.v_start_id", this->v_start_id);
        loadCppDataType(config_file, "refTask.v_end_id", this->v_end_id);
        loadCppDataType(config_file, "refTask.q_start_id", this->q_start_id);
        loadCppDataType(config_file, "refTask.q_end_id", this->q_end_id);
        loadCppDataType(config_file, "refTask.q_dot_start_id", this->q_dot_start_id);
        loadCppDataType(config_file, "refTask.q_dot_end_id", this->q_dot_end_id);
        loadCppDataType(config_file, "refTask.p_start_id", this->p_start_id);
        loadCppDataType(config_file, "refTask.p_end_id", this->p_end_id);
        loadCppDataType(config_file, "refTask.num_tracklink", this->num_tracklink);
    }
};

} // namespace humanoid_controller
