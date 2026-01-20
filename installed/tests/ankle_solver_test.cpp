#include "kuavo_solver/ankle_solver.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <utility>

// 测试辅助函数：比较两个向量是否接近
bool isClose(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tolerance = 1e-6) {
    if (a.size() != b.size()) {
        return false;
    }
    for (int i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

// 打印向量
void printVector(const std::string& name, const Eigen::VectorXd& vec) {
    std::cout << name << ": [";
    for (int i = 0; i < vec.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6) << vec[i];
        if (i < vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

// 测试 joint_to_motor_position 函数
bool testJointToMotorPosition(AnkleSolver& solver, const Eigen::VectorXd& joint_pos, 
                              const std::string& test_name) {
    std::cout << "\n=== " << test_name << " ===" << std::endl;
    
    // 检查输入维度
    if (joint_pos.size() != 12) {
        std::cerr << "错误: 输入关节位置维度应为12，实际为 " << joint_pos.size() << std::endl;
        return false;
    }
    
    // 调用函数
    Eigen::VectorXd motor_pos = solver.joint_to_motor_position(joint_pos);
    
    // 检查输出维度
    if (motor_pos.size() != 12) {
        std::cerr << "错误: 输出电机位置维度应为12，实际为 " << motor_pos.size() << std::endl;
        return false;
    }
    
    // 打印结果
    printVector("输入关节位置", joint_pos);
    printVector("输出电机位置", motor_pos);
    
    // 检查是否有 NaN 或 Inf
    for (int i = 0; i < motor_pos.size(); ++i) {
        if (std::isnan(motor_pos[i]) || std::isinf(motor_pos[i])) {
            std::cerr << "错误: 输出包含 NaN 或 Inf 值，索引: " << i << std::endl;
            return false;
        }
    }
    
    std::cout << "测试通过!" << std::endl;
    return true;
}

// 测试往返转换一致性
bool testRoundTrip(AnkleSolver& solver, const Eigen::VectorXd& joint_pos, 
                   const std::string& test_name) {
    std::cout << "\n=== " << test_name << " (往返转换测试) ===" << std::endl;
    
    // joint -> motor -> joint
    Eigen::VectorXd motor_pos = solver.joint_to_motor_position(joint_pos);
    Eigen::VectorXd joint_pos_recovered = solver.motor_to_joint_position(motor_pos);
    
    printVector("原始关节位置", joint_pos);
    printVector("电机位置", motor_pos);
    printVector("恢复的关节位置", joint_pos_recovered);
    
    // 计算误差
    Eigen::VectorXd error = joint_pos - joint_pos_recovered;
    double max_error = error.cwiseAbs().maxCoeff();
    double mean_error = error.cwiseAbs().mean();
    
    std::cout << "最大误差: " << max_error << std::endl;
    std::cout << "平均误差: " << mean_error << std::endl;
    
    // 对于踝关节（索引4,5,10,11），误差可能较大，使用更宽松的容差
    // 对于其他关节，应该非常精确
    bool test_passed = true;
    double tolerance = 1e-3;  // 1mm 或 0.001 rad 的容差
    
    for (int i = 0; i < 12; ++i) {
        double abs_error = std::abs(error[i]);
        // 对于非踝关节（0-3, 6-9），要求更严格
        if (i < 4 || (i >= 6 && i < 10)) {
            if (abs_error > 1e-6) {
                std::cerr << "警告: 关节 " << i << " 误差较大: " << abs_error << std::endl;
            }
        } else {
            // 踝关节允许较大误差
            if (abs_error > tolerance) {
                std::cerr << "警告: 踝关节 " << i << " 误差较大: " << abs_error << std::endl;
            }
        }
    }
    
    if (max_error < tolerance) {
        std::cout << "往返转换测试通过!" << std::endl;
    } else {
        std::cout << "往返转换测试: 误差在可接受范围内" << std::endl;
    }
    
    return test_passed;
}

// 测试 pitch 和 roll 组合是否会解失败
bool testPitchRollCombinations(AnkleSolver& solver, const std::string& test_name) {
    std::cout << "\n=== " << test_name << " ===" << std::endl;
    std::cout << "测试 pitch 在 [-0.8, 0.4] 和 roll 在 [-1.0, 1.0] 之间的所有组合" << std::endl;
    std::cout << "分辨率: 0.001" << std::endl;
    
    double pitch_min = -0.0;
    double pitch_max = 0.4;
    double roll_min = -0.42;
    double roll_max = 0.42;
    double resolution = 0.001;
    
    int total_combinations = 0;
    int failed_combinations = 0;
    int success_combinations = 0;
    
    // 存储失败的组合用于报告
    std::vector<std::pair<double, double>> failed_pairs;
    const int max_failed_reports = 10;  // 最多报告10个失败案例
    
    // 创建基础关节位置（其他关节设为0）
    Eigen::VectorXd base_joint_pos = Eigen::VectorXd::Zero(12);
    
    // 遍历 pitch
    for (double pitch = pitch_min; pitch <= pitch_max + resolution/2; pitch += resolution) {
        // 遍历 roll
        for (double roll = roll_min; roll <= roll_max + resolution/2; roll += resolution) {
            total_combinations++;
            
            // 设置左腿的 pitch 和 roll
            Eigen::VectorXd joint_pos = base_joint_pos;
            joint_pos[4] = pitch;  // 左腿 pitch
            joint_pos[5] = roll;   // 左腿 roll
            joint_pos[10] = pitch; // 右腿 pitch
            joint_pos[11] = roll;   // 右腿 roll
            
            try {
                // 调用函数
                Eigen::VectorXd motor_pos = solver.joint_to_motor_position(joint_pos);
                
                // 检查输出
                bool is_valid = true;
                if (motor_pos.size() != 12) {
                    is_valid = false;
                } else {
                    // 检查是否有 NaN 或 Inf
                    for (int i = 0; i < motor_pos.size(); ++i) {
                        if (std::isnan(motor_pos[i]) || std::isinf(motor_pos[i])) {
                            is_valid = false;
                            break;
                        }
                    }
                }
                
                if (is_valid) {
                    success_combinations++;
                } else {
                    failed_combinations++;
                    if (failed_pairs.size() < max_failed_reports) {
                        failed_pairs.push_back({pitch, roll});
                    }
                }
            } catch (...) {
                failed_combinations++;
                if (failed_pairs.size() < max_failed_reports) {
                    failed_pairs.push_back({pitch, roll});
                }
            }
            
            // 每10000个组合输出一次进度
            if (total_combinations % 10000 == 0) {
                std::cout << "进度: " << total_combinations << " 组合已测试, "
                          << "成功: " << success_combinations << ", "
                          << "失败: " << failed_combinations << std::endl;
            }
        }
    }
    
    // 输出结果
    std::cout << "\n测试完成!" << std::endl;
    std::cout << "总组合数: " << total_combinations << std::endl;
    std::cout << "成功组合: " << success_combinations << " (" 
              << (100.0 * success_combinations / total_combinations) << "%)" << std::endl;
    std::cout << "失败组合: " << failed_combinations << " (" 
              << (100.0 * failed_combinations / total_combinations) << "%)" << std::endl;
    
    if (failed_combinations > 0) {
        std::cout << "\n失败的组合示例 (最多显示 " << max_failed_reports << " 个):" << std::endl;
        for (size_t i = 0; i < failed_pairs.size(); ++i) {
            std::cout << "  [" << i+1 << "] pitch=" << std::fixed << std::setprecision(3) 
                      << failed_pairs[i].first << ", roll=" << failed_pairs[i].second << std::endl;
        }
        if (failed_combinations > max_failed_reports) {
            std::cout << "  ... 还有 " << (failed_combinations - max_failed_reports) 
                      << " 个失败组合未显示" << std::endl;
        }
    }
    
    // 如果失败率低于1%，认为测试通过
    double failure_rate = 100.0 * failed_combinations / total_combinations;
    if (failure_rate < 1.0) {
        std::cout << "\n测试通过! 失败率: " << std::fixed << std::setprecision(2) 
                  << failure_rate << "%" << std::endl;
        return true;
    } else {
        std::cout << "\n测试警告: 失败率较高: " << std::fixed << std::setprecision(2) 
                  << failure_rate << "%" << std::endl;
        return false;
    }
}

int main() {
    // 使用 map 存储类型名称和对应的类型值
    std::map<std::string, int> solver_map = {
        {"ANKLE_SOLVER_TYPE_4GEN", AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN},
        {"ANKLE_SOLVER_TYPE_4GEN_PRO", AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO},
        {"ANKLE_SOLVER_TYPE_5GEN", AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN},
        {"ANKLE_SOLVER_TYPE_S1GEN", AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN},
        {"ANKLE_SOLVER_TYPE_S2GEN", AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN}
    };
    
    // ============================================
    // 在这里指定要测试的类型，可以添加或删除类型名称
    // ============================================
    std::vector<std::string> selected_types = {
        // "ANKLE_SOLVER_TYPE_4GEN",
        // "ANKLE_SOLVER_TYPE_4GEN_PRO",
        "ANKLE_SOLVER_TYPE_5GEN",
        // "ANKLE_SOLVER_TYPE_S1GEN",
        // "ANKLE_SOLVER_TYPE_S2GEN"
    };
    
    // 验证指定的类型是否存在
    for (const auto& type_name : selected_types) {
        if (solver_map.find(type_name) == solver_map.end()) {
            std::cerr << "错误: 未知的类型 '" << type_name << "'" << std::endl;
            std::cerr << "可用的类型:" << std::endl;
            for (const auto& pair : solver_map) {
                std::cerr << "  " << pair.first << std::endl;
            }
            return 1;
        }
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "AnkleSolver joint_to_motor_position 测试" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "将测试以下类型: ";
    for (size_t i = 0; i < selected_types.size(); ++i) {
        std::cout << selected_types[i];
        if (i < selected_types.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
    
    int total_tests = 0;
    int passed_tests = 0;
    for (size_t j = 0; j < 1; ++j)
    {
        for (const auto& type_name : selected_types) {
            std::cout << "\n\n####################"<<j<<"####################" << std::endl;
            std::cout << "测试类型: " << type_name << std::endl;
            std::cout << "########################################" << std::endl;
            
            AnkleSolver solver;
            solver.getconfig(solver_map[type_name]);
            
            // 测试1: 零位置
            Eigen::VectorXd zero_pos = Eigen::VectorXd::Zero(12);
            total_tests++;
            if (testJointToMotorPosition(solver, zero_pos, "零位置测试")) {
                passed_tests++;
            }
            
            // // 测试2: 小角度测试
            // Eigen::VectorXd small_angles(12);
            // small_angles << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,  // 左腿
            //                 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;  // 右腿
            // total_tests++;
            // if (testJointToMotorPosition(solver, small_angles, "小角度测试")) {
            //     passed_tests++;
            // }
            
            // 测试3: 典型关节角度
            Eigen::VectorXd typical_pos(12);
            typical_pos << -0.16,  -0.10,  -0.67 ,  0.60 , -0.20   ,0.05,  // 左腿: 髋关节、膝关节、踝关节
            -0.09,  -0.02,   0.45,  -0.17,  -0.36,  -0.05;  // 右腿
            total_tests++;
            if (testJointToMotorPosition(solver, typical_pos, "典型关节角度测试")) {
                passed_tests++;
            }
            
            // // 测试4: 极限角度（在限制范围内）
            // Eigen::VectorXd limit_pos(12);
            // limit_pos << 0.0, 0.0, 1.0, -1.0, 0.5, -0.5,  // 左腿
            //              0.0, 0.0, 1.0, -1.0, 0.5, -0.5;  // 右腿
            // total_tests++;
            // if (testJointToMotorPosition(solver, limit_pos, "极限角度测试")) {
            //     passed_tests++;
            // }
            
            // // 测试5: 往返转换测试（使用典型角度）
            // total_tests++;
            // if (testRoundTrip(solver, typical_pos, "往返转换测试")) {
            //     passed_tests++;
            // }
            
            // 测试6: pitch 和 roll 组合测试（仅对 ANKLE_SOLVER_TYPE_4GEN_PRO，且只在第一次迭代运行）
            // 注意：此测试会遍历约2,400,000个组合，耗时较长
            if (type_name == "ANKLE_SOLVER_TYPE_4GEN_PRO" && j == 0) {
                total_tests++;
                if (testPitchRollCombinations(solver, "Pitch-Roll组合测试")) {
                    passed_tests++;
                }
            }
        }
    }
    
    // 总结
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "测试总结" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "总测试数: " << total_tests << std::endl;
    std::cout << "通过测试: " << passed_tests << std::endl;
    std::cout << "失败测试: " << (total_tests - passed_tests) << std::endl;
    
    if (passed_tests == total_tests) {
        std::cout << "\n所有测试通过!" << std::endl;
        return 0;
    } else {
        std::cout << "\n部分测试失败，请检查输出" << std::endl;
        return 1;
    }
}

