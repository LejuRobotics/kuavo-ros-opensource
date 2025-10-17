#include "EcLogFile.h"
#include "EcLogging.h"
#include "EcInterfaceCommon.h"

// 从log文件读取cyclic command: working counter error
bool readWkcErrorFromLog(const std::string& logFilePath, std::vector<EC_T_WKCERR_DESC>& error_list)
{
    std::ifstream file(logFilePath);
    if (!file.is_open()) {
        printf("\033[1;31m[错误]\033[0m 无法打开文件: %s\n", logFilePath.c_str());
        return false;
    }

    std::string line;
    std::regex pattern(R"(EC_NOTIFY_CYCCMD_WKC_ERROR\((\d+), 0x([0-9a-fA-F]+), (\d+), (\d+)\))");
    
    struct ErrorKey {
        EC_T_BYTE byCmd;
        EC_T_DWORD dwAddr;
        EC_T_WORD wWkcAct;
        EC_T_WORD wWkcSet;
        
        bool operator<(const ErrorKey& other) const {
            if (byCmd != other.byCmd) return byCmd < other.byCmd;
            if (dwAddr != other.dwAddr) return dwAddr < other.dwAddr;
            if (wWkcAct != other.wWkcAct) return wWkcAct < other.wWkcAct;
            return wWkcSet < other.wWkcSet;
        }
    };
    
    std::map<ErrorKey, int> error_count;
    
    while (std::getline(file, line)) {
        std::smatch matches;
        if (std::regex_search(line, matches, pattern)) {
            EC_T_WKCERR_DESC error;
            error.byCmd = static_cast<EC_T_BYTE>(std::stoi(matches[1].str()));
            error.dwAddr = std::stoul(matches[2].str(), nullptr, 16);
            error.wWkcAct = static_cast<EC_T_WORD>(std::stoi(matches[3].str()));
            error.wWkcSet = static_cast<EC_T_WORD>(std::stoi(matches[4].str()));
            error_list.push_back(error);
            
            ErrorKey key{error.byCmd, error.dwAddr, error.wWkcAct, error.wWkcSet};
            error_count[key]++;
        }
    }
    file.close();

    if (error_list.empty()) {
        printf("\033[32m✔ 未发现丢帧错误\033[0m\n");
        return true;
    }

    printf("\n\033[1;34m====== 丢帧错误检查报告 ======\033[0m\n");
    printf("共发现 \033[1;33m%zu\033[0m 种EC_NOTIFY_CYCCMD_WKC_ERROR错误:\n", error_count.size());
    for (const auto& pair : error_count) {
        const auto& key = pair.first;
        const auto& count = pair.second;
        printf("\033[1;31m[错误]\033[0m ");
        printf("命令: \033[1;33m%d\033[0m, 地址: \033[1;33m0x%08X\033[0m, 实际/期望应答从站数量: \033[1;31m%d/%d\033[0m, 次数: \033[1;31m%d\033[0m\n", 
            key.byCmd, key.dwAddr, key.wWkcAct, key.wWkcSet, count);
    }
    printf("\033[1;34m-----------------------------\033[0m\n");

    return true;
}

/**
 * @brief 从日志文件中读取冗余线路断开事件信息
 * 
 * 该函数用于分析EtherCAT主站日志文件，提取最后一次冗余线路断开事件的相关信息。
 * 主要功能包括：
 * 1. 检测冗余线路断开事件
 * 2. 记录最后一次断开时的主线路和冗余线路从站数量
 * 3. 检测冗余线路是否被修复
 * 
 * @param logFilePath 日志文件的完整路径
 * @return bool 函数执行是否成功
 *         - true: 成功读取并分析日志文件
 *         - false: 无法打开日志文件
 */
bool readRedundancyLineFromLog(const std::string& logFilePath)
{
    // 定义冗余线路事件信息结构体
    struct RedundancyInfo {
        int mainSlaves;        // 主线路从站数量
        int redSlaves;         // 冗余线路从站数量
    };
    
    // 打开日志文件
    std::ifstream file(logFilePath);
    if (!file.is_open()) {
        printf("\033[1;31m[错误]\033[0m 无法打开文件: %s\n", logFilePath.c_str());
        return false;
    }

    // 定义正则表达式模式
    std::string line;
    std::regex pattern(R"(ERROR: Redundancy Line break (\d+) slaves on main and (\d+) slaves on red)");
    std::regex fixed_pattern(R"(Redundancy - Line fixed)");
    
    // 存储最后一次断开事件的信息
    RedundancyInfo* lastBreakEvent = nullptr;
    bool lineFixed = false;
    
    // 逐行读取日志文件
    while (std::getline(file, line)) {
        std::smatch matches;
        // 检查是否匹配冗余线路断开事件
        if (std::regex_search(line, matches, pattern)) {
            // 如果已有事件，先删除
            if (lastBreakEvent != nullptr) {
                delete lastBreakEvent;
            }
            // 创建新事件
            lastBreakEvent = new RedundancyInfo();
            // 解析主线路从站数量
            lastBreakEvent->mainSlaves = std::stoi(matches[1].str());
            // 解析冗余线路从站数量
            lastBreakEvent->redSlaves = std::stoi(matches[2].str());
        }
        // 检查是否匹配冗余线路修复事件
        else if (std::regex_search(line, fixed_pattern)) {
            lineFixed = true;
        }
    }
    
    file.close();

    // 如果没有发现任何断开事件
    if (lastBreakEvent == nullptr) {
        printf("\033[32m✔ 未发现冗余线路断开事件\033[0m\n");
        return true;
    }

    printf("\n\033[1;34m====== 冗余线路检查报告 ======\033[0m\n");
    // 输出最后一次断开事件的详细信息
    printf("冗余线路断开 - 主线路从站数: \033[1;33m%d\033[0m, 冗余线路从站数: \033[1;33m%d\033[0m\n", 
        lastBreakEvent->mainSlaves, lastBreakEvent->redSlaves);
    
    if(lastBreakEvent->mainSlaves != lastBreakEvent->redSlaves)
    {
        printf("\033[1;31m[错误]\033[0m 主线路或冗余线路可能断连，请检查EC线路连接\n");
        delete lastBreakEvent;
        printf("\033[1;34m-----------------------------\033[0m\n");
        return false;
    }
    
    // 如果检测到线路修复事件，输出修复信息
    if (lineFixed) {
        printf("\033[32m✔ 冗余线路已修复\033[0m\n");
    }
    
    printf("\033[1;34m-----------------------------\033[0m\n");

    // 清理内存
    delete lastBreakEvent;
    return true;
}
