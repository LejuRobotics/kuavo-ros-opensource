#ifndef __ECSDO_H
#define __ECSDO_H

#include <fstream>
#include <sstream>
#include <vector>
#include <thread>

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include <cstdlib>    // for realpath
#include <limits.h>   // for PATH_MAX
#include <cstring>    // for strerror
#include "ObjectDiction.h"

#pragma pack(1)

typedef struct
{
  std::string joint_name;
  uint32_t product_code; //0x1018 subindex 2 
  uint8_t motorParameterCode; //电机参数组
  uint16_t software_ver; //驱动器软件版本
  uint16_t driverWorkMode;//0x3507 驱动器工作模式
  uint16_t TorqueModeLimitOfVelocity;//0x3401  力矩模式速度限制
  uint32_t driverRatingCount;//0x6075 关节额定电流值
  uint16_t joint_csp_kp;//0x3500 位置环P增益
  uint16_t joint_csp_offset;//0x3502 位置环前馈
  uint16_t joint_csv_kp;//0x3504 速度环P增益
  uint16_t joint_csv_ki;//0x3505 速度环积分增益
  uint16_t joint_csp_command_filter;//0x3532位置指令滤波
  uint16_t EncoderFeedbackMode;//0x381C 编码器多圈模式
} EC_DriversConfigParameter_t;

// JSON字段描述符，用于定义配置文件中字段的元数据和转换规则
struct JsonFieldDescriptor {
    std::string name;           // 字段名(JSON中的字段名)
    std::string unit;           // 单位
    float scale_factor;         // 缩放因子
    std::string format;         // 格式化字符串
    uint16_t obIndex;          // 对象索引
    uint16_t subIndex;         // 子索引
    
    // 获取值的函数
    using ToStringFunc = std::function<std::string(const EC_DriversConfigParameter_t&)>;
    ToStringFunc toString;

    // 获取原始int值的函数
    using ToRawIntFunc = std::function<int32_t(const EC_DriversConfigParameter_t&)>;
    ToRawIntFunc toRawInt;
    
    // 根据json设置值的函数
    using FromJsonFunc = std::function<void(EC_DriversConfigParameter_t&, const nlohmann::json&)>;
    FromJsonFunc fromJson;
    
    // 直接设置原始int值的函数
    using SetRawIntFunc = std::function<void(EC_DriversConfigParameter_t&, int32_t)>;
    SetRawIntFunc setRawInt;
};

// 错误信息结构体（增加关节索引和详细描述）
struct ConfigMismatch {
    size_t joint_index;       // 关节序号（从0开始）
    std::string joint_name;   // 关节名称
    std::string field_name;   // 参数名称
    
    std::vector<EC_DriversConfigParameter_t>json_value;   // JSON配置值
    std::vector<EC_DriversConfigParameter_t> driver_value; // 驱动器实际值
    // std::string json_value;   // JSON配置值
    // std::string driver_value; // 驱动器实际值
    
    std::string description;  // 错误描述
};


typedef struct
{
  uint8_t SlaveId;
  uint16_t ObIndex;
  uint16_t SubIndex;
  int32_t read_data;
}SdoRead_t;

typedef struct
{
  uint8_t SlaveId;
  uint16_t ObIndex;
  uint16_t SubIndex;
  int32_t write_data;
  bool save;
}SdoWrite_t;

typedef enum
{
  SDO_STATUS_NONE = 0,        // 无操作
  SDO_STATUS_READ_ALL = 1,    // 读取所有驱动器配置
  SDO_STATUS_WRITE_ALL = 2,   // 写入所有驱动器配置
  SDO_STATUS_READ_SINGLE = 3, // 读取单个配置
  SDO_STATUS_WRITE_SINGLE = 4, // 写入单个配置
  SDO_STATUS_SELF_LEARN = 5, // 电机自学习
  SDO_STATUS_READ_ERROR_CODE = 6 // 读取error code
}en_SdoStatus;

#pragma pack()

//读单个索引
bool readSingleSdo(const uint8_t SlaveId,const uint16_t ObIndex,const uint16_t SubIndex,int32_t* read_data);
//对单个配置进行写入
bool writeSingleSdo(const uint8_t SlaveId,const uint16_t ObIndex,const uint16_t SubIndex,int32_t* write_data, bool save);
//按照json表写入所有配置
bool robotSetAllConfig(const uint8_t all_num_slave, const char *conJsonPath);
//获得所有配置，并保存到json表
std::vector<EC_DriversConfigParameter_t> getRobotAllDriverConfigParameter(const uint8_t all_num_slave);
//获得json表配置
std::vector<EC_DriversConfigParameter_t> getJsonDriverConfigParameter(const char* filename_);
//对比配置，获得错误列表
std::vector<ConfigMismatch> validateDriverConfigurations(const std::vector<EC_DriversConfigParameter_t>& jsonConfigs,const std::vector<EC_DriversConfigParameter_t>& realConfigs);
//打印错误信息
void printValidationResults(const std::vector<ConfigMismatch>& mismatches);

void saveStructToJson(const std::vector<EC_DriversConfigParameter_t>& driver_data, const std::string& filePath);

uint32_t hexStringToNumber(const std::string& hex_str);


en_SdoStatus getSdoStatus();
void setSdoStatus(en_SdoStatus status);

extern SdoRead_t SdoRead; // SDO读取
extern SdoWrite_t SdoWrite; // SDO写入
extern bool Sdo_RW_Res; // SDO读写结果
extern std::string g_conJsonPath; // 配置文件路径
extern std::vector<EC_DriversConfigParameter_t> json_config_parameter;
extern std::vector<EC_DriversConfigParameter_t> real_config_parameter;

// 字段描述符注册与查找
namespace FieldRegistry {
    extern const std::vector<JsonFieldDescriptor> ALL_FIELD_DESCRIPTORS;
    const JsonFieldDescriptor* getFieldDescriptor(const std::string& field_name);
    std::vector<const JsonFieldDescriptor*> getFieldDescriptors(const std::vector<std::string>& field_names);
    const JsonFieldDescriptor* getFieldDescriptorByIndex(uint16_t obIndex, uint16_t subIndex);
}

// 字段名列表管理
namespace FieldNameList {
    extern const std::vector<std::string> READ_FIELD_NAMES;
    extern const std::vector<std::string> WRITE_FIELD_NAMES;
    std::vector<std::string> getReadFieldNames();
    std::vector<std::string> getWriteFieldNames();
    bool isReadField(const std::string& field_name);
    bool isWriteField(const std::string& field_name);
}

bool motorSelfLearning(uint8_t slave_id, int32_t learn_mode, uint8_t timeout_seconds);
bool readAllErrorCode(uint8_t all_num_slave,std::vector<int32_t>& error_code_list);
uint8_t checkErrorCode(const std::vector<int32_t>& error_code_list);

#endif
