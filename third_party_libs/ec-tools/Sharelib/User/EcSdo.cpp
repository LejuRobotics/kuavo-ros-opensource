// ==============================================
// ================== INCLUDES ==================
// ==============================================
#include "EcDemoApp.h"
#include "EcSdo.h"
#include "util.h"
#include <sys/stat.h>

using json = nlohmann::json;

// ==============================================
// ================== 全局变量区 ==================
// ==============================================
static en_SdoStatus Sdo_status = SDO_STATUS_NONE; // SDO状态 选择
SdoRead_t SdoRead;                         // SDO读取
SdoWrite_t SdoWrite;                       // SDO写入
bool Sdo_RW_Res = false;                   // SDO读写结果

std::string g_conJsonPath = "./config/sdo_config/YouDa.json";

std::vector<EC_DriversConfigParameter_t> json_config_parameter{};
std::vector<EC_DriversConfigParameter_t> real_config_parameter{};

// ==============================================
// ============ 字段描述符与字段名管理区 ============
// ==============================================
#define FIELD_DESC(field_name, unit, scale, format, type, ob_index, sub_index) \
    {#field_name, unit, scale, format, ob_index, sub_index, \
    [](const auto& p) { return std::to_string((double)(p.field_name) / (scale)); }, \
    [](const auto& p) { return static_cast<int32_t>(p.field_name); }, \
    [](auto& p, const nlohmann::json& j) { \
        if (j.contains(#field_name)) { \
            p.field_name = j[#field_name].get<type>() * (scale); \
            p.field_name = j[#field_name].get<type>() * (scale); \
        } else { \
            p.field_name = 0; \
        } \
    }, \
    [](auto& p, int32_t value) { p.field_name = value; }}

#define FIELD_DESC_DOUBLE(field_name, unit, scale, format, ob_index, sub_index) \
    FIELD_DESC(field_name, unit, scale, format, double, ob_index, sub_index)

#define FIELD_DESC_STRING(field_name, unit, scale, format, ob_index, sub_index) \
    {#field_name, unit, scale, format, ob_index, sub_index, \
    [](const auto& p) { return p.field_name; }, \
    [](const auto& p) { return 0; }, \
    [](auto& p, const nlohmann::json& j) { \
        if (j.contains(#field_name)) { \
            p.field_name = j[#field_name].get<std::string>(); \
        } \
    }, \
    nullptr }

#define FIELD_DESC_HEXSTR(field_name, unit, scale, format, ob_index, sub_index) \
    {#field_name, unit, scale, format, ob_index, sub_index, \
    [](const auto& p) { return "0x" + std::to_string(p.field_name); }, \
    [](const auto& p) { return static_cast<int32_t>(p.field_name); }, \
    [](auto& p, const nlohmann::json& j) { \
        if (j.contains(#field_name)) { \
            p.field_name = hexStringToNumber(j[#field_name].get<std::string>()); \
        } else { \
            p.field_name = 0; \
        } \
    }, \
    nullptr }

namespace FieldRegistry {
    const std::vector<JsonFieldDescriptor> ALL_FIELD_DESCRIPTORS = {
        FIELD_DESC_STRING(joint_name, "", 1, "%s", 0, 0),
        FIELD_DESC_HEXSTR(product_code, "", 1, "0x%x", 0, 0),
        FIELD_DESC_DOUBLE(software_ver, "", 1, "%u", DRIVER_SOFTWARE_VERSION, 0),
        FIELD_DESC_DOUBLE(motorParameterCode, "组", 1, "%u", MOTOR_PARAMETER_CODE, 0),
        FIELD_DESC_DOUBLE(driverWorkMode, "模式", 1, "%u", DRIVER_WORK_MODE, 0),
        FIELD_DESC_DOUBLE(TorqueModeLimitOfVelocity, "RPM", 1, "%u", TORQUE_MODE_VELOCITY_LIMIT, 0),
        FIELD_DESC_DOUBLE(driverRatingCount, "A", 1000, "%.1f", INDEX_MOTOR_RATED_CURRENT, 0),
        FIELD_DESC_DOUBLE(joint_csp_kp, "", 10, "%.1f", JOINT_CSP_KP, 0),
        FIELD_DESC_DOUBLE(joint_csp_offset, "", 10, "%.1f", JOINT_CSP_OFFSET, 0),
        FIELD_DESC_DOUBLE(joint_csv_kp, "", 1000, "%.1f", JOINT_CSV_KP, 0),
        FIELD_DESC_DOUBLE(joint_csv_ki, "", 10, "%.1f", JOINT_CSV_KI, 0),
        FIELD_DESC_DOUBLE(joint_csp_command_filter, "", 100, "%.1f", JOINT_CSP_COMMAND_FILTER, 0),
        FIELD_DESC_DOUBLE(EncoderFeedbackMode, "", 1, "%u", ENCODER_FEEDBACK_MODE, 0)
    };
    const JsonFieldDescriptor* getFieldDescriptor(const std::string& field_name) {
        for (const auto& desc : ALL_FIELD_DESCRIPTORS) {
            if (desc.name == field_name) {
                return &desc;
            }
        }
        return nullptr;
    }
    std::vector<const JsonFieldDescriptor*> getFieldDescriptors(const std::vector<std::string>& field_names) {
        std::vector<const JsonFieldDescriptor*> descriptors;
        descriptors.reserve(field_names.size());
        for (const auto& name : field_names) {
            if (const auto* desc = getFieldDescriptor(name)) {
                descriptors.push_back(desc);
            }
        }
        return descriptors;
    }
    const JsonFieldDescriptor* getFieldDescriptorByIndex(uint16_t obIndex, uint16_t subIndex) {
        for (const auto& desc : ALL_FIELD_DESCRIPTORS) {
            if (desc.obIndex == obIndex && desc.subIndex == subIndex) {
                return &desc;
            }
        }
        return nullptr;
    }
}

namespace FieldNameList {
    const std::vector<std::string> READ_FIELD_NAMES = {
        "motorParameterCode",
        "software_ver",
        "driverWorkMode",
        "TorqueModeLimitOfVelocity",
        "driverRatingCount",
        "joint_csp_kp",
        "joint_csp_offset",
        "joint_csv_kp",
        "joint_csv_ki",
        "joint_csp_command_filter",
        "EncoderFeedbackMode"
    };
    const std::vector<std::string> WRITE_FIELD_NAMES = {
        "motorParameterCode",
        "driverWorkMode",
        "TorqueModeLimitOfVelocity",
        "joint_csp_kp",
        "joint_csp_offset",
        "joint_csv_kp",
        "joint_csv_ki",
        "joint_csp_command_filter",
        "EncoderFeedbackMode"
    };
    std::vector<std::string> getReadFieldNames() { return READ_FIELD_NAMES; }
    std::vector<std::string> getWriteFieldNames() { return WRITE_FIELD_NAMES; }
    bool isReadField(const std::string& field_name) {
        return std::find(READ_FIELD_NAMES.begin(), READ_FIELD_NAMES.end(), field_name) != READ_FIELD_NAMES.end();
    }
    bool isWriteField(const std::string& field_name) {
        return std::find(WRITE_FIELD_NAMES.begin(), WRITE_FIELD_NAMES.end(), field_name) != WRITE_FIELD_NAMES.end();
    }
}

static const std::map<int32_t, std::string> errorMsgMap = {
    {0x001, "硬件短路保护  处理方式：启驱动器，如果重启后还会就换驱动器"},
    {0x006, "AD采样故障  处理方式：换驱动器"},
    {0x007, "编码器断线  处理方式：检查编码器连接"},
    {0x008, "编码器CRC校验错误  处理方式：检查编码器连接与供电，不行就换编码器"},
    {0x009, "编码器位置计数错误  处理方式：换编码器"},
    {0x00A, "母线电压低于15V  处理方式：检查大板供电"},
    {0x00B, "母线电压高于72V  处理方式：检查大板供电"},
    {0x00C, "电机欠输出  处理方式：电机三相线断了一路或是驱动器功率模块烧坏，先查电机三相线，没解决就换驱动器"},
    {0x00D, "电机过载  处理方式：重启驱动器"},
    {0x00E, "驱动器过载  处理方式：重启驱动器"},
    {0x010, "驱动器过温  处理方式：检查驱动器散热，通过0x300F查看驱动器温度是否正常，如果只有单个驱动器温度异常，建议重新自学习"},
    {0x012, "驱动器过速  处理方式：查询0x3E06、0x3E07参数是否符合电机参数组的数值，不符合则重新刷驱动器固件，符合则转研发查算法"},
    {0x013, "位置偏差过大  处理方式：根据关节电机是否失控判断是否没自学习成功，运行的时候有没有发热异常，有以上现象先自学习"},
    {0x019, "编码器电池电压低于3.1V  处理方式：优先多圈复位，其次检查编码器电池"},
    {0x01A, "编码器电池电压低于2.5V  处理方式：优先多圈复位，其次检查编码器电池"},
    {0x01B, "驱动器与电机匹配错误  处理方式：如果开了多圈，检查0x3E15是否为1，0x381C是否为2"},
    {0x022, "ECAT通过网线与主机通讯中断  处理方式：重启或重新跑程序都可以解决"},
    {0x036, "电机自学习完成  处理方式：重启驱动器"}
};

// ==============================================
// ================== SDO操作区 ==================
// ==============================================
en_SdoStatus getSdoStatus()
{
  return Sdo_status;
}
void setSdoStatus(en_SdoStatus status)
{ 
  Sdo_status = status;
}

/// @brief 单次sdo读取
/// @param SlaveId 驱动器id
/// @param ObIndex 对象索引
/// @param SubIndex 子索引
/// @param out_data 数据指针
/// @return
bool readSingleSdo(const uint8_t SlaveId, const uint16_t ObIndex, const uint16_t SubIndex, int32_t *read_data)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  uint8_t buf[4];
  int32_t value;
  uint32_t outdata_len;

  dwRes = emCoeSdoUpload(0, SlaveId, ObIndex, SubIndex, buf, 4, &outdata_len, 100, 0);
  if (EC_E_NOERROR != dwRes)
  {
    printf("Failed to emCoeSdoUpload, Slave %d, Err (0x%x)\n", SlaveId+1, dwRes);
    return false;
  }
  if (outdata_len == 2)
  {
    // 读取的是16位数据
    *read_data = (buf[1] << 8) | buf[0];
  }
  else if (outdata_len == 4)
  {
    *read_data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]; // 对读出来的值进行转换拼接
  }
  return true;
}

/// @brief 单次sdo写入
/// @param SlaveId 驱动器id
/// @param ObIndex 对象索引
/// @param SubIndex 子索引
/// @param in_data 数据指针
/// @param save 本次写入是否需要断电后能保存？true:保存，false:不保存
/// @return
bool writeSingleSdo(const uint8_t SlaveId, const uint16_t ObIndex, const uint16_t SubIndex, int32_t *write_data, bool save)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  uint8_t buf[4];
  static int32_t save_value = 11;
  int32_t value;

  toLittleEndian32(*write_data, buf);

  dwRes = emCoeSdoDownload(0, SlaveId, ObIndex, SubIndex, buf, 4, 100, 0);
  if (EC_E_NOERROR != dwRes)
  {
    printf("Failed to emCoeSdoDownload, Slave %d, Err (0x%x)\n", SlaveId+1, dwRes);
    return false;
  }
  // 需要保存的修改配置进行写入保存
  if (save == true)
  {
    printf("writeSingleSdo save\n");
    toLittleEndian32(save_value, buf);
    dwRes = emCoeSdoDownload(0, SlaveId, DRIVER_SAVE_SETTING_PARAMETER, 0, buf, 4, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      printf("Failed to emCoeSdoDownload of save, Slave %d, Err (0x%x)\n", SlaveId+1, dwRes);
      return false;
    }
  }
  return true;
}

// ==============================================
// ================ 配置文件读写区 ================
// ==============================================
// 将十六进制字符串转换为数字
uint32_t hexStringToNumber(const std::string &hex_str)
{
  // 移除可能存在的 "0x" 前缀
  std::string clean_hex = hex_str;
  if (hex_str.substr(0, 2) == "0x")
  {
    clean_hex = hex_str.substr(2);
  }
  return std::stoul(clean_hex, nullptr, 16);
}

/// @brief 将结构体数据保存为JSON文件
/// @param driver_data 结构体数据
/// @param filePath 文件路径
/// @return
void saveStructToJson(const std::vector<EC_DriversConfigParameter_t> &driver_data, const std::string &filePath)
{
  // 如果路径所在的目录不存在，则创建目录
  std::string dirPath = filePath.substr(0, filePath.find_last_of('/'));
  if (!dirPath.empty())
  {
    int status = mkdir(dirPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0 && errno != EEXIST)
    {
      std::cerr << "无法创建目录: " << strerror(errno) << std::endl;
      return;
    }
  }

  nlohmann::ordered_json j;
  j["YoudaDrivers"] = nlohmann::ordered_json::array();

  // 只保存READ_FIELD_NAMES中的字段
  const auto& read_field_names = FieldNameList::READ_FIELD_NAMES;
  auto read_field_descriptors = FieldRegistry::getFieldDescriptors(read_field_names);

  for (const auto &driver : driver_data)
  {
    nlohmann::ordered_json driver_json;
    for (const auto* field : read_field_descriptors) {
        driver_json[field->name] = field->toString(driver);
    }
    j["YoudaDrivers"].push_back(driver_json);
  }

  // 打开文件并写入JSON数据
  std::ofstream ofs(filePath);
  if (!ofs)
  {
    std::cerr << "无法打开文件写入: " << filePath << std::endl;
    return;
  }
  // 使用缩进4个空格格式化输出
  ofs << j.dump(4);
  ofs.close();
}

std::vector<EC_DriversConfigParameter_t> getJsonDriverConfigParameter(const char *filename)
{
  printf("开始读取配置文件: %s\n", filename);
  if (!filename){
    printf("错误：文件路径为空\n");
    return {};
  }
  std::ifstream file(filename);
  if (!file.is_open()){
    printf("错误：无法打开文件\n");
    return {};
  }

  nlohmann::json ConfigJson;
  file >> ConfigJson;
  file.close();

  // 验证JSON结构
  if (!ConfigJson.contains("YoudaDrivers")){
    printf("错误：配置文件中缺少YoudaDrivers字段\n");
    return {};
  }
  const auto &drivers = ConfigJson["YoudaDrivers"];
  const size_t driver_count = drivers.size();
  printf("JSON文件中找到 %zu 个优达驱动器配置\n", driver_count);

  std::vector<EC_DriversConfigParameter_t> parameter_configs;
  parameter_configs.reserve(driver_count);
  for (const auto &driver : drivers)
  {
    EC_DriversConfigParameter_t parameter{};
    for (const auto &field : FieldRegistry::ALL_FIELD_DESCRIPTORS)
    {
      // if (field.name == "joint_csv_kp") {
      //   printf("\n调试 - joint_csv_kp转换:\n");
      //   printf("JSON原始值: %f\n", driver[field.name].get<double>());
        field.fromJson(parameter, driver);
      //   printf("转换后值: %d\n", parameter.joint_csv_kp);
      //   printf("显示值: %f\n", (double)parameter.joint_csv_kp / field.scale_factor);
      // } else {
      //   field.fromJson(parameter, driver);
      // }
    }
    parameter_configs.push_back(parameter);
  }

  // printf("\n=== getJson 驱动器配置参数 ===\n");
  // for (size_t i = 0; i < driver_count; i++)
  // {
  //   const auto &param = parameter_configs[i];
  //   printf("\n驱动器 %zu (%s):\n", i + 1, param.joint_name.c_str());
  //   for (const auto &field : FieldRegistry::ALL_FIELD_DESCRIPTORS)
  //   {
  //     printf("  %s: %s%s\n",
  //            field.name.c_str(),
  //            field.toString(param).c_str(),
  //            field.unit.c_str());
  //   }
  // }
  // printf("\n=== 配置打印完成 ===\n");
  return parameter_configs;
}

/// @brief 从整机实物驱动器中读取优达驱动器的配置参数
/// @param 从站总数
/// @return parameter_configs
std::vector<EC_DriversConfigParameter_t> getRobotAllDriverConfigParameter(const uint8_t all_num_slave)
{
  int32_t temp_value;
  // 驱动器类型分布
  // std::vector<std::string> driverTypes = {"H1", "H2", "H3", "H4", "H5", "H6","H1",
  //                                         "H2", "H3", "H4", "H5", "H6", "A1","A1"};
  std::vector<EC_DriversConfigParameter_t> parameter_configs;
  for (uint8_t i = 0; i < all_num_slave; i++)
  {
    EC_DriversConfigParameter_t parameter{};
    // parameter.joint_name = driverTypes[i];

    // TODO 驱动器product code 索引不确定
    // readSingleSdo(i,INDEX_PRODUCT_CODE,0,temp_value); //读product code
    // parameter.product_code = temp_value;

    // 遍历read列表
    const auto& read_fields = FieldNameList::READ_FIELD_NAMES;
    for (const auto& field_name : read_fields)
    {
      const auto* field_desc = FieldRegistry::getFieldDescriptor(field_name);
      if (field_desc && field_desc->obIndex != 0)  // 只处理有SDO索引的字段
      {
        if (!readSingleSdo(i, field_desc->obIndex, field_desc->subIndex, &temp_value))
        {
          printf("getRobotAllDriverConfigParameter: Failed to read field %s for slave %d\n", 
                 field_name.c_str(), i+1);
          return {};
        }
        field_desc->setRawInt(parameter, temp_value);
      }
    }
    parameter_configs.push_back(parameter);
  }
  return parameter_configs;
}

/// @brief 写入驱动器配置
/// @param pAppContext
/// @return
bool robotSetAllConfig(const uint8_t all_num_slave, const char *conJsonPath)
{
  uint8_t buf_32_t[4];
  auto configs = getJsonDriverConfigParameter(conJsonPath);
  if (configs.empty()){
    printf("错误：无法读取配置文件或配置文件为空\n");
    return false;
  }
  for (uint32_t i = 0; i < all_num_slave; i++)
  {
    const int32_t to_save = 11;
    for (const auto& field : FieldRegistry::getFieldDescriptors(FieldNameList::WRITE_FIELD_NAMES))
    {
      if (field && field->obIndex != 0)
      {
        int32_t value = field->toRawInt(configs[i]);
        toLittleEndian32(value, buf_32_t);
        if (EC_E_NOERROR != emCoeSdoDownload(0, i, field->obIndex, field->subIndex, buf_32_t, 2, 100, 0)){
          printf("Failed to emCoeSdoDownload slave: %d, field: %s\n", i+1, field->name.c_str());
          return false;
        }
      }
    }
    toLittleEndian32(to_save, buf_32_t);
    if (EC_E_NOERROR != emCoeSdoDownload(0, i, DRIVER_SAVE_SETTING_PARAMETER, 0, buf_32_t, 2, 100, 0)){
      printf("Slave_id: %d, Failed to save emCoeSdoDownload!\n", i+1);
      return false;
    }
    printf("Slave_id: %d, Save emCoeSdoDownload successfully!\n", i+1);
  }
  return true;
}
// ==============================================
// =============== 配置校验与打印区 ===============
// ==============================================
// 校验函数实现
std::vector<ConfigMismatch> validateDriverConfigurations(const std::vector<EC_DriversConfigParameter_t> &jsonConfigs, const std::vector<EC_DriversConfigParameter_t> &realConfigs)
{
  if (jsonConfigs.empty() || realConfigs.empty())
    return {};

  std::vector<ConfigMismatch> mismatches;
  const size_t jsonConfigSize = jsonConfigs.size();
  const size_t realConfigSize = realConfigs.size();

  const size_t mixsize = jsonConfigSize < realConfigSize ? jsonConfigSize : realConfigSize;
  for (size_t i = 0; i < mixsize; ++i)
  {
    const auto &realParam = realConfigs[i];
    const auto &jsonParam = jsonConfigs[i];
    // printf("realParam.joint_name: %s, jsonParam.joint_name: %s\n", realParam.joint_name.c_str(), jsonParam.joint_name.c_str());

    // // 基础校验：关节名称匹配
    // if (realParam.joint_name != jsonParam.joint_name)
    // {
    //   mismatches.push_back({i,
    //                         realParam.joint_name,
    //                         "joint_name",
    //                         {jsonParam},
    //                         {realParam},
    //                         "关节名称不匹配，可能配置顺序错误"});
    //   continue;
    // }

    // 只对比READ_FIELD_NAMES中的字段
    for (const auto& field_name : FieldNameList::READ_FIELD_NAMES) {
        const auto* field = FieldRegistry::getFieldDescriptor(field_name);
        if (!field) continue;
        const std::string jsonVal = field->toString(jsonParam);
        const std::string realVal = field->toString(realParam);
        if (jsonVal != realVal) {
            mismatches.push_back({i,
                                realParam.joint_name,
                                field->name,
                                {jsonParam},
                                {realParam},
                                "参数值不匹配: 预期=" + jsonVal + ", 实际=" + realVal});
        }
    }
  }

  return mismatches;
}

// 增强版结果打印函数
void printValidationResults(const std::vector<ConfigMismatch> &mismatches)
{
  if (mismatches.empty())
  {
    std::cout << "\033[32m✔ 所有驱动器配置校验通过\033[0m\n";
    return;
  }

  // 按关节分组错误
  std::map<size_t, std::vector<ConfigMismatch>> groupedErrors;
  for (const auto &err : mismatches)
  {
    groupedErrors[err.joint_index].push_back(err);
  }

  // 打印报告头
  std::cerr << "\n\033[31m▓▓▓ 配置校验报告 ▓▓▓\033[0m\n";
  std::cerr << "共发现 " << mismatches.size() << " 个问题，涉及 "
            << groupedErrors.size() << " 个关节\n\n";

  // 打印每个关节的问题
  for (const auto &pair : groupedErrors)
  {
    size_t index = pair.first;
    const std::vector<ConfigMismatch> &errors = pair.second;

    std::cerr << "\033[33m■ 关节 " << (index + 1) << "\033[0m\n";
    // << " (" << errors[0].joint_name << ")\033[0m\n";

    for (const auto &err : errors)
    {
      // std::cerr << "  ├─ \033[36m" << err.field_name << "\033[0m ";

      // const auto &jsonParam = err.json_value[0];
      // const auto &realParam = err.driver_value[0];

      // auto it = std::find_if(FieldRegistry::ALL_FIELD_DESCRIPTORS.begin(), 
      //                       FieldRegistry::ALL_FIELD_DESCRIPTORS.end(),
      //                       [&err](const JsonFieldDescriptor& desc) { 
      //                           return desc.name == err.field_name; 
      //                       });
      
      // if (it != FieldRegistry::ALL_FIELD_DESCRIPTORS.end())
      // {
      //   std::string expected = it->toString(jsonParam);
      //   std::string actual = it->toString(realParam);
      //   std::cerr << "预期: \033[32m" << expected << "\033[0m / "
      //             << "实际: \033[31m" << actual << "\033[0m\n";
      // }
      // else
      // {
      //   // 如果找不到转换器，显示N/A
      //   std::cerr << "预期: \033[32mN/A\033[0m / "
      //             << "实际: \033[31mN/A\033[0m\n";
      // }
      std::cerr << "  ├─ \033[36m" << err.field_name << "\033[0m\n";
      std::cerr << "  └─ \033[90m" << err.description << "\033[0m\n";
    }
    std::cerr << std::endl;
  }
}

bool motorSelfLearning(uint8_t slave_id, int32_t learn_mode, uint8_t timeout_seconds)
{
  int32_t write_data = learn_mode;
  int32_t error_code = 0;
  uint8_t count = 0;
  count = 0;
  if (!writeSingleSdo(slave_id,MOTOR_SELF_LEARN_PARAMETER,0,&write_data,false))
  {
    printf("Slave %d Failed to write motor self learn parameter\n", (slave_id+1));
    return false;
  }
  while(1)
  {
    if(!readSingleSdo(slave_id,INDEX_ERROR_CODE,0,&error_code))
    {
      printf("Slave %d Failed to read error code\n", (slave_id+1));
      return false;
    }
    if(error_code == MOTOR_SELF_LEARN_COMPLETE_FLAG)
    {
      break;
    }
    count++;
    if(count > timeout_seconds)
    {
      printf("Slave %d motor self learn timeout\n", (slave_id+1));
      return false;
    }
    sleep(1);
    printf("Slave %d is learning...\n", (slave_id+1));
  }
  printf("Slave %d motor self learn complete!\n", (slave_id+1));
  // printf("error_code: %d\n", error_code);
  return true;
}

// 读取error code
bool readAllErrorCode(uint8_t all_num_slave,std::vector<int32_t> &error_code_list)
{
  error_code_list.reserve(all_num_slave);
  error_code_list.clear();
  int32_t error_code = 0;
  for (uint8_t i = 0; i < all_num_slave; i++)
  {
    if(!readSingleSdo(i,INDEX_ERROR_CODE,0,&error_code))
    {
      printf("Slave %d Failed to read error code\n", (i+1));
      return false;
    }
    error_code_list.push_back(error_code);
  }
  return true;
}

// 检查error code
uint8_t checkErrorCode(const std::vector<int32_t> &error_code_list)
{
  uint8_t error_count = 0;
  if (error_code_list.empty())
  {
    printf("\033[32m✔ 没有检测到任何错误码\033[0m\n");
    return 0;
  }
  printf("\n\033[1;34m====== 错误码检查报告 ======\033[0m\n");
  for (size_t i = 0; i < error_code_list.size(); ++i)
  {
    int32_t error_code = error_code_list[i];
    if (error_code == 0)
      continue;
    printf("\033[1;31m[错误]\033[0m ");
    printf("关节编号: \033[1;33m%-2zu\033[0m | 错误码: \033[1;31m0x%04X\033[0m\n", (i+1), error_code);
    auto it = errorMsgMap.find(error_code);
    if (it != errorMsgMap.end()) {
      printf("  \033[1;36m说明: %s\033[0m\n", it->second.c_str());
    }
    else
    {
      printf("  \033[1;35m未知错误\033[0m\n");
    }
    error_count++;
  }
  printf("\033[1;31m共检测到 %u 个关节存在错误码\033[0m\n", error_count);
  printf("\033[1;34m-----------------------------\033[0m\n");
  return error_count;
}
