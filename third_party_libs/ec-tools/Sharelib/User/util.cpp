#include <fstream>
#include <sstream>
#include <vector>
#include <thread>

#include <iostream>
#include <nlohmann/json.hpp>

#include <pwd.h>
#include <unistd.h>

#include <filesystem>
#include <cstdlib>    // for realpath
#include <limits.h>   // for PATH_MAX
#include <cstring>    // for strerror 


#include "EcMotor.h"




#include <sys/stat.h> 
#include <cstdio>

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

double pos_offset[NUM_SLAVE_MAX] = {0};

uint16_t sw2cw(const uint16_t state_word) // state to control
{
  if (!(state_word & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
  {
    if (!(state_word & (1 << STATUSWORD_SWITCHED_ON_BIT)))
    {
      if (!(state_word & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
      {
        if ((state_word & (1 << STATUSWORD_FAULT_BIT)))
        {
          return 0x80; // fault reset
        }
        else
        {
          return 0x06; // shutdown
        }
      }
      else
      {
        return 0x07; // switch on
      }
    }
    else
    {
      return 0x0F; // switch on
    }
  }
  else
  {
    return 0x0F; // switch on
  }

  return 0;
}
/**
 * @brief double value conver to char* string
 *
 * @param value
 * @return char* snprint char malloc
 * @details EcLogMsg() can't log float, snprint float ot chars
 */
char *floatToChar(double value)
{
  int preLen = snprintf(0, 0, "%.8lf", value);
  char *logTempFloat = (char *)malloc(preLen);
  if (logTempFloat == NULL)
  {
    free(logTempFloat);
    char *strNull = (char *)malloc(5);
    strcpy(strNull, "NULL");
    return strNull;
  }
  snprintf(logTempFloat, preLen, "%.8lf", value);
  return logTempFloat;
}

//将多关节数据转换成多个字节的数据，按小端字节序排列
void toLittleEndian32(int32_t value,uint8_t* buf) {
    buf[0] = static_cast<uint8_t>(value & 0xFF);//获取最低位
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}
//将相对路径转为绝对路径并输出
void printAbsolutePath(const char* relativePath) {
    char resolvedPath[PATH_MAX]; // 存放绝对路径
    if (realpath(relativePath, resolvedPath) != nullptr) {
        std::cout << "Absolute path: " << resolvedPath << std::endl;
    } else {
        std::cerr << "Error resolving path: " << strerror(errno) << std::endl;
    }
}
bool csvRead(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data)
{
  std::ifstream inFile(file_name);
  if (!inFile.is_open())
  {
    return false;
  }

  std::string line;
  if (skip_header)
  {
    std::getline(inFile, line);
  }

  while (std::getline(inFile, line))
  {
    std::vector<double> line_vec;
    std::string number;
    std::istringstream readstr(line);
    while (std::getline(readstr, number, ','))
    {
      line_vec.push_back(atof(number.c_str()));
    }
    data.push_back(line_vec);
  }
  return true;
}
std::string getHomePath()
{
  uid_t uid;
  char *sudoUser = getenv("SUDO_USER");

  if (sudoUser != NULL)
  {
    struct passwd *pw = getpwnam(sudoUser);
    if (pw)
    {
      return std::string(pw->pw_dir);
    }
  }
  else
  {
    uid = getuid();
    struct passwd *pw = getpwuid(uid);
    if (pw)
    {
      return std::string(pw->pw_dir);
    }
  }
  return "";
}
std::string getOffsetFilePath()
{
  std::string homePath = getHomePath();
  if (!homePath.empty())
  {
    return homePath + "/.config/lejuconfig/offset.csv";
  }
  return "";
}
bool loadOffset()
{
  std::vector<std::vector<double_t>> offset;
  std::string offset_file = getOffsetFilePath();
  if (offset_file.empty())
  {
    std::cerr << "[ECmaster]: Failed to get offset file path!\n" << offset_file << std::endl;
    return false;
  }
  if (!csvRead(offset_file.c_str(), false, offset))
  {
    std::cerr << "[ECmaster]: Failed to load offset file!\n" << offset_file << std::endl;
    return false;
  }
  uint32_t num_onelen_item = 1;
  uint32_t i, j;
  for (i = 0; i < offset.size(); i++)
  {
    for (j = 0; j < num_onelen_item; j++)
    {
      if (i * num_onelen_item + j < NUM_SLAVE_MAX)
      {
        // mtx_io.lock(); 
        pos_offset[i * num_onelen_item + j] = offset[i][j];
        // mtx_io.unlock();
      }
    }
  }
  return true;
}


double calcCos(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

double get_sin_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  out = A * sin(2 * M_PI / T * time) + b;
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double get_cos_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  out = A * -cos(2 * M_PI / T * time) + b;
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double get_square_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = A + b;
  }
  else
  {
    out = -A + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}


// using json = nlohmann::json;

// void saveStructToJson(const MyStruct& data, const std::string& filePath) {
//     // 如果路径所在的目录不存在，则创建目录
//     std::filesystem::path pathObj(filePath);
//     if (!std::filesystem::exists(pathObj.parent_path())) {
//         try {
//             std::filesystem::create_directories(pathObj.parent_path());
//         } catch (std::filesystem::filesystem_error &e) {
//             std::cerr << "无法创建目录: " << e.what() << std::endl;
//             return;
//         }
//     }

//     // 如果文件不存在，则创建空文件（下面写入时会自动覆盖）
//     if (!std::filesystem::exists(filePath)) {
//         std::ofstream ofs(filePath);
//         if (!ofs) {
//             std::cerr << "无法创建文件: " << filePath << std::endl;
//             return;
//         }
//         ofs.close();
//     }

//     // 将结构体数据转换为 JSON 对象
//     json j;
//     j["id"]    = data.id;
//     j["name"]  = data.name;
//     j["value"] = data.value;

//     // 打开文件并写入 JSON 数据
//     std::ofstream ofs(filePath);
//     if (!ofs) {
//         std::cerr << "无法打开文件写入: " << filePath << std::endl;
//         return;
//     }
//     // 使用缩进4个空格格式化输出
//     ofs << j.dump(4);
//     ofs.close();
// }