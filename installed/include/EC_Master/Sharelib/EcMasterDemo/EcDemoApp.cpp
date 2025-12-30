/*-----------------------------------------------------------------------------
 * EcDemoApp.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              EC-Master demo application
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <cstdlib>
#include <sys/stat.h> 
#include <cstdio>
#include <cassert>
#include <mutex>
/*-DEFINES-------------------------------------------------------------------*/
#define DCM_ENABLE_LOGFILE

#define ELMO_VENDOR_ID 0x0000009A
#define ELMO_PRODUCT_CODE 0x00030924

#define YD_VENDOR_ID 0x0000005A
#define YD_PRODUCT_CODE 0x00000003

#define PERF_myAppWorkpd 0
#define PERF_DCM_Logfile 1
#define MAX_JOB_NUM 2

#define NUM_SLAVE_MAX 30
#define NUM_SLAVE_ELMO_MAX 14
#define NUM_SLAVE_YD_MAX (NUM_SLAVE_MAX - NUM_SLAVE_ELMO_MAX)

#define BIT_17 (1 << 17)
#define BIT_17_8 (BIT_17 * 8)
#define BIT_17_9 (BIT_17 * 9)
#define BIT_17_10 (BIT_17 * 10)
#define BIT_17_18 (BIT_17 * 18)
#define BIT_17_36 (BIT_17 * 36)
#define MAX_TORQUE (31.2)
#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

#define SENSOR_FEEDBACK_ERROR 0x2311 // 编码器反馈错误
#define FEEDBACK_ERROR 0x7300        // 反馈错误
#define SPEED_TRACKING_ERROR 0x8480  // 速度跟踪超限
#define SPEED_LIMIT_EXCEEDED 0x8481
#define POSITION_TRACKING_ERROR 0x8611 // 位置跟踪超限
#define FAILED_TO_START_MOTOR 0XFF10   // 电机启动错误

#define ANTHROPOMORPHIC_GAIT 1 //是否开启拟人步态
/* 优达对象词典索引*/
#define INDEX_POSITION_ACTUAL_VALUE 0x6064
#define INDEX_MOTOR_RATED_CURRENT 0x6075 //额定电流

#define INDEX_PRODUCT_CODE 0x1026 //厂商识别码，未确定
#define TORQUE_MODE_VELOCITY_LIMIT 0x3401  //力矩模式速度限制
#define DRIVER_RATING_COUNT 0x6075  //关节额定电流值
#define DRIVER_WORK_MODE 0x3507  //驱动器工作模式
#define JOINT_CSP_KP 0X3500   //位置环P增益
#define JOINT_CSV_KP 0x3504   //速度环P增益
#define JOINT_CSV_KI 0x3505   //速度环积分增益
#define JOINT_CSP_OFFSET 0x3502  //位置环前馈
#define JOINT_CSP_COMMAND_FILTER 0x3532  //位置指令滤波
#define ENCODER_FEEDBACK_MODE 0x381C  //编码器工作模式
#define MOTOR_PARAMETER_CODE 0x3E01 //电机参数代码
#define DRIVER_SAVE_SETTING_PARAMETER 0x313D  //驱动器保存参数
/*-LOCAL VARIABLES-----------------------------------------------------------*/
static EC_T_PERF_MEAS_INFO_PARMS S_aPerfMeasInfos[MAX_JOB_NUM] =
    {
        {"myAppWorkPd                    ", 0},
        {"Write DCM logfile              ", 0}};

static double pos_offset[NUM_SLAVE_MAX] = {0}; // 注意这个是保留值，只能被setoffset和csvRead 刷新

static uint32_t rated_current[NUM_SLAVE_MAX] = {0};

/**
 * @brief 编码器系数
 * @details PA100,PA81,PA100,PA100,AK70,AK70
 * PA100,PA81,PA100,PA100,AK70,AK70
 * AK10,AK70,AK70,AK70
 * AK10,AK70,AK70,AK70
 */

logger_callback_stdvector g_ecmaster_vector_logger_=nullptr;
void set_EClogger_callback(logger_callback_stdvector callback){
	g_ecmaster_vector_logger_=callback;
  std::cout << "set_EClogger_callback\n" << "g_ecmaster_vector_logger_ "<< std::to_string( g_ecmaster_vector_logger_==nullptr);
}
void log_vector(const std::string& name, const std::vector<double>& vec)
{
	if (g_ecmaster_vector_logger_!=nullptr)
	{
		g_ecmaster_vector_logger_(name, vec);
	}
}

static uint32_t encoder_range[NUM_SLAVE_MAX] = {
    BIT_17_18, BIT_17_10, BIT_17_10, BIT_17_18, BIT_17_36, BIT_17_36,
    BIT_17_18, BIT_17_10, BIT_17_10, BIT_17_18, BIT_17_36, BIT_17_36,
    BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36,
    BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36, BIT_17_36};

static uint16_t ids[NUM_SLAVE_MAX] = {
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30};

//通过枚举来辨别不同类型驱动器，用数组确定不同位置所用的驱动器    
enum EcMasterType driver_type[NUM_SLAVE_MAX] = {
    YD  , YD  , YD  , YD  , YD  , YD  ,
    YD  , YD  , YD  , YD  , YD  , YD  ,
    YD  , YD  , ELMO, ELMO, ELMO, ELMO,
    ELMO, ELMO, ELMO, ELMO, ELMO, ELMO,
    ELMO, ELMO, ELMO, ELMO, ELMO, ELMO} ;

typedef struct
{
  double target_position;
  double target_velocity;
  double target_torque;
  double kp = 0.0;
  double kd = 0.0;
} TorqueFeedbackTarget_t;

static YD_SlaveRead_t *yd_slave_input[NUM_SLAVE_YD_MAX];
static YD_SlaveWrite_t *yd_slave_output[NUM_SLAVE_YD_MAX];
static uint32_t num_yd_slave = 0;

static ELMO_SlaveRead_t *elmo_slave_input[NUM_SLAVE_ELMO_MAX];
static ELMO_SlaveWrite_t *elmo_slave_output[NUM_SLAVE_ELMO_MAX];
static uint32_t num_elmo_slave = 0;

// 创建两个缓冲区实例
static SlaveBuffersIn buffersAIn;
static SlaveBuffersIn buffersBIn;
static std::atomic<SlaveBuffersIn*> currentBuffersIn;

// 创建两个缓冲区实例
static SlaveBuffersOut buffersAOut;
static SlaveBuffersOut buffersBOut;
static std::atomic<SlaveBuffersOut*> currentBuffersOut;


// Instantiate the new buffers
struct SlaveBuffersOut buffersCOut;
struct SlaveBuffersOut buffersDOut;
static std::atomic<SlaveBuffersOut*> currentPositionBuffersOut;

// Instantiate the new buffers
struct SlaveBuffersOut buffersEOut;
struct SlaveBuffersOut buffersFOut;
static std::atomic<SlaveBuffersOut*> currentTorqueBuffersOut;

//static TorqueFeedbackTarget_t torque_feedback_target[NUM_SLAVE_MAX];

// 双缓冲区定义
static TorqueFeedbackTarget_t torque_feedback_targetA[NUM_SLAVE_MAX];
static TorqueFeedbackTarget_t torque_feedback_targetB[NUM_SLAVE_MAX];

// 原子指针指向当前活跃的缓冲区
static std::atomic<TorqueFeedbackTarget_t*> currentTorqueFeedback;

static uint8_t motorStatusMapA[NUM_SLAVE_MAX] = {0};
static uint8_t motorStatusMapB[NUM_SLAVE_MAX] = {0};
static std::atomic<uint8_t*> currentMotorStatusMap;

static std::mutex mtx_io;
static uint32_t num_slave = 0;
static bool motor_enabled = false;
static bool ecMasterExit = false;

static std::mutex mtx_j;
static uint16_t cm = 2;
static double motorAcceleration[NUM_SLAVE_MAX] = {0};

int restartCounter = 0;
bool restartMotorFlag = false;
//uint8_t motorStatusMap[NUM_SLAVE_MAX] = {0};
int motorErrorCodeMap[NUM_SLAVE_MAX] = {0};
clock_t last_restart_time[NUM_SLAVE_MAX] = {0};

static bool g_motor_disabled[NUM_SLAVE_MAX] = {false};  // 被 _disable 的电机Id
static bool use_anthropomorphic_gait = false;//拟人步态默认关闭
static std::vector<int32_t> joint_kp_write_vec;
static std::vector<int32_t> joint_kd_write_vec;

std::mutex mtx_joint_kp_read_vec;
std::mutex mtx_joint_kd_read_vec;
static std::vector<int32_t> joint_kp_read_vec;
static std::vector<int32_t> joint_kd_read_vec;

#define IS_MOTOR_DISABLED(id) (g_motor_disabled[id])

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
static EC_T_VOID EcMasterJobTask(EC_T_VOID *pvAppContext);
static EC_T_DWORD EcMasterNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS *pParms);
#if (defined INCLUDE_RAS_SERVER)
static EC_T_DWORD RasNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS *pParms);
#endif

/*-MYAPP---------------------------------------------------------------------*/
typedef struct _T_MY_APP_DESC
{
  EC_T_DWORD dwFlashPdBitSize; /* Size of process data memory */
  EC_T_DWORD dwFlashPdBitOffs; /* Process data offset of data */
  EC_T_DWORD dwFlashTimer;
  EC_T_DWORD dwFlashInterval;
  EC_T_BYTE byFlashVal;      /* flash pattern */
  EC_T_BYTE *pbyFlashBuf;    /* flash buffer */
  EC_T_DWORD dwFlashBufSize; /* flash buffer size */
} T_MY_APP_DESC;
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT *pAppContext);
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT *pAppContext);
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT *pAppContext);
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT *pAppContext);
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT *pAppContext);
static EC_T_DWORD myAppNotify(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS *pParms);

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3

#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

static uint16_t sw2cw(const uint16_t state_word) // state to control
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
static char *floatToChar(double value)
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

//////////////////////////////////////////////////////////////////////////////
// 打印 `EcMasterType` 的字符串表示
const char *getEcMasterTypeString(EcMasterType type)
{
  switch (type)
  {
  case ELMO:
    return "ELMO";
  case YD:
    return "YD";
  default:
    return "UNKNOWN";
  }
}

// 将多关节数据转换成多个字节的数据，按小端字节序排列
static void toLittleEndian32(int32_t value, uint8_t *buf)
{
  buf[0] = static_cast<uint8_t>(value & 0xFF); // 获取最低位
  buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buf[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buf[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}
// 将相对路径转为绝对路径并输出
static void printAbsolutePath(const char *relativePath)
{
  char resolvedPath[PATH_MAX]; // 存放绝对路径
  if (realpath(relativePath, resolvedPath) != nullptr)
  {
    std::cout << "Absolute path: " << resolvedPath << std::endl;
  }
  else
  {
    std::cerr << "Error resolving path: " << strerror(errno) << std::endl;
  }
}

/// @brief 单次sdo写入
/// @param SlaveId 驱动器id
/// @param ObIndex 对象索引
/// @param SubIndex 子索引
/// @param in_data 数据指针
/// @return
bool ReadSingleSdo(const uint8_t SlaveId, const uint16_t ObIndex, const uint16_t SubIndex, int32_t &read_data)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  uint8_t buf[4];
  uint32_t outdata_len;
  dwRes = emCoeSdoUpload(0, SlaveId, ObIndex, 0, buf, 4, &outdata_len, 100, 0);
  if (dwRes != 0)
  {
    return false;
  }

  // 将读取到的数据转换为 int32_t 类型
  read_data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
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
  int32_t value;
  toLittleEndian32(*write_data, buf);

  dwRes = emCoeSdoDownload(0, SlaveId, ObIndex, SubIndex, buf, 4, 100, 0);
  if (EC_E_NOERROR != dwRes)
  {
    return false;
  }
  // 需要保存的修改配置进行写入保存,目前只有优达支持
  if (save == true)
  {
    dwRes = emCoeSdoDownload(0, SlaveId, DRIVER_SAVE_SETTING_PARAMETER, 0, buf, 4, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      return false;
    }
  }
  return true;
}

// 记录输入缓冲区的数据
void log_buffers_in_real() {
    // 获取当前生效的输入缓冲区
    SlaveBuffersIn* currentBuffer = currentBuffersIn.load(std::memory_order_acquire);
    //const char* currentBufferName = (currentBuffer == &buffersAIn) ? "buffersAIn" : "buffersBIn";
    const std::string currentBufferName = "buffersC_In";
    // 处理 ELMO_SlaveRead_t 结构体的各字段
    std::vector<double> position_actual_values;
    std::vector<double> torque_actual_values;
    std::vector<double> status_words;
    std::vector<double> mode_of_opration_displays;
    std::vector<double> position_demand_raws;
    std::vector<double> velocity_demand_raws;
    std::vector<double> velocity_actual_values;
    std::vector<double> torque_demand_raws;
    std::vector<double> error_codes;

    // 预分配内存以提高性能
    position_actual_values.reserve(num_elmo_slave);
    torque_actual_values.reserve(num_elmo_slave);
    status_words.reserve(num_elmo_slave);
    mode_of_opration_displays.reserve(num_elmo_slave);
    position_demand_raws.reserve(num_elmo_slave);
    velocity_demand_raws.reserve(num_elmo_slave);
    velocity_actual_values.reserve(num_elmo_slave);
    torque_demand_raws.reserve(num_elmo_slave);
    error_codes.reserve(num_elmo_slave);

    // 遍历当前缓冲区中的所有 ELMO 从设备
    for (uint32_t i = 0; i < num_elmo_slave; ++i) {
        position_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].position_actual_value));
        torque_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].torque_actual_value));
        // status_words.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].status_word));
        // mode_of_opration_displays.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].mode_of_opration_display));
        // position_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].position_demand_raw));
        // velocity_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].velocity_demand_raw));
        // velocity_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].velocity_actual_value));
        // torque_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].torque_demand_raw));
        // error_codes.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].error_code));
    }

    // 记录 ELMO 的各字段数据
    log_vector("/sensor_data/joint_data/" + currentBufferName + "_Elmo_position_actual_value_Real", position_actual_values);
    log_vector("/sensor_data/joint_data/" + currentBufferName + "_Elmo_torque_actual_value_Real", torque_actual_values);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_status_word_Real", status_words);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_mode_of_opration_display_Real", mode_of_opration_displays);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_position_demand_raw_Real", position_demand_raws);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_demand_raw_Real", velocity_demand_raws);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_actual_value_Real", velocity_actual_values);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_torque_demand_raw_Real", torque_demand_raws);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_error_code_Real", error_codes);

    // // 处理 YD_SlaveRead_t 结构体的各字段
    // std::vector<double> yd_status_words;
    // std::vector<double> yd_position_actual_values;
    // std::vector<double> yd_velocity_actual_values;
    // std::vector<double> yd_torque_actual_values;
    // std::vector<double> yd_mode_of_opration_displays;
    // std::vector<double> yd_error_codes;
    // std::vector<double> yd_torque_demand_raws;
    // std::vector<double> yd_velocity_demand_raws;

    // yd_status_words.reserve(num_yd_slave);
    // yd_position_actual_values.reserve(num_yd_slave);
    // yd_velocity_actual_values.reserve(num_yd_slave);
    // yd_torque_actual_values.reserve(num_yd_slave);
    // yd_mode_of_opration_displays.reserve(num_yd_slave);
    // yd_error_codes.reserve(num_yd_slave);
    // yd_torque_demand_raws.reserve(num_yd_slave);
    // yd_velocity_demand_raws.reserve(num_yd_slave);

    // // 遍历当前缓冲区中的所有 YD 从设备
    // for (uint32_t i = 0; i < num_yd_slave; ++i) {
    //     yd_status_words.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].status_word));
    //     yd_position_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].position_actual_value));
    //     yd_velocity_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].velocity_actual_value));
    //     yd_torque_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].torque_actual_value));
    //     yd_mode_of_opration_displays.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].mode_of_opration_display));
    //     yd_error_codes.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].error_code));
    //     yd_torque_demand_raws.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].torque_demand_raw));
    //     yd_velocity_demand_raws.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].velocity_demand_raw));
    // }

    // // 记录 YD 的各字段数据
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_status_word", yd_status_words);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_position_actual_value", yd_position_actual_values);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_actual_value", yd_velocity_actual_values);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_actual_value", yd_torque_actual_values);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_mode_of_opration_display", yd_mode_of_opration_displays);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_error_code", yd_error_codes);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_demand_raw", yd_torque_demand_raws);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_demand_raw", yd_velocity_demand_raws);
}
// 记录输入缓冲区的数据
// 记录输出缓冲区的数据
void log_buffers_out_real() {
    // 获取当前生效的输出缓冲区
    SlaveBuffersOut* currentBuffer = currentBuffersOut.load(std::memory_order_acquire);
    //const char* currentBufferName = (currentBuffer == &buffersAOut) ? "buffersAOut" : "buffersBOut";
    const std::string currentBufferName = "buffersC_Out";
    // 处理 ELMO_SlaveWrite_t 结构体的各字段
    std::vector<double> target_positions;
    std::vector<double> target_velocities;
    std::vector<double> target_torques;
    std::vector<double> max_torques;
    std::vector<double> control_words;
    std::vector<double> mode_of_oprations;
    std::vector<double> position_offsets;
    std::vector<double> velocity_offsets;
    std::vector<double> torque_offsets;

    // 预分配内存以提高性能
    target_positions.reserve(num_elmo_slave);
    target_velocities.reserve(num_elmo_slave);
    target_torques.reserve(num_elmo_slave);
    max_torques.reserve(num_elmo_slave);
    control_words.reserve(num_elmo_slave);
    mode_of_oprations.reserve(num_elmo_slave);
    position_offsets.reserve(num_elmo_slave);
    velocity_offsets.reserve(num_elmo_slave);
    torque_offsets.reserve(num_elmo_slave);

    // 遍历当前缓冲区中的所有 ELMO 从设备
    for (uint32_t i = 0; i < num_elmo_slave; ++i) {
        target_positions.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_position));
        // target_velocities.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_velocity));
        target_torques.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_torque));
        // max_torques.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].max_torque));
        // control_words.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].control_word));
        // mode_of_oprations.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].mode_of_opration));
        //position_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].position_offset));
        // velocity_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].velocit_offset));
        // torque_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].torque_offset));
    }

    // 记录 ELMO 的各字段数据
    log_vector("/sensor_data/joint_data/" + currentBufferName + "_Elmo_target_position_Real", target_positions);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_target_velocity", target_velocities);
    log_vector("/sensor_data/joint_data/" + currentBufferName + "_Elmo_target_torque", target_torques);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_max_torque", max_torques);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_control_word", control_words);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_mode_of_opration", mode_of_oprations);
    //log_vector("/sensor_data/joint_data/" + currentBufferName + "_Elmo_position_offset_Real", position_offsets);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_offset", velocity_offsets);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_torque_offset", torque_offsets);

    // // 处理 YD_SlaveWrite_t 结构体的各字段
    // std::vector<double> yd_control_words;
    // std::vector<double> yd_target_positions;
    // std::vector<double> yd_target_velocities;
    // std::vector<double> yd_target_torques;
    // std::vector<double> yd_velocity_offsets;
    // std::vector<double> yd_torque_offsets;
    // std::vector<double> yd_mode_of_oprations;

    // yd_control_words.reserve(num_yd_slave);
    // yd_target_positions.reserve(num_yd_slave);
    // yd_target_velocities.reserve(num_yd_slave);
    // yd_target_torques.reserve(num_yd_slave);
    // yd_velocity_offsets.reserve(num_yd_slave);
    // yd_torque_offsets.reserve(num_yd_slave);
    // yd_mode_of_oprations.reserve(num_yd_slave);

    // // 遍历当前缓冲区中的所有 YD 从设备
    // for (uint32_t i = 0; i < num_yd_slave; ++i) {
    //     yd_control_words.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].control_word));
    //     yd_target_positions.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_position));
    //     yd_target_velocities.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_velocity));
    //     yd_target_torques.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_torque));
    //     yd_velocity_offsets.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].velocity_offset));
    //     yd_torque_offsets.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].torque_offset));
    //     yd_mode_of_oprations.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].mode_of_opration));
    // }

    // // 记录 YD 的各字段数据
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_control_word", yd_control_words);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_position", yd_target_positions);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_velocity", yd_target_velocities);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_torque", yd_target_torques);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_offset", yd_velocity_offsets);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_offset", yd_torque_offsets);
    // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_mode_of_opration", yd_mode_of_oprations);
}
void log_buffers_in() {
    // 定义缓冲区列表和名称
    SlaveBuffersIn* buffersList[2] = { &buffersAIn, &buffersBIn };
    const char* bufferNames[2] = { "buffersAIn", "buffersBIn" };

    for (int buf = 0; buf < 2; ++buf) {
        SlaveBuffersIn* currentBuffer = buffersList[buf];
        const char* currentBufferName = bufferNames[buf];

        // 处理 ELMO_SlaveRead_t 结构体的各字段
        std::vector<double> position_actual_values;
        std::vector<double> torque_actual_values;
        std::vector<double> status_words;
        std::vector<double> mode_of_opration_displays;
        std::vector<double> position_demand_raws;
        std::vector<double> velocity_demand_raws;
        std::vector<double> velocity_actual_values;
        std::vector<double> torque_demand_raws;
        std::vector<double> error_codes;

        for (uint32_t i = 0; i < num_elmo_slave; ++i) {
            position_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].position_actual_value));
            torque_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].torque_actual_value));
            // status_words.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].status_word));
            // mode_of_opration_displays.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].mode_of_opration_display));
            //position_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].position_demand_raw));
            // velocity_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].velocity_demand_raw));
            // velocity_actual_values.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].velocity_actual_value));
            // torque_demand_raws.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].torque_demand_raw));
            // error_codes.push_back(static_cast<double>(currentBuffer->elmo_slave_input[i].error_code));
        }

        // 记录 ELMO 的各字段数据
        log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_position_actual_value", position_actual_values);
        log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_torque_actual_value", torque_actual_values);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_status_word", status_words);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_mode_of_opration_display", mode_of_opration_displays);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_position_demand_raw", position_demand_raws);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_demand_raw", velocity_demand_raws);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_actual_value", velocity_actual_values);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_torque_demand_raw", torque_demand_raws);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_error_code", error_codes);

        // // 处理 YD_SlaveRead_t 结构体的各字段
        // std::vector<double> yd_status_words;
        // std::vector<double> yd_position_actual_values;
        // std::vector<double> yd_velocity_actual_values;
        // std::vector<double> yd_torque_actual_values;
        // std::vector<double> yd_mode_of_opration_displays;
        // std::vector<double> yd_error_codes;
        // std::vector<double> yd_torque_demand_raws;
        // std::vector<double> yd_velocity_demand_raws;

        // for (uint32_t i = 0; i < num_yd_slave; ++i) {
        //     yd_status_words.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].status_word));
        //     yd_position_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].position_actual_value));
        //     yd_velocity_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].velocity_actual_value));
        //     yd_torque_actual_values.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].torque_actual_value));
        //     yd_mode_of_opration_displays.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].mode_of_opration_display));
        //     yd_error_codes.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].error_code));
        //     yd_torque_demand_raws.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].torque_demand_raw));
        //     yd_velocity_demand_raws.push_back(static_cast<double>(currentBuffer->yd_slave_input[i].velocity_demand_raw));
        // }

        // // 记录 YD 的各字段数据
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_status_word", yd_status_words);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_position_actual_value", yd_position_actual_values);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_actual_value", yd_velocity_actual_values);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_actual_value", yd_torque_actual_values);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_mode_of_opration_display", yd_mode_of_opration_displays);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_error_code", yd_error_codes);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_demand_raw", yd_torque_demand_raws);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_demand_raw", yd_velocity_demand_raws);
    }
 }

// 记录输出缓冲区的数据
void log_buffers_out() {
    // 定义缓冲区列表和名称
    SlaveBuffersOut* buffersList[2] = { &buffersAOut, &buffersBOut };
    const char* bufferNames[2] = { "buffersAOut", "buffersBOut" };

    for (int buf = 0; buf < 2; ++buf) {
        SlaveBuffersOut* currentBuffer = buffersList[buf];
        const char* currentBufferName = bufferNames[buf];

        // 处理 ELMO_SlaveWrite_t 结构体的各字段
        std::vector<double> target_positions;
        std::vector<double> target_velocities;
        std::vector<double> target_torques;
        std::vector<double> max_torques;
        std::vector<double> control_words;
        std::vector<double> mode_of_oprations;
        std::vector<double> position_offsets;
        std::vector<double> velocity_offsets;
        std::vector<double> torque_offsets;

        for (uint32_t i = 0; i < num_elmo_slave; ++i) {
            target_positions.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_position));
            // target_velocities.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_velocity));
            target_torques.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].target_torque));
            // max_torques.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].max_torque));
            // control_words.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].control_word));
            // mode_of_oprations.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].mode_of_opration));
            // position_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].position_offset));
            // velocity_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].velocit_offset));
            // torque_offsets.push_back(static_cast<double>(currentBuffer->elmo_slave_output[i].torque_offset));
        }

        // 记录 ELMO 的各字段数据
        log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_target_position", target_positions);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_target_velocity", target_velocities);
        log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_target_torque", target_torques);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_max_torque", max_torques);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_control_word", control_words);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_mode_of_opration", mode_of_oprations);
       // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_position_offset", position_offsets);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_velocity_offset", velocity_offsets);
        //log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_Elmo_torque_offset", torque_offsets);

        // // 处理 YD_SlaveWrite_t 结构体的各字段
        // std::vector<double> yd_control_words;
        // std::vector<double> yd_target_positions;
        // std::vector<double> yd_target_velocities;
        // std::vector<double> yd_target_torques;
        // std::vector<double> yd_velocity_offsets;
        // std::vector<double> yd_torque_offsets;
        // std::vector<double> yd_mode_of_oprations;

        // for (uint32_t i = 0; i < num_yd_slave; ++i) {
        //     yd_control_words.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].control_word));
        //     yd_target_positions.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_position));
        //     yd_target_velocities.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_velocity));
        //     yd_target_torques.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].target_torque));
        //     yd_velocity_offsets.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].velocity_offset));
        //     yd_torque_offsets.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].torque_offset));
        //     yd_mode_of_oprations.push_back(static_cast<double>(currentBuffer->yd_slave_output[i].mode_of_opration));
        // }

        // // 记录 YD 的各字段数据
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_control_word", yd_control_words);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_position", yd_target_positions);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_velocity", yd_target_velocities);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_target_torque", yd_target_torques);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_velocity_offset", yd_velocity_offsets);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_torque_offset", yd_torque_offsets);
        // log_vector("/sensor_data/joint_data/" + std::string(currentBufferName) + "_YD_mode_of_opration", yd_mode_of_oprations);
    }
}

/**
 * @brief return motor status from status map
 *
 * @param id
 * @return uint8_t
 */
uint8_t motorStatus(const uint16_t id)
{
    // 获取当前 motorStatusMap 缓冲区
    uint8_t* currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  return currentMotorStatus[id - 1];
}

/**
 * @brief 获取驱动某些值配置并打印
 *
 * @param pAppContext ec_master
 * @return true
 * @return false
 * @details 获取地址和格式参照驱动板的xml文件，不需要出现在 PDO map 中
 */
#define INDEX_POSITION_ACTUAL_VALUE 0x6064
#define INDEX_MOTOR_RATED_CURRENT 0x6075

static bool motorGetConfig(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  bool kd_write_status;
  bool kp_write_status;
  int32_t value;
  uint8_t buf[4];
  uint32_t outdata_len;
  
  for (uint32_t i = 0; i < num_slave; i++)
  {
    dwRes = emCoeSdoUpload(0, i, INDEX_POSITION_ACTUAL_VALUE, 0, buf, 4, &outdata_len, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed ot emCoeSdoUpload, Slave %d, Err (0x%lx)\n", i, dwRes));
      return false;
    }
    value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

    char *tempfloat = floatToChar(value * 360.0 / encoder_range[i]);
    char *tempEncoder = floatToChar(value);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave %d actual position %s,Encoder %s\n", i + 1, tempfloat, tempEncoder));
    free(tempfloat);
    free(tempEncoder);

    dwRes = emCoeSdoUpload(0, i, INDEX_MOTOR_RATED_CURRENT, 0, buf, 4, &outdata_len, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed ot emCoeSdoUpload, Slave %d, Err (0x%lx)\n", i, dwRes));
      return false;
    }
    value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    rated_current[i] = value;

    char *tempfloat1 = floatToChar(value / 1000.0);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Rated current %s\n", tempfloat1));
    free(tempfloat1);

    if (driver_type[i] == YD)
    {
      int32_t joint_kd = joint_kd_write_vec[i];
      int32_t joint_kp = joint_kp_write_vec[i];

      int32_t init_kd, init_kp, act_kp, act_kd;
      auto kp_read_status = ReadSingleSdo(i, JOINT_CSP_KP, 0, init_kp);
      if (!kp_read_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to read init kp, Slave %d\n", i));
        return false;
      }

      auto kd_read_status = ReadSingleSdo(i, JOINT_CSV_KP, 0, init_kd);
      if (!kd_read_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to read init kd, Slave %d\n", i + 1));
        return false;
      }

      kd_write_status = writeSingleSdo(i, JOINT_CSV_KP, 0, &joint_kd, false);
      if (!kd_write_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to change joint kd, Slave %d\n", i));
        return false;
      }
      kp_write_status = writeSingleSdo(i, JOINT_CSP_KP, 0, &joint_kp, false);
      if (!kp_write_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to change joint kp, Slave %d\n", i));
        return false;
      }

      kp_read_status = ReadSingleSdo(i, JOINT_CSP_KP, 0, act_kp);
      if (!kp_read_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to read kp after write, Slave %d\n", i));
        return false;
      }

      kd_read_status = ReadSingleSdo(i, JOINT_CSV_KP, 0, act_kd);
      if (!kd_read_status)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to read kd after write, Slave %d\n", i + 1));
        return false;
      }

      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Read Slave %d init kp value %d, and set kp value %d successfully\n", i + 1, init_kp, act_kp));
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Read Slave %d init kd value %d, and set kd value %d successfully\n", i + 1, init_kd, act_kd));

      // //判断是否开启拟人步态，如果是的话在本次运行阶段将膝关节速度P增益调小，否则写为原来的值,这里的写入没有开启断电保存
      // if(use_anthropomorphic_gait == true && (i == 3||i == 9))
      // {
      //   write_status = writeSingleSdo(i,JOINT_CSV_KP,0,&use_pd_gain_value,false);
      //   printf("***use_anthropomorphic_gait == true ***\n");
      //   if ( write_status == false)
      //   {
      //     EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to change knee joint gain, Slave %d\n", i));
      //     return false;
      //   }
      // }
      // else if(use_anthropomorphic_gait == false && (i == 3||i == 9))
      // {
      //   write_status = writeSingleSdo(i,JOINT_CSV_KP,0,&no_pd_gain_value,false);
      //   if ( write_status == false)
      //   {
      //     EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to change knee joint gain, Slave %d\n", i));
      //     return false;
      //   }
      // }
    }
  }
  return true;
}

static int motorReadJointKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t> &joint_kp)
{
  if(driver_type != EcMasterType::YD) {
    std::cout << "[EcMaster] Invalid driver type, expected YD\n";
    return 1; // 1 表示驱动类型不支持该操作
  }

  joint_kp.resize(ids.size(), 0);

  for (uint32_t i = 0; i < ids.size(); i++)
  {
    int32_t kp;
    if (!ReadSingleSdo(ids[i]-1, JOINT_CSP_KP, 0, kp))
    {
      std::cout << "Failed to read joint kp, Slave " << ids[i] << std::endl;
      return 2; // 1 表示读取失败
    }
    joint_kp[i] = kp;
  }

  return 0;
}

static int motorReadJointKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t> &joint_kd)
{
  if(driver_type != EcMasterType::YD) {
    std::cout << "[EcMaster] Invalid driver type, expected YD\n";
    return 1; // 1 表示驱动类型不支持该操作
  }

  joint_kd.resize(ids.size(), 0);
  for (uint32_t i = 0; i < ids.size(); i++)
  {
      int32_t kd;
      if (!ReadSingleSdo(ids[i]-1, JOINT_CSV_KP, 0, kd))
      {
        std::cout << "Failed to read joint kd, Slave " << ids[i] << std::endl;
        return 2; // 2 表示读取失败
      }
      joint_kd[i] = kd;
  }

  return 0;
}

static int motorWriteJointKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t> &joint_kp)
{
  if(driver_type != EcMasterType::YD) {
    std::cout << "[EcMaster] Invalid driver type, expected YD\n";
    return 1; // 1 表示驱动类型不支持该操作
  }

  if(joint_kp.size() != ids.size())
  {
    std::cout << "[EcMaster] Invalid joint kp size, expected " << ids.size() << ", got " << joint_kp.size() << std::endl;
    return 3; // 3 表示joint kp size不匹配
  }
  
  for(uint32_t i = 0; i < ids.size(); i++) 
  {
    int32_t kp = joint_kp[i];
    bool write_status = writeSingleSdo(ids[i]-1, JOINT_CSP_KP, 0, &kp, false);
    if(!write_status)
    {
      std::cout << "[EcMaster] Failed to write joint kp, Slave " << ids[i] << std::endl;
      return 2; // 2 表示写入失败
    }
  }
  
  return 0;
}

static int motorWriteJointKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t> &joint_kd)
{
  if(driver_type != EcMasterType::YD) {
    std::cout << "[EcMaster] Invalid driver type, expected YD\n";
    return 1; // 1 表示驱动类型不支持该操作
  }

  if(joint_kd.size() != ids.size())
  {
    std::cout << "[EcMaster] Invalid joint kd size, expected " << ids.size() << ", got "<< joint_kd.size() << std::endl;
    return 3; // 3 表示joint kd size不匹配
  }

  for(uint32_t i = 0; i < ids.size(); i++)
  {
      int32_t kd = joint_kd[i];
      bool write_status = writeSingleSdo(ids[i] - 1, JOINT_CSV_KP, 0, &kd, false);
      if(!write_status)
      {
        std::cout << "[EcMaster] Failed to write joint kd, Slave" << ids[i] << std::endl;
        return 2; // 2 表示写入失败
      }
  }

  return 0;
}



// void motorGetCurrent(double *current_actual)
// {
//   mtx_io.lock();
//   for (uint32_t i = 0; i < num_slave; i++)
//   {
//     current_actual[i] = elmo_slave_input[i]->current_actual_value / 100.0;
//   }
//   mtx_io.unlock();
// }

bool motorIsEnable(const uint16_t id)
{
  if (id < 1)
  {
    return false;
  }
  uint16_t index = id - 1;
  mtx_io.lock();
  uint16_t sw = elmo_slave_input[index]->status_word & 0x6f;
  mtx_io.unlock();
  if (sw == 0x27)
  {
    return true;
  }
  return false;
}

static bool motorEnable(const uint16_t id, EcMasterType* driver)
{
  if (id < 1)
  {
    return false;
  }
  uint16_t sw;
  uint16_t index = id - 1;
  static uint8_t elmo_Number = 0, yd_Number = 0;
  
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *nextOut = (currentOut == &buffersAOut) ? &buffersBOut : &buffersAOut;

  if(driver[index] == ELMO)
  {
    sw = currentIn->elmo_slave_input[elmo_Number].status_word & 0x6f;
  }
  else if(driver[index] == YD)
  {
    sw = currentIn->yd_slave_input[yd_Number].status_word & 0x6f;
  }
  //mtx_io.unlock();
  if (sw == 0x27)
  {
    switch (driver[index])
    {
      case ELMO: {
        elmo_Number++;
      }
      break;

      case YD  : { 
        yd_Number++;
      }
      break;
      default:
      break;
    }
    return true;
  }

  //mtx_io.lock();
  if(driver[index] == ELMO)
  {
    if(!IS_MOTOR_DISABLED(id)) {
      nextOut->elmo_slave_output[elmo_Number].target_position = currentIn->elmo_slave_input[elmo_Number].position_actual_value;
      nextOut->elmo_slave_output[elmo_Number].position_offset = 0;
      nextOut->elmo_slave_output[elmo_Number].velocit_offset = 0;
      nextOut->elmo_slave_output[elmo_Number].torque_offset  = 0;
      nextOut->elmo_slave_output[elmo_Number].max_torque  = 1000;
      nextOut->elmo_slave_output[elmo_Number].mode_of_opration = MODE_CSP;
      nextOut->elmo_slave_output[elmo_Number].control_word = sw2cw(sw);
      torque_feedback_targetA[index].target_position = currentIn->elmo_slave_input[elmo_Number].position_actual_value * (360.0 / encoder_range[index]);
      torque_feedback_targetB[index].target_position = currentIn->elmo_slave_input[elmo_Number].position_actual_value * (360.0 / encoder_range[index]);
    } else {
      nextOut->elmo_slave_output[elmo_Number].target_torque = 0;
      nextOut->elmo_slave_output[elmo_Number].torque_offset = 0;
      nextOut->elmo_slave_output[elmo_Number].max_torque = 10; // safety
      nextOut->elmo_slave_output[elmo_Number].mode_of_opration = MODE_CST;
      nextOut->elmo_slave_output[elmo_Number].control_word = sw2cw(sw);
    }
  }
  else if(driver[index] == YD)
  { 
    if(!IS_MOTOR_DISABLED(id)) {
      nextOut->yd_slave_output[yd_Number].target_position = currentIn->yd_slave_input[yd_Number].position_actual_value;
      nextOut->yd_slave_output[yd_Number].velocity_offset = 0;
      nextOut->yd_slave_output[yd_Number].torque_offset = 0;
      nextOut->yd_slave_output[yd_Number].mode_of_opration = MODE_CSP;
      nextOut->yd_slave_output[yd_Number].control_word    = sw2cw(sw);
      torque_feedback_targetA[index].target_position = currentIn->yd_slave_input[yd_Number].position_actual_value * (360.0 / encoder_range[index]);
      torque_feedback_targetB[index].target_position = currentIn->yd_slave_input[yd_Number].position_actual_value * (360.0 / encoder_range[index]);
    } else {
      nextOut->yd_slave_output[yd_Number].target_torque = 0;
      nextOut->yd_slave_output[yd_Number].torque_offset = 0;
      nextOut->yd_slave_output[yd_Number].mode_of_opration = MODE_CST;
      nextOut->yd_slave_output[yd_Number].control_word = sw2cw(sw);
    }
  }
  
  torque_feedback_targetA[index].target_velocity = 0;
  torque_feedback_targetA[index].target_torque = 0;
  torque_feedback_targetA[index].kp = 0;
  torque_feedback_targetA[index].kd = 0;

  torque_feedback_targetB[index].target_velocity = 0;
  torque_feedback_targetB[index].target_torque = 0;
  torque_feedback_targetB[index].kp = 0;
  torque_feedback_targetB[index].kd = 0;
  currentBuffersOut.store(nextOut, std::memory_order_release);
  //mtx_io.unlock();
  return false;
}

void motorGetData(const uint16_t *ids, const EcMasterType* driver, uint32_t num,
                  MotorParam_t *data)
{
  uint16_t index = 0;
  uint8_t elmo_Number = 0, yd_Number = 0;

  static uint8_t get_change_mode_old = 0;
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //printf("current ----------InbufferREAD address: %p\n", (void*)currentIn);
  // 获取当前 motorStatusMap 缓冲区
  uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  //mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
   if(driver[i] == ELMO)
    {  
      data[i].position = (currentIn->elmo_slave_input[elmo_Number].position_actual_value * (360.0 / encoder_range[index])  - pos_offset[index]) ;
      data[i].velocity = currentIn->elmo_slave_input[elmo_Number].velocity_actual_value * (360.0 / encoder_range[index]);
      data[i].torque = currentIn->elmo_slave_input[elmo_Number].torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 ;
      data[i].acceleration = motorAcceleration[elmo_Number] * (360.0 / encoder_range[index]);
      data[i].status = currentMotorStatus[i];

      // 驱动器控制环路底层数据
      data[i].error_code = currentIn->elmo_slave_input[elmo_Number].error_code;
      data[i].status_word = currentIn->elmo_slave_input[elmo_Number].status_word;
      data[i].torque_demand_trans = currentIn->elmo_slave_input[elmo_Number].torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0;
      elmo_Number++;
    }
    else if(driver[i] == YD)
    { 
      data[i].position = (currentIn->yd_slave_input[yd_Number].position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
      data[i].velocity = currentIn->yd_slave_input[yd_Number].velocity_actual_value * (360.0 / encoder_range[index]) ;
      data[i].torque = currentIn->yd_slave_input[yd_Number].torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
      data[i].acceleration = motorAcceleration[yd_Number] * (360.0 / encoder_range[index]);
      data[i].status = currentMotorStatus[i];
      
      // 驱动器控制环路底层数据
      data[i].error_code = currentIn->yd_slave_input[yd_Number].error_code;
      data[i].status_word = currentIn->yd_slave_input[yd_Number].status_word;
      data[i].torque_demand_trans = currentIn->yd_slave_input[yd_Number].torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
      yd_Number++;
    }
  }
}

static void motorGetStatus(const uint16_t *ids, EcMasterType* driver, uint32_t num, uint16_t *status_word, uint16_t *error_code)
{
  // 从原子指针获取当前生效的缓冲区
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  uint16_t index = 0;
  uint8_t elmo_Number = 0, yd_Number = 0;
  //mtx_io.lock();
  for (u_int16_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if(driver[index] == ELMO)
    {
      status_word[i] = currentIn->elmo_slave_input[elmo_Number].status_word;
      error_code[i] =  currentIn->elmo_slave_input[elmo_Number].error_code;
      elmo_Number++;
    }
    else if(driver[index] == YD)
    {
      status_word[i] = currentIn->yd_slave_input[yd_Number].status_word;
      error_code[i] = currentIn->yd_slave_input[yd_Number].error_code;
      yd_Number++;
    }
  }
  //mtx_io.unlock();
}

void setMotorPositionOffset(const std::vector<double>& offsets)
{
    std::lock_guard<std::mutex> lock(mtx_io); 
    for (size_t i = 0; i < offsets.size() && i < NUM_SLAVE_MAX; i++)
    {
        pos_offset[i] = offsets[i]; 
    }
}

void getMotorPositionOffset(std::vector<double>& output)
{
    std::lock_guard<std::mutex> lock(mtx_io); 
    output.resize(NUM_SLAVE_MAX); 
    for (size_t i = 0; i < NUM_SLAVE_MAX; i++)
    {
        output[i] = pos_offset[i];
    }
}

void motorSetPosition(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  uint16_t index = 0;
  // mtx_io.lock();
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentPositionBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *nextOut = (currentOut == &buffersCOut) ? &buffersDOut : &buffersCOut;
  // 获取当前 motorStatusMap 缓冲区
  uint8_t* currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);

  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if (currentMotorStatus[index])
      continue;

      
    if(driver[index] == ELMO)
    {
      if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
        nextOut->elmo_slave_output[index].target_position = (params[i].position + pos_offset[index]) * (encoder_range[index] / 360.0) ;
        nextOut->elmo_slave_output[index].position_offset = params[i].positionOffset * (encoder_range[index] / 360.0)  ;
        nextOut->elmo_slave_output[index].velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0)  ;
        nextOut->elmo_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000  ;
        nextOut->elmo_slave_output[index].max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
        nextOut->elmo_slave_output[index].mode_of_opration = MODE_CSP;
        nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
      }
      else {
        // Ignore this Id, skip it.
        nextOut->elmo_slave_output[index].target_torque = 0;
        nextOut->elmo_slave_output[index].torque_offset = 0;
        nextOut->elmo_slave_output[index].max_torque = 10;
        nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
        nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
      }
    } 
    else if(driver[index] == YD)
    {
      if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
        nextOut->yd_slave_output[index].target_position = (params[i].position+ pos_offset[index]) * (encoder_range[index] / 360.0);
        nextOut->yd_slave_output[index].velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0)  ;
        nextOut->yd_slave_output[index].torque_offset = (params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000.0 )  / 1.414;
        nextOut->yd_slave_output[index].mode_of_opration = MODE_CSP;
        nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
      }
      else {
        nextOut->yd_slave_output[index].target_torque = 0;
        nextOut->yd_slave_output[index].torque_offset = 0;
        nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
        nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
      }
        
    }
  }
  currentPositionBuffersOut.store(nextOut, std::memory_order_release);
  //mtx_io.unlock();
}

void motorSetVelocity(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *nextOut = (currentOut == &buffersAOut) ? &buffersBOut : &buffersAOut;
  // 获取当前 motorStatusMap 缓冲区
  uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  uint16_t index = 0;
  //mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    
    index = ids[i] - 1;
    if (currentMotorStatus[index])
      continue;
    if(driver[index] == ELMO)
    {
      if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
        nextOut->elmo_slave_output[index].target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
        nextOut->elmo_slave_output[index].velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
        nextOut->elmo_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
        nextOut->elmo_slave_output[index].max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
        nextOut->elmo_slave_output[index].mode_of_opration = MODE_CSV;
        nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
      }
      else {
        // Ignore this Id, skip it.
        nextOut->elmo_slave_output[index].target_torque = 0;
        nextOut->elmo_slave_output[index].torque_offset = 0;
        nextOut->elmo_slave_output[index].max_torque = 10;
        nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
        nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
      }
    }
    else if(driver[index] == YD)
    {
      if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
        nextOut->yd_slave_output[index].target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
        nextOut->yd_slave_output[index].velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
        nextOut->yd_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
        nextOut->yd_slave_output[index].mode_of_opration = MODE_CSV;
        nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
      }
      else {
        nextOut->yd_slave_output[index].target_torque = 0;
        nextOut->yd_slave_output[index].torque_offset = 0;
        nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
        nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
      }
    }
  }
  currentBuffersOut.store(nextOut, std::memory_order_release);
  //mtx_io.unlock();
}

void motorSetTorqueWithFeedback(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentTorqueBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *nextOut = (currentOut == &buffersEOut) ? &buffersFOut : &buffersEOut;
  
  TorqueFeedbackTarget_t* currentTorqueFeedbackPtr = currentTorqueFeedback.load(std::memory_order_acquire);
  TorqueFeedbackTarget_t* nextTorqueFeedbackPtr = (currentTorqueFeedbackPtr == torque_feedback_targetA) ? torque_feedback_targetB : torque_feedback_targetA;
  // 获取当前 motorStatusMap 缓冲区
  uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  uint16_t index = 0;
  static uint8_t change_mode_old = 0;
  static uint64_t cnt_number;
  std::vector<double> target_tau_vec, target_tau_vecaa, feedback_torque_vec,error_vec;
  target_tau_vec.resize(num_slave);
  target_tau_vecaa.resize(num_slave);
  feedback_torque_vec.resize(num_slave);
  error_vec.resize(num_slave);
  for (auto i = 0; i < num_slave; i++)
  {
    error_vec[i] = currentMotorStatus[i];
  }
  log_vector("/sensor_data/joint_data/error_vec", error_vec);

  //mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    
    index = ids[i] - 1;
    if (currentMotorStatus[index])
      continue;
    // for feedback
    auto &target = nextTorqueFeedbackPtr[index];
    if(!IS_MOTOR_DISABLED(ids[i])) {
      target.target_position = (params[i].position + pos_offset[index]);
      target.target_velocity = params[i].velocityOffset;
      target.target_torque = params[i].torque;
      target.kp = params[i].kp;
      target.kd = params[i].kd;
      feedback_torque_vec[index] = params[i].torque;
    }
    else { // Disabled
      target.target_position = 0;
      target.target_velocity = 0;
      target.target_torque = 0;
      target.kp = 0;
      target.kd = 0;
    }


    double current_position,current_velocity;
    if(driver[index] == ELMO)
    {
      current_position = currentIn->elmo_slave_input[index].position_actual_value * (360.0 / encoder_range[index]);
      current_velocity = currentIn->elmo_slave_input[index].velocity_actual_value * (360.0 / encoder_range[index]);
    }
    else if(driver[index] == YD)
    {
      current_position = currentIn->yd_slave_input[index].position_actual_value * (360.0 / encoder_range[index]);
      current_velocity = currentIn->yd_slave_input[index].velocity_actual_value* (360.0 / encoder_range[index]);
    }
    //
    uint16_t max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
    int16_t target_tau = (target.target_torque + (target.target_position - current_position)* TO_RADIAN * params[i].kp + (target.target_velocity - current_velocity) * TO_RADIAN * params[i].kd) * (1000.0 / rated_current[index]) * 1000;
    int32_t max_torque_32 = static_cast<int32_t>(max_torque);
    int32_t target_tau_32 = static_cast<int32_t>(target_tau);
    if (target_tau_32 > max_torque_32)
    {
      target_tau = static_cast<int16_t>(max_torque);
    }else if (target_tau_32 < -max_torque_32)
    {
      target_tau = static_cast<int16_t>(-max_torque);
    }
    target_tau_vecaa[index] = target_tau;
    if(driver[index] == ELMO)
    {
      // FIX-QA: https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/825
        if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
          nextOut->elmo_slave_output[index].target_torque = target_tau;
          nextOut->elmo_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
          nextOut->elmo_slave_output[index].max_torque = max_torque;
          nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
        }
        else { // Motor Disabled.
          nextOut->elmo_slave_output[index].target_torque = 0;
          nextOut->elmo_slave_output[index].torque_offset = 0;
          nextOut->elmo_slave_output[index].max_torque = 10;
          nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
        }
    }
    else if(driver[index] == YD)
    {
        if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
          nextOut->yd_slave_output[index].target_torque = target_tau / 1.414;
          nextOut->yd_slave_output[index].torque_offset = (params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000) / 1.414;
          nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
          target_tau_vec[index] = currentOut->yd_slave_output[index].target_torque;
        }
        else {
          nextOut->yd_slave_output[index].target_torque = 0;
          nextOut->yd_slave_output[index].torque_offset = 0;
          nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
        }
    }
  }
  currentTorqueBuffersOut.store(nextOut, std::memory_order_release);
  currentTorqueFeedback.store(nextTorqueFeedbackPtr, std::memory_order_release);
  //mtx_io.unlock();
  // log_vector("/sensor_data/joint_data/set_feedback_torqueint16", target_tau_vecaa);
  // log_vector("/sensor_data/joint_data/feedback_torque_vec", feedback_torque_vec);
  // log_vector("/sensor_data/joint_data/set_feedback_torque", target_tau_vec);
}
void motorSetTorque(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *nextOut = (currentOut == &buffersAOut) ? &buffersBOut : &buffersAOut;
  // 获取当前 motorStatusMap 缓冲区
  uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  // 得到另一块缓冲区
  //SlaveBuffers *next = (current == &buffersA) ? &buffersB : &buffersA;
  uint16_t index = 0;
  //mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    
    index = ids[i] - 1;
    if (currentMotorStatus[index])
      continue;
    if (driver[index] == ELMO)
    {
        if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
          nextOut->elmo_slave_output[index].target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000;
          nextOut->elmo_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
          nextOut->elmo_slave_output[index].max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
          nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
        }
        else { // Motor Disabled.
          nextOut->elmo_slave_output[index].target_torque = 0;
          nextOut->elmo_slave_output[index].torque_offset = 0;
          nextOut->elmo_slave_output[index].max_torque = 10;
          nextOut->elmo_slave_output[index].mode_of_opration = MODE_CST;
          nextOut->elmo_slave_output[index].control_word = sw2cw(currentIn->elmo_slave_input[index].status_word & 0x6f);
        }
    }
    else if(driver[index] == YD)
    {
        if(!IS_MOTOR_DISABLED(ids[i])) { // NOTES: This motor is not disabled, configure from kuavo.json.
          nextOut->yd_slave_output[index].target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000 / 1.414;
          nextOut->yd_slave_output[index].torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
          nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
          // nextOut->yd_slave_output[index].control_word = sw2cw(currentIn->yd_slave_input[index].status_word & 0x6f);
        }
        else {
          nextOut->yd_slave_output[index].target_torque = 0;
          nextOut->yd_slave_output[index].torque_offset = 0;
          nextOut->yd_slave_output[index].mode_of_opration = MODE_CST;
        }
    }
  }
  currentBuffersOut.store(nextOut, std::memory_order_release);
  // printf("elmo %d \n",elmo_Number);
    // printf("command motor mode5 = %d,mode6 = %d \n", yd_slave_output[4]->mode_of_opration, yd_slave_output[5]->mode_of_opration);
  //mtx_io.unlock();

}

void motorSetkp(const std::vector<int32_t> &joint_kp)
{
  for (int i = 0; i < joint_kp.size(); i++)
  {
    joint_kp_write_vec.push_back(joint_kp[i]);
  }
}

void motorSetkd(const std::vector<int32_t> &joint_kd)
{
  for (int i = 0; i < joint_kd.size(); i++)
  {
    joint_kd_write_vec.push_back(joint_kd[i]);
  }
}

int motorReadKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t> &joint_kp)
{
  return motorReadJointKp(ids, driver_type, joint_kp);
}

int motorReadKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t> &joint_kd)
{
  return motorReadJointKd(ids, driver_type, joint_kd);
}

int motorWriteKp(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t> &joint_kp)
{
  return motorWriteJointKp(ids, driver_type, joint_kp);
}

int motorWriteKd(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t> &joint_kd)
{
  return motorWriteJointKd(ids, driver_type, joint_kd);
}
bool isMotorEnable(void)
{
  return motor_enabled;
}

uint32_t getNumSlave(void)
{
  return num_slave;
}

void setEcEncoderRange(uint32_t *encoder_range_set, uint16_t num)
{
  mtx_io.lock();
  num > NUM_SLAVE_MAX ? num = NUM_SLAVE_MAX : num = num;
  for (uint32_t i = 0; i < num; i++)
  {
    encoder_range[i] = encoder_range_set[i];
  }
  mtx_io.unlock();
}

/**
 * @brief coe emergency request 错误处理
 *
 * @param slaveAddr 从站地址
 * @param errorCode 错误码
 * @param errorReg 错误寄存器
 * @return true
 * @return false
 */
void fixEmergencyRequest(const int motorId, const int errorCode)
{
  // 获取当前 motorStatusMap 缓冲区
  uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
  if (errorCode == SENSOR_FEEDBACK_ERROR)
  {
    setEcMasterExit();
    ecatDeinitMaster();
    return;
  }
  else if (errorCode == FAILED_TO_START_MOTOR)
  {
    setEcMasterExit();
    ecatDeinitMaster();
    return;
  }
  else if (errorCode == FEEDBACK_ERROR || errorCode == POSITION_TRACKING_ERROR || errorCode == SPEED_TRACKING_ERROR)
  {
    /* 重启性能测试计算 */

    // std::clock_t start;
    // double duration;
    // start = std::clock();
    while (true)
    {
      if (motorEnable(motorId, driver_type))
      {
        break;
      }
      OsSleep(1);
    }
    // duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    currentMotorStatus[motorId - 1] = MOTOR_STATUS_NO_ERROR;
    restartMotorFlag = false;
    // OsPrintf("restart %d motor use time %lf\n", motorId, duration);
  }
}

static double calcCos(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void motorToPosition(T_EC_DEMO_APP_CONTEXT *pAppContext, const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt)
{
  double q0[num_id];
  double T[num_id];

  char *q0str;
  char *qdstr;

  MotorParam_t initMotorParam[NUM_SLAVE_MAX];

  motorGetData(ids, driver_type, num_id, initMotorParam);

  for (uint32_t i = 0; i < num_id; i++)
  {
    q0[i] = initMotorParam[i].position;
    T[i] = fabs(q_d[i] - q0[i]) / speed;
    q0str = floatToChar(q0[i]);
    qdstr = floatToChar(q_d[i]);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                                 "Motor %d from %s to %s\n", i + 1, q0str, qdstr));
  }
  free(q0str);
  free(qdstr);

  double max_T = T[0];
  for (uint32_t i = 1; i < num_id; i++)
  {
    if (max_T < T[i])
    {
      max_T = T[i];
    }
  }
  if (max_T < 0.5)
  {
    max_T = 0.5;
  }
  char *tempfloat = floatToChar(max_T);
  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Duration %s\n", tempfloat));
  free(tempfloat);
  double t = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    for (uint32_t i = 0; i < num_id; i++)
    {
      initMotorParam[i].position = calcCos(q0[i], q_d[i], max_T, t);
      initMotorParam[i].velocity = 0;
      initMotorParam[i].torque = 0;
      initMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, num_id, initMotorParam);

    t += dt;
    if (t > max_T)
    {
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

void motorToPosition(const uint16_t *ids, uint32_t num_id, const double *q_d, double speed, double dt)
{
  double q0[num_id];
  double T[num_id];

  MotorParam_t initMotorParam[NUM_SLAVE_MAX];

  motorGetData(ids, driver_type, num_id, initMotorParam);

  for (uint32_t i = 0; i < num_id; i++)
  {
    q0[i] = initMotorParam[i].position;
    T[i] = fabs(q_d[i] - q0[i]) / speed;
  }

  double max_T = T[0];
  for (uint32_t i = 1; i < num_id; i++)
  {
    if (max_T < T[i])
    {
      max_T = T[i];
    }
  }
  if (max_T < 0.5)
  {
    max_T = 0.5;
  }
  char *tempfloat = floatToChar(max_T);
  double t = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    for (uint32_t i = 0; i < num_id; i++)
    {
      initMotorParam[i].position = calcCos(q0[i], q_d[i], max_T, t);
      initMotorParam[i].velocity = 0;
      initMotorParam[i].torque = 0;
      initMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, num_id, initMotorParam);

    t += dt;
    if (t > max_T)
    {
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
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

/********************************************************************************/
/** \brief EC-Master demo application.
 *
 * This is an EC-Master demo application.
 *
 * \return  Status value.
 */
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT *pAppContext)
{

  std::vector<double> real_tau= {0,0,0,0,0,0};
  log_vector("/sensor_data/joint_data/real_tau",real_tau);
  log_vector("/sensor_data/joint_data/set_feedback_torque", real_tau);
  log_vector("/sensor_data/joint_data/set_feedback_torqueint16", real_tau);
  log_vector("/sensor_data/joint_data/feedback_torque_vec", real_tau);
  log_vector("/sensor_data/joint_data/error_vec", real_tau);
  log_vector("/sensor_data/joint_data/error_code",real_tau);

  // log_vector("/sensor_data/joint_data/real_mode",real_tau);



  EC_T_DWORD dwRetVal = EC_E_NOERROR;
  EC_T_DWORD dwRes = EC_E_NOERROR;

  T_EC_DEMO_APP_PARMS *pAppParms = &pAppContext->AppParms;

  EC_T_VOID *pvJobTaskHandle = EC_NULL;

  EC_T_REGISTERRESULTS RegisterClientResults;
  OsMemset(&RegisterClientResults, 0, sizeof(EC_T_REGISTERRESULTS));

  CEcTimer oAppDuration;
  EC_T_BOOL bFirstDcmStatus = EC_TRUE;
  CEcTimer oDcmStatusTimer;

#if (defined INCLUDE_RAS_SERVER)
  EC_T_VOID *pvRasServerHandle = EC_NULL;
#endif

#if (defined INCLUDE_PCAP_RECORDER)
  CPcapRecorder *pPcapRecorder = EC_NULL;
#endif

  /* check link layer parameter */
  if (EC_NULL == pAppParms->apLinkParms[0])
  {
    dwRetVal = EC_E_INVALIDPARM;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Missing link layer parameter\n"));
    goto Exit;
  }

  /* check if polling mode is selected */
  if (pAppParms->apLinkParms[0]->eLinkMode != EcLinkMode_POLLING)
  {
    dwRetVal = EC_E_INVALIDPARM;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Link layer in 'interrupt' mode is not supported by %s. Please select 'polling' mode.\n", EC_DEMO_APP_NAME));
    goto Exit;
  }

  pAppContext->pNotificationHandler = EC_NEW(CEmNotification(pAppContext));
  if (EC_NULL == pAppContext->pNotificationHandler)
  {
    dwRetVal = EC_E_NOMEMORY;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create notification handler\n"));
    goto Exit;
  }

  pAppContext->pMyAppDesc = (T_MY_APP_DESC *)OsMalloc(sizeof(T_MY_APP_DESC));
  if (EC_NULL == pAppContext->pMyAppDesc)
  {
    dwRetVal = EC_E_NOMEMORY;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create myApp descriptor\n"));
    goto Exit;
  }
  OsMemset(pAppContext->pMyAppDesc, 0, sizeof(T_MY_APP_DESC));
  
  dwRes = myAppInit(pAppContext);
  if (EC_E_NOERROR != dwRes)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppInit failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }

#ifdef INCLUDE_RAS_SERVER
  /* start RAS server if enabled */
  if (pAppParms->bStartRasServer)
  {
    ATEMRAS_T_SRVPARMS oRemoteApiConfig;

    OsMemset(&oRemoteApiConfig, 0, sizeof(ATEMRAS_T_SRVPARMS));
    oRemoteApiConfig.dwSignature = ATEMRASSRV_SIGNATURE;
    oRemoteApiConfig.dwSize = sizeof(ATEMRAS_T_SRVPARMS);
    oRemoteApiConfig.oAddr.dwAddr = 0; /* INADDR_ANY */
    oRemoteApiConfig.wPort = pAppParms->wRasServerPort;
    oRemoteApiConfig.dwCycleTime = ATEMRAS_CYCLE_TIME;
    oRemoteApiConfig.dwCommunicationTimeout = ATEMRAS_MAX_WATCHDOG_TIMEOUT;
    oRemoteApiConfig.oAcceptorThreadCpuAffinityMask = pAppParms->CpuSet;
    oRemoteApiConfig.dwAcceptorThreadPrio = MAIN_THREAD_PRIO;
    oRemoteApiConfig.dwAcceptorThreadStackSize = JOBS_THREAD_STACKSIZE;
    oRemoteApiConfig.oClientWorkerThreadCpuAffinityMask = pAppParms->CpuSet;
    oRemoteApiConfig.dwClientWorkerThreadPrio = MAIN_THREAD_PRIO;
    oRemoteApiConfig.dwClientWorkerThreadStackSize = JOBS_THREAD_STACKSIZE;
    oRemoteApiConfig.pfnRasNotify = RasNotifyCallback;                    /* RAS notification callback function */
    oRemoteApiConfig.pvRasNotifyCtxt = pAppContext->pNotificationHandler; /* RAS notification callback function context */
    oRemoteApiConfig.dwMaxQueuedNotificationCnt = 100;                    /* pre-allocation */
    oRemoteApiConfig.dwMaxParallelMbxTferCnt = 50;                        /* pre-allocation */
    oRemoteApiConfig.dwCycErrInterval = 500;                              /* span between to consecutive cyclic notifications of same type */

    if (1 <= pAppParms->nVerbose)
    {
      OsMemcpy(&oRemoteApiConfig.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
      oRemoteApiConfig.LogParms.dwLogLevel = EC_LOG_LEVEL_ERROR;
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Start Remote API Server now\n"));
    dwRes = emRasSrvStart(&oRemoteApiConfig, &pvRasServerHandle);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot spawn Remote API Server\n"));
    }
  }
#endif

  /* initialize EtherCAT master */
  {
    EC_T_INIT_MASTER_PARMS oInitParms;

    OsMemset(&oInitParms, 0, sizeof(EC_T_INIT_MASTER_PARMS));
    oInitParms.dwSignature = ATECAT_SIGNATURE;
    oInitParms.dwSize = sizeof(EC_T_INIT_MASTER_PARMS);
    oInitParms.pOsParms = &pAppParms->Os;
    oInitParms.pLinkParms = pAppParms->apLinkParms[0];
    oInitParms.pLinkParmsRed = pAppParms->apLinkParms[1];
    oInitParms.dwBusCycleTimeUsec = pAppParms->dwBusCycleTimeUsec;
    oInitParms.dwMaxBusSlaves = MASTER_CFG_ECAT_MAX_BUS_SLAVES;
    oInitParms.dwMaxAcycFramesQueued = MASTER_CFG_MAX_ACYC_FRAMES_QUEUED;
    if (oInitParms.dwBusCycleTimeUsec >= 1000)
    {
      oInitParms.dwMaxAcycBytesPerCycle = MASTER_CFG_MAX_ACYC_BYTES_PER_CYC;
    }
    else
    {
      oInitParms.dwMaxAcycBytesPerCycle = 1500;
      oInitParms.dwMaxAcycFramesPerCycle = 1;
      oInitParms.dwMaxAcycCmdsPerCycle = 20;
    }
    oInitParms.dwEcatCmdMaxRetries = MASTER_CFG_MAX_ACYC_CMD_RETRIES;

    OsMemcpy(&oInitParms.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
    oInitParms.LogParms.dwLogLevel = pAppParms->dwMasterLogLevel;

    if (pAppParms->dwPerfMeasLevel > 0)
    {
      oInitParms.PerfMeasInternalParms.bEnabled = EC_TRUE;

      if (pAppParms->dwPerfMeasLevel > 1)
      {
        oInitParms.PerfMeasInternalParms.HistogramParms.dwBinCount = 200;
      }
    }
    else
    {
      oInitParms.PerfMeasInternalParms.bEnabled = EC_FALSE;
    }

    dwRes = ecatInitMaster(&oInitParms);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot initialize EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    if (0 != OsStrlen(pAppParms->szLicenseKey))
    {
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Try to verify EtherCAT license!\n"));
      dwRes = ecatSetLicenseKey(pAppParms->szLicenseKey);
      if (dwRes != EC_E_NOERROR)
      {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot set license key: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
      }
      else
      {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "EtherCAT license is verified!\n"));
      }
    }
  }

  /* initalize performance measurement */
  if (pAppParms->dwPerfMeasLevel > 0)
  {
    EC_T_PERF_MEAS_APP_PARMS oPerfMeasAppParms;
    OsMemset(&oPerfMeasAppParms, 0, sizeof(EC_T_PERF_MEAS_APP_PARMS));
    oPerfMeasAppParms.dwNumMeas = MAX_JOB_NUM;
    oPerfMeasAppParms.aPerfMeasInfos = S_aPerfMeasInfos;

    dwRes = ecatPerfMeasAppCreate(&oPerfMeasAppParms, &pAppContext->pvPerfMeas);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot initialize app performance measurement: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    pAppContext->dwPerfMeasLevel = pAppParms->dwPerfMeasLevel;
  }

  /* print MAC address */
  {
    ETHERNET_ADDRESS oSrcMacAddress;
    OsMemset(&oSrcMacAddress, 0, sizeof(ETHERNET_ADDRESS));

    dwRes = ecatGetSrcMacAddress(&oSrcMacAddress);
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get MAC address: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "EtherCAT network adapter MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
                                 oSrcMacAddress.b[0], oSrcMacAddress.b[1], oSrcMacAddress.b[2], oSrcMacAddress.b[3], oSrcMacAddress.b[4], oSrcMacAddress.b[5]));
  }

  /* EtherCAT traffic logging */
#if (defined INCLUDE_PCAP_RECORDER)
  if (pAppParms->bPcapRecorder)
  {
    pPcapRecorder = EC_NEW(CPcapRecorder());
    if (EC_NULL == pPcapRecorder)
    {
      dwRetVal = EC_E_NOMEMORY;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Creating PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRetVal), dwRetVal));
      goto Exit;
    }
    dwRes = pPcapRecorder->InitInstance(pAppContext->dwInstanceId, pAppParms->dwPcapRecorderBufferFrameCnt, pAppParms->szPcapRecorderFileprefix);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Initialize PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRes), dwRes));
      goto Exit;
    }
  }
#endif /* INCLUDE_PCAP_RECORDER */

#if (defined INCLUDE_SLAVE_STATISTICS)
  /* Slave statistics polling for error diagnostic */
  {
    EC_T_DWORD dwPeriodMs = 1000;

    dwRes = ecatIoCtl(EC_IOCTL_SET_SLVSTAT_PERIOD, (EC_T_BYTE *)&dwPeriodMs, sizeof(EC_T_DWORD), EC_NULL, 0, EC_NULL);
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatIoControl(EC_IOCTL_SET_SLVSTAT_PERIOD) returns with error=0x%x\n", dwRes));
      goto Exit;
    }
  }
#endif /* INCLUDE_SLAVE_STATISTICS */

  /* create cyclic task to trigger jobs */
  {
    CEcTimer oTimeout(2000);

    pAppContext->bJobTaskRunning = EC_FALSE;
    pAppContext->bJobTaskShutdown = EC_FALSE;
    pvJobTaskHandle = OsCreateThread((EC_T_CHAR *)"EcMasterJobTask", EcMasterJobTask, pAppParms->CpuSet,
                                     JOBS_THREAD_PRIO, JOBS_THREAD_STACKSIZE, (EC_T_VOID *)pAppContext);

    /* wait until thread is running */
    while (!oTimeout.IsElapsed() && !pAppContext->bJobTaskRunning)
    {
      OsSleep(10);
    }
    if (!pAppContext->bJobTaskRunning)
    {
      dwRetVal = EC_E_TIMEOUT;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Timeout starting JobTask\n"));
      goto Exit;
    }
  }

  /* set OEM key if available */
  if (0 != pAppParms->qwOemKey)
  {
    dwRes = ecatSetOemKey(pAppParms->qwOemKey);
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot set OEM key at master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
  }

  /* configure master */
  dwRes = ecatConfigureMaster(pAppParms->eCnfType, pAppParms->pbyCnfData, pAppParms->dwCnfDataLen);
  if (dwRes != EC_E_NOERROR)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }

  /* register client */
  dwRes = ecatRegisterClient(EcMasterNotifyCallback, pAppContext, &RegisterClientResults);
  if (dwRes != EC_E_NOERROR)
  {
    dwRetVal = dwRes;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot register client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    goto Exit;
  }
  pAppContext->pNotificationHandler->SetClientID(RegisterClientResults.dwClntId);

  /* configure DC/DCM master is started with ENI */
  if (EC_NULL != pAppParms->pbyCnfData)
  {
    /* configure DC */
    {
      EC_T_DC_CONFIGURE oDcConfigure;

      OsMemset(&oDcConfigure, 0, sizeof(EC_T_DC_CONFIGURE));
      oDcConfigure.dwTimeout = ETHERCAT_DC_TIMEOUT;
      oDcConfigure.dwDevLimit = ETHERCAT_DC_DEV_LIMIT;
      oDcConfigure.dwSettleTime = ETHERCAT_DC_SETTLE_TIME;
      if (eDcmMode_MasterRefClock == pAppParms->eDcmMode)
      {
        oDcConfigure.dwTotalBurstLength = 10000;
        oDcConfigure.dwBurstBulk = 1;
      }
      else
      {
        oDcConfigure.dwTotalBurstLength = ETHERCAT_DC_ARMW_BURSTCYCLES;
        if (pAppParms->dwBusCycleTimeUsec < 1000)
        {
          /* if the cycle time is below 1000 usec, we have to reduce the number of frames sent within one cycle */
          oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP / 2;
        }
        else
        {
          oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP;
        }
      }
#if (defined INCLUDE_DCX)
      if (eDcmMode_Dcx == pAppParms->eDcmMode)
      {
        oDcConfigure.bAcycDistributionDisabled = EC_FALSE; /* Enable acyclic distribution if cycle time is above 1000 usec to get DCX in sync */
      }
      else
      {
        oDcConfigure.bAcycDistributionDisabled = EC_TRUE;
      }
#else
      oDcConfigure.bAcycDistributionDisabled = EC_TRUE;
#endif

      dwRes = ecatDcConfigure(&oDcConfigure);
      if (dwRes != EC_E_NOERROR)
      {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DC! (Result = 0x%x)\n", dwRes));
        goto Exit;
      }
    }
    /* configure DCM */
    if (pAppParms->bDcmLogEnabled && !pAppParms->bDcmConfigure)
    {
      EC_T_BOOL bBusShiftConfiguredByEni = EC_FALSE;
      dwRes = ecatDcmGetBusShiftConfigured(&bBusShiftConfiguredByEni);
      if (dwRes != EC_E_NOERROR)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot check if BusShift is configured  (Result = 0x%x)\n", dwRes));
      }
      if (bBusShiftConfiguredByEni)
      {
        pAppParms->bDcmConfigure = EC_TRUE;
        pAppParms->eDcmMode = eDcmMode_BusShift;
      }
    }
    if (pAppParms->bDcmConfigure)
    {
      EC_T_DWORD dwCycleTimeNsec = pAppParms->dwBusCycleTimeUsec * 1000; /* cycle time in nsec */
      EC_T_INT nCtlSetValNsec = dwCycleTimeNsec * 2 / 3 /* 66% */;       /* distance between cyclic frame send time and DC base on bus */
      EC_T_DWORD dwInSyncLimitNsec = dwCycleTimeNsec / 5 /* 20% */;      /* limit for DCM InSync monitoring */

      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "cycle time: %d nsec  \n", dwCycleTimeNsec));
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "distance between cyclic frame: %d nsec  \n", nCtlSetValNsec));
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "sync limit time: %d nsec  \n", dwInSyncLimitNsec));

      EC_T_DCM_CONFIG oDcmConfig;
      OsMemset(&oDcmConfig, 0, sizeof(EC_T_DCM_CONFIG));

      switch (pAppParms->eDcmMode)
      {
      case eDcmMode_Off:
        oDcmConfig.eMode = eDcmMode_Off;
        break;
      case eDcmMode_BusShift:
        oDcmConfig.eMode = eDcmMode_BusShift;
        oDcmConfig.u.BusShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.BusShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.BusShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));
          oDcmConfig.u.BusShift.bCtlOff = EC_TRUE;
        }
        break;
      case eDcmMode_MasterShift:
        oDcmConfig.eMode = eDcmMode_MasterShift;
        oDcmConfig.u.MasterShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));
          oDcmConfig.u.MasterShift.bCtlOff = EC_TRUE;
        }
        break;
      case eDcmMode_MasterRefClock:
        oDcmConfig.eMode = eDcmMode_MasterRefClock;
        oDcmConfig.u.MasterRefClock.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.MasterRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
        break;
      case eDcmMode_LinkLayerRefClock:
        oDcmConfig.eMode = eDcmMode_LinkLayerRefClock;
        oDcmConfig.u.LinkLayerRefClock.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.LinkLayerRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
        break;
#if (defined INCLUDE_DCX)
      case eDcmMode_Dcx:
        oDcmConfig.eMode = eDcmMode_Dcx;
        /* Mastershift */
        oDcmConfig.u.Dcx.MasterShift.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.Dcx.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.Dcx.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
        /* Dcx Busshift */
        oDcmConfig.u.Dcx.nCtlSetVal = nCtlSetValNsec;
        oDcmConfig.u.Dcx.dwInSyncLimit = dwInSyncLimitNsec;
        oDcmConfig.u.Dcx.bLogEnabled = pAppParms->bDcmLogEnabled;
        oDcmConfig.u.Dcx.dwExtClockTimeout = 1000;
        oDcmConfig.u.Dcx.wExtClockFixedAddr = 0; /* 0 only when clock adjustment in external mode configured by EcEngineer */
        if (pAppParms->bDcmControlLoopDisabled)
        {
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled for diagnosis!\n"));

          oDcmConfig.u.Dcx.MasterShift.bCtlOff = EC_TRUE;
          oDcmConfig.u.Dcx.bCtlOff = EC_TRUE;
        }
        break;
#endif
      default:
        dwRetVal = EC_E_NOTSUPPORTED;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM mode is not supported!\n"));
        goto Exit;
      }
      dwRes = ecatDcmConfigure(&oDcmConfig, 0);
      switch (dwRes)
      {
      case EC_E_NOERROR:
        break;
      case EC_E_FEATURE_DISABLED:
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode!\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Start with -dcmmode off to run the DC demo without DCM, or prepare the ENI file to support the requested DCM mode\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "In ET9000 for example, select under "
                                                                         "Advanced settings\\Distributed clocks"
                                                                         " "
                                                                         "DC in use"
                                                                         " and "
                                                                         "Slave Mode"
                                                                         "\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "to support BusShift and MasterRefClock modes.\n"));
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Please refer to the class A manual for further information\n"));
        dwRetVal = dwRes;
        goto Exit;
      default:
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode! %s (Result = 0x%x)\n", ecatGetText(dwRes), dwRes));
        dwRetVal = dwRes;
        goto Exit;
      }
    }
  }

  /* print found slaves */
  if (pAppParms->dwAppLogLevel >= EC_LOG_LEVEL_VERBOSE)
  {
    dwRes = ecatScanBus(ETHERCAT_SCANBUS_TIMEOUT);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    switch (dwRes)
    {
    case EC_E_NOERROR:
    case EC_E_BUSCONFIG_MISMATCH:
    case EC_E_LINE_CROSSED:
      PrintSlaveInfos(pAppContext);
      break;
    default:
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot scan bus: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
      break;
    }
    if (dwRes != EC_E_NOERROR)
    {
      dwRetVal = dwRes;
      goto Exit;
    }
  }

  /* set master to INIT */
  dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
  pAppContext->pNotificationHandler->ProcessNotificationJobs();
  if (dwRes != EC_E_NOERROR)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to INIT: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  dwRes = myAppPrepare(pAppContext);
  if (EC_E_NOERROR != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  /* set master to PREOP */
  dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_PREOP);
  pAppContext->pNotificationHandler->ProcessNotificationJobs();
  if (dwRes != EC_E_NOERROR)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to PREOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    dwRetVal = dwRes;
    goto Exit;
  }

  /* skip this step if demo started without ENI */
  if (EC_NULL != pAppParms->pbyCnfData)
  {
    dwRes = myAppSetup(pAppContext);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "myAppSetup failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      dwRetVal = dwRes;
      goto Exit;
    }

    /* set master to SAFEOP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_SAFEOP);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to SAFEOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));

      /* most of the time SAFEOP is not reachable due to DCM */
      if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
      {
        EC_T_DWORD dwStatus = 0;
        EC_T_INT nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

        dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
        if (dwRes == EC_E_NOERROR)
        {
          if (dwStatus != EC_E_NOERROR)
          {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
          }
        }
        else
        {
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
        }
      }
      dwRetVal = dwRes;
      goto Exit;
    }

    /* set master to OP */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_OP);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to OP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      dwRetVal = dwRes;
      goto Exit;
    }
  }
  else
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "No ENI file provided. EC-Master started with generated ENI file.\n"));
  }

  {
    bRun = EC_TRUE;
    uint32_t i = 0, count = 0;
    uint32_t enalbe_elmo_number = 0,enalbe_yd_number = 0;
   
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Wait for %d motors to enable\n", num_slave));
    OsSleep(10);
    while (bRun)
    {
      if (motorEnable(i + 1, driver_type))
      {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Motor %d enabled successfully\n", i + 1));
        i++;
        if(driver_type[i] == ELMO)
        {
          enalbe_elmo_number++;
        }
        else if(driver_type[i] == YD)
        {
          enalbe_yd_number++;
        }
        if (i >= num_slave)
        {
          break;
        }
        count = 0;
      }
      OsSleep(1); // 1000 usec
      count++;
      if (count >= 5000)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to enable motor %d\n", i + 1));
        if(driver_type[i] == ELMO)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "elmo_Status word 0x%x\n", elmo_slave_input[enalbe_elmo_number]->status_word));
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "elmo_Error code 0x%x\n", elmo_slave_input[enalbe_elmo_number]->error_code));
        }
        else if (driver_type[i] == YD)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "yd_Status word 0x%x\n", yd_slave_input[enalbe_yd_number]->status_word));
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "yd_Error code 0x%x\n", yd_slave_input[enalbe_yd_number]->error_code));
        }
        printf("tuichukle\n");
        goto Exit;
      }
    }

    // 启动回到0位置，有死区 编码器不会回到0.0
    // motorToPosition(pAppContext, ids, num_slave, pos_offset, 90, 1e-3);

    motor_enabled = true;
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "motor enabled\n"));
  }
  OsSleep(10);

  if (pAppContext->dwPerfMeasLevel > 0)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times during startup <INIT> to <%s>:\n", ecatStateToStr(ecatGetMasterState())));
    PRINT_PERF_MEAS();
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\n"));
    /* clear job times of startup phase */
    ecatPerfMeasAppReset(pAppContext->pvPerfMeas, EC_PERF_MEAS_ALL);
    ecatPerfMeasInternalReset(EC_PERF_MEAS_ALL);
  }

#if (defined DEBUG) && (defined EC_VERSION_XENOMAI)
  /* Enabling mode switch warnings for shadowed task (mallocs before may have switched mode) */
  dwRes = rt_task_set_mode(0, T_WARNSW, NULL);
  if (0 != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "EnableRealtimeEnvironment: rt_task_set_mode returned error 0x%lx\n", dwRes));
    OsDbgAssert(EC_FALSE);
  }
#endif

  /* run the demo */
  if (pAppParms->dwDemoDuration != 0)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s will stop in %ds...\n", EC_DEMO_APP_NAME, pAppParms->dwDemoDuration / 1000));
    oAppDuration.Start(pAppParms->dwDemoDuration);
  }
  bRun = EC_TRUE;
  {
    CEcTimer oPerfMeasPrintTimer;

    if (pAppParms->bPerfMeasShowCyclic)
    {
      oPerfMeasPrintTimer.Start(2000);
    }

    while (bRun)
    {
      if (oPerfMeasPrintTimer.IsElapsed())
      {
        PRINT_PERF_MEAS();
        oPerfMeasPrintTimer.Restart();
      }

      /* check if demo shall terminate */
      bRun = !(OsTerminateAppRequest() || oAppDuration.IsElapsed());
      myAppDiagnosis(pAppContext);

      if (EC_NULL != pAppParms->pbyCnfData)
      {
        if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
        {
          EC_T_DWORD dwStatus = 0;
          EC_T_BOOL bWriteDiffLog = EC_FALSE;
          EC_T_INT nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

          if (!oDcmStatusTimer.IsStarted() || oDcmStatusTimer.IsElapsed())
          {
            bWriteDiffLog = EC_TRUE;
            oDcmStatusTimer.Start(5000);
          }

          dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
          if (dwRes == EC_E_NOERROR)
          {
            if (bFirstDcmStatus)
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
            }
            if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
            }
            if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
            {
              EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Diff (cur/avg/max) [nsec]: %7d/ %7d/ %7d\n", nDiffCur, nDiffAvg, nDiffMax));
            }
          }
          else
          {
            if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
            }
          }
#if (defined INCLUDE_DCX)
          if (eDcmMode_Dcx == pAppParms->eDcmMode && EC_E_NOERROR == dwRes)
          {
            EC_T_INT64 nTimeStampDiff = 0;
            dwRes = ecatDcxGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax, &nTimeStampDiff);
            if (EC_E_NOERROR == dwRes)
            {
              if (bFirstDcmStatus)
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
              }
              if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
              }
              if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
              {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Diff(ns): Cur=%7d, Avg=%7d, Max=%7d, TimeStamp=%7d\n", nDiffCur, nDiffAvg, nDiffMax, nTimeStampDiff));
              }
            }
            else
            {
              if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
              {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCX status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
              }
            }
          }
#endif
          if (bFirstDcmStatus && (EC_E_NOERROR == dwRes))
          {
            bFirstDcmStatus = EC_FALSE;
            ecatDcmResetStatus();
          }
        }
      }
      /* process notification jobs */
      pAppContext->pNotificationHandler->ProcessNotificationJobs();

      OsSleep(5);
    }
  }

  if (pAppParms->dwAppLogLevel >= EC_LOG_LEVEL_VERBOSE)
  {
    EC_T_DWORD dwCurrentUsage = 0;
    EC_T_DWORD dwMaxUsage = 0;
    dwRes = ecatGetMemoryUsage(&dwCurrentUsage, &dwMaxUsage);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
      goto Exit;
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage Master     (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));

#if (defined INCLUDE_RAS_SERVER)
    if (EC_NULL != pvRasServerHandle)
    {
      dwRes = emRasGetMemoryUsage(pvRasServerHandle, &dwCurrentUsage, &dwMaxUsage);
      if (EC_E_NOERROR != dwRes)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of RAS: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
      }
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage RAS Server (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));
    }
#endif
  }

Exit:
  ecMasterExit = true;

  /* set master state to INIT */
  if (eEcatState_UNKNOWN != ecatGetMasterState())
  {
    if (pAppParms->dwPerfMeasLevel > 0)
    {
      EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times before shutdown\n"));
      PRINT_PERF_MEAS();
    }
    if (pAppParms->dwPerfMeasLevel > 1)
    {
      PRINT_HISTOGRAM();
    }

    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot stop EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
  }

#if (defined INCLUDE_PCAP_RECORDER)
  SafeDelete(pPcapRecorder);
#endif /* INCLUDE_PCAP_RECORDER */

  /* unregister client */
  if (0 != RegisterClientResults.dwClntId)
  {
    dwRes = ecatUnregisterClient(RegisterClientResults.dwClntId);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot unregister client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
    }
  }
#if (defined DEBUG) && (defined EC_VERSION_XENOMAI)
  /* disable PRIMARY to SECONDARY MODE switch warning */
  dwRes = rt_task_set_mode(T_WARNSW, 0, NULL);
  if (dwRes != 0)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: rt_task_set_mode returned error %d\n", dwRes));
    OsDbgAssert(EC_FALSE);
  }
#endif

#if (defined INCLUDE_RAS_SERVER)
  /* stop RAS server */
  if (EC_NULL != pvRasServerHandle)
  {
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Stop Remote Api Server\n"));
    dwRes = emRasSrvStop(pvRasServerHandle, 2000);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Remote API Server shutdown failed\n"));
    }
  }
#endif

  /* shutdown JobTask */
  {
    CEcTimer oTimeout(2000);
    pAppContext->bJobTaskShutdown = EC_TRUE;
    while (pAppContext->bJobTaskRunning && !oTimeout.IsElapsed())
    {
      OsSleep(10);
    }
    if (EC_NULL != pvJobTaskHandle)
    {
      OsDeleteThreadHandle(pvJobTaskHandle);
    }
  }

  /* deinitialize master */
  dwRes = ecatDeinitMaster();
  if (EC_E_NOERROR != dwRes)
  {
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot de-initialize EtherCAT-Master: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
  }

  SafeDelete(pAppContext->pNotificationHandler);
  if (EC_NULL != pAppContext->pMyAppDesc)
  {
    SafeOsFree(pAppContext->pMyAppDesc->pbyFlashBuf);
    SafeOsFree(pAppContext->pMyAppDesc);
  }

  return dwRetVal;
}

bool isEcMasterExit()
{
  return ecMasterExit;
}
void setEcMasterExit()
{
  ecMasterExit = true;
}

/********************************************************************************/
/** \brief  Trigger jobs to drive master, and update process data.
 *
 * \return N/A
 */
static EC_T_VOID EcMasterJobTask(EC_T_VOID *pvAppContext)
{
  EC_T_DWORD dwRes = EC_E_ERROR;
  EC_T_INT nOverloadCounter = 0; /* counter to check if cycle time is to short */
  T_EC_DEMO_APP_CONTEXT *pAppContext = (T_EC_DEMO_APP_CONTEXT *)pvAppContext;
  T_EC_DEMO_APP_PARMS *pAppParms = &pAppContext->AppParms;

  EC_T_USER_JOB_PARMS oJobParms;
  EC_T_USER_JOB_PARMS oTaskJobParms;
  OsMemset(&oJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));
  OsMemset(&oTaskJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));

  // motorToPosition(pAppContext, ids, num_slave, q_d, 90, 4);

  /* demo loop */
  pAppContext->bJobTaskRunning = EC_TRUE;
  do
  {
    /* wait for next cycle (event from scheduler task) */
    OsWaitForEvent(pAppContext->pvJobTaskEvent, EC_WAITINFINITE);

    /* start Task (required for enhanced performance measurement) */
    oTaskJobParms.StartTask.dwTaskId = 0;
    dwRes = ecatExecJob(eUsrJob_StartTask, &oTaskJobParms);
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StartTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }

    /* process all received frames (read new input values) */
    //mtx_io.lock(); 
    dwRes = ecatExecJob(eUsrJob_ProcessAllRxFrames, &oJobParms);
    //log_buffers_in_real();
    //mtx_io.unlock();
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob( eUsrJob_ProcessAllRxFrames): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }

    if (EC_E_NOERROR == dwRes)
    {
      if (!oJobParms.bAllCycFramesProcessed)
      {
        /* it is not reasonable, that more than 5 continuous frames are lost */
        nOverloadCounter += 10;
        if (nOverloadCounter >= 50)
        {
          if ((pAppContext->dwPerfMeasLevel > 0) && (nOverloadCounter < 60))
          {
            PRINT_PERF_MEAS();
          }
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Error: System overload: Cycle time too short or huge jitter!\n"));
        }
        else
        {
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "eUsrJob_ProcessAllRxFrames - not all previously sent frames are received/processed (frame loss)!\n"));
        }
      }
      else
      {
        /* everything o.k.? If yes, decrement overload counter */
        if (nOverloadCounter > 0)
          nOverloadCounter--;
      }
    }
    /* Handle DCM logging */
#ifdef DCM_ENABLE_LOGFILE
    if (pAppParms->bDcmLogEnabled)
    {
      EC_T_CHAR *pszLog = EC_NULL;

      if (pAppContext->dwPerfMeasLevel > 0)
      {
        ecatPerfMeasAppStart(pAppContext->pvPerfMeas, PERF_DCM_Logfile);
      }
      ecatDcmGetLog(&pszLog);
      if ((EC_NULL != pszLog))
      {
        ((CAtEmLogging *)pEcLogContext)->LogDcm(pszLog);
      }
      if (pAppContext->dwPerfMeasLevel > 0)
      {
        ecatPerfMeasAppEnd(pAppContext->pvPerfMeas, PERF_DCM_Logfile);
      }
    }
#endif

      // 获取当前指针指向的缓冲区和另一个缓冲区
      SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
      SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;

      EC_T_STATE eMasterState = ecatGetMasterState();

      EC_T_BYTE* pbyPdIn = ecatGetProcessImageInputPtr();
      
      uint32_t elmo_rl = sizeof(ELMO_SlaveRead_t);
      uint32_t elmo_wl = sizeof(ELMO_SlaveWrite_t);
      uint32_t elmo_l = (elmo_rl >= elmo_wl ? elmo_rl : elmo_wl);

      uint32_t yd_rl = sizeof(YD_SlaveRead_t);
      uint32_t yd_wl = sizeof(YD_SlaveWrite_t);
        // 拷贝数据到 IN 缓冲区中
      for (uint32_t i = 0; i < num_elmo_slave; i++)
      {
        memcpy(&nextIn->elmo_slave_input[i], pbyPdIn + elmo_l * i, sizeof(ELMO_SlaveRead_t));
      }
      for (uint32_t i = 0; i < num_yd_slave; i++)
      {
        memcpy(&nextIn->yd_slave_input[i], pbyPdIn + elmo_l * num_elmo_slave + yd_rl * i, sizeof(YD_SlaveRead_t));
      }
      currentBuffersIn.store(nextIn, std::memory_order_release);
      
      {
        if (pAppContext->dwPerfMeasLevel > 0)
        {
          ecatPerfMeasAppStart(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }
        if ((eEcatState_SAFEOP == eMasterState) || (eEcatState_OP == eMasterState))
        {
          myAppWorkpd(pAppContext);
        }
        if (pAppContext->dwPerfMeasLevel > 0)
        {
          ecatPerfMeasAppEnd(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }
      }
    /* write output values of current cycle, by sending all cyclic frames */
    //mtx_io.lock();
    //log_buffers_out_real();
    SlaveBuffersOut *currentOut = currentBuffersOut.load(std::memory_order_acquire);
    EC_T_BYTE* pbyPdOut = ecatGetProcessImageOutputPtr();
    elmo_rl = sizeof(ELMO_SlaveRead_t);
    elmo_wl = sizeof(ELMO_SlaveWrite_t);
    elmo_l = (elmo_rl >= elmo_wl ? elmo_rl : elmo_wl);

    yd_rl = sizeof(YD_SlaveRead_t);
    yd_wl = sizeof(YD_SlaveWrite_t);
    // 拷贝数据到 OUT 缓冲区中
    for (uint32_t i = 0; i < num_elmo_slave; i++)
    {
      memcpy(pbyPdOut + elmo_l * i, &currentOut->elmo_slave_output[i], sizeof(ELMO_SlaveWrite_t));
    }
    for (uint32_t i = 0; i < num_yd_slave; i++)
    {
      memcpy(pbyPdOut + elmo_l * num_elmo_slave + yd_wl * i, &currentOut->yd_slave_output[i], sizeof(YD_SlaveWrite_t));
    }
    
    dwRes = ecatExecJob(eUsrJob_SendAllCycFrames, &oJobParms);
    //mtx_io.unlock();
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob( eUsrJob_SendAllCycFrames,    EC_NULL ): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }

    /* remove this code when using licensed version */
    if (EC_E_EVAL_EXPIRED == dwRes)
    {
      bRun = EC_FALSE;
    }

    /* execute some administrative jobs. No bus traffic is performed by this function */
    dwRes = ecatExecJob(eUsrJob_MasterTimer, EC_NULL);
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_MasterTimer, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }

    /* send queued acyclic EtherCAT frames */
    //mtx_io.lock();
    dwRes = ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL);
    //mtx_io.unlock();
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }
    /* stop Task (required for enhanced performance measurement) */
    oTaskJobParms.StopTask.dwTaskId = 0;
    dwRes = ecatExecJob(eUsrJob_StopTask, &oTaskJobParms);
    if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StopTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }
#if !(defined NO_OS)
  } while (!pAppContext->bJobTaskShutdown);

  pAppContext->bJobTaskRunning = EC_FALSE;
#else
    /* in case of NO_OS the job task function is called cyclically within the timer ISR */
  } while (EC_FALSE);
  pAppContext->bJobTaskRunning = !pAppContext->bJobTaskShutdown;
#endif

  return;
}

/********************************************************************************/
/** \brief  Handler for master notifications
 *
 * \return  Status value.
 */
static EC_T_DWORD EcMasterNotifyCallback(
    EC_T_DWORD dwCode,       /**< [in]   Notification code */
    EC_T_NOTIFYPARMS *pParms /**< [in]   Notification parameters */
)
{
  EC_T_DWORD dwRetVal = EC_E_NOERROR;
  CEmNotification *pNotificationHandler = EC_NULL;

  if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
  {
    dwRetVal = EC_E_INVALIDPARM;
    goto Exit;
  }

  pNotificationHandler = ((T_EC_DEMO_APP_CONTEXT *)pParms->pCallerData)->pNotificationHandler;

  if ((dwCode >= EC_NOTIFY_APP) && (dwCode <= EC_NOTIFY_APP + EC_NOTIFY_APP_MAX_CODE))
  {
    /* notification for application */
    dwRetVal = myAppNotify(dwCode - EC_NOTIFY_APP, pParms);
  }
  else
  {
    /* default notification handler */
    dwRetVal = pNotificationHandler->ecatNotify(dwCode, pParms);
  }

Exit:
  return dwRetVal;
}

/********************************************************************************/
/** \brief  RAS notification handler
 *
 * \return EC_E_NOERROR or error code
 */
#ifdef INCLUDE_RAS_SERVER
static EC_T_DWORD RasNotifyCallback(
    EC_T_DWORD dwCode,
    EC_T_NOTIFYPARMS *pParms)
{
  EC_T_DWORD dwRetVal = EC_E_NOERROR;
  CEmNotification *pNotificationHandler = EC_NULL;

  if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
  {
    dwRetVal = EC_E_INVALIDPARM;
    goto Exit;
  }

  pNotificationHandler = (CEmNotification *)pParms->pCallerData;
  dwRetVal = pNotificationHandler->emRasNotify(dwCode, pParms);

Exit:
  return dwRetVal;
}
#endif

/*-MYAPP---------------------------------------------------------------------*/

/***************************************************************************************************/
/**
\brief  Initialize Application

\return EC_E_NOERROR on success, error code otherwise.
*/

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


// 获取文件权限
mode_t getFilePermissions(const std::string& file_path) {
    struct stat file_stats;
    if (stat(file_path.c_str(), &file_stats) == 0) {
        return file_stats.st_mode;
    }
    return 0; // 如果获取权限失败，返回0
}

// 设置文件权限
void setFilePermissions(const std::string& file_path, mode_t permissions) {
    chmod(file_path.c_str(), permissions);
}


bool saveOffset()
{
    std::string file_path = getOffsetFilePath();
    std::string backup_path = file_path + ".bak";

    // 先备份文件
    std::ifstream src(file_path, std::ios::binary);
    std::ofstream dst(backup_path, std::ios::binary);
    
    if (src.is_open())
    {
        dst << src.rdbuf();
        src.close();
        dst.close();

        // 获取原文件权限并应用到备份文件
        mode_t original_permissions = getFilePermissions(file_path);
        setFilePermissions(backup_path, original_permissions);
    }
    else
    {
        std::cerr << "[ECmaster]: No existing file to backup: " << file_path << std::endl;
    }

    std::ofstream ofs(file_path);
    
    // 检查文件是否成功打开
    if (!ofs.is_open())
    {
        std::cerr << "[ECmaster]: Failed to open the file for writing: " << file_path << std::endl;
        mtx_io.unlock(); // 解锁互斥量
        return false;
    }

    // 写入数据，每行一个数据
    for (uint32_t i = 0; i < NUM_SLAVE_MAX; i++)
    {
        ofs << pos_offset[i] << std::endl;
    }

    ofs.close(); // 关闭文件
    return true;
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
        mtx_io.lock();
        pos_offset[i * num_onelen_item + j] = offset[i][j];
        mtx_io.unlock();
      }
    }
  }
  return true;
}
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_UNREFPARM(pAppContext);

  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Enter my app Init\n"));

#ifdef ERROR_FIX
  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "电机错误已启动!请在cmakefile 中开关此宏\n"));
#endif
  loadOffset();

  return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  Initialize Slave Instance.

Find slave parameters.
\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  T_MY_APP_DESC *pMyAppDesc = pAppContext->pMyAppDesc;
  EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
  OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

  // 根据参商id查找 设备, 并统计数目
  EC_T_DWORD dwSlaveIdx = 0;
  EC_T_DWORD dwNumConfiguredSlaves = emGetNumConfiguredSlaves(pAppContext->dwInstanceId);
  num_slave = dwNumConfiguredSlaves;//总的从站数目

  //初始化从站计数器
  num_elmo_slave = 0;
  num_yd_slave = 0; 

  //识别从站并将个数统计到各自的num_slave
  for(dwSlaveIdx = 0; dwSlaveIdx < dwNumConfiguredSlaves; dwSlaveIdx++)
  {
     EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

    /* get config slave information */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR) 
    dwRes = emGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
    dwRes = esGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#endif
  //根据不同的厂商的id统计从站数目 （Vendor 厂商ID）
    if(oCfgSlaveInfo.dwVendorId == YD_VENDOR_ID)
    {
      num_yd_slave++;
      // printf("num_yd_slave = %d\n",num_yd_slave);   
    }
    else if(oCfgSlaveInfo.dwVendorId == ELMO_VENDOR_ID)
    {      
      num_elmo_slave++;
      // printf("num_elmo_slave = %d\n",num_elmo_slave);
    }
  }
  std::cout << "[ECmaster]: num_elmo_slave = " << num_elmo_slave << std::endl;
  std::cout << "[ECmaster]: num_yd_slave = " << num_yd_slave << std::endl;

  // 获取 收发 image
  EC_T_BYTE *pbyPdIn = ecatGetProcessImageInputPtr();
  EC_T_BYTE *pbyPdOut = ecatGetProcessImageOutputPtr();
  
  uint32_t elmo_rl = sizeof(ELMO_SlaveRead_t);
  uint32_t elmo_wl = sizeof(ELMO_SlaveWrite_t);
  uint32_t elmo_l = elmo_rl >= elmo_wl ? elmo_rl : elmo_wl; // ELMO的需要对齐 ENI BitOffs
  
  uint32_t yd_rl = sizeof(YD_SlaveRead_t);
  uint32_t yd_wl = sizeof(YD_SlaveWrite_t);

  // 分配内存给buffersA和buffersB
  buffersAIn.elmo_slave_input = (ELMO_SlaveRead_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveRead_t));
  buffersAIn.yd_slave_input = (YD_SlaveRead_t *)malloc(num_yd_slave * sizeof(YD_SlaveRead_t));
  buffersBIn.elmo_slave_input = (ELMO_SlaveRead_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveRead_t));
  buffersBIn.yd_slave_input = (YD_SlaveRead_t *)malloc(num_yd_slave * sizeof(YD_SlaveRead_t));
  

  buffersAOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersAOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));
  buffersBOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersBOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));

  buffersCOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersCOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));
  buffersDOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersDOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));

  buffersEOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersEOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));
  buffersFOut.elmo_slave_output = (ELMO_SlaveWrite_t *)malloc(num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  buffersFOut.yd_slave_output = (YD_SlaveWrite_t *)malloc(num_yd_slave * sizeof(YD_SlaveWrite_t));


  // 清空初始数据
  memset(buffersAIn.elmo_slave_input, 0, num_elmo_slave * sizeof(ELMO_SlaveRead_t));
  memset(buffersAIn.yd_slave_input, 0, num_yd_slave * sizeof(YD_SlaveRead_t));
  memset(buffersBIn.elmo_slave_input, 0, num_elmo_slave * sizeof(ELMO_SlaveRead_t));
  memset(buffersBIn.yd_slave_input, 0, num_yd_slave * sizeof(YD_SlaveRead_t));

  memset(buffersAOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersAOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));
  memset(buffersBOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersBOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));

  // Clear initial data
  memset(buffersCOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersCOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));
  memset(buffersDOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersDOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));

    // Clear initial data
  memset(buffersEOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersEOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));
  memset(buffersFOut.elmo_slave_output, 0, num_elmo_slave * sizeof(ELMO_SlaveWrite_t));
  memset(buffersFOut.yd_slave_output, 0, num_yd_slave * sizeof(YD_SlaveWrite_t));

  // 拷贝数据到 IN 缓冲区中
  for (uint32_t i = 0; i < num_elmo_slave; i++)
  {
    memcpy(&buffersAIn.elmo_slave_input[i], pbyPdIn + elmo_l * i, sizeof(ELMO_SlaveRead_t));
    memcpy(&buffersBIn.elmo_slave_input[i], pbyPdIn + elmo_l * i, sizeof(ELMO_SlaveRead_t));
  }

  for (uint32_t i = 0; i < num_yd_slave; i++)
  {
    memcpy(&buffersAIn.yd_slave_input[i], pbyPdIn + elmo_l * num_elmo_slave + yd_rl * i, sizeof(YD_SlaveRead_t));
    memcpy(&buffersBIn.yd_slave_input[i], pbyPdIn + elmo_l * num_elmo_slave + yd_rl * i, sizeof(YD_SlaveRead_t));
  }

  // 初始化原子指针指向buffersA或buffersB中的一个，假设指向A作为初始可读数据
  currentBuffersIn.store(&buffersAIn, std::memory_order_release);
  currentBuffersOut.store(&buffersAOut, std::memory_order_release);
  currentPositionBuffersOut.store(&buffersCOut, std::memory_order_release);
  currentTorqueBuffersOut.store(&buffersEOut, std::memory_order_release);


  memset(motorStatusMapA, 0, sizeof(motorStatusMapA));
  memset(motorStatusMapB, 0, sizeof(motorStatusMapB));
  currentMotorStatusMap.store(motorStatusMapA, std::memory_order_release);

  // 初始化 torque_feedback_target 双缓冲区
  memset(torque_feedback_targetA, 0, sizeof(torque_feedback_targetA));
  memset(torque_feedback_targetB, 0, sizeof(torque_feedback_targetB));
  currentTorqueFeedback.store(torque_feedback_targetA, std::memory_order_release);

  if (pAppContext->AppParms.bFlash)
  {
    EC_T_WORD wFlashSlaveAddr = pAppContext->AppParms.wFlashSlaveAddr;

    /* check if slave address is provided */
    if (wFlashSlaveAddr != INVALID_FIXED_ADDR)
    {
      /* get slave's process data offset and some other infos */
      dwRes = ecatGetCfgSlaveInfo(EC_TRUE, wFlashSlaveAddr, &oCfgSlaveInfo);
      if (dwRes != EC_E_NOERROR)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatGetCfgSlaveInfo() returns with error=0x%x, slave address=%d\n", dwRes, wFlashSlaveAddr));
        goto Exit;
      }

      if (oCfgSlaveInfo.dwPdSizeOut != 0)
      {
        pMyAppDesc->dwFlashPdBitSize = oCfgSlaveInfo.dwPdSizeOut;
        pMyAppDesc->dwFlashPdBitOffs = oCfgSlaveInfo.dwPdOffsOut;
      }
      else
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: Slave address=%d has no outputs, therefore flashing not possible\n", wFlashSlaveAddr));
      }
    }
    else
    {
      /* get complete process data output size */
      EC_T_MEMREQ_DESC oPdMemorySize;
      OsMemset(&oPdMemorySize, 0, sizeof(EC_T_MEMREQ_DESC));

      dwRes = ecatIoCtl(EC_IOCTL_GET_PDMEMORYSIZE, EC_NULL, 0, &oPdMemorySize, sizeof(EC_T_MEMREQ_DESC), EC_NULL);
      if (dwRes != EC_E_NOERROR)
      {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatIoControl(EC_IOCTL_GET_PDMEMORYSIZE) returns with error=0x%x\n", dwRes));
        goto Exit;
      }
      pMyAppDesc->dwFlashPdBitSize = oPdMemorySize.dwPDOutSize * 8;
    }
    if (pMyAppDesc->dwFlashPdBitSize > 0)
    {
      pMyAppDesc->dwFlashInterval = 20000; /* flash every 20 msec */
      pMyAppDesc->dwFlashBufSize = BIT2BYTE(pMyAppDesc->dwFlashPdBitSize);
      pMyAppDesc->pbyFlashBuf = (EC_T_BYTE *)OsMalloc(pMyAppDesc->dwFlashBufSize);
      OsMemset(pMyAppDesc->pbyFlashBuf, 0, pMyAppDesc->dwFlashBufSize);
    }
  }

  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "App prepare down!\n"));

Exit:
  return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  Setup slave parameters (normally done in PREOP state)

  - SDO up- and Downloads
  - Read Object Dictionary

\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_UNREFPARM(pAppContext);

  motorGetConfig(pAppContext);

  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "App setup down!\n"));

  return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  demo application working process data function.

  This function is called in every cycle after the the master stack is started.

*/
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  T_MY_APP_DESC *pMyAppDesc = pAppContext->pMyAppDesc;
  EC_T_BYTE *pbyPdOut = ecatGetProcessImageOutputPtr();
  SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
  //SlaveBuffersIn *nextIn = (currentIn == &buffersAIn) ? &buffersBIn : &buffersAIn;
  SlaveBuffersOut *currentOut = currentBuffersOut.load(std::memory_order_acquire);
  TorqueFeedbackTarget_t* currentTorqueFeedbackPtr = currentTorqueFeedback.load(std::memory_order_acquire);
  
  struct timespec t0;
  static double prev_v[NUM_SLAVE_MAX] = {0};
  double v[NUM_SLAVE_MAX] = {0};
  double vd[NUM_SLAVE_MAX] = {0};

  MotorParam_t motorData[NUM_SLAVE_MAX];
  // ****************** add pos vel feedback to torque ************************
  uint8_t elmo_number = 0, yd_number = 0;
  std::vector<double> real_tau;
  // std::vector<double> real_mode;
  real_tau.resize(num_slave);
  // real_mode.resize(num_slave);
  //mtx_io.lock();
  SlaveBuffersOut *currentPositionOut = currentPositionBuffersOut.load(std::memory_order_acquire);
  SlaveBuffersOut *currentTorqueOut = currentTorqueBuffersOut.load(std::memory_order_acquire);
  if (motor_enabled)
  {
    for (uint32_t i = 0; i < num_slave; i++)
    {
      // real_mode[i] =yd_slave_output[yd_number]->mode_of_opration;
      if (driver_type[i] == ELMO)
      {
        if (currentTorqueOut->elmo_slave_output[elmo_number].mode_of_opration == MODE_CST)
        {
          currentOut->elmo_slave_output[elmo_number].target_torque = currentTorqueOut->elmo_slave_output[elmo_number].target_torque;
          currentOut->elmo_slave_output[elmo_number].torque_offset = currentTorqueOut->elmo_slave_output[elmo_number].torque_offset;
          currentOut->elmo_slave_output[elmo_number].max_torque = currentTorqueOut->elmo_slave_output[elmo_number].max_torque;
          currentOut->elmo_slave_output[elmo_number].mode_of_opration = currentTorqueOut->elmo_slave_output[elmo_number].mode_of_opration;
          currentOut->elmo_slave_output[elmo_number].control_word = currentTorqueOut->elmo_slave_output[elmo_number].control_word;
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          double current_position = currentIn->elmo_slave_input[elmo_number].position_actual_value * (360.0 / encoder_range[i]);
          double current_velocity = currentIn->elmo_slave_input[elmo_number].velocity_actual_value * (360.0 / encoder_range[i]);
          TorqueFeedbackTarget_t &target = currentTorqueFeedbackPtr[i];

          uint16_t max_torque = currentOut->elmo_slave_output[elmo_number].max_torque;
          int16_t target_tau = (target.target_torque + (target.target_position - current_position) * TO_RADIAN * target.kp + (target.target_velocity - current_velocity) * TO_RADIAN * target.kd) * (1000.0 / rated_current[i]) * 1000;
          int32_t max_torque_32 = static_cast<int32_t>(max_torque);
          int32_t target_tau_32 = static_cast<int32_t>(target_tau);
          if (target_tau_32 > max_torque_32)
          {
            target_tau = static_cast<int16_t>(max_torque);
          }
          else if (target_tau_32 < -max_torque_32)
          {
            target_tau = static_cast<int16_t>(-max_torque);
          }
          currentOut->elmo_slave_output[elmo_number].target_torque = target_tau;
        }
        else if (currentPositionOut->elmo_slave_output[elmo_number].mode_of_opration == MODE_CSP)
        {
          currentOut->elmo_slave_output[elmo_number].target_position = currentPositionOut->elmo_slave_output[elmo_number].target_position;
          currentOut->elmo_slave_output[elmo_number].position_offset = currentPositionOut->elmo_slave_output[elmo_number].position_offset;
          currentOut->elmo_slave_output[elmo_number].velocit_offset = currentPositionOut->elmo_slave_output[elmo_number].velocit_offset;
          currentOut->elmo_slave_output[elmo_number].torque_offset = currentPositionOut->elmo_slave_output[elmo_number].torque_offset;
          currentOut->elmo_slave_output[elmo_number].max_torque = currentPositionOut->elmo_slave_output[elmo_number].max_torque;
          currentOut->elmo_slave_output[elmo_number].mode_of_opration = currentPositionOut->elmo_slave_output[elmo_number].mode_of_opration;
          currentOut->elmo_slave_output[elmo_number].control_word = currentPositionOut->elmo_slave_output[elmo_number].control_word;
        }

        real_tau[i] = currentOut->elmo_slave_output[elmo_number].target_torque;
        elmo_number++;
      }
      else if (driver_type[i] == YD)
      {
        
        if (currentTorqueOut->yd_slave_output[yd_number].mode_of_opration == MODE_CST)
        {
          currentOut->yd_slave_output[yd_number].target_torque = currentTorqueOut->yd_slave_output[yd_number].target_torque;
          currentOut->yd_slave_output[yd_number].torque_offset = currentTorqueOut->yd_slave_output[yd_number].torque_offset;
          currentOut->yd_slave_output[yd_number].mode_of_opration = currentTorqueOut->yd_slave_output[yd_number].mode_of_opration;
          currentOut->yd_slave_output[yd_number].control_word = currentTorqueOut->yd_slave_output[yd_number].control_word;
          /////////////////////////////////////////////////////////////////////////////////////////////////
          double current_position = currentIn->yd_slave_input[yd_number].position_actual_value * (360.0 / encoder_range[i]);
          double current_velocity = currentIn->yd_slave_input[yd_number].velocity_actual_value * (360.0 / encoder_range[i]);
          TorqueFeedbackTarget_t &target = currentTorqueFeedbackPtr[i];

          int16_t target_tau = (target.target_torque + (target.target_position - current_position) * TO_RADIAN * target.kp + (target.target_velocity - current_velocity) * TO_RADIAN * target.kd) * (1000.0 / rated_current[i]) * 1000;

          currentOut->yd_slave_output[yd_number].target_torque = target_tau / 1.414;
        }
        else if (currentPositionOut->yd_slave_output[yd_number].mode_of_opration == MODE_CSP)
        {
          currentOut->yd_slave_output[yd_number].target_position = currentPositionOut->yd_slave_output[yd_number].target_position;
          currentOut->yd_slave_output[yd_number].velocity_offset = currentPositionOut->yd_slave_output[yd_number].velocity_offset;
          currentOut->yd_slave_output[yd_number].torque_offset = currentPositionOut->yd_slave_output[yd_number].torque_offset;
          currentOut->yd_slave_output[yd_number].mode_of_opration = currentPositionOut->yd_slave_output[yd_number].mode_of_opration;
          currentOut->yd_slave_output[yd_number].control_word = currentPositionOut->yd_slave_output[yd_number].control_word;
        }

        real_tau[i] = currentOut->yd_slave_output[yd_number].target_torque;
        yd_number++;
      }
      else
      {
        real_tau[i] = -1;
      }
    }
  }
    // mtx_io.unlock();
    //  log_vector("/sensor_data/joint_data/real_tau",real_tau);
    //  log_vector("/sensor_data/joint_data/real_mode",real_mode);
    if (motor_enabled)
    {
      // if (restartMotorFlag == false)
      {
        // 获取当前 motorStatusMap 缓冲区指针
        uint8_t *currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
        // 确定下一个motorStatusMap缓冲区
        uint8_t *nextMotorStatus = (currentMotorStatus == motorStatusMapA) ? motorStatusMapB : motorStatusMapA;
        uint8_t yd_number_status = 0, elmo_number_status = 0;
        for (uint8_t i = 0; i < num_slave; i++)
        {
          // if motor status_word is fault
          // mtx_io.lock();
          bool isFault;
          if (driver_type[i] == ELMO)
          {
            isFault = currentIn->elmo_slave_input[elmo_number_status].status_word & 1 << 3;
            elmo_number_status++;
          }
          else if (driver_type[i] == YD)
          {
            isFault = currentIn->yd_slave_input[yd_number_status].status_word & 1 << 3;
            yd_number_status++;
          }
          // mtx_io.unlock();
          if (isFault && currentMotorStatus[i] == MOTOR_STATUS_NO_ERROR)
          {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d is fault!\n", ids[i]));
            restartMotorFlag = true;
            nextMotorStatus[i] = MOTOR_STATUS_ERROR;
          }
          else if (!isFault && currentMotorStatus[i] != MOTOR_STATUS_NO_ERROR)
          {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d has been fixed!\n", ids[i]));
            nextMotorStatus[i] = MOTOR_STATUS_NO_ERROR;
          }
        }
        // 在这里切换 currentMotorStatusMap 指针，让 nextMotorStatus 成为新生效的数据
        currentMotorStatusMap.store(nextMotorStatus, std::memory_order_release);
      }
      // mtx_j.lock();
      for (uint32_t i = 0; i < num_slave; i++)
      {
        prev_v[i] = v[i];
      }
      motorGetData(ids, driver_type, num_slave, motorData);
      for (uint32_t i = 0; i < num_slave; i++)
      {
        v[i] = motorData[i].velocity;
        vd[i] = (v[i] - prev_v[i]) / (pAppContext->AppParms.dwBusCycleTimeUsec / 1e6);
        motorAcceleration[i] = vd[i];
      }
      // mtx_j.unlock();
    }
    // log_buffers_in();
    // log_buffers_out();
  /* demo code flashing */
  if (pMyAppDesc->dwFlashPdBitSize != 0)
  {
    pMyAppDesc->dwFlashTimer += pAppContext->AppParms.dwBusCycleTimeUsec;
    if (pMyAppDesc->dwFlashTimer >= pMyAppDesc->dwFlashInterval)
    {
      pMyAppDesc->dwFlashTimer = 0;

      /* flash with pattern */
      pMyAppDesc->byFlashVal++;
      OsMemset(pMyAppDesc->pbyFlashBuf, pMyAppDesc->byFlashVal, pMyAppDesc->dwFlashBufSize);

      /* update PdOut */
      EC_COPYBITS(pbyPdOut, pMyAppDesc->dwFlashPdBitOffs, pMyAppDesc->pbyFlashBuf, 0, pMyAppDesc->dwFlashPdBitSize);
    }
  }
  return EC_E_NOERROR;
  }

/***************************************************************************************************/
/**
\brief  demo application doing some diagnostic tasks

  This function is called in sometimes from the main demo task
*/
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_UNREFPARM(pAppContext);

  if (ecMasterExit)
  {
    motor_enabled = false;
  }

  // if (restartMotorFlag)
  {
    SlaveBuffersIn *currentIn = currentBuffersIn.load(std::memory_order_acquire);
    bool has_error = false;
    std::vector<double> error_code_vec;
    // 获取当前 motorStatusMap 缓冲区
    uint8_t* currentMotorStatus = currentMotorStatusMap.load(std::memory_order_acquire);
    for (uint8_t i = 0; i < num_slave; i++)
    {
      if (currentMotorStatus[i] == MOTOR_STATUS_ERROR)
      {
        has_error = true;
        if(driver_type[i] == ELMO)
        {
          motorErrorCodeMap[i] = currentIn->elmo_slave_input[i].error_code;
        }
        else if(driver_type[i] == YD)
        {
          motorErrorCodeMap[i] = currentIn->yd_slave_input[i].error_code;
        }
        // 错误的同时不一定会读到错误码
        if (motorErrorCodeMap[i])
        {
          if (std::clock() - last_restart_time[i] < 3e+5)
          {
            continue;
          }
          last_restart_time[i] = std::clock();
          currentMotorStatus[i] = MOTOR_STATUS_REINIT;
// #ifdef ERROR_FIX // 是否需要修复错误
//           std::thread restartThread(fixEmergencyRequest, ids[i], motorErrorCodeMap[i]);
//           restartThread.detach();
//           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d fix error %x!\n", ids[i], motorErrorCodeMap[i]));
// #else
          restartMotorFlag = false;
          int errorCode = motorErrorCodeMap[i];
          if(driver_type[i] == ELMO)
          {
            if (errorCode == SENSOR_FEEDBACK_ERROR)
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 传感器错误\n", ids[i], motorErrorCodeMap[i]));
            }
            else if (errorCode == FAILED_TO_START_MOTOR)
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 传感器错误\n", ids[i], motorErrorCodeMap[i]));
            }
            else if (errorCode == FEEDBACK_ERROR || errorCode == POSITION_TRACKING_ERROR)
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 位置跟踪错误\n", ids[i], motorErrorCodeMap[i]));
            }
            else if (errorCode == SPEED_TRACKING_ERROR)
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 速度跟踪错误\n", ids[i], motorErrorCodeMap[i]));
            }
            else
            {
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! \n", ids[i], motorErrorCodeMap[i]));
            }
          }
          else if(driver_type[i] == YD)
          {
            switch (errorCode)
            {
            case 0x001:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 检测到功率器件的短路保护\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x004:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! EPROM存储异常\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x006:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! AD 采样故障\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x007:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 编码器断线\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x008:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 编码器CRC校验错误\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x009:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 编码器内部计数错误\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x00A:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 供电电压过低,低于15V\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x00B:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 供电电压过高,超过72V\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x00C:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 电机三相线异常\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x00D:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 电机长时间过载\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x00E:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 驱动器长时间过载\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x010:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 驱动器过热停机\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x012:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 电机转速超过驱动器速度上限\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x013:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 位置跟踪误差过大\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x019:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 多圈编码器电池电压过低\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x01A:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 多圈编码器电池断联,请按Pr8.01 = 2复位\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x025:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x!  PDO 通信设置同步时间超范围\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x026:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x!  PDO 通信数据超范围\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x027:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x!  长时间未接收到 PDO 通信\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x028:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 看门狗周期寄存器时间为零\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0x029:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 位置指令对应得速度指令超过额定转速 2倍\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE01:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 电机过热警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE02:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 驱动器温度过高警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE03:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 电机过载警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE04:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 驱动器过载警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE05:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 位置偏差过大警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE06:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 制动过载警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE07:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 正向超程警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            case 0xE08:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! 反向超程警告\n", ids[i], motorErrorCodeMap[i]));
              break;
            default:
              EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Motor %d err %x! \n", ids[i], motorErrorCodeMap[i]));
              break;
            }
          }
          // motorStatusMap[i] = MOTOR_STATUS_NO_ERROR;
// #endif
        }
        error_code_vec.push_back(static_cast<double>(motorErrorCodeMap[i]));
      }else
      {
        error_code_vec.push_back(0.0);
      }
    }
    if (has_error)
    {
      log_vector("/sensor_data/joint_data/error_code",error_code_vec);
    }
  }

  return EC_E_NOERROR;
}

/********************************************************************************/
/** \brief  Handler for application notifications
 *
 *  !!! No blocking API shall be called within this function!!!
 *  !!! Function is called by cylic task                    !!!
 *
 * \return  Status value.
 */
static EC_T_DWORD myAppNotify(
    EC_T_DWORD dwCode,       /* [in] Application notification code */
    EC_T_NOTIFYPARMS *pParms /* [in] Notification parameters */
)
{
  EC_T_DWORD dwRetVal = EC_E_ERROR;
  T_EC_DEMO_APP_CONTEXT *pAppContext = (T_EC_DEMO_APP_CONTEXT *)pParms->pCallerData;

  /* dispatch notification code */
  switch (dwCode)
  {
  case 1:
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "myAppNotify: Notification code=0x%lx received\n", dwCode));
    /* dwRetVal = EC_E_NOERROR; */
    break;
  case 2:
    break;
  default:
    break;
  }

  return dwRetVal;
}

EC_T_VOID ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  const EC_T_CHAR *szAppUsage = "<LinkLayer> [-f ENI-FileName] [-t time] [-b cycle time] [-a affinity] [-v lvl] [-perf [level]] [-log prefix [msg cnt]] [-lic key] [-oem key]  [-flash address]"
#if (defined INCLUDE_RAS_SERVER)
                                " [-sp [port]]"
#endif
#if (defined AUXCLOCK_SUPPORTED)
                                " [-auxclk period]"
#endif
                                " [-dcmmode mode] [-ctloff]"
#if (defined INCLUDE_PCAP_RECORDER)
                                " [-rec [prefix [frame cnt]]]"
#endif
      ;
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s V%s for %s %s\n", EC_DEMO_APP_NAME, ATECAT_FILEVERSIONSTR, ATECAT_PLATFORMSTR, ATECAT_COPYRIGHT));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Syntax:\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s %s", EC_DEMO_APP_NAME, szAppUsage));
}

EC_T_VOID ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -flash            Flash outputs\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     address         0=all, >0 = slave station address\n"));
#if (defined AUXCLOCK_SUPPORTED)
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -auxclk           use auxiliary clock\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     period          clock period in usec\n"));
#endif
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -dcmmode          Set DCM mode\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     off                Off\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     busshift           BusShift mode (default)\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     mastershift        MasterShift mode\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     masterrefclock     MasterRefClock mode\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     linklayerrefclock  LinkLayerRefClock mode\n"));
#if (defined INCLUDE_DCX)
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     dcx                External synchronization mode\n"));
#endif
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -ctloff           Disable DCM control loop for diagnosis\n"));
#if (defined INCLUDE_PCAP_RECORDER)
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rec              Record network traffic to pcap file\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [prefix          pcap file name prefix\n"));
  EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [frame cnt]      Frame count for log buffer allocation (default = %d, with %d bytes per message)]\n", PCAP_RECORDER_BUF_FRAME_CNT, ETHERNET_MAX_FRAMEBUF_LEN));
#endif
}

void disableMotor(const uint16_t *ids, uint32_t num_id)
{
  
  for (uint32_t i = 0; i < num_id; i++)
  {
    auto id = ids[i];
    assert(id < NUM_SLAVE_MAX && id >= 0);
    g_motor_disabled[id] = true;
  }

  // Print!
  std::cout << ">>>>>>>>>>>>>>>>> EcMaster Disabled Id: [";
  for(int i = 0; i < NUM_SLAVE_MAX; i++) {
    if(g_motor_disabled[i]) {
      std::cout << i << " ";
    }
  }
  std::cout << "]" << std::endl;
}
/*-END OF SOURCE FILE--------------------------------------------------------*/
