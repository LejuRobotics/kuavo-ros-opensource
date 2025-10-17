#include "EcDemoApp.h"
#include "EcMotor.h"
#include "util.h"
#include <mutex>
#include <cmath>
#include "ObjectDiction.h"

#define DCM_ENABLE_LOGFILE

#define ELMO_VENDOR_ID 0x0000009A
#define ELMO_PRODUCT_CODE 0x00030924

#define YD_VENDOR_ID 0x0000005A
#define YD_PRODUCT_CODE 0x00000003

#define PERF_myAppWorkpd 0
#define PERF_DCM_Logfile 1
#define MAX_JOB_NUM 2


#define BIT_17 (1 << 17)
#define BIT_17_8 (BIT_17 * 8)
#define BIT_17_9 (BIT_17 * 9)
#define BIT_17_10 (BIT_17 * 10)
#define BIT_17_18 (BIT_17 * 18)
#define BIT_17_36 (BIT_17 * 36)
#define MAX_TORQUE (31.2)
#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)


#define MODE_CSP (8)
#define MODE_CSV (9)
#define MODE_CST (10)

static double pos_offset[NUM_SLAVE_MAX] = {0}; // 注意这个是保留值，只能被setoffset和csvRead 刷新

static uint32_t rated_current[NUM_SLAVE_MAX] = {0};//额定电流

static uint32_t encoder_range[NUM_SLAVE_MAX] = {0};

//通过枚举来辨别不同类型驱动器，用数组确定不同位置所用的驱动器    
enum EcMasterType driver_type[NUM_SLAVE_MAX] = {YD} ;

YD_SlaveRead_t *yd_slave_input[NUM_SLAVE_YD_MAX];
YD_SlaveWrite_t *yd_slave_output[NUM_SLAVE_YD_MAX];
uint32_t num_yd_slave = 0;

ELMO_SlaveRead_t *elmo_slave_input[NUM_SLAVE_ELMO_MAX];
ELMO_SlaveWrite_t *elmo_slave_output[NUM_SLAVE_ELMO_MAX];
uint32_t num_elmo_slave = 0;

static std::mutex mtx_io;
uint32_t num_motor_slave = 0;

double motor_data::motorAcceleration[NUM_SLAVE_MAX] = {0};
uint16_t motor_data::ids[NUM_SLAVE_MAX] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30
};

int restartCounter = 0;
bool restartMotorFlag = false;
uint8_t motorStatusMap[NUM_SLAVE_MAX] = {0};
int motorErrorCodeMap[NUM_SLAVE_MAX] = {0};

MotorInfo_t g_motor_info[NUM_SLAVE_MAX] = {0};

// 机器人关节映射，Slave-1 -> Joint-1
static uint8_t slave2joint[NUM_SLAVE_MAX] = {0};
uint8_t slaveTojoint(uint8_t SlaveId)
{
  return slave2joint[SlaveId];
}

/**
 * @brief 获取驱动器基础配置并打印
 *
 * @param pAppContext ec_master
 * @return true
 * @return false
 * @details 获取地址和格式参照驱动板的xml文件，不需要出现在 PDO map 中
 */
static bool motorGetBasicConfig(T_EC_DEMO_APP_CONTEXT *pAppContext)
{
  EC_T_DWORD dwRes = EC_E_NOERROR;
  uint32_t value;
  uint8_t buf[4];
  uint32_t outdata_len;
  for (uint32_t i = 0; i < num_motor_slave; i++)
  {
    dwRes = emCoeSdoUpload(0, g_motor_info[i].slave_id-1, INDEX_POSITION_ACTUAL_VALUE, 0, buf, 4, &outdata_len, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed to emCoeSdoUpload, Joint %d, Slave %d, Err (0x%lx)\n", g_motor_info[i].logical_id+1, g_motor_info[i].slave_id, dwRes));
      return false;
    }
    value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

    char *tempfloat = floatToChar(value * 360.0 / encoder_range[i]);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Joint %d, Slave %d actual position encoder %s", g_motor_info[i].logical_id+1, g_motor_info[i].slave_id, tempfloat));
    free(tempfloat);
    tempfloat = floatToChar(value);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Joint %d, Slave %d actual position %s", g_motor_info[i].logical_id+1, g_motor_info[i].slave_id, tempfloat));
    free(tempfloat);

    dwRes = emCoeSdoUpload(0, g_motor_info[i].slave_id-1, INDEX_MOTOR_RATED_CURRENT, 0, buf, 4, &outdata_len, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed ot emCoeSdoUpload, Joint %d, Slave %d, Err (0x%lx)\n", g_motor_info[i].logical_id+1, g_motor_info[i].slave_id, dwRes));
      return false;
    }
    value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    rated_current[i] = value;

    char *tempfloat1 = floatToChar(value / 1000.0);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Rated current %s\n", tempfloat));
    free(tempfloat1);
  }
  return true;
}


bool motorEnable(const uint16_t id)
{
  if (id < 1 || id > num_motor_slave)
  {
    return false;
  }
  uint16_t sw;
  uint16_t index = id - 1;
  mtx_io.lock();
  if(g_motor_info[index].driver_type == ELMO)
  {
    sw = elmo_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f;
  }
  else if(g_motor_info[index].driver_type == YD)
  {
    sw = yd_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f;
  }
  mtx_io.unlock();
  if (sw == 0x27)
  {
    return true;
  }
  mtx_io.lock();
  if(g_motor_info[index].driver_type == ELMO)
  {
    elmo_slave_output[g_motor_info[index].pdo_id]->target_position = elmo_slave_input[g_motor_info[index].pdo_id]->position_actual_value;
    elmo_slave_output[g_motor_info[index].pdo_id]->position_offset = 0;
    elmo_slave_output[g_motor_info[index].pdo_id]->velocit_offset = 0;
    elmo_slave_output[g_motor_info[index].pdo_id]->torque_offset = 0;
    elmo_slave_output[g_motor_info[index].pdo_id]->max_torque = 1000;
    elmo_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
    elmo_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(sw);
  }
  else if(g_motor_info[index].driver_type == YD)
  { 
    yd_slave_output[g_motor_info[index].pdo_id]->target_position = yd_slave_input[g_motor_info[index].pdo_id]->position_actual_value;
    yd_slave_output[g_motor_info[index].pdo_id]->velocity_offset = 0;
    yd_slave_output[g_motor_info[index].pdo_id]->torque_offset = 0;
    yd_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
    yd_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(sw);
  }
  
  mtx_io.unlock();
  // printf("end elmo_Number: %d, yd_Number: %d, num_motor_slave: %d\n", elmo_Number, yd_Number, num_motor_slave);
  return false;
}

void motorGetData(const uint16_t *ids, uint32_t num,
                  MotorParam_t *data)
{
  uint16_t index = 0;

  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if(g_motor_info[index].driver_type == ELMO)
    {
      data[i].position = (elmo_slave_input[g_motor_info[index].pdo_id]->position_actual_value * (360.0 / encoder_range[index])  - pos_offset[index]) ;
      data[i].velocity = elmo_slave_input[g_motor_info[index].pdo_id]->velocity_actual_value * (360.0 / encoder_range[index]);
      data[i].torque = elmo_slave_input[g_motor_info[index].pdo_id]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0;
      data[i].acceleration = motor_data::motorAcceleration[index] * (360.0 / encoder_range[index]);
      data[i].status = motorStatusMap[index];

      // 驱动器控制环路底层数据
      data[i].error_code = elmo_slave_input[g_motor_info[index].pdo_id]->error_code;
      data[i].status_word = elmo_slave_input[g_motor_info[index].pdo_id]->status_word;
    }
    else if(g_motor_info[index].driver_type == YD)
    {
      data[i].position = (yd_slave_input[g_motor_info[index].pdo_id]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
      data[i].velocity = yd_slave_input[g_motor_info[index].pdo_id]->velocity_actual_value * (360.0 / encoder_range[index]) ;
      data[i].torque = yd_slave_input[g_motor_info[index].pdo_id]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
      data[i].acceleration = motor_data::motorAcceleration[index] * (360.0 / encoder_range[index]);
      data[i].status = motorStatusMap[index];
      
      // 驱动器控制环路底层数据
      data[i].error_code = yd_slave_input[g_motor_info[index].pdo_id]->error_code;
      data[i].status_word = yd_slave_input[g_motor_info[index].pdo_id]->status_word;
      data[i].torque_demand_trans = yd_slave_input[g_motor_info[index].pdo_id]->torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
    }
  }
  mtx_io.unlock();
}

/**
 * @brief 获取单个电机的数据
 * @param id 电机ID（从1开始）
 * @param driver 电机驱动类型数组（EcMasterType）
 * @param data 用于存放返回数据的MotorParam_t结构体指针
 */
void motorGetSingleData(uint16_t id, MotorParam_t *data)
{
  uint16_t index = id - 1;
  mtx_io.lock();
  if (g_motor_info[index].driver_type == ELMO) {
    if (elmo_slave_input[g_motor_info[index].pdo_id] == nullptr) {
        printf("Error:[motorGetSingleData]:elmo_slave_input[%d] is nullptr!\n", g_motor_info[index].pdo_id);
        mtx_io.unlock();
        return;
    }
    data->position = (elmo_slave_input[g_motor_info[index].pdo_id]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
    data->velocity = elmo_slave_input[g_motor_info[index].pdo_id]->velocity_actual_value * (360.0 / encoder_range[index]);
    data->torque = elmo_slave_input[g_motor_info[index].pdo_id]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0;
    data->acceleration = motor_data::motorAcceleration[index] * (360.0 / encoder_range[index]);
    data->status = motorStatusMap[index];
    data->error_code = elmo_slave_input[g_motor_info[index].pdo_id]->error_code;
    data->status_word = elmo_slave_input[g_motor_info[index].pdo_id]->status_word;
  }
  else if (g_motor_info[index].driver_type == YD) {
    if (yd_slave_input[g_motor_info[index].pdo_id] == nullptr) {
        printf("Error:[motorGetSingleData]:yd_slave_input[%d] is nullptr!\n", g_motor_info[index].pdo_id);
        mtx_io.unlock();
        return;
    }
    data->position = (yd_slave_input[g_motor_info[index].pdo_id]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
    data->velocity = yd_slave_input[g_motor_info[index].pdo_id]->velocity_actual_value * (360.0 / encoder_range[index]);
    data->torque = yd_slave_input[g_motor_info[index].pdo_id]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
    data->acceleration = motor_data::motorAcceleration[index] * (360.0 / encoder_range[index]);
    data->status = motorStatusMap[index];
    data->error_code = yd_slave_input[g_motor_info[index].pdo_id]->error_code;
    data->status_word = yd_slave_input[g_motor_info[index].pdo_id]->status_word;
    data->torque_demand_trans = yd_slave_input[g_motor_info[index].pdo_id]->torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
  }
  mtx_io.unlock();
}

void setMotorPositionOffset(double *offset, uint16_t len)
{
  mtx_io.lock();
  for (uint16_t i = 0; i < len; i++)
  {
    pos_offset[i] = offset[i];
  }
  mtx_io.unlock();
}

void motorSetPosition(const uint16_t *ids, uint32_t num, MotorParam_t *params)
{
  uint16_t index = 0;
  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if (motorStatusMap[index])
      continue;
    if(g_motor_info[index].driver_type == ELMO)
    {
      elmo_slave_output[g_motor_info[index].pdo_id]->target_position = (params[i].position + pos_offset[index]) * (encoder_range[index] / 360.0) ;
      elmo_slave_output[g_motor_info[index].pdo_id]->position_offset = params[i].positionOffset * (encoder_range[index] / 360.0);
      elmo_slave_output[g_motor_info[index].pdo_id]->velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      elmo_slave_output[g_motor_info[index].pdo_id]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
      elmo_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(elmo_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    } 
    else if(g_motor_info[index].driver_type == YD)
    {
      yd_slave_output[g_motor_info[index].pdo_id]->target_position = (params[i].position+ pos_offset[index]) * (encoder_range[index] / 360.0);
      yd_slave_output[g_motor_info[index].pdo_id]->velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      yd_slave_output[g_motor_info[index].pdo_id]->torque_offset = (params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000.0)/ 1.414;
      yd_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
      yd_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(yd_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    }
  }
  mtx_io.unlock();
}

void motorSetVelocity(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  uint16_t index = 0;
  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if (motorStatusMap[index])
      continue;
    if(g_motor_info[index].driver_type == ELMO)
    {
      elmo_slave_output[g_motor_info[index].pdo_id]->target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
      elmo_slave_output[g_motor_info[index].pdo_id]->velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      elmo_slave_output[g_motor_info[index].pdo_id]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSV;
      elmo_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(elmo_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    }
    else if(g_motor_info[index].driver_type == YD)
    {
      yd_slave_output[g_motor_info[index].pdo_id]->target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
      yd_slave_output[g_motor_info[index].pdo_id]->velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      yd_slave_output[g_motor_info[index].pdo_id]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSV;
      yd_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(yd_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    }
  }
  mtx_io.unlock();
}

void motorSetTorque(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  uint16_t index = 0;
  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    
    index = ids[i] - 1;
    if (motorStatusMap[index])
      continue;
    if(g_motor_info[index].driver_type == ELMO)
    {
      elmo_slave_output[g_motor_info[index].pdo_id]->target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CST;
      elmo_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(elmo_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    }
    else if(g_motor_info[index].driver_type == YD)
    {
      yd_slave_output[g_motor_info[index].pdo_id]->target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[g_motor_info[index].pdo_id]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CST;
      yd_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(yd_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
    }
  }
  mtx_io.unlock();

}

uint8_t motorStatus(const uint16_t id)
{
  return motorStatusMap[id - 1];
}

std::vector<uint16_t> motorStatusAll(EcMasterType driver_type,uint32_t motor_num)
{
  std::vector<uint16_t> motorErrorMap;
  motorErrorMap.resize(motor_num); 
  return motorErrorMap;
}

bool workpdMotorCspSin(const uint16_t *ids, uint32_t motor_num, MotorCspSinParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[NUM_SLAVE_MAX] = {0};
  static double time[NUM_SLAVE_MAX] = {0};
  static int8_t runMode = 0;
  static double countTimeS = 0;
  // printf("workpdMotorCspSin runMode = %d\n", runMode);
  if(runMode == 0)
  {
    motorGetData(ids, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      printf("start_pos[%d] = %f\n", i, start_pos[i]);
    }
    runMode = 1;
    // printf("workpdMotorCspSin runMode = %d\n", runMode);
  }
  if(runMode == 1)
  {
    // printf("countTimeS = %f, time_total = %f\n", countTimeS, param->time_total);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      setMotorParam[i].position = param->A * sin(2 * M_PI / param->T * time[i]) + start_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
      time[i] = fmod(time[i] + param->dt, param->T);
    }
    motorSetPosition(ids, motor_num, setMotorParam);
    countTimeS += param->dt;
    if(countTimeS >= param->time_total)
    {
      runMode = -1;
      // printf("workpdMotorCspSin runMode = %d\n", runMode);
    }
  }
  if(runMode == -1)
  {
    memset(start_pos, 0, sizeof(start_pos));
    memset(time, 0, sizeof(time));
    countTimeS = 0;
    runMode = 0;
    // printf("workpdMotorCspSin runMode = %d\n", runMode);
    return true;
  }

  // printf("runMode = %d, countTimeS = %f, time_total = %f\n", runMode, countTimeS, param->time_total);
  return false;
}

/**
 * @brief 多电机动作序列执行函数
 * @param ids 电机ID数组
 * @param motor_num 电机数量
 * @param param 动作序列参数
 * @return true表示动作完成，false表示继续执行
 */
bool workpdMotorMultiAction(const uint16_t *ids, uint32_t motor_num, MotorMultiActionParam_t *param)
{
  static MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  static MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[NUM_SLAVE_MAX] = {0};
  static double target_pos[NUM_SLAVE_MAX] = {0};
  static double current_pos[NUM_SLAVE_MAX] = {0};
  static double frame_start_pos[NUM_SLAVE_MAX] = {0};  // 每帧开始位置
  static double initial_pos[NUM_SLAVE_MAX] = {0};  // 保存最开始的初始位置
  static double elapsed_time = 0;
  static double total_elapsed = 0;
  static double total_program_time = 0; // 程序总运行时间（从程序启动时计算）
  static uint32_t current_action = 0;
  static int8_t runMode = 0;  // 0:初始化, 1:等待同步点, 2:运动到目标动作
  static bool first_run = true;
  static double sync_wait_time = 0;     // 同步等待时间
  static uint32_t current_cycle = 0;    // 当前同步周期数
  static uint32_t actions_per_cycle = 0; // 每个同步周期的动作数
  
  if(runMode == 0)  // 初始化阶段 - 立即获取电机位置并激活
  {
    if(param->motor_actions.empty() || param->motor_actions[0].empty()) {
      printf("Error: No actions provided\n");
      return true;
    }

    // 读取所有电机当前位置
    motorGetData(ids, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      current_pos[i] = start_pos[i];
      initial_pos[i] = start_pos[i];  // 保存最开始的初始位置
      printf("Motor %d initial_pos = %f\n", ids[i], initial_pos[i]);
    }
    
    // 立即发布当前位置作为目标位置（激活电机）
    for (uint32_t i = 0; i < motor_num; i++)
    {
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, motor_num, setMotorParam);
    
    elapsed_time = 0;
    total_elapsed = 0;
    total_program_time = 0;
    current_action = 0;
    runMode = 1;  // 进入等待同步点阶段
    first_run = true;
    sync_wait_time = 0;
    current_cycle = 0;
    // 计算每个同步周期的动作数（基于15秒同步周期）
    double sync_cycle_time = 15.0;  // 15秒同步周期
    // 动态计算动作帧数：根据实际动作序列长度
    actions_per_cycle = param->action_sequence_length;  // 使用实际的动作序列长度
    
    printf("Multi motor action initialized with %d motors, motion duration: %.1f seconds, total time: %.1f seconds\n", 
           motor_num, param->motion_duration, param->time_total);
    printf("🔄 同步周期信息: 每个周期 %d 帧 (%.1f秒动作 + %.1f秒保持)\n", 
           actions_per_cycle, actions_per_cycle * param->motion_duration, sync_cycle_time - actions_per_cycle * param->motion_duration);
    printf("电机已激活，等待同步点到达...\n");
    
    // 发送同步信号，通知手臂腿部已准备就绪
    FILE* sync_file = fopen("/tmp/leg_ready_signal", "w");
    if (sync_file != NULL) {
        fprintf(sync_file, "leg_ready");
        fclose(sync_file);
        printf("✅ 腿部准备完成，已发送同步信号到 /tmp/leg_ready_signal\n");
    } else {
        printf("⚠️ 无法创建同步信号文件\n");
    }
  }
  
  if(runMode == 1)  // 等待同步点阶段 - 持续发布当前位置
  {
    total_program_time += param->dt;  // 只计算程序总时间，不计算执行时间
    sync_wait_time += param->dt;
    
    // 检查手臂失能信号
    FILE* arm_disable_file = fopen("/tmp/arm_disable_signal", "r");
    if (arm_disable_file != NULL) {
      printf("❌ 检测到手臂失能信号，腿部磨线程序提前退出以避免冲突\n");
      fclose(arm_disable_file);
      // 重置所有静态变量
      memset(start_pos, 0, sizeof(start_pos));
      memset(target_pos, 0, sizeof(target_pos));
      memset(current_pos, 0, sizeof(current_pos));
      memset(frame_start_pos, 0, sizeof(frame_start_pos));
      memset(initial_pos, 0, sizeof(initial_pos));
      elapsed_time = 0;
      total_elapsed = 0;
      total_program_time = 0;
      current_action = 0;
      runMode = 0;
      first_run = true;
      sync_wait_time = 0;
      current_cycle = 0;
      actions_per_cycle = 0;
      return true;  // 任务完成（提前退出）
    }
    
    // 持续发布当前位置作为目标位置
    for (uint32_t i = 0; i < motor_num; i++)
    {
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, motor_num, setMotorParam);
    
    // 检查是否到达同步点（15秒周期）
    time_t current_time = time(NULL);
    int current_second = (int)current_time % 15;
    
    // 每5秒打印一次等待状态
    static double last_print_time = 0;
    if (sync_wait_time - last_print_time >= 5.0) {
      printf("⏳ 等待同步点... 当前秒数: %d, 等待时间: %.1f秒\n", current_second, sync_wait_time);
      last_print_time = sync_wait_time;
    }
    
    if (current_second == 0 && sync_wait_time > 1.0) {  // 确保已经等待了至少1秒
      printf("🎯 同步点到达！开始执行动作序列\n");
      if (first_run) {
        printf("⏰ 等待时间: %.1f秒，现在开始计算%.1f秒执行时间\n", sync_wait_time, param->time_total);
        first_run = false;
      } else {
        printf("⏰ 等待时间: %.1f秒，继续执行剩余时间\n", sync_wait_time);
      }
      runMode = 2;  // 进入动作执行阶段
      elapsed_time = 0;
      // 不重置total_elapsed，保持累计执行时间
      current_action = 0;  // 重置动作计数
      
      // 设置第一个动作为目标（初始位置 + 偏移值）
      for (uint32_t i = 0; i < motor_num; i++)
      {
        frame_start_pos[i] = current_pos[i];  // 当前位置作为起始位置
        target_pos[i] = initial_pos[i] + param->motor_actions[i][0];  // 初始位置 + 第一个动作的偏移值
      }
    }
    
    return false;  // 在等待同步点期间，不执行动作逻辑
  }

  if(runMode == 2)  // 运动到当前目标动作
  {
    // 在执行过程中也检查手臂失能信号
    FILE* arm_disable_file = fopen("/tmp/arm_disable_signal", "r");
    if (arm_disable_file != NULL) {
      printf("❌ 执行过程中检测到手臂失能信号，腿部磨线程序立即停止以避免冲突\n");
      fclose(arm_disable_file);
      // 重置所有静态变量
      memset(start_pos, 0, sizeof(start_pos));
      memset(target_pos, 0, sizeof(target_pos));
      memset(current_pos, 0, sizeof(current_pos));
      memset(frame_start_pos, 0, sizeof(frame_start_pos));
      memset(initial_pos, 0, sizeof(initial_pos));
      elapsed_time = 0;
      total_elapsed = 0;
      total_program_time = 0;
      current_action = 0;
      runMode = 0;
      first_run = true;
      sync_wait_time = 0;
      current_cycle = 0;
      actions_per_cycle = 0;
      return true;  // 任务完成（提前退出）
    }
    
    elapsed_time += param->dt;
    total_elapsed += param->dt;      // 执行时间（从开始动作时计算）
    total_program_time += param->dt; // 程序总时间（从程序启动时计算）
    double progress = elapsed_time / param->motion_duration;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动 - 从帧起始位置插值到目标位置
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      
      // 从帧起始位置插值到目标位置，确保连续平滑过渡
      double interp_pos = frame_start_pos[i] + (target_pos[i] - frame_start_pos[i]) * smooth_progress;
      
      // 更新当前位置为插值后的位置
      current_pos[i] = interp_pos;
      
      setMotorParam[i].position = interp_pos;
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, motor_num, setMotorParam);

    if(progress >= 1.0)
    {
      printf("动作 %d 完成 (当前动作/总动作: %d/%zu)\n", current_action, current_action, param->motor_actions[0].size());
      current_action++;
      
      // 检查是否完成一个同步周期
      if(actions_per_cycle > 0 && current_action % actions_per_cycle == 0)
      {
        current_cycle++;
        printf("✅ 第 %d 轮同步周期完成！(动作 %d/%zu)\n", current_cycle, current_action, param->motor_actions[0].size());
      }
      
      if(current_action >= param->motor_actions[0].size())
      {
        // 一轮动作完成，检查是否还有足够时间进行下一个周期
        double one_cycle_time = param->motor_actions[0].size() * param->motion_duration;
        
        if(total_elapsed >= param->time_total)
        {
          // 已达到目标执行时间，结束程序
          printf("✅ 已达到目标执行时间 %.1f秒，程序结束\n", param->time_total);
          return true;  // 任务完成
        }
        
        if(total_elapsed + one_cycle_time > param->time_total)
        {
          // 剩余时间不足以完成下一个完整周期，提前结束
          printf("❌ 一个周期完成。剩余时间不足以完成下一个周期，停止\n");
          return true;  // 任务完成
        }
        
        // 有足够时间，开始下一个周期
        current_action = 0;
        printf("🔄 一个周期完成，等待下一个15秒同步点开始新周期\n");
        runMode = 1;  // 回到等待同步点阶段
        sync_wait_time = 0;  // 重置同步等待时间
        return false;  // 继续运行，等待同步点
      }
      
      // 准备下一个动作 - 基于初始位置计算绝对目标位置
      for (uint32_t i = 0; i < motor_num; i++)
      {
        frame_start_pos[i] = current_pos[i];  // 当前位置作为新的起始位置
        // 目标位置始终基于最开始的初始位置计算，而不是基于当前位置
        target_pos[i] = initial_pos[i] + param->motor_actions[i][current_action];  // 初始位置 + 下一个动作的偏移值
        
        // printf("电机 %d 动作 %d: 从 %.2f 运动到 %.2f (初始位置 %.2f + 偏移 %.2f)\n", 
        //        ids[i], current_action, frame_start_pos[i], target_pos[i], initial_pos[i], param->motor_actions[i][current_action]);
      }
      elapsed_time = 0;
      runMode = 2;  // 继续运动到下一个动作
    }
  }
  
  // 检查总运行时间是否超时
  if(total_elapsed >= param->time_total)
  {
    printf("Multi motor action completed after %.1f seconds\n", total_elapsed);
    // 重置所有静态变量
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    memset(frame_start_pos, 0, sizeof(frame_start_pos));
    memset(initial_pos, 0, sizeof(initial_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    total_program_time = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    sync_wait_time = 0;
    current_cycle = 0;
    actions_per_cycle = 0;
    return true;  // 任务完成
  }

  return false;  // 持续运行
}

/**
 * @brief 设置单个电机的位置参数（线程安全）
 * @param id 电机ID（从1开始）
 * @param driver 电机驱动类型（EcMasterType）
 * @param param 目标参数指针
 */
void motorSetSinglePosition(uint16_t id, EcMasterType driver, MotorParam_t* param)
{
  uint16_t index = id - 1;
  mtx_io.lock();
  if (motorStatusMap[index]) {
    mtx_io.unlock();
    return;
  }
  if (g_motor_info[index].driver_type == ELMO) {
    elmo_slave_output[g_motor_info[index].pdo_id]->target_position = (param->position + pos_offset[index]) * (encoder_range[index] / 360.0);
    elmo_slave_output[g_motor_info[index].pdo_id]->position_offset = param->positionOffset * (encoder_range[index] / 360.0);
    elmo_slave_output[g_motor_info[index].pdo_id]->velocit_offset = param->velocityOffset * (encoder_range[index] / 360.0);
    elmo_slave_output[g_motor_info[index].pdo_id]->torque_offset = param->torqueOffset * (1000.0 / rated_current[index]) * 1000;
    elmo_slave_output[g_motor_info[index].pdo_id]->max_torque = param->maxTorque * (1000.0 / rated_current[index]) * 1000;
    elmo_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
    elmo_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(elmo_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
  } else if (g_motor_info[index].driver_type == YD) {
    yd_slave_output[g_motor_info[index].pdo_id]->target_position = (param->position + pos_offset[index]) * (encoder_range[index] / 360.0);
    yd_slave_output[g_motor_info[index].pdo_id]->velocity_offset = param->velocityOffset * (encoder_range[index] / 360.0);
    yd_slave_output[g_motor_info[index].pdo_id]->torque_offset = (param->torqueOffset * (1000.0 / rated_current[index]) * 1000.0) / 1.414;
    yd_slave_output[g_motor_info[index].pdo_id]->mode_of_opration = MODE_CSP;
    yd_slave_output[g_motor_info[index].pdo_id]->control_word = sw2cw(yd_slave_input[g_motor_info[index].pdo_id]->status_word & 0x6f);
  }
  mtx_io.unlock();
}

/**
 * @brief 单个电机相对当前位置转动deltaPos角度（周期调用，余弦插值）
 * @param id 电机ID（从1开始）
 * @param driver 电机驱动类型数组（EcMasterType）
 * @param deltaPos 相对转动角度（单位：度，正负均可）
 * @param totalTime 运动总时长（单位：秒）
 * @param dt 每步插值时间（单位：秒），建议与主循环周期一致
 * @return true 到达目标，false 未到达
 * @note 该函数内部使用静态变量保存插值状态，适合单电机单目标周期调用。
 *       若需多电机或多目标并发插值，请自行管理插值状态。
 */
bool motorMoveRelativeInterpStep(uint16_t id, double deltaPos, double totalTime, double dt)
{
  static double startPos = 0;
  static double targetPos = 0;
  static double elapsed = 0;
  static bool running = false;
  static double lastDelta = 0;

  if (totalTime <= 0 || dt <= 0) {
    printf("error: motorMoveRelativeInterpStep totalTime or dt is 0\n");
    return true;
  }

  MotorParam_t data = {0};
  motorGetSingleData(id, &data);

  // 新目标或首次调用，重置插值状态
  if (!running || lastDelta != deltaPos) {
    startPos = data.position;
    targetPos = startPos + deltaPos;
    elapsed = 0;
    running = true;
    lastDelta = deltaPos;
  }
  
  if (elapsed >= totalTime) {
    MotorParam_t param = {0};
    param.position = targetPos;
    param.velocity = 0;
    param.torque = 0;
    param.maxTorque = MAX_TORQUE;
    motorSetPosition(&id, 1, &param);
    running = false;
    elapsed = 0;
    return true;
  } else {
    double ratio = elapsed / totalTime;
    if (ratio > 1.0) ratio = 1.0;
    double s = (1 - cos(M_PI * ratio)) / 2;
    double interpPos = startPos + (targetPos - startPos) * s;
    MotorParam_t param = {0};
    param.position = interpPos;
    param.velocity = 0;
    param.torque = 0;
    param.maxTorque = MAX_TORQUE;
    motorSetPosition(&id, 1, &param);
    elapsed += dt;
    return false;
  }
}

void setEncoderRange(uint16_t id, uint32_t encoderRange)
{
  encoder_range[id - 1] = encoderRange;
}

uint32_t getEncoderRange(uint16_t id)
{
  return encoder_range[id - 1];
}

void setSlave2Joint(uint8_t SlaveId, uint8_t JointId)
{
  slave2joint[SlaveId-1] = JointId-1;
}

uint8_t getSlave2Joint(uint8_t SlaveId)
{
  return (slave2joint[SlaveId-1]+1);
}
