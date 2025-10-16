#include "EcDemoApp.h"
#include "EcMotor.h"
#include "util.h"
#include <mutex>
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

static uint32_t rated_current[NUM_SLAVE_MAX] = {0};//额定电流

static uint32_t encoder_range[NUM_SLAVE_MAX] = {
    BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
    BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
    BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
    BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
    BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9};

uint16_t motor_data::ids[NUM_SLAVE_MAX] = {
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

YD_SlaveRead_t *yd_slave_input[NUM_SLAVE_YD_MAX];
YD_SlaveWrite_t *yd_slave_output[NUM_SLAVE_YD_MAX];
uint32_t num_yd_slave = 0;

ELMO_SlaveRead_t *elmo_slave_input[NUM_SLAVE_ELMO_MAX];
ELMO_SlaveWrite_t *elmo_slave_output[NUM_SLAVE_ELMO_MAX];
uint32_t num_elmo_slave = 0;

static std::mutex mtx_io;
uint32_t num_slave = 0;

double motor_data::motorAcceleration[NUM_SLAVE_MAX] = {0};

int restartCounter = 0;
bool restartMotorFlag = false;
uint8_t motorStatusMap[NUM_SLAVE_MAX] = {0};
int motorErrorCodeMap[NUM_SLAVE_MAX] = {0};
clock_t last_restart_time[NUM_SLAVE_MAX] = {0};

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
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave %d actual position encoder %s, ", i + 1, tempfloat));
    free(tempfloat);
    tempfloat = floatToChar(value);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave %d actual position %s, ", i + 1, tempfloat));
    free(tempfloat);

    dwRes = emCoeSdoUpload(0, i, INDEX_MOTOR_RATED_CURRENT, 0, buf, 4, &outdata_len, 100, 0);
    if (EC_E_NOERROR != dwRes)
    {
      EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed ot emCoeSdoUpload, Slave %d, Err (0x%lx)\n", i, dwRes));
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


bool motorEnable(const uint16_t id, EcMasterType* driver)
{
  if (id < 1)
  {
    return false;
  }
  uint16_t sw;
  uint16_t index = id - 1;
  static uint8_t elmo_Number = 0, yd_Number = 0;
  // printf("start elmo_Number: %d, yd_Number: %d, num_slave: %d\n", elmo_Number, yd_Number, num_slave);
  mtx_io.lock();
  if(driver[index] == ELMO)
  {
    sw = elmo_slave_input[elmo_Number]->status_word & 0x6f;
  }
  else if(driver[index] == YD)
  {
    sw = yd_slave_input[yd_Number]->status_word & 0x6f;
  }
  // static uint16_t old_status_word = 0;
  // static uint8_t old_yd_Number = 0;
  // if(yd_slave_input[yd_Number]->status_word != old_status_word || old_yd_Number != yd_Number)
  // {
  //   old_yd_Number = yd_Number;
  //   old_status_word = yd_slave_input[yd_Number]->status_word;
  //   printf("yd_slave_input[%d]->status_word: 0X%x\n", yd_Number, yd_slave_input[yd_Number]->status_word);
  // }
  // static uint16_t old_id = 0;
  // static EcMasterType old_driver = YD;
  // static uint16_t old_sw = 0;
  // if(id != old_id || driver[index] != old_driver || sw != old_sw)
  // {
  //   printf("motorEnable, slave_id: %d, driver: %d, sw: 0X%x\n", id, driver[index], sw);
  //   old_id = id;
  //   old_driver = driver[index];
  //   old_sw = sw;
  // }
  mtx_io.unlock();
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

    // printf("Motor %d is already enabled, elmo_Number: %d, yd_Number: %d, num_slave: %d\n", id, elmo_Number, yd_Number, num_slave);
    if(elmo_Number + yd_Number >= num_slave)
    {
      // printf("elmo_Number: %d, yd_Number: %d, num_slave: %d\n", elmo_Number, yd_Number, num_slave);
      elmo_Number = 0;
      yd_Number = 0;
    }
    return true;
  }
  mtx_io.lock();
  if(driver[index] == ELMO)
  {
    elmo_slave_output[elmo_Number]->target_position = elmo_slave_input[elmo_Number]->position_actual_value;
    elmo_slave_output[elmo_Number]->position_offset = 0;
    elmo_slave_output[elmo_Number]->velocit_offset = 0;
    elmo_slave_output[elmo_Number]->torque_offset = 0;
    elmo_slave_output[elmo_Number]->max_torque = 1000;
    elmo_slave_output[elmo_Number]->mode_of_opration = MODE_CSP;
    elmo_slave_output[elmo_Number]->control_word = sw2cw(sw);
  }
  else if(driver[index] == YD)
  { 
    yd_slave_output[yd_Number]->target_position = yd_slave_input[yd_Number]->position_actual_value;
    yd_slave_output[yd_Number]->velocity_offset = 0;
    yd_slave_output[yd_Number]->torque_offset = 0;
    yd_slave_output[yd_Number]->mode_of_opration = MODE_CSP;
    yd_slave_output[yd_Number]->control_word = sw2cw(sw);
  }
  
  mtx_io.unlock();
  // printf("end elmo_Number: %d, yd_Number: %d, num_slave: %d\n", elmo_Number, yd_Number, num_slave);
  return false;
}

void motorGetData(const uint16_t *ids, const EcMasterType* driver, uint32_t num,
                  MotorParam_t *data)
{
  uint16_t index = 0;
  uint8_t elmo_Number = 0, yd_Number = 0;

  static uint8_t get_change_mode_old = 0;
  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
   if(driver[i] == ELMO)
    {  
      data[i].position = (elmo_slave_input[elmo_Number]->position_actual_value * (360.0 / encoder_range[index])  - pos_offset[index]) ;
      data[i].velocity = elmo_slave_input[elmo_Number]->velocity_actual_value * (360.0 / encoder_range[index]);
      data[i].torque = elmo_slave_input[elmo_Number]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 ;
      data[i].acceleration = motor_data::motorAcceleration[elmo_Number] * (360.0 / encoder_range[index]);
      data[i].status = motorStatusMap[i];

      // 驱动器控制环路底层数据
      data[i].error_code = elmo_slave_input[elmo_Number]->error_code;
      data[i].status_word = elmo_slave_input[elmo_Number]->status_word;
      elmo_Number++;
    }
    else if(driver[i] == YD)
    { 
      data[i].position = (yd_slave_input[yd_Number]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
      data[i].velocity = yd_slave_input[yd_Number]->velocity_actual_value * (360.0 / encoder_range[index]) ;
      data[i].torque = yd_slave_input[yd_Number]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
      data[i].acceleration = motor_data::motorAcceleration[yd_Number] * (360.0 / encoder_range[index]);
      data[i].status = motorStatusMap[i];
      
      // 驱动器控制环路底层数据
      data[i].error_code = yd_slave_input[yd_Number]->error_code;
      data[i].status_word = yd_slave_input[yd_Number]->status_word;
      data[i].torque_demand_trans = yd_slave_input[yd_Number]->torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
      yd_Number++;
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
void motorGetSingleData(uint16_t id, const EcMasterType* driver, MotorParam_t *data)
{
  uint8_t elmo_Number = 0, yd_Number = 0;
  uint16_t index = id - 1;
  // 遍历所有电机，找到id对应的elmo/yd编号
  for (uint16_t i = 0; i < index; i++) {
    if (driver[i] == ELMO) {
        elmo_Number++;
    } else if (driver[i] == YD) {
        yd_Number++;
    }
  }
  // printf("motorGetSingleData id: %d\n", id); 
  mtx_io.lock();
  if (driver[index] == ELMO) {
    if (elmo_slave_input[elmo_Number] == nullptr) {
        printf("Error:[motorGetSingleData]:elmo_slave_input[%d] is nullptr!\n", elmo_Number);
        mtx_io.unlock();
        return;
    }
    data->position = (elmo_slave_input[elmo_Number]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
    data->velocity = elmo_slave_input[elmo_Number]->velocity_actual_value * (360.0 / encoder_range[index]);
    data->torque = elmo_slave_input[elmo_Number]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0;
    data->acceleration = motor_data::motorAcceleration[elmo_Number] * (360.0 / encoder_range[index]);
    data->status = motorStatusMap[index];
    data->error_code = elmo_slave_input[elmo_Number]->error_code;
    data->status_word = elmo_slave_input[elmo_Number]->status_word;
  } else if (driver[index] == YD) {
    if (yd_slave_input[yd_Number] == nullptr) {
        printf("Error:[motorGetSingleData]:yd_slave_input[%d] is nullptr!\n", yd_Number);
        mtx_io.unlock();
        return;
    }
    data->position = (yd_slave_input[yd_Number]->position_actual_value * (360.0 / encoder_range[index]) - pos_offset[index]);
    data->velocity = yd_slave_input[yd_Number]->velocity_actual_value * (360.0 / encoder_range[index]);
    data->torque = yd_slave_input[yd_Number]->torque_actual_value * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
    data->acceleration = motor_data::motorAcceleration[yd_Number] * (360.0 / encoder_range[index]);
    data->status = motorStatusMap[index];
    data->error_code = yd_slave_input[yd_Number]->error_code;
    data->status_word = yd_slave_input[yd_Number]->status_word;
    data->torque_demand_trans = yd_slave_input[yd_Number]->torque_demand_raw * (rated_current[index] / 1000.0) / 1000.0 * 1.414;
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

void motorSetPosition(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params)
{
  uint16_t index = 0;
  mtx_io.lock();
  for (uint32_t i = 0; i < num; i++)
  {
    index = ids[i] - 1;
    if (motorStatusMap[index])
      continue;
    if(driver[index] == ELMO)
      {
        elmo_slave_output[index]->target_position = (params[i].position + pos_offset[index]) * (encoder_range[index] / 360.0) ;
        elmo_slave_output[index]->position_offset = params[i].positionOffset * (encoder_range[index] / 360.0)  ;
        elmo_slave_output[index]->velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0)  ;
        elmo_slave_output[index]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000  ;
        elmo_slave_output[index]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
        elmo_slave_output[index]->mode_of_opration = MODE_CSP;
        elmo_slave_output[index]->control_word = sw2cw(elmo_slave_input[index]->status_word & 0x6f);
      } 
      else if(driver[index] == YD)
      {
        yd_slave_output[index]->target_position = (params[i].position+ pos_offset[index]) * (encoder_range[index] / 360.0);
        yd_slave_output[index]->velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0)  ;
        yd_slave_output[index]->torque_offset = (params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000.0 )  / 1.414;
        yd_slave_output[index]->mode_of_opration = MODE_CSP;
        yd_slave_output[index]->control_word = sw2cw(yd_slave_input[index]->status_word & 0x6f);
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
   if(driver[index] == ELMO)
    {
      elmo_slave_output[index]->target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
      elmo_slave_output[index]->velocit_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      elmo_slave_output[index]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[index]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[index]->mode_of_opration = MODE_CSV;
      elmo_slave_output[index]->control_word = sw2cw(elmo_slave_input[index]->status_word & 0x6f);
    }
    else if(driver[index] == YD)
    {
      yd_slave_output[index]->target_velocity = params[i].velocity * (encoder_range[index] / 360.0);
      yd_slave_output[index]->velocity_offset = params[i].velocityOffset * (encoder_range[index] / 360.0);
      yd_slave_output[index]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[index]->mode_of_opration = MODE_CSV;
      yd_slave_output[index]->control_word = sw2cw(yd_slave_input[index]->status_word & 0x6f);
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
    if(driver[index] == ELMO)
    {
      elmo_slave_output[index]->target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[index]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[index]->max_torque = params[i].maxTorque * (1000.0 / rated_current[index]) * 1000;
      elmo_slave_output[index]->mode_of_opration = MODE_CST;
      elmo_slave_output[index]->control_word = sw2cw(elmo_slave_input[index]->status_word & 0x6f);
    }
    else if(driver[index] == YD)
    {
      yd_slave_output[index]->target_torque = params[i].torque * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[index]->torque_offset = params[i].torqueOffset * (1000.0 / rated_current[index]) * 1000 / 1.414;
      yd_slave_output[index]->mode_of_opration = MODE_CST;
    
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
  static uint32_t last_motor_num = 0;  // 记录上次的电机数量
  // 如果电机数量发生变化，重置状态
  if (last_motor_num != motor_num) {
    memset(start_pos, 0, sizeof(start_pos));
    memset(time, 0, sizeof(time));
    countTimeS = 0;
    runMode = 0;
    last_motor_num = motor_num;
  }
  
  // printf("workpdMotorCspSin runMode = %d, motor_num = %d\n", runMode, motor_num);
  if(runMode == 0)
  {
    motorGetData(ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      printf("Motor %d start_pos[%d] = %f\n", ids[i], i, start_pos[i]);
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
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);
    countTimeS += param->dt;
    if(countTimeS >= param->time_total)
    {
      runMode = -1;
      // printf("workpdMotorCspSin runMode = %d\n", runMode);
    }
  }
  if(runMode == -1)
  {
    printf("CSP正弦运动任务完成，总时间: %.2f秒\n", param->time_total);
    memset(start_pos, 0, sizeof(start_pos));
    memset(time, 0, sizeof(time));
    countTimeS = 0;
    runMode = 0;
    last_motor_num = 0;  // 重置电机数量记录
    // printf("workpdMotorCspSin runMode = %d\n", runMode);
    return true;
  }

  // printf("runMode = %d, countTimeS = %f, time_total = %f\n", runMode, countTimeS, param->time_total);
  return false;
}

/**
 * @brief 左脚电机位置控制函数 - 根据给定位置列表运动，然后往复递增
 * @param ids 电机ID数组
 * @param motor_num 电机数量
 * @param param 位置控制参数
 * @return true 任务完成，false 任务进行中
 */
bool workpdMotorPositionControl(const uint16_t *ids, uint32_t motor_num, MotorPositionControlParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[NUM_SLAVE_MAX] = {0};
  static double target_pos[NUM_SLAVE_MAX] = {0};
  static double current_pos[NUM_SLAVE_MAX] = {0};
  static double elapsed_time = 0;
  static double hold_elapsed = 0;
  static double total_elapsed = 0;  // 总执行时间
  static int8_t runMode = 0;  // 0:初始化, 1:运动到目标位置, 2:保持, 3:递增并运动, 4:返回初始位置
  static bool direction = true;  // true:递增, false:递减
  static bool first_run = true;

  if(runMode == 0)  // 初始化阶段
  {
    motorGetData(ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      target_pos[i] = param->positions[i];
      current_pos[i] = start_pos[i];
      printf("Motor %d: start_pos = %f, target_pos = %f\n", ids[i], start_pos[i], target_pos[i]);
    }
    elapsed_time = 0;
    hold_elapsed = 0;
    total_elapsed = 0;
    runMode = 1;
    direction = true;
    first_run = true;
    printf("Position control initialized, total time: %.1f seconds\n", param->total_time);
  }

  if(runMode == 1)  // 运动到目标位置
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->move_time;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      current_pos[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * smooth_progress;
      
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);

    if(progress >= 1.0)
    {
      runMode = 2;
      hold_elapsed = 0;
      printf("Reached target positions, starting hold phase\n");
    }
  }

  if(runMode == 2)  // 保持阶段
  {
    hold_elapsed += param->dt;
    total_elapsed += param->dt;
    
    // 保持当前位置
    for (uint32_t i = 0; i < motor_num; i++)
    {
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);

    if(hold_elapsed >= param->hold_time)
    {
      runMode = 3;
      elapsed_time = 0;
      printf("Hold phase completed, starting increment phase\n");
    }
  }

  if(runMode == 3)  // 递增并运动
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->move_time;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < motor_num; i++)
    {
      double new_target = target_pos[i];
      if(direction)
      {
        new_target += param->increment;
      }
      else
      {
        new_target -= param->increment;
      }

      // 使用余弦插值平滑运动
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      current_pos[i] = target_pos[i] + (new_target - target_pos[i]) * smooth_progress;
      
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);

    if(progress >= 1.0)
    {
      // 更新目标位置
      for (uint32_t i = 0; i < motor_num; i++)
      {
        if(direction)
        {
          target_pos[i] += param->increment;
        }
        else
        {
          target_pos[i] -= param->increment;
        }
      }
      
      // 切换方向
      direction = !direction;
      runMode = 2;  // 回到保持阶段
      hold_elapsed = 0;
      printf("Increment completed, direction: %s\n", direction ? "increment" : "decrement");
    }
  }

  // 检查总执行时间是否超时
  if(total_elapsed >= param->total_time)
  {
    printf("Position control completed after %.1f seconds\n", total_elapsed);
    // 重置所有静态变量
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    elapsed_time = 0;
    hold_elapsed = 0;
    total_elapsed = 0;
    runMode = 0;
    direction = true;
    first_run = true;
    return true;  // 任务完成
  }

  return false;  // 持续运行
}

/**
 * @brief 左脚电机动作序列控制函数 - 按动作序列运动
 * @param ids 电机ID数组
 * @param motor_num 电机数量
 * @param param 动作序列控制参数
 * @return true 任务完成，false 任务进行中
 */
bool workpdMotorActionSequence(const uint16_t *ids, uint32_t motor_num, MotorActionSequenceParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[NUM_SLAVE_MAX] = {0};
  static double target_pos[NUM_SLAVE_MAX] = {0};
  static double current_pos[NUM_SLAVE_MAX] = {0};
  static double elapsed_time = 0;
  static double total_elapsed = 0;  // 总运行时间
  static uint32_t current_action = 0;
  static int8_t runMode = 0;  // 0:初始化, 1:运动到目标动作, 2:等待下一个动作
  static bool first_run = true;

  if(runMode == 0)  // 初始化阶段
  {
    if(param->action_count == 0) {
      printf("Error: No actions provided\n");
      return true;
    }

    motorGetData(ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      current_pos[i] = start_pos[i];
      printf("Motor %d: start_pos = %f\n", ids[i], start_pos[i]);
    }
    
    // 设置第一个动作为目标（零点位置 + 相对偏移）
    for (uint32_t i = 0; i < motor_num; i++)
    {
      target_pos[i] = pos_offset[ids[i]-1] + param->actions[0][i];  // ids[i]-1 因为电机ID从1开始，数组从0开始
    }
    
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 1;
    first_run = true;
    printf("Action sequence initialized with %d actions, motion duration: %.1f seconds, total time: %.1f seconds\n", 
           param->action_count, param->motion_duration, param->total_time);
  }

  if(runMode == 1)  // 运动到当前目标动作
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->motion_duration;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      current_pos[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * smooth_progress;
      
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);

    if(progress >= 1.0)
    {
      printf("动作 %d 完成\n", current_action);
      current_action++;
      
      if(current_action >= param->action_count)
      {
        // 一轮动作完成，回到第一个动作继续循环
        current_action = 0;
        printf("一个周期完成，开始新周期\n");
      }
      
      // 准备下一个动作（零点位置 + 相对偏移）
      for (uint32_t i = 0; i < motor_num; i++)
      {
        start_pos[i] = current_pos[i];  // 当前位置作为新的起始位置
        target_pos[i] = pos_offset[ids[i]-1] + param->actions[current_action][i];  // 零点位置 + 相对偏移
      }
      elapsed_time = 0;
      runMode = 1;  // 继续运动到下一个动作
    }
  }

  // 检查总运行时间是否超时
  if(total_elapsed >= param->total_time)
  {
    printf("Action sequence completed after %.1f seconds\n", total_elapsed);
    // 重置所有静态变量
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    return true;  // 任务完成
  }

  return false;  // 持续运行
}

/**
 * @brief 双脚电机动作序列控制 - 支持对称运动
 * @param left_ids 左脚电机ID数组
 * @param right_ids 右脚电机ID数组
 * @param motor_num 每条腿的电机数量
 * @param param 控制参数
 * @return true表示任务完成，false表示继续运行
 */
bool workpdMotorDualLegActionSequence(const uint16_t *left_ids, const uint16_t *right_ids, uint32_t motor_num, MotorDualLegActionSequenceParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double left_start_pos[NUM_SLAVE_MAX] = {0};
  static double left_target_pos[NUM_SLAVE_MAX] = {0};
  static double left_current_pos[NUM_SLAVE_MAX] = {0};
  static double right_start_pos[NUM_SLAVE_MAX] = {0};
  static double right_target_pos[NUM_SLAVE_MAX] = {0};
  static double right_current_pos[NUM_SLAVE_MAX] = {0};
  static double waist_start_pos = 0;
  static double waist_target_pos = 0;
  static double waist_current_pos = 0;
  static double elapsed_time = 0;
  static double total_elapsed = 0;  // 总运行时间
  static uint32_t current_action = 0;
  static int8_t runMode = 0;  // 0:初始化, 1:运动到目标动作, 2:等待下一个动作
  static bool first_run = true;

  if(runMode == 0)  // 初始化阶段
  {
    if(param->action_count == 0) {
      printf("Error: No actions provided\n");
      return true;
    }

    // 读取左脚当前位置
    motorGetData(left_ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      left_current_pos[i] = getMotorData[i].position;
      printf("左腿电机 %d: 当前位置 = %f, 零点值 = %f, 差值 = %f\n", 
             left_ids[i], left_current_pos[i], pos_offset[left_ids[i]-1], 
             left_current_pos[i] - pos_offset[left_ids[i]-1]);
    }

    // 读取右脚当前位置
    motorGetData(right_ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      right_current_pos[i] = getMotorData[i].position;
      printf("右腿电机 %d: 当前位置 = %f, 零点值 = %f, 差值 = %f\n", 
             right_ids[i], right_current_pos[i], pos_offset[right_ids[i]-1], 
             right_current_pos[i] - pos_offset[right_ids[i]-1]);
    }

    // 读取腰部当前位置
    motorGetSingleData(1, driver_type, getMotorData);  // 腰部电机ID为1
    waist_current_pos = getMotorData[0].position;
    printf("腰部电机 1: 当前位置 = %f, 零点值 = %f, 差值 = %f\n", 
           waist_current_pos, pos_offset[0], waist_current_pos - pos_offset[0]);

    // 设置腰部目标位置
    waist_start_pos = waist_current_pos;
    waist_target_pos = pos_offset[0] + param->actions[0][0];  // 腰部：零点位置 + 第1个值（相对偏移）

    // 设置第一个动作为目标（当前位置 + 相对偏移）
    for (uint32_t i = 0; i < motor_num; i++)
    {
      left_start_pos[i] = left_current_pos[i];
      left_target_pos[i] = pos_offset[left_ids[i]-1] + param->actions[0][i+1];  // 左腿：零点位置 + 相对偏移（使用第2-7个值）
      
      if(param->symmetric_motion)
      {
        // 右腿运动：前三个关节对称运动，后三个关节同向运动
        right_start_pos[i] = right_current_pos[i];
        if(i < 3) {
          // 前三个关节（ID 8,9,10对应左腿ID 5,6,7）：同向运动
          right_target_pos[i] = pos_offset[right_ids[i]-1] + param->actions[0][motor_num-i];  // 右腿：零点位置 + 反转的相对偏移
        } else {
          // 后三个关节（ID 11,12,13对应左腿ID 2,3,4）：对称运动
          right_target_pos[i] = pos_offset[right_ids[i]-1] - param->actions[0][motor_num-i];  // 右腿：零点位置 - 反转的相对偏移
        }
      }
      else
      {
        // 右腿同向运动：当前位置 + 相对偏移，右脚关节从下往上排序，需要反转动作序列
        right_start_pos[i] = right_current_pos[i];
        right_target_pos[i] = pos_offset[right_ids[i]-1] + param->actions[0][motor_num-i];  // 右腿：零点位置 + 反转的相对偏移（使用第2-7个值）
      }
      
    }
    
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 1;
    first_run = true;
    printf("双脚动作序列初始化完成：%d个动作，每个动作%.1f秒，总时间%.1f秒，对称运动：%s\n", 
           param->action_count, param->motion_duration, param->total_time, param->symmetric_motion ? "是" : "否");
    
  }

  if(runMode == 1)  // 运动到当前目标动作
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->motion_duration;
    if(progress > 1.0) progress = 1.0;

    // 腰部插值计算
    double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
    waist_current_pos = waist_start_pos + (waist_target_pos - waist_start_pos) * smooth_progress;

    // 左脚插值计算
    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动
      left_current_pos[i] = left_start_pos[i] + (left_target_pos[i] - left_start_pos[i]) * smooth_progress;
    }

    // 右脚插值计算
    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动
      right_current_pos[i] = right_start_pos[i] + (right_target_pos[i] - right_start_pos[i]) * smooth_progress;
    }

    // 设置所有13个电机参数（腰部1个 + 左腿6个 + 右腿6个）
    uint16_t all_motor_ids[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};  // 所有电机ID
    MotorParam_t all_motor_params[13] = {0};
    
    // 腰部电机（ID 1）：使用插值后的位置
    all_motor_params[0].position = waist_current_pos - pos_offset[0];  // 减去零点值，传递相对位置
    all_motor_params[0].velocity = 0;
    all_motor_params[0].torque = 0;
    all_motor_params[0].maxTorque = MAX_TORQUE;
    
    // 左腿电机（ID 2-7）：传递相对位置
    for (uint32_t i = 0; i < motor_num; i++)
    {
      all_motor_params[left_ids[i]-1].position = left_current_pos[i] - pos_offset[left_ids[i]-1];  // 减去零点值，传递相对位置
      all_motor_params[left_ids[i]-1].velocity = 0;
      all_motor_params[left_ids[i]-1].torque = 0;
      all_motor_params[left_ids[i]-1].maxTorque = MAX_TORQUE;
    }
    
    // 右腿电机（ID 8-13）：传递相对位置
    for (uint32_t i = 0; i < motor_num; i++)
    {
      all_motor_params[right_ids[i]-1].position = right_current_pos[i] - pos_offset[right_ids[i]-1];  // 减去零点值，传递相对位置
      all_motor_params[right_ids[i]-1].velocity = 0;
      all_motor_params[right_ids[i]-1].torque = 0;
      all_motor_params[right_ids[i]-1].maxTorque = MAX_TORQUE;
    }
    
    // 一次性发布所有13个电机的位置
    motorSetPosition(all_motor_ids, driver_type, 13, all_motor_params);
    

    if(progress >= 1.0)
    {
      printf("动作 %d 完成\n", current_action);
      current_action++;
      
      if(current_action >= param->action_count)
      {
        // 一轮动作完成，检查是否还有足够时间进行下一个周期
        double one_cycle_time = param->action_count * param->motion_duration;
        if(total_elapsed + one_cycle_time > param->total_time)
        {
          // 剩余时间不足以完成下一个完整周期，提前结束
          printf("一个周期完成。剩余时间不足以完成下一个周期，停止\n");
          return true;  // 任务完成
        }
        
        // 有足够时间，开始下一个周期
        current_action = 0;
        printf("一个周期完成，开始新周期\n");
      }
      
      // 准备下一个动作（零点位置 + 相对偏移）
      // 设置腰部目标位置
      waist_start_pos = waist_current_pos;
      waist_target_pos = pos_offset[0] + param->actions[current_action][0];  // 腰部：零点位置 + 第1个值（相对偏移）

      for (uint32_t i = 0; i < motor_num; i++)
      {
        left_start_pos[i] = left_current_pos[i];  // 当前位置作为新的起始位置
        left_target_pos[i] = pos_offset[left_ids[i]-1] + param->actions[current_action][i+1];  // 左腿：零点位置 + 相对偏移（使用第2-7个值）
        
        if(param->symmetric_motion)
        {
          // 右腿运动：前三个关节对称运动，后三个关节同向运动
          right_start_pos[i] = right_current_pos[i];
          if(i < 3) {
            // 前三个关节（ID 8,9,10对应左腿ID 5,6,7）：同向运动
            right_target_pos[i] = pos_offset[right_ids[i]-1] + param->actions[current_action][motor_num-i];  // 右腿：零点位置 + 反转的相对偏移
          } else {
            // 后三个关节（ID 11,12,13对应左腿ID 2,3,4）：对称运动
            right_target_pos[i] = pos_offset[right_ids[i]-1] - param->actions[current_action][motor_num-i];  // 右腿：零点位置 - 反转的相对偏移
          }
        }
        else
        {
          // 右腿同向运动：当前位置 + 相对偏移，右脚关节从下往上排序，需要反转动作序列
          right_start_pos[i] = right_current_pos[i];
          right_target_pos[i] = pos_offset[right_ids[i]-1] + param->actions[current_action][motor_num-i];  // 右腿：零点位置 + 反转的相对偏移（使用第2-7个值）
        }
        
      }
      elapsed_time = 0;
      runMode = 1;  // 继续运动到下一个动作
    }
  }

  // 检查总运行时间是否超时
  if(total_elapsed >= param->total_time)
  {
    printf("双脚动作序列在%.1f秒后完成\n", total_elapsed);
    // 重置所有静态变量
    memset(left_start_pos, 0, sizeof(left_start_pos));
    memset(left_target_pos, 0, sizeof(left_target_pos));
    memset(left_current_pos, 0, sizeof(left_current_pos));
    memset(right_start_pos, 0, sizeof(right_start_pos));
    memset(right_target_pos, 0, sizeof(right_target_pos));
    memset(right_current_pos, 0, sizeof(right_current_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
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
  if (driver == ELMO) {
    elmo_slave_output[index]->target_position = (param->position + pos_offset[index]) * (encoder_range[index] / 360.0);
    elmo_slave_output[index]->position_offset = param->positionOffset * (encoder_range[index] / 360.0);
    elmo_slave_output[index]->velocit_offset = param->velocityOffset * (encoder_range[index] / 360.0);
    elmo_slave_output[index]->torque_offset = param->torqueOffset * (1000.0 / rated_current[index]) * 1000;
    elmo_slave_output[index]->max_torque = param->maxTorque * (1000.0 / rated_current[index]) * 1000;
    elmo_slave_output[index]->mode_of_opration = MODE_CSP;
    elmo_slave_output[index]->control_word = sw2cw(elmo_slave_input[index]->status_word & 0x6f);
  } else if (driver == YD) {
    yd_slave_output[index]->target_position = (param->position + pos_offset[index]) * (encoder_range[index] / 360.0);
    yd_slave_output[index]->velocity_offset = param->velocityOffset * (encoder_range[index] / 360.0);
    yd_slave_output[index]->torque_offset = (param->torqueOffset * (1000.0 / rated_current[index]) * 1000.0) / 1.414;
    yd_slave_output[index]->mode_of_opration = MODE_CSP;
    yd_slave_output[index]->control_word = sw2cw(yd_slave_input[index]->status_word & 0x6f);
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
bool motorMoveRelativeInterpStep(uint16_t id, const EcMasterType* driver, double deltaPos, double totalTime, double dt)
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
  motorGetSingleData(id, driver, &data);

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
    motorSetPosition(&id, driver, 1, &param);
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
    motorSetPosition(&id, driver, 1, &param);
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

/**
 * @brief 单电机动作帧控制函数 - 基于初始位置的偏移值进行动作序列控制
 * @param ids 电机ID数组
 * @param motor_num 电机数量
 * @param param 动作帧控制参数
 * @return true 任务完成，false 任务进行中
 */
bool workpdMotorActionFrame(const uint16_t *ids, uint32_t motor_num, MotorActionFrameParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[NUM_SLAVE_MAX] = {0};
  static double target_pos[NUM_SLAVE_MAX] = {0};
  static double current_pos[NUM_SLAVE_MAX] = {0};
  static double initial_pos[NUM_SLAVE_MAX] = {0};  // 保存最开始的初始位置
  static double elapsed_time = 0;
  static double total_elapsed = 0;  // 总运行时间
  static uint32_t current_action = 0;
  static int8_t runMode = 0;  // 0:初始化, 1:运动到目标动作, 2:等待下一个动作
  static bool first_run = true;
  static uint32_t last_motor_num = 0;  // 记录上次的电机数量

  // 如果电机数量发生变化，重置状态
  if (last_motor_num != motor_num) {
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    memset(initial_pos, 0, sizeof(initial_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    last_motor_num = motor_num;
  }

  if(runMode == 0)  // 初始化阶段
  {
    if(param->action_count == 0) {
      printf("Error: No actions provided\n");
      return true;
    }

    motorGetData(ids, driver_type, motor_num, getMotorData);
    for (uint32_t i = 0; i < motor_num; i++)
    {
      start_pos[i] = getMotorData[i].position;
      current_pos[i] = start_pos[i];
      initial_pos[i] = start_pos[i];  // 保存最开始的初始位置
      printf("Motor %d: initial_pos[%d] = %f\n", ids[i], i, initial_pos[i]);
    }
    
    // 设置第一个动作为目标（初始位置 + 偏移值）
    for (uint32_t i = 0; i < motor_num; i++)
    {
      target_pos[i] = initial_pos[i] + param->actions[0];  // 初始位置 + 第一个动作的偏移值
    }
    
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 1;
    first_run = true;
    printf("Action frame initialized with %d actions, motion duration: %.1f seconds, total time: %.1f seconds\n", 
           param->action_count, param->motion_duration, param->total_time);
  }

  if(runMode == 1)  // 运动到当前目标动作
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->motion_duration;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < motor_num; i++)
    {
      // 使用余弦插值平滑运动
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      current_pos[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * smooth_progress;
      
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(ids, driver_type, motor_num, setMotorParam);

    if(progress >= 1.0)
    {
      printf("动作 %d 完成\n", current_action);
      current_action++;
      
      if(current_action >= param->action_count)
      {
        // 一轮动作完成，检查是否还有足够时间进行下一个周期
        double one_cycle_time = param->action_count * param->motion_duration;
        if(total_elapsed + one_cycle_time > param->total_time)
        {
          // 剩余时间不足以完成下一个完整周期，提前结束
          printf("一个周期完成。剩余时间不足以完成下一个周期，停止\n");
          return true;  // 任务完成
        }
        
        // 有足够时间，开始下一个周期
        current_action = 0;
        printf("一个周期完成，开始新周期\n");
      }
      
      // 准备下一个动作（始终基于初始位置 + 偏移值）
      for (uint32_t i = 0; i < motor_num; i++)
      {
        start_pos[i] = current_pos[i];  // 当前位置作为新的起始位置
        // 目标位置始终基于最开始的初始位置计算，而不是基于当前位置
        target_pos[i] = initial_pos[i] + param->actions[current_action];  // 初始位置 + 下一个动作的偏移值
      }
      elapsed_time = 0;
      runMode = 1;  // 继续运动到下一个动作
    }
  }

  // 检查总运行时间是否超时
  if(total_elapsed >= param->total_time)
  {
    printf("Action frame completed after %.1f seconds\n", total_elapsed);
    // 重置所有静态变量
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    memset(initial_pos, 0, sizeof(initial_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    last_motor_num = 0;  // 重置电机数量记录
    return true;  // 任务完成
  }

  return false;  // 持续运行
}

/**
 * @brief 通用多电机动作帧控制函数 - 基于初始位置的偏移值进行动作序列控制
 * @param param 多电机动作帧控制参数
 * @return true 任务完成，false 任务进行中
 */
bool workpdMotorMultiActionFrame(MotorMultiActionFrameParam_t *param)
{
  MotorParam_t setMotorParam[NUM_SLAVE_MAX] = {0};
  MotorParam_t getMotorData[NUM_SLAVE_MAX] = {0};
  static double start_pos[13] = {0};    // 13个电机的起始位置
  static double target_pos[13] = {0};   // 13个电机的目标位置
  static double current_pos[13] = {0};  // 13个电机的当前位置
  static double initial_pos[13] = {0};  // 13个电机的最初位置
  static double elapsed_time = 0;
  static double total_elapsed = 0;      // 总运行时间
  static uint32_t current_action = 0;
  static int8_t runMode = 0;            // 0:初始化, 1:运动到目标动作
  static bool first_run = true;
  static uint32_t last_motor_count = 0; // 记录上次的电机数量

  // 如果电机数量发生变化，重置状态
  if (last_motor_count != param->motor_count) {
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    memset(initial_pos, 0, sizeof(initial_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    last_motor_count = param->motor_count;
  }

  if(runMode == 0)  // 初始化阶段
  {
    if(param->action_count == 0) {
      printf("Error: No actions provided\n");
      return true;
    }

    motorGetData(param->motor_ids, driver_type, param->motor_count, getMotorData);
    for (uint32_t i = 0; i < param->motor_count; i++)
    {
      start_pos[i] = getMotorData[i].position;
      current_pos[i] = start_pos[i];
      initial_pos[i] = start_pos[i];  // 保存最开始的初始位置
      printf("Motor %d: initial_pos = %f\n", param->motor_ids[i], initial_pos[i]);
    }
    
    // 设置第一个动作为目标（初始位置 + 偏移值）
    for (uint32_t i = 0; i < param->motor_count; i++)
    {
      target_pos[i] = initial_pos[i] + param->actions[i][0];  // 初始位置 + 第一个动作的偏移值
    }
    
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 1;
    first_run = true;
    printf("Multi motor action frame initialized with %d motors, %d actions, motion duration: %.1f seconds, total time: %.1f seconds\n", 
           param->motor_count, param->action_count, param->motion_duration, param->total_time);
  }

  if(runMode == 1)  // 运动到当前目标动作
  {
    elapsed_time += param->dt;
    total_elapsed += param->dt;
    double progress = elapsed_time / param->motion_duration;
    if(progress > 1.0) progress = 1.0;

    for (uint32_t i = 0; i < param->motor_count; i++)
    {
      // 使用余弦插值平滑运动
      double smooth_progress = 0.5 * (1.0 - cos(M_PI * progress));
      current_pos[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * smooth_progress;
      
      setMotorParam[i].position = current_pos[i];
      setMotorParam[i].velocity = 0;
      setMotorParam[i].torque = 0;
      setMotorParam[i].maxTorque = MAX_TORQUE;
    }
    motorSetPosition(param->motor_ids, driver_type, param->motor_count, setMotorParam);

    if(progress >= 1.0)
    {
      printf("动作 %d 完成\n", current_action);
      current_action++;
      
      if(current_action >= param->action_count)
      {
        // 一轮动作完成，检查是否还有足够时间进行下一个周期
        double one_cycle_time = param->action_count * param->motion_duration;
        if(total_elapsed + one_cycle_time > param->total_time)
        {
          // 剩余时间不足以完成下一个完整周期，提前结束
          printf("一个周期完成。剩余时间不足以完成下一个周期，停止\n");
          return true;  // 任务完成
        }
        
        // 有足够时间，开始下一个周期
        current_action = 0;
        printf("一个周期完成，开始新周期\n");
      }
      
      // 准备下一个动作（始终基于初始位置 + 偏移值）
      for (uint32_t i = 0; i < param->motor_count; i++)
      {
        start_pos[i] = current_pos[i];  // 当前位置作为新的起始位置
        // 目标位置始终基于最开始的初始位置计算，而不是基于当前位置
        target_pos[i] = initial_pos[i] + param->actions[i][current_action];  // 初始位置 + 下一个动作的偏移值
      }
      elapsed_time = 0;
      runMode = 1;  // 继续运动到下一个动作
    }
  }

  // 检查总运行时间是否超时
  if(total_elapsed >= param->total_time)
  {
    printf("Multi motor action frame completed after %.1f seconds\n", total_elapsed);
    // 重置所有静态变量
    memset(start_pos, 0, sizeof(start_pos));
    memset(target_pos, 0, sizeof(target_pos));
    memset(current_pos, 0, sizeof(current_pos));
    memset(initial_pos, 0, sizeof(initial_pos));
    elapsed_time = 0;
    total_elapsed = 0;
    current_action = 0;
    runMode = 0;
    first_run = true;
    last_motor_count = 0;  // 重置电机数量记录
    return true;  // 任务完成
  }

  return false;  // 持续运行
}
