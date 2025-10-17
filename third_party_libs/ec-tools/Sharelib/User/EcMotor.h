#ifndef EC_MOTOR_H_
#define EC_MOTOR_H_

#include <vector>

#define NUM_SLAVE_MAX 30
#define NUM_SLAVE_ELMO_MAX 14
#define NUM_SLAVE_YD_MAX (NUM_SLAVE_MAX - NUM_SLAVE_ELMO_MAX)

#pragma pack(1)

typedef struct
{
  int32_t position_actual_value;
  int16_t torque_actual_value;
  uint16_t status_word;
  int16_t mode_of_opration_display;
  int32_t position_demand_raw;
  int32_t velocity_demand_raw;
  int32_t velocity_actual_value;
  int16_t torque_demand_raw;
  uint16_t error_code;
} ELMO_SlaveRead_t;

typedef struct
{
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  uint16_t max_torque;
  uint16_t control_word;
  int16_t mode_of_opration;
  int32_t position_offset;
  int32_t velocit_offset;
  int16_t torque_offset;
} ELMO_SlaveWrite_t;


/* yd */
typedef struct
{
  uint16_t status_word;
  int32_t position_actual_value;
  int32_t velocity_actual_value;
  int16_t torque_actual_value;
  int32_t velocity_demand_raw;
  int16_t torque_demand_raw;
  int16_t error_code;
  uint16_t igbt_temperature;
  int8_t mode_of_opration_display;
} YD_SlaveRead_t;

typedef struct
{
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  int32_t velocity_offset;
  int16_t torque_offset;
  int8_t mode_of_opration;//选模式，例：CSP、CSV等。。。
} YD_SlaveWrite_t;

enum EcMasterType
{
  ELMO = 0,
  YD = 1,
  LEJU = 2
};

typedef struct
{
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double maxTorque = 0.0;
  double positionOffset = 0.0;
  double velocityOffset = 0.0;
  double torqueOffset = 0.0;
  double acceleration = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  uint8_t status = 0;
  uint16_t status_word = 0;
  uint16_t error_code = 0;
  double torque_demand_trans = 0.0;
} MotorParam_t;

#pragma pack()

extern enum EcMasterType driver_type[30];
extern uint32_t num_motor_slave;
extern uint32_t num_slave;
extern uint32_t num_elmo_slave;
extern uint32_t num_yd_slave;
extern YD_SlaveRead_t *yd_slave_input[NUM_SLAVE_YD_MAX];
extern YD_SlaveWrite_t *yd_slave_output[NUM_SLAVE_YD_MAX];
extern ELMO_SlaveRead_t *elmo_slave_input[NUM_SLAVE_ELMO_MAX];
extern ELMO_SlaveWrite_t *elmo_slave_output[NUM_SLAVE_ELMO_MAX];

namespace motor_data {
  extern uint16_t ids[NUM_SLAVE_MAX];
  extern double motorAcceleration[NUM_SLAVE_MAX];
}

extern void motorGetData(const uint16_t *ids, uint32_t num, MotorParam_t *data);
extern void motorGetSingleData(uint16_t id, MotorParam_t *data);

extern void motorSetPosition(const uint16_t *ids, uint32_t num, MotorParam_t *params);
extern void motorSetVelocity(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetTorque(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);


bool motorEnable(const uint16_t id);
extern uint32_t getNumSlave(void);
void setEcEncoderRange(uint32_t *encoder_range_set, uint16_t num);


extern void setMotorPositionOffset(double *offset, uint16_t len);

#pragma pack(1)
typedef struct
{
  double A = 0;
  double T = 0;
  double time_total = 0;
  double dt = 0;
} MotorCspSinParam_t;

// 多电机动作序列参数结构体
typedef struct
{
  std::vector<std::vector<double>> motor_actions;  // 每个电机的动作序列
  double motion_duration;                         // 每个动作的持续时间
  double time_total;                              // 总运行时间
  double dt;                                      // 时间步长
  uint32_t action_sequence_length;                // 动作序列长度
} MotorMultiActionParam_t;

typedef struct
{
  uint8_t slave_id;   // EC从站ID,即真实的物理id 从1开始
  uint8_t logical_id; // 逻辑id,即urdf的id 从0开始
  uint8_t pdo_id;     // 用于PDO读写的索引 从0开始
  uint8_t physical_id;// 除去分支器的motor连接顺序 从0开始
  EcMasterType driver_type; // 驱动类型
} MotorInfo_t;

#pragma pack()

extern MotorInfo_t g_motor_info[NUM_SLAVE_MAX];

bool workpdMotorCspSin(const uint16_t *ids, uint32_t motor_num, MotorCspSinParam_t *param);

// 多电机动作序列执行函数
bool workpdMotorMultiAction(const uint16_t *ids, uint32_t motor_num, MotorMultiActionParam_t *param);

void motorSetSinglePosition(uint16_t id, EcMasterType driver, MotorParam_t* param);

bool motorMoveRelativeInterpStep(uint16_t id, double targetPos, double totalTime, double dt);


void setEncoderRange(uint16_t id, uint32_t encoderRange);
uint32_t getEncoderRange(uint16_t id);

uint8_t slaveTojoint(uint8_t SlaveId);
void setSlave2Joint(uint8_t SlaveId, uint8_t JointId);
uint8_t getSlave2Joint(uint8_t SlaveId);

#endif // EC_MOTOR_H_

