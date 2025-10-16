#ifndef EC_MOTOR_H_
#define EC_MOTOR_H_

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

extern void motorGetData(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *data);
extern void motorGetSingleData(uint16_t id, const EcMasterType* driver, MotorParam_t *data);

extern void motorSetPosition(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetVelocity(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);
extern void motorSetTorque(const uint16_t *ids, const EcMasterType* driver, uint32_t num, MotorParam_t *params);


bool motorEnable(const uint16_t id, EcMasterType* driver);
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

// 左脚电机位置控制参数结构体
typedef struct _MotorPositionControlParam_t
{
  double positions[6];     // 6个关节的目标位置
  double increment;        // 每次递增的角度
  double hold_time;        // 保持时间（秒）
  double move_time;        // 运动时间（秒）
  double total_time;       // 总执行时间（秒）
  double dt;              // 时间步长
} MotorPositionControlParam_t;

// 左脚电机动作序列控制参数结构体
typedef struct _MotorActionSequenceParam_t
{
  double actions[32][6];   // 最多32个动作，每个动作6个关节位置
  uint32_t action_count;   // 动作数量
  double motion_duration;  // 每个动作的执行时间（秒）
  double total_time;       // 总运行时间（秒）
  double dt;              // 时间步长
} MotorActionSequenceParam_t;

// 双脚电机动作序列控制参数结构体
typedef struct _MotorDualLegActionSequenceParam_t
{
  double actions[32][7];   // 最多32个动作，每个动作7个值（腰部1个 + 左腿6个）
  uint32_t action_count;   // 动作数量
  double motion_duration;  // 每个动作的执行时间（秒）
  double total_time;       // 总运行时间（秒）
  double dt;              // 时间步长
  bool symmetric_motion;   // 是否对称运动（右脚与左脚相对）
} MotorDualLegActionSequenceParam_t;

// 单电机动作帧控制参数结构体
typedef struct _MotorActionFrameParam_t
{
  double actions[32];      // 最多32个动作，每个动作是相对于初始位置的偏移值
  uint32_t action_count;   // 动作数量
  double motion_duration;  // 每个动作的执行时间（秒）
  double total_time;       // 总运行时间（秒）
  double dt;              // 时间步长
} MotorActionFrameParam_t;

// 通用多电机动作帧控制参数结构体
typedef struct _MotorMultiActionFrameParam_t
{
  uint16_t motor_ids[13];  // 最多13个电机ID
  double actions[13][32];  // 最多13个电机，每个电机最多32个动作，每个动作是相对于初始位置的偏移值
  uint32_t motor_count;    // 电机数量
  uint32_t action_count;   // 动作数量
  double motion_duration;  // 每个动作的执行时间（秒）
  double total_time;       // 总运行时间（秒）
  double dt;              // 时间步长
} MotorMultiActionFrameParam_t;
#pragma pack()

bool workpdMotorCspSin(const uint16_t *ids, uint32_t motor_num, MotorCspSinParam_t *param);
bool workpdMotorPositionControl(const uint16_t *ids, uint32_t motor_num, MotorPositionControlParam_t *param);
bool workpdMotorActionSequence(const uint16_t *ids, uint32_t motor_num, MotorActionSequenceParam_t *param);
bool workpdMotorDualLegActionSequence(const uint16_t *left_ids, const uint16_t *right_ids, uint32_t motor_num, MotorDualLegActionSequenceParam_t *param);
bool workpdMotorActionFrame(const uint16_t *ids, uint32_t motor_num, MotorActionFrameParam_t *param);
bool workpdMotorMultiActionFrame(MotorMultiActionFrameParam_t *param);

void motorSetSinglePosition(uint16_t id, EcMasterType driver, MotorParam_t* param);

bool motorMoveRelativeInterpStep(uint16_t id, const EcMasterType* driver, double targetPos, double totalTime, double dt);


void setEncoderRange(uint16_t id, uint32_t encoderRange);
uint32_t getEncoderRange(uint16_t id);

#endif // EC_MOTOR_H_

