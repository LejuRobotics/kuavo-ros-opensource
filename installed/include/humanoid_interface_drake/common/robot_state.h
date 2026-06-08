#pragma once
#include <map>

#include <iostream>
#include "Eigen/Core"
#include <vector>
typedef enum
{
  phase_loss,
  pre_walk,
  walk,
  take_off,
  flight,
  touch_down,
  squat,
  None
} Phase_t; // TODO: remove old phase

typedef enum
{
  P_stand, // 站立
  P_walk,  // 行走
  P_jump,  // 跳跃
  P_squat, // 下蹲
  P_ERROR, // 错误
  P_None
} mainPhase_t;

static std::map<mainPhase_t, std::string> phase_name_map = {
    {P_stand, "P_stand"},
    {P_walk, "P_walk"},
    {P_jump, "P_jump"},
    {P_squat, "P_squat"},
    {P_ERROR, "P_ERROR"},
    {P_None, "P_None"},
};

typedef enum
{
  // walk
  walk_pre,
  walking,
  walk_stop,

  // jump
  jump_pre,
  jump_take_off,
  jump_flight,
  jump_touch_down,
  jump_to_ready,

  // squat
  squat_normal,
  squat_quick, // 快速蹲起

  sub_phase_none,

} subPhase_t;

typedef enum
{
  R_Contact = -1,
  Double_contact,
  L_Contact,
} Contact_states;

typedef enum
{
  PositionControl,
  VelocityControl
} controlMode_t;

typedef struct
{
  Eigen::Vector3d contact;
  Contact_states walk_contact;
} ContactPoint_t;

typedef struct
{
  Eigen::Vector3d l_contact;
  Eigen::Vector3d r_contact;
} JumpContactPoint_t;

enum EndEffectorType
{
  none,
  jodell,
  qiangnao,
  lejuclaw,
};
enum MotorDriveType
{
  EC_MASTER,
  DYNAMIXEL,
  REALMAN,
  RUIWO
};

typedef struct
{
  EndEffectorType type;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd torque;
  void resize(uint8_t num_joints)
  {
    position.resize(num_joints);
    velocity.resize(num_joints);
    torque.resize(num_joints);
  }
} EndEffectorData;

typedef struct // TODO: remove old EstimateState_t
{
  Eigen::VectorXd q;               // 广义位置:包含姿态四元数(0-3)、躯干位置(4-6)、关节角度(7-19)
  Eigen::VectorXd v;               // 广义速度:包含角速度(0-2)、躯干速度(3-5)、关节速度(6-18)
  Eigen::VectorXd vd;              // 加速度：由广义速度算出的相应加速度
  Eigen::Vector3d torsoR;          // 躯干角度
  Eigen::Vector3d torsoRd;         // 躯干角速度
  Eigen::Vector3d torsoRdd;        // 躯干角加速度
  Eigen::Vector3d r;               // 质心位置
  Eigen::Vector3d rd;              // 质心线速度
  Eigen::Vector3d rdd;             // 质心线加速度
  Eigen::Vector3d r_est;           // 和 r 一致?
  Eigen::Vector3d rd_est;          // 和 rd 一致?
  Eigen::Matrix<double, 6, 1> lf;  // 左脚的姿态和位置向量(rpy,xyz)
  Eigen::Matrix<double, 6, 1> rf;  // 右脚的姿态和位置向量
  Eigen::Matrix<double, 6, 1> lh;  // 左手的姿态位置向量
  Eigen::Matrix<double, 6, 1> rh;  // 右手的姿态位置向量
  Eigen::Matrix<double, 6, 1> lfv; // 左脚的速度向量
  Eigen::Matrix<double, 6, 1> rfv; // 右脚的速度向量
  Eigen::Matrix<double, 6, 1> lfvd;
  Eigen::Matrix<double, 6, 1> rfvd;
  Eigen::Matrix<double, 6, 1> lhv;
  Eigen::Matrix<double, 6, 1> rhv;
  Eigen::Vector3d contact;           // 不再使用
  Eigen::Matrix<double, 6, 1> cm;    // 质心空间动量
  Eigen::Matrix<double, 6, 1> lf_sm; // 左脚空间动量
  Eigen::Matrix<double, 6, 1> rf_sm; // 右脚空间动量
  Eigen::Matrix<double, 6, 1> contact_sm;
  Eigen::Vector3d p_touch;
  Eigen::Vector2d hiproll_offset;
  Eigen::Vector2d hipyaw_damp;
  Eigen::Vector2d hiproll_damp;
  Eigen::Vector2d hippitch_damp;
  Eigen::Vector2d anklepitch_damp;
  ContactPoint_t walk_touch;   // 行走的接地状态和接地点
  Phase_t phase;               // 机器人状态
  Phase_t prev_phase;          // 存储机器人上一个状态
  Contact_states walk_contact; // 机器人脚接地状态，左脚、右脚、双脚
  Eigen::Vector3d SpM_des;
  double phase_time;         // 状态计时时间
  Eigen::VectorXd tau;       // 关节力矩
  Eigen::VectorXd tau_max;   // 最大力矩/电流
  Eigen::VectorXd tau_ratio; // 力矩比例系数
  // Eigen::Vector3d position_cmd;
  // Eigen::Vector3d vel_cmd;
} EstimateState_t; // TODO: remove old EstimateState_t

typedef struct : public EstimateState_t
{
  mainPhase_t phase;      // 机器人状态
  mainPhase_t prev_phase; // 存储机器人上一个状态
  subPhase_t sub_phase;   // 储存机器人子状态
  Eigen::Vector3d position_cmd;
  Eigen::Vector3d vel_cmd;
  // JumpContactPoint_t jump_touch; // 跳跃左右脚接触点
  Eigen::VectorXd arm_q;  // 手臂关节位置
  Eigen::VectorXd arm_v;  // 手臂关节速度
  Eigen::VectorXd arm_vd; // 手臂关节加速度
  // JumpContactPoint_t jump_touch; // 跳跃左右脚接触点
  Eigen::Matrix<double, 9, 1> contact_force;
  std::vector<int> control_modes;
  std::vector<EndEffectorData> end_effectors;

} RobotState_t;
