#pragma once

#include <iostream>
#include "Eigen/Core"
#include "kuavo_common/common/robot_state.h"
#include <deque>
typedef struct
{
  double time;
  Eigen::VectorXd joint_q;
  Eigen::VectorXd joint_v;
  Eigen::VectorXd joint_vd;
  Eigen::VectorXd joint_current;
  Eigen::VectorXd joint_velocity_demand;
  Eigen::VectorXd joint_torque_demand;
  Eigen::VectorXd joint_igbt_temperature;
  Eigen::VectorXd joint_ntc_temperature;
  Eigen::Vector3d gyro;
  Eigen::Vector3d acc;
  Eigen::Vector3d free_acc;
  Eigen::Vector4d quat;
  Eigen::Vector3d gyro_W;
  Eigen::Vector3d acc_W;
  Eigen::Vector3d free_acc_W;
  Eigen::Vector4d quat_W;
  std::vector<EndEffectorData> end_effectors_data;
  void resizeJoint(size_t n)
  {
    joint_q.resize(n);
    joint_v.resize(n);
    joint_vd.resize(n);
    joint_current.resize(n);
    joint_velocity_demand.resize(n);
    joint_torque_demand.resize(n);
    joint_igbt_temperature.resize(n);
    joint_ntc_temperature.resize(n);
  }
} SensorData_t;

typedef struct
{
  Eigen::Vector3d omegaBody;
  Eigen::Vector3d omegaWorld;
  Eigen::Vector3d aBody;
  Eigen::Vector3d aWorld;
  Eigen::Vector3d Angu_xyz;
  Eigen::Vector3d VelBody;
  Eigen::Vector3d VelWorld;
  Eigen::Vector3d PosBody;
  Eigen::Vector3d PosWorld;
  Eigen::Vector4d quat;
  Eigen::Matrix3d rBody;
} _SensorOrientationData;
