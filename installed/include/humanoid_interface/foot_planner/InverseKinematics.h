/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <humanoid_interface/common/Types.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2
{
namespace humanoid
{
class InverseKinematics
{
public:
  void setParam(std::shared_ptr<PinocchioInterface> pinocchioInterface, std::shared_ptr<CentroidalModelInfo> info)
  {
    pinocchio_interface_ = pinocchioInterface;
    info_ = info;
    qr_.setThreshold(0.01);
  }

  vector6_t computeTranslationIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz);
  vector6_t computeRotationIK(vector_t init_q, int leg, matrix3_t des_foot_R_des);
  vector6_t computeRotationIK(vector_t init_q, int leg, vector3_t des_foot_eular_zyx);

  vector6_t computeIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz, matrix3_t des_foot_R_des);
  vector6_t computeIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz, vector3_t des_foot_eular_zyx);

  vector6_t computeDIK(vector_t q, int leg, vector3_t foot_linear_vel, vector3_t foot_angular_vel);

  feet_array_t<vector3_t> computeFootPos(const vector_t& state);

  static int leg2index(int leg)
  {
    int index = 0;
    if (leg == 0)
      index = 0;
    else if (leg == 1)
      index = 6;
    return index;
  }
  static int leg2point(int leg)
  {
    int point = 0;
    if (leg == 0)
      point = 2;
    else if (leg == 1)
      point = 7;//右脚后跟
    return point;
  }

private:
  std::shared_ptr<PinocchioInterface> pinocchio_interface_;
  std::shared_ptr<CentroidalModelInfo> info_;

  Eigen::ColPivHouseholderQR<matrix_t> qr_;
};

}  // namespace humanoid
}  // namespace ocs2
