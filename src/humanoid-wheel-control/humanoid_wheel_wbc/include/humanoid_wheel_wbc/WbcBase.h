//
// Created by qiayuan on 2022/7/1.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wheel_wbc/Task.h"
#include <humanoid_wheel_interface/ManipulatorModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ros/ros.h"
#include "ocs2_mpc/SystemObservation.h"

#include <ocs2_core/misc/LoadData.h>

namespace ocs2
{
  namespace mobile_manipulator
  {
    class WbcBase
    {
      using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
      using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
      using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

    public:
      WbcBase(const PinocchioInterface &pinocchioInterface, const ManipulatorModelInfo& info);

      virtual void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real);

      virtual vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation);

      void setArmNums(int arm_nums)
      {
        arm_nums_ = arm_nums;
        lowJoint_nums_ = info_.armDim - arm_nums_;
      }

      void setUseVrArmAccelTask(bool useVrArmAccelTask)
      {
        useVrArmAccelTask_ = useVrArmAccelTask;
      }
      
    protected:
      size_t getNumDecisionVars() const
      {
        return numDecisionVars_;
      }

      Task formulateFloatingBaseEomTask();
      Task formulateTorqueLimitsTask();
      Task formulateLowJointAccelTask();
      Task formulateArmJointAccelTask();
      Task formulateTorsoZeroAccTask();
      Task formulateBaseAccTask();

      Eigen::Vector3d bodyToWorldVelocity(double v_body, double vyaw_body, double yaw);

      void updateMeasured(const vector_t &stateMeasured, const vector_t &inputMeasured);
      void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);

      size_t numDecisionVars_;

      PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
      ManipulatorModelInfo info_;

      vector_t qMeasured_, vMeasured_;
      vector_t qDesired_, vDesired_;

      size_t lowJoint_nums_{};
      size_t arm_nums_{};

      vector_t armJointKp_, armJointKd_;
      vector_t vrArmJointKp_, vrArmJointKd_;
      vector_t lowJointKp_, lowJointKd_;
      bool useVrArmAccelTask_{false};
      bool hasVrArmAccelTask_{false};

      vector_t torqueLimits_;
      matrix_t j_torso_, dj_torso_;
      size_t torso_id_{0};
      size_t base_id_{0};
      std::vector<size_t> ee_ids_; 
    };

  } // namespace mobile_manipulator
} // namespace ocs2
