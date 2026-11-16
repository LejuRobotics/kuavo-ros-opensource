//
// Created by qiayuan on 2022/7/1.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once
#include <pinocchio/fwd.hpp>  // Configure Boost before ROS/OCS2 headers.
#include <humanoid_wheel_wbc/WbcBase.h>
#include <array>
#include <cstdint>

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
    class AbsoluteWbcBase : public WbcBase
    {
      using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
      using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
      using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

    public:
      AbsoluteWbcBase(const PinocchioInterface &pinocchioInterface, const ManipulatorModelInfo& info);

      virtual ~AbsoluteWbcBase() = default;

      virtual void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real);

      virtual vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation);

      void writeE2eAudit(const vector_t& solution, int status, int iterations, double solveTime);
      // Observation context only. uint64 identities split into exact 32-bit halves.
      void setTaskAuditReference(uint64_t session, uint64_t generation,
          uint64_t sequence, uint64_t epoch, bool applied, double ffGain) {
        const uint64_t ids[]={session,generation,sequence,epoch};
        for(int i=0;i<4;++i) { taskAuditReference_[2*i]=ids[i]>>32;
          taskAuditReference_[2*i+1]=ids[i]&0xffffffffULL; }
        taskAuditReference_[8]=applied;taskAuditReference_[9]=ffGain;
      }

      void setArmNums(int arm_nums)
      {
        arm_nums_ = arm_nums;
        lowJoint_nums_ = info_.armDim - arm_nums_;
        armAccelerationFeedforward_.setZero(arm_nums_);
        lowJointAccelerationFeedforward_.setZero(lowJoint_nums_);
      }

      void setUseVrArmAccelTask(bool useVrArmAccelTask)
      {
        useVrArmAccelTask_ = useVrArmAccelTask;
      }

      /**
       * Set acceleration from the same atomic q/dq/ddq reference used in this
       * controller cycle. Passing enabled=false preserves legacy feedback-only
       * behaviour.
       */
      void setArmAccelerationFeedforward(const vector_t& acceleration,
                                         bool enabled)
      {
        jointAccelerationFeedforwardValid_ = false;
        lowJointAccelerationFeedforward_.setZero(lowJoint_nums_);
        armAccelerationFeedforwardValid_ =
            enabled &&
            acceleration.size() == static_cast<Eigen::Index>(arm_nums_) &&
            acceleration.array().isFinite().all();
        if (armAccelerationFeedforwardValid_)
        {
          armAccelerationFeedforward_ = acceleration;
        }
        else
        {
          armAccelerationFeedforward_.setZero(arm_nums_);
        }
      }

      /** Set lower-body and arm ddq from one atomic whole-body reference. */
      void setJointAccelerationFeedforward(const vector_t& acceleration,
                                           bool enabled)
      {
        jointAccelerationFeedforwardValid_ =
            enabled &&
            acceleration.size() == static_cast<Eigen::Index>(info_.armDim) &&
            acceleration.array().isFinite().all();
        if (jointAccelerationFeedforwardValid_)
        {
          lowJointAccelerationFeedforward_ =
              acceleration.head(lowJoint_nums_);
          armAccelerationFeedforward_ = acceleration.tail(arm_nums_);
          armAccelerationFeedforwardValid_ = true;
        }
        else
        {
          lowJointAccelerationFeedforward_.setZero(lowJoint_nums_);
          armAccelerationFeedforward_.setZero(arm_nums_);
          armAccelerationFeedforwardValid_ = false;
        }
      }

    protected:
      std::array<double,384> taskAuditRow_{};
      std::array<double,10> taskAuditReference_{};
      std::array<bool,14> taskAuditJump_{};
      uint64_t taskAuditCycle_=0;
      bool taskAuditReady_=false, taskAuditForcePd_=false;
      double taskAuditArmWeight_=0.0;
      size_t getNumDecisionVars() const
      {
        return numDecisionVars_;
      }

      Task formulateFloatingBaseEomTask();
      Task formulateLowJointAccelTask();
      Task formulateArmJointAccelTask();
      Task formulateTorsoZeroAccTask();
      Task formulateBaseAccTask();

      void processArmJointErrorsWithSafe(vector_t& pos_error, vector_t& vel_error);

      Eigen::Vector3d bodyToWorldVelocity(double v_body, double vyaw_body, double yaw);

      void updateMeasured(const vector_t &stateMeasured, const vector_t &inputMeasured);
      void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);

      vector_t armAccelerationFeedforward_;
      bool armAccelerationFeedforwardValid_{false};
      vector_t lowJointAccelerationFeedforward_;
      bool jointAccelerationFeedforwardValid_{false};

      bool retainLowerArmCoupling_{false};
    };

  } // namespace mobile_manipulator
} // namespace ocs2
