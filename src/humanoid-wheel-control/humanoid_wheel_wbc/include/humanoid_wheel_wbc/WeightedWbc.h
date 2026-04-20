//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wheel_wbc/WbcBase.h"

namespace ocs2
{
  namespace mobile_manipulator
  {
    struct Wbc_weight_t {
      scalar_t weightLowJointAccel_;
      scalar_t weightArmAccel_;
      scalar_t weightTorsoZeroAccel_;
    };

    class WeightedWbc : public WbcBase
    {
    public:
      using WbcBase::WbcBase;

      vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation) override;

      void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) override;
    protected:
      virtual Task formulateConstraints();
      virtual Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired);
      
      // 访问权重的方法，供派生类使用
      scalar_t getWeightArmAccel() const { return wbc_weight_wheel_.weightArmAccel_; }
      void setWeightArmAccel(scalar_t weight) { wbc_weight_wheel_.weightArmAccel_ = weight; }
      scalar_t getWeightLowJointAccel() const { return wbc_weight_wheel_.weightLowJointAccel_; }
      scalar_t getWeightTorsoZeroAccel() const { return wbc_weight_wheel_.weightTorsoZeroAccel_; }
    
    private:
      vector_t last_qpSol;
      Wbc_weight_t wbc_weight_wheel_;
    };

  } // namespace mobile_manipulator
} // namespace ocs2
