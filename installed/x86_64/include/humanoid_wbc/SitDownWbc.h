//
// Created for Kuavo seat P3 (sit-down offset) WBC.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wbc/WbcBase.h"

namespace ocs2
{
  namespace humanoid
  {
    /** WBC for seated posture adjustment (P3): COM + stance feet + leg/arm joint tracking. */
    class SitDownWbc : public WbcBase
    {
    public:
      using WbcBase::WbcBase;

      vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                      size_t mode, scalar_t period, bool mpc_update = false) override;

      void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) override;

    protected:
      Task formulateConstraints(const vector_t &inputDesired);
      Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
      void loadSitDownJointAccelTaskSetting(const std::string &taskFile, bool verbose, bool is_real);

    private:
      scalar_t weightBaseLinear_{}, weightBaseAngular_{}, weightJointAccel_{}, weightContactForce_{},
          weightStanceLeg_{80};

      vector_t last_qpSol;
    };

  } // namespace humanoid
} // namespace ocs2
