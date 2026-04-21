//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_wbc/WbcBase.h"

namespace ocs2
{
  namespace humanoid
  {
    class StandUpWbc : public WbcBase
    {
    public:
      using WbcBase::WbcBase;

      vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                      size_t mode, scalar_t period, bool mpc_update = false) override;

      void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) override;

    protected:
      Task formulateConstraints(const vector_t &inputDesired);
      Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

    private:
      scalar_t weightBaseLinear_, weightBaseAngular_, weightJointAccel_, weightContactForce_;

      vector_t last_qpSol;
    };

  } // namespace humanoid
} // namespace ocs2
