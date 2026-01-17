//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_wheel_wbc/WeightedWbc.h"

#include <qpOASES.hpp>
#include <fstream>

namespace ocs2
{
  namespace mobile_manipulator
  {
    using Duration = std::chrono::duration<double>;
    using Clock = std::chrono::high_resolution_clock;

    vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation)
    {
      WbcBase::update(stateDesired, inputDesired, observation);
      // Constraints
      Task constraints = formulateConstraints();
      size_t numConstraints = constraints.b_.size() + constraints.f_.size();

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
      vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
      A << constraints.a_,
          constraints.d_;

      lbA << constraints.b_,
            -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
      ubA << constraints.b_,
            constraints.f_; // clang-format on

      // Cost
      Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
          weighedTask.a_.transpose() * weighedTask.a_;
      vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

      // Solve
      auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_LOW;
      options.enableEqualities = qpOASES::BT_TRUE;
      qpProblem.setOptions(options);
      int nWsr = 200;
      qpOASES::real_t cpu_time = 0.002;
      qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr, &cpu_time);
      vector_t qpSol(getNumDecisionVars());

      int ret = qpProblem.getPrimalSolution(qpSol.data());

      if (ret != qpOASES::SUCCESSFUL_RETURN)
      {
        ROS_ERROR_STREAM("WeightWBC Not Solved!!!");
        if (last_qpSol.size() > 0)
          qpSol = last_qpSol;
        else
          qpSol.setZero();
      }

      last_qpSol = qpSol;

      // std::cout << "numConstraints: " << numConstraints << std::endl;
      return qpSol;
    }

    Task WeightedWbc::formulateConstraints()
    {
      Task totalConstraints = formulateFloatingBaseEomTask() +  formulateTorqueLimitsTask();
      return totalConstraints;
    }

    Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired)
    {
      Task totalTask = formulateLowJointAccelTask() * wbc_weight_wheel_.weightLowJointAccel_ + 
                       formulateArmJointAccelTask() * wbc_weight_wheel_.weightArmAccel_ + 
                       formulateTorsoZeroAccTask() * wbc_weight_wheel_.weightTorsoZeroAccel_;
      return totalTask;
    }

    void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      WbcBase::loadTasksSetting(taskFile, verbose, is_real);

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "weight.";
      if (verbose)
      {
        std::cerr << "\n #### WBC weight:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, wbc_weight_wheel_.weightLowJointAccel_, prefix + "accLowJoint", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_wheel_.weightArmAccel_, prefix + "accArm", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_wheel_.weightTorsoZeroAccel_, prefix + "accTorsoZero", verbose);
      
    }

  } // namespace mobile_manipulator
} // namespace ocs2
