/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_mpc/MPC_MRT_Interface.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_MRT_Interface::MPC_MRT_Interface(MPC_BASE& mpc) : mpc_(mpc) {
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::resetMpcNode(const TargetTrajectories& initTargetTrajectories) {
  mpc_.reset();
  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::setCurrentObservation(const SystemObservation& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ReferenceManagerInterface& MPC_MRT_Interface::getReferenceManager() {
  return mpc_.getSolverPtr()->getReferenceManager();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const ReferenceManagerInterface& MPC_MRT_Interface::getReferenceManager() const {
  return mpc_.getSolverPtr()->getReferenceManager();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::advanceMpc() {
  // measure the delay in running MPC
  mpcTimer_.startTimer();

  SystemObservation currentObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    currentObservation = currentObservation_;
  }

  bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
  if (!controllerIsUpdated) {
    return;
  }
  copyToBuffer(currentObservation);

  // measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }
  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "[MPC_MRT_Interface::advanceMpc] WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // measure the delay
  if (mpc_.settings().debugPrint_) {
    std::cerr << "\n### MPC_MRT Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::copyToBuffer(const SystemObservation& mpcInitObservation) {
  // policy
  auto primalSolutionPtr = std::make_unique<PrimalSolution>();
  const scalar_t startTime = mpcInitObservation.time;
  const scalar_t finalTime =
      (mpc_.settings().solutionTimeWindow_ < 0) ? mpc_.getSolverPtr()->getFinalTime() : startTime + mpc_.settings().solutionTimeWindow_;
  mpc_.getSolverPtr()->getPrimalSolution(finalTime, primalSolutionPtr.get());

  // command
  auto commandPtr = std::make_unique<CommandData>();
  commandPtr->mpcInitObservation_ = mpcInitObservation;
  commandPtr->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();

  // performance indices
  auto performanceIndicesPtr = std::make_unique<PerformanceIndex>();
  try {
    *performanceIndicesPtr = mpc_.getSolverPtr()->getPerformanceIndeces();
  } catch (const std::runtime_error& e) {
    // 检查是否是性能日志为空的错误（reset期间常见）
    std::string error_msg(e.what());
    if (error_msg.find("No performance log yet") != std::string::npos) {
      // 创建一个默认的性能指标，避免中断MPC线程
      std::cout << "[MPC_MRT_Interface::copyToBuffer] No performance log yet, creating default performance indices" << std::endl;
      performanceIndicesPtr->merit = 0.0;
      performanceIndicesPtr->cost = 0.0;
      performanceIndicesPtr->dualFeasibilitiesSSE = 0.0;
      performanceIndicesPtr->dynamicsViolationSSE = 0.0;
      performanceIndicesPtr->equalityConstraintsSSE = 0.0;
      performanceIndicesPtr->inequalityConstraintsSSE = 0.0;
      performanceIndicesPtr->equalityLagrangian = 0.0;
      performanceIndicesPtr->inequalityLagrangian = 0.0;
      // 注意：这里不输出警告信息，因为这是正常的reset过程
    } else {
      // 其他运行时错误重新抛出
      throw;
    }
  }

  this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t MPC_MRT_Interface::getLinearFeedbackGain(scalar_t time) {
  auto controller = dynamic_cast<LinearController*>(this->getPolicy().controllerPtr_.get());
  if (controller == nullptr) {
    throw std::runtime_error("[MPC_MRT_Interface::getLinearFeedbackGain] Feedback gains only available with linear controller!");
  }
  matrix_t K;
  controller->getFeedbackGain(time, K);
  return K;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MPC_MRT_Interface::getValueFunction(scalar_t time, const vector_t& state) const {
  return mpc_.getSolverPtr()->getValueFunction(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MPC_MRT_Interface::getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const {
  return mpc_.getSolverPtr()->getStateInputEqualityConstraintLagrangian(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiplierCollection MPC_MRT_Interface::getIntermediateDualSolution(scalar_t time) const {
  return mpc_.getSolverPtr()->getIntermediateDualSolution(time);
}

}  // namespace ocs2
