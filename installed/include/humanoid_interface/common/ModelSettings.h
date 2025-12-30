/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace humanoid {

struct humanoidModelInfo{
  std::string baseFrame;                    // name of the base frame of the robot
  std::vector<std::string> eeFrame;         // name of the end-effector frame of the robot
};

struct ModelSettings {
  scalar_t positionErrorGain = 0.0;
  scalar_t positionErrorGain_xy = 0.0;
  scalar_t velocityErrorGain_xy = 0.0;
  scalar_t positionErrorGain_zero = 0.0; // velocity error gain for zero velocity

  scalar_t velocityErrorGainXYStepControl = 0.0;
  scalar_t positionErrorGainXYStepControl = 0.0;

  int mpcArmsDof = 14;
  int mpcLegsDof = 12;
  int modelDof = 26;



  scalar_t phaseTransitionStanceTime = 0.4;

  bool verboseCppAd = true;
  bool recompileLibrariesCppAd = true;
  std::string modelFolderCppAd = "/tmp/ocs2";

  // This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
  std::vector<std::string> jointNames{}; // joint names with MPC dof
  std::vector<std::string> jointNamesReal{}; // joint names with same dof as the real robot
  std::vector<std::string> simplifiedJointNames{}; // jointNamesReal - jointNames
  std::vector<std::string> contactNames6DoF{};
  std::vector<std::string> contactNames3DoF{};

  // For humanoid Whole body
  humanoidModelInfo info;
};

ModelSettings loadModelSettings(const std::string& filename, const std::string& urdfFile = "default", const std::string& fieldName = "model_settings", bool verbose = false);

}  // namespace humanoid
}  // namespace ocs2
