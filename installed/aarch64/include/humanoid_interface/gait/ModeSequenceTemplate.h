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
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>

#include "humanoid_interface/gait/Gait.h"
#include "humanoid_interface/gait/MotionPhaseDefinition.h"

namespace ocs2 {
namespace humanoid {

/**
 * ModeSequenceTemplate describes a periodic sequence of modes. It is defined by
 * - switching times (size N+1), where the first time is 0, and the last time denotes the period of the cycle
 * - modeSequence (size N), indicating the mode between the switching times.
 */
struct ModeSequenceTemplate {
  /**
   * Constructor for a ModeSequenceTemplate. The number of modes must be greater than zero (N > 0)
   * @param [in] switchingTimesInput : switching times of size N + 1
   * @param [in] modeSequenceInput : mode sequence of size N
   */
  ModeSequenceTemplate(std::vector<scalar_t> switchingTimesInput, std::vector<size_t> modeSequenceInput)
      : switchingTimes(std::move(switchingTimesInput)), modeSequence(std::move(modeSequenceInput)) {
    assert(!modeSequence.empty());
    assert(switchingTimes.size() == modeSequence.size() + 1);
  }

  /**
   * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration
   * of the template logic. t_1 to t_n are the event moments.
   */
  std::vector<scalar_t> switchingTimes;

  /**
   * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is
   * active in period [t_i, t_(i+1)]
   */
  std::vector<size_t> modeSequence;
  
  /**
   * Replace the specified modes with a new mode.
   * @param modesToReplace : List of mode IDs to replace
   * @param newMode : New mode ID to replace the specified modes with
   */
  void replaceModes(const std::vector<size_t>& modesToReplace, size_t newMode) {
    for (size_t& mode : modeSequence) {
        if (std::find(modesToReplace.begin(), modesToReplace.end(), mode) != modesToReplace.end()) {
            mode = newMode;
        }
    }
  }

  void replaceModes(size_t oldMode, size_t newMode) {
    for (size_t& mode : modeSequence) {
        if (mode == oldMode) {
            mode = newMode;
        }
    }
  }

  /**
   * Swap two modes in the mode sequence.
   * @param modeA : The first mode ID to swap
   * @param modeB : The second mode ID to swap
   */
  void swapModes(size_t modeA, size_t modeB) {
      for (size_t& mode : modeSequence) {
          if (mode == modeA) {
              mode = modeB;
          } else if (mode == modeB) {
              mode = modeA;
          }
      }
  }
  
    /**
   * Scales the time duration of specified modes by a given factor.
   * @param modesToScale : List of mode IDs to scale
   * @param scaleFactor : Factor to scale the specified modes by
   */
  void scaleModeTimes(const std::vector<size_t>& modesToScale, scalar_t scaleFactor) {
    if (modeSequence.empty() || switchingTimes.size() < 2) {
      return;  // Empty or invalid template, return immediately
    }

    const scalar_t epsilon = 1e-6;  // Threshold for considering scaleFactor as zero

    std::vector<scalar_t> newSwitchingTimes;
    std::vector<size_t> newModeSequence;
    newSwitchingTimes.push_back(switchingTimes[0]);  // Keep the start time unchanged

    for (size_t i = 0; i < modeSequence.size(); ++i) {
      scalar_t duration = switchingTimes[i+1] - switchingTimes[i];
      
      if (std::find(modesToScale.begin(), modesToScale.end(), modeSequence[i]) != modesToScale.end()) {
        // If the current mode needs to be scaled
        if (std::abs(scaleFactor) < epsilon) {
          // If scaleFactor is close to zero, skip this mode
          continue;
        }
        duration *= scaleFactor;
      }

      newSwitchingTimes.push_back(newSwitchingTimes.back() + duration);
      newModeSequence.push_back(modeSequence[i]);
    }

    // Update the template
    switchingTimes = std::move(newSwitchingTimes);
    modeSequence = std::move(newModeSequence);
  }

  bool containsAnyMode(const std::vector<size_t>& modes) const {
      return std::any_of(modes.begin(), modes.end(), [this](size_t mode) {
          return std::find(this->modeSequence.begin(), this->modeSequence.end(), mode) != this->modeSequence.end();
      });
  }

  static bool areFloatsEqual(scalar_t a, scalar_t b, scalar_t epsilon = 1e-9) {
      return std::abs(a - b) < epsilon;
  }

  static bool areFloatVectorsEqual(const std::vector<scalar_t>& v1, const std::vector<scalar_t>& v2, scalar_t epsilon = 1e-9) {
      if (v1.size() != v2.size()) return false;
      for (size_t i = 0; i < v1.size(); ++i) {
          if (!areFloatsEqual(v1[i], v2[i], epsilon)) return false;
      }
      return true;
  }

  bool isEqual(const ModeSequenceTemplate& other, scalar_t epsilon = 1e-9) const {
      return areFloatVectorsEqual(switchingTimes, other.switchingTimes, epsilon) && 
              modeSequence == other.modeSequence;
  }

  bool operator==(const ModeSequenceTemplate& other) const {
      return isEqual(other);
  }

  bool operator!=(const ModeSequenceTemplate& other) const {
      return !(*this == other);
  }

  bool areModesEqual(const ModeSequenceTemplate& other) const {
      if (modeSequence.size() != other.modeSequence.size()) 
          return false;
      for (size_t i = 0; i < modeSequence.size(); ++i) {
          if (modeSequence[i] != other.modeSequence[i]) {
              return false;
          }
      }
    return true;
    
  }
};

/** Swap two modesequence templates */
inline void swap(ModeSequenceTemplate& lh, ModeSequenceTemplate& rh) {
  lh.switchingTimes.swap(rh.switchingTimes);
  lh.modeSequence.swap(rh.modeSequence);
}

/** Print the modesequence template */
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate);

/** Converts a mode sequence template to a gait */
Gait toGait(const ModeSequenceTemplate& modeSequenceTemplate);

/**
 * Load a modesequence template from file.  The template needs to be declared as:
 *
 * topicName
 * {
 *   modeSequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *   }
 *   switchingTimes
 *   {
 *     [0]     0.0
 *     [1]     t1
 *     [2]     T
 *   }
 * }
 */
ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose = true);

/**
 * Load a mode schedule template from file.  The schedule needs to be declared as:
 *
 * topicName
 * {
 *   modeSequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *     [2]     mode2
 *   }
 *   eventTimes
 *   {
 *     [0]     t0
 *     [1]     t1
 *   }
 * }
 */
ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose);
std::string getModeScheduleName(std::map<std::string, ModeSequenceTemplate>& gaitMap_, const ModeSequenceTemplate& modeSequenceTemplate);

}  // namespace humanoid
}  // namespace ocs2
