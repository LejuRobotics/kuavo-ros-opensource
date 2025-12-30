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

#include "ocs2_core/reference/ModeSchedule.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule::ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput)
    : eventTimes(std::move(eventTimesInput)), modeSequence(std::move(modeSequenceInput)) {
  if (modeSequence.empty()) {
    throw std::runtime_error("[ModeSchedule] ModeSequence cannot be empty!");
  }
  if (modeSequence.size() != eventTimes.size() + 1) {
    throw std::runtime_error("[ModeSchedule] Incompatible event times and mode sequence lengths!");
  }
  
  // Initialize other sequences with default values
  enableFootSequence.resize(modeSequence.size(), false);
  enableFullBodySequence.resize(modeSequence.size(), false);
  footPoseSequence.resize(modeSequence.size(), Eigen::Vector4d::Zero());
  torsoPoseSequence.resize(modeSequence.size(), Eigen::Vector4d::Zero());
  swingHeightSequence.resize(modeSequence.size(), 0.06);
  fullBodyStateSequence.resize(modeSequence.size(), vector_array_t());
  additionalFootPoseSequence.resize(modeSequence.size(), std::vector<Eigen::Vector4d>());
  timeTrajectorySequence.resize(modeSequence.size(), scalar_array_t());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule::ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput,
               std::vector<bool> enableFootSequenceInput, std::vector<Eigen::Vector4d> footPoseSequenceInput,
               std::vector<Eigen::Vector4d> torsoPoseSequenceInput)
    : eventTimes(std::move(eventTimesInput))
    , modeSequence(std::move(modeSequenceInput))
    , enableFootSequence(std::move(enableFootSequenceInput))
    , footPoseSequence(std::move(footPoseSequenceInput))
    , torsoPoseSequence(std::move(torsoPoseSequenceInput)) {
  

  if (modeSequence.empty()) {
    throw std::runtime_error("[ModeSchedule] ModeSequence cannot be empty!");
  }
  if (modeSequence.size() != eventTimes.size() + 1) {
    throw std::runtime_error("[ModeSchedule] Incompatible event times and mode sequence lengths!");
  }
  if (modeSequence.size() != enableFootSequence.size()) {
    throw std::runtime_error("[ModeSchedule] Incompatible mode sequence and enable foot sequence lengths!");
  }
  if (modeSequence.size() != footPoseSequence.size()) {
    throw std::runtime_error("[ModeSchedule] Incompatible mode sequence and foot pose sequence lengths!");
  }
  if (modeSequence.size() != torsoPoseSequence.size()) {
    throw std::runtime_error("[ModeSchedule] Incompatible mode sequence and torso pose sequence lengths!");
  }
  
  // Initialize remaining sequences with default values
  enableFullBodySequence.resize(modeSequence.size(), false);
  swingHeightSequence.resize(modeSequence.size(), 0.06);
  fullBodyStateSequence.resize(modeSequence.size(), vector_array_t());
  additionalFootPoseSequence.resize(modeSequence.size(), std::vector<Eigen::Vector4d>());
  timeTrajectorySequence.resize(modeSequence.size(), scalar_array_t());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ModeSchedule::getSubSchedule(scalar_t startTime, scalar_t endTime, ModeSchedule& subSchedule) const {
  subSchedule.clear();
  
  // 遍历所有事件时间，找到在时间范围内的事件
  bool last_event = false;
  size_t last_index = 0;
  for(size_t i = 0; i < eventTimes.size(); ++i) {
    if(eventTimes[i] <= endTime) {
      subSchedule.eventTimes.push_back(eventTimes[i]);
      subSchedule.modeSequence.push_back(modeSequence[i]);
      subSchedule.enableFootSequence.push_back(enableFootSequence[i]);
      subSchedule.enableFullBodySequence.push_back(enableFullBodySequence[i]);
      subSchedule.footPoseSequence.push_back(footPoseSequence[i]);
      subSchedule.torsoPoseSequence.push_back(torsoPoseSequence[i]);
      subSchedule.swingHeightSequence.push_back(swingHeightSequence[i]);
      subSchedule.fullBodyStateSequence.push_back(fullBodyStateSequence[i]);
      subSchedule.timeTrajectorySequence.push_back(timeTrajectorySequence[i]);
      subSchedule.additionalFootPoseSequence.push_back(additionalFootPoseSequence[i]);
    }
    else{
      if(last_event)
        break;
      last_index = i;
      subSchedule.eventTimes.push_back(eventTimes[i]);
      subSchedule.modeSequence.push_back(modeSequence[i]);
      subSchedule.enableFootSequence.push_back(enableFootSequence[i]);
      subSchedule.enableFullBodySequence.push_back(enableFullBodySequence[i]);
      subSchedule.footPoseSequence.push_back(footPoseSequence[i]);
      subSchedule.torsoPoseSequence.push_back(torsoPoseSequence[i]);
      subSchedule.swingHeightSequence.push_back(swingHeightSequence[i]);
      subSchedule.fullBodyStateSequence.push_back(fullBodyStateSequence[i]);
      subSchedule.timeTrajectorySequence.push_back(timeTrajectorySequence[i]);
      subSchedule.additionalFootPoseSequence.push_back(additionalFootPoseSequence[i]);
      last_event = true;
    }
  }
  
  // 如果找到了有效事件，添加最后一个模式
  if(!subSchedule.eventTimes.empty()) {
    // std::cout << "last_index: " << last_index << std::endl;
    // size_t lastIndex = std::lower_bound(eventTimes.begin(), eventTimes.end(), endTime) - eventTimes.begin();
    // if(lastIndex < modeSequence.size()) {
      subSchedule.modeSequence.push_back(modeSequence[last_index+1]);
      subSchedule.enableFootSequence.push_back(enableFootSequence[last_index+1]);
      subSchedule.enableFullBodySequence.push_back(enableFullBodySequence[last_index+1]);
      subSchedule.footPoseSequence.push_back(footPoseSequence[last_index+1]);
      subSchedule.torsoPoseSequence.push_back(torsoPoseSequence[last_index+1]);
      subSchedule.swingHeightSequence.push_back(swingHeightSequence[last_index+1]);
      subSchedule.fullBodyStateSequence.push_back(fullBodyStateSequence[last_index+1]);
      subSchedule.additionalFootPoseSequence.push_back(additionalFootPoseSequence[last_index+1]);
      subSchedule.timeTrajectorySequence.push_back(timeTrajectorySequence[last_index+1]);
      // std::cout << "subSchedule.eventTimes.size(): " << subSchedule.eventTimes.size() << std::endl;
      // std::cout << "subSchedule.modeSequence.size(): " << subSchedule.modeSequence.size() << std::endl;
    // }
  }
  
  return !subSchedule.eventTimes.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ModeSchedule::modeAtTime(scalar_t time) const {
  const auto ind = lookup::findIndexInTimeArray(eventTimes, time);
  return modeSequence[ind];
}

int ModeSchedule::modeBeforeId(scalar_t time) const
{
  int ind = lookup::findIndexInTimeArray(eventTimes, time);
  ind--;
  return ind;
}

int ModeSchedule::modeNextId(scalar_t time) const
{
  int ind = lookup::findIndexInTimeArray(eventTimes, time);
  ind++;
  return ind;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(ModeSchedule& lh, ModeSchedule& rh) {
  lh.eventTimes.swap(rh.eventTimes);
  lh.modeSequence.swap(rh.modeSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSchedule& modeSchedule) {
  stream << "event times:   {" << toDelimitedString(modeSchedule.eventTimes) << "}\n";
  stream << "mode sequence: {" << toDelimitedString(modeSchedule.modeSequence) << "}\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getNumberOfPrecedingEvents(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, scalar_t eventTime) {
  // the case of empty time trajectory and eventTime smaller or equal to the initial time
  if (timeTrajectory.empty() || eventTime <= timeTrajectory.front()) {
    return 0;
  }

  const auto eventIndexItr = std::find_if(postEventIndices.cbegin(), postEventIndices.cend(), [&](size_t postEventInd) {
    return numerics::almost_eq(eventTime, timeTrajectory[postEventInd - 1]);
  });

  // if the given time did not match any event time but it is smaller than the final time
  if (eventIndexItr == postEventIndices.cend() && eventTime < timeTrajectory.back()) {
    throw std::runtime_error(
        "[getNumberOfPrecedingEvents] The requested time is within the time trajectory but it is not marked as an event!");
  }

  return std::distance(postEventIndices.cbegin(), eventIndexItr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, scalar_t> findIntersectionToExtendableInterval(const scalar_array_t& timeTrajectory, const scalar_array_t& eventTimes,
                                                                   const std::pair<scalar_t, scalar_t>& timePeriod) {
  // no interpolation: a bit before initial time
  const std::pair<scalar_t, scalar_t> emptyInterpolatableInterval{timePeriod.first, timePeriod.first - 1e-4};

  if (timeTrajectory.empty() || timePeriod.first > timePeriod.second) {
    return emptyInterpolatableInterval;

  } else {
    const auto pastEventItr =
        std::find_if(eventTimes.crbegin(), eventTimes.crend(), [&](const scalar_t& te) { return te <= timeTrajectory.front(); });
    const auto initialTime = (pastEventItr != eventTimes.crend()) ? std::max(*pastEventItr, timePeriod.first) : timePeriod.first;

    const auto nextEventItr = std::lower_bound(eventTimes.cbegin(), eventTimes.cend(), timeTrajectory.back());
    const auto finalTime = (nextEventItr != eventTimes.cend()) ? std::min(*nextEventItr, timePeriod.second) : timePeriod.second;

    return (initialTime < finalTime) ? std::make_pair(initialTime, finalTime) : emptyInterpolatableInterval;
  }
}

}  // namespace ocs2
