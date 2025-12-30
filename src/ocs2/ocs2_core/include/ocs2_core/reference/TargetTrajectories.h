/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ostream>

#include "ocs2_core/Types.h"
#include <iostream>
namespace ocs2 {

/**
 * This class is an interface class for the user defined target trajectories.
 */
struct TargetTrajectories {
  explicit TargetTrajectories(size_t size = 0);
  TargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                     vector_array_t desiredInputTrajectory = vector_array_t());
  void clear();
  bool empty() const { return timeTrajectory.empty() || stateTrajectory.empty(); }
  size_t size() const { return timeTrajectory.size(); }
  TargetTrajectories segmentState(int startIndex, int length) const
  {
    vector_array_t newStates;
    for (auto state : stateTrajectory)
    {
      newStates.push_back(state.segment(startIndex, length));
    }
    return TargetTrajectories(timeTrajectory, newStates, inputTrajectory);
  };

  TargetTrajectories segmentTargetTrajectories(scalar_t startTime, scalar_t endTime, bool trim = true)
  {
    TargetTrajectories targetTrajectories;
    targetTrajectories.timeTrajectory.push_back(startTime);
    targetTrajectories.stateTrajectory.push_back(getDesiredState(startTime));
    targetTrajectories.inputTrajectory.push_back(getDesiredInput(startTime));
    for (size_t i = 0; i < timeTrajectory.size(); ++i)
    {
      if (timeTrajectory[i] > startTime && timeTrajectory[i] < endTime)
      {
        targetTrajectories.timeTrajectory.push_back(timeTrajectory[i]);
        targetTrajectories.stateTrajectory.push_back(stateTrajectory[i]);
        targetTrajectories.inputTrajectory.push_back(inputTrajectory[i]);
      }
    }

    targetTrajectories.timeTrajectory.push_back(endTime);
    targetTrajectories.stateTrajectory.push_back(getDesiredState(endTime));
    targetTrajectories.inputTrajectory.push_back(getDesiredInput(endTime));

    if (trim && timeTrajectory.size() > 2)
    {
      for (size_t i = 1; i < timeTrajectory.size(); ++i)
      {// 清理前前一个轨迹点
        if (timeTrajectory.size() <= 2)
          break;
        if (timeTrajectory[i] < startTime)
        {
          timeTrajectory.erase(timeTrajectory.begin());
          stateTrajectory.erase(stateTrajectory.begin());
          inputTrajectory.erase(inputTrajectory.begin());
        }else{
          break;
        }
      }
    }
    return targetTrajectories;
  }
  // fill the target trajectories with the other one
  void fillTargetTrajectories(const TargetTrajectories &other,
                              int startIndex)
  {
    auto length = other.stateTrajectory[0].size();
    mergeTargetTrajectories(other, startIndex, 0, length);
  }

  // 合并两个目标轨迹
  void mergeTargetTrajectories(const TargetTrajectories &other,
                               int startIndex, int length)
  {
    mergeTargetTrajectories(other, startIndex, startIndex, length);
  }
  void mergeTargetTrajectories(const TargetTrajectories &other,
                               int startIndex, int otherStartIndex, int length)
  {

    // 获取所有时间节点
    std::vector<scalar_t> allTimes;
    allTimes.reserve(timeTrajectory.size() + other.timeTrajectory.size());
    allTimes.insert(allTimes.end(), timeTrajectory.begin(), timeTrajectory.end());
    allTimes.insert(allTimes.end(), other.timeTrajectory.begin(), other.timeTrajectory.end());

    // 对时间节点进行排序并去重
    std::sort(allTimes.begin(), allTimes.end());
    allTimes.erase(std::unique(allTimes.begin(), allTimes.end()), allTimes.end());

    scalar_array_t newTimeTrajectory = allTimes;

    vector_array_t newStateTrajectory;
    newStateTrajectory.reserve(allTimes.size());
    vector_array_t newInputTrajectory;
    newInputTrajectory.reserve(allTimes.size());

    for (const auto &t : allTimes)
    {
      vector_t state1 = this->getDesiredState(t);
      vector_t state2 = other.getDesiredState(t);

      // 替换指定部分
      state1.segment(startIndex, length) = state2.segment(otherStartIndex, length);
      newStateTrajectory.push_back(state1);
      newInputTrajectory.push_back(this->getDesiredInput(t));
    }
    timeTrajectory = newTimeTrajectory;
    stateTrajectory = newStateTrajectory;
    inputTrajectory = newInputTrajectory;
  }

  void trimTargetTrajectories(scalar_t startTime)
  {
    if (timeTrajectory.empty() || timeTrajectory.front() >= startTime)
      return;
    
    auto initState = getDesiredState(startTime);
    auto initInput = getDesiredInput(startTime);
    //找到第一个大于或等于 startTime 的元素
    auto index = std::lower_bound(timeTrajectory.begin(), timeTrajectory.end(), startTime);
    size_t eraseCount = std::distance(timeTrajectory.begin(), index);

    timeTrajectory.erase(timeTrajectory.begin(), index);
    stateTrajectory.erase(stateTrajectory.begin(), stateTrajectory.begin() + eraseCount);
    inputTrajectory.erase(inputTrajectory.begin(), inputTrajectory.begin() + eraseCount);
    if (timeTrajectory.size() < 2)
    {
      timeTrajectory.insert(timeTrajectory.begin(), startTime);
      stateTrajectory.insert(stateTrajectory.begin(), initState);
      inputTrajectory.insert(inputTrajectory.begin(), initInput);
    }
  }

  bool operator==(const TargetTrajectories& other);
  bool operator!=(const TargetTrajectories& other) { return !(*this == other); }

  vector_t getDesiredState(scalar_t time) const;
  vector_t getDesiredInput(scalar_t time) const;

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
};

void swap(TargetTrajectories& lh, TargetTrajectories& rh);
std::ostream& operator<<(std::ostream& out, const TargetTrajectories& targetTrajectories);

}  // namespace ocs2
