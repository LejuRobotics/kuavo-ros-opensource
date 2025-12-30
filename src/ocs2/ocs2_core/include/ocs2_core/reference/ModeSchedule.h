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
#include <vector>
#include <iostream>

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Defines a sequence of N modes, separated by N-1 event times
 */
struct ModeSchedule {
  /**
   * Default constructor.
   */
  ModeSchedule() : ModeSchedule(std::vector<scalar_t>{}, std::vector<size_t>{0}) {}

  /**
   * Constructor for a ModeSchedule. The number of modes must be greater than zero (N > 0)
   * @param [in] eventTimesInput : event times of size N - 1
   * @param [in] modeSequenceInput : mode sequence of size N
   * @param [in] enableFootSequenceInput : enable foot sequence of size N
   * @param [in] footPoseSequenceInput : foot pose sequence of size N
   * @param [in] torsoPoseSequenceInput : torso pose sequence of size N
   */
  ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput,
               std::vector<bool> enableFootSequenceInput, std::vector<Eigen::Vector4d> footPoseSequenceInput,
               std::vector<Eigen::Vector4d> torsoPoseSequenceInput);

  /**
   * Constructor for a ModeSchedule. The number of modes must be greater than zero (N > 0)
   * @param [in] eventTimesInput : event times of size N - 1
   * @param [in] modeSequenceInput : mode sequence of size N
   */
  ModeSchedule(std::vector<scalar_t> eventTimesInput, std::vector<size_t> modeSequenceInput);

  /**
   *  Returns the mode based on the query time.
   *  Events are counted as follows:
   *      ------ | ------ | ------ | ...  ------ | ------
   *         t[0]     t[1]     t[2]        t[n-1]
   *  mode: m[0]    m[1]     m[2] ...     m[n-1]    m[n]
   *
   *  If time equal to a switch time is requested, the lower count is taken
   *
   *  @param [in] time: The inquiry time.
   *  @return the associated mode for the input time.
   */
  size_t modeAtTime(scalar_t time) const;
  
  int modeBeforeId(scalar_t time) const;
  
  size_t modeBefore(scalar_t time) const{return modeSequence[modeBeforeId(time)];};

  scalar_t timeBefore(scalar_t time) const{return eventTimes[modeBeforeId(time)];};

  int modeNextId(scalar_t time) const;

  size_t modeNext(scalar_t time) const{return modeSequence[modeNextId(time)];};

  scalar_t timeNext(scalar_t time) const{return eventTimes[modeNextId(time)];};

  scalar_t timeSwitch(scalar_t time) const{return eventTimes[modeNextId(time)-1];};

  bool existValidFootPose() const{
    size_t count = 0;
    for (int i = 0; i < enableFootSequence.size(); i++)
    {
      if(enableFootSequence[i])
        count++;
      if(count > 1)//最后一个
        return true;
    }
    
    return false;
  }

  bool existValidFootPose(scalar_t time) const{
    size_t count = 0;
    const size_t index_start = std::lower_bound(eventTimes.begin(), eventTimes.end(), time) - eventTimes.begin();
    for(size_t i = index_start+1; i < enableFootSequence.size(); i++)
    {
      if(enableFootSequence[i])
        count++;
      if(count > 0)//最后一个
        return true;
    }
    
    return false;
  }

  /** Clears modeSchedule */
  void clear() {
    eventTimes.clear();
    modeSequence.clear();
    enableFootSequence.clear();
    enableFullBodySequence.clear();
    footPoseSequence.clear();
    torsoPoseSequence.clear();
    swingHeightSequence.clear();
    fullBodyStateSequence.clear();
    timeTrajectorySequence.clear();
    additionalFootPoseSequence.clear();
  }

  /**
   * Gets a sub-schedule within the specified time range
   * @param [in] startTime: start time of the sub-schedule
   * @param [in] endTime: end time of the sub-schedule
   * @param [out] subSchedule: output ModeSchedule containing events within [startTime, endTime]
   * @return true if successfully found modes within the time range, false otherwise
   */
  bool getSubSchedule(scalar_t startTime, scalar_t endTime, ModeSchedule& subSchedule) const;

  std::vector<scalar_t> eventTimes;  // event times of size N - 1
  std::vector<size_t> modeSequence;  // mode sequence of size N
  std::vector<bool> enableFootSequence;  // decide whether foot is enabled or not at each mode at each event time
  std::vector<bool> enableFullBodySequence;
  std::vector<Eigen::Vector4d> footPoseSequence;  // foot pos(xyz yaw) of each mode at each event time
  std::vector<Eigen::Vector4d> torsoPoseSequence; // torso pos(xyz yaw) of each mode at each event time
  std::vector<std::vector<Eigen::Vector4d>> additionalFootPoseSequence; // additional foot pos(xyz yaw) of each mode at each event time
  std::vector<double> swingHeightSequence; // swing height of each mode at swing phase
  // bool with_pos_sequence{false};  // whether posSequence is available or not
  std::vector<vector_array_t> fullBodyStateSequence;
  std::vector<scalar_array_t> timeTrajectorySequence;
};

/** Exchanges the given values. */
void swap(ModeSchedule& lh, ModeSchedule& rh);

/** Inserts modeSchedule into the output stream. */
std::ostream& operator<<(std::ostream& stream, const ModeSchedule& modeSchedule);

/**
 * Gets the number of events that have preceded the given event time.
 *
 * @param [in] timeTrajectory: A trajectory of timestamps.
 * @param [in] postEventIndices: An array of post-event time index in the time trajectory.
 * @param [in] eventTime: The requested event time.
 * @return The number of events that have preceded the given event time.
 */
size_t getNumberOfPrecedingEvents(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, scalar_t eventTime);

/**
 * Finds the intersection of the requested period i.e., timePeriod, to the time interval of the modes that timeTrajectory
 * covers, i.e., extendable interval.
 *
 * Case 1: non-empty interval
 * eventTimes       s0             s2         s2          s3
 * ------------------------|--------------|----------|-----------|-------
 * timeTrajectory ---------------****************------------------------
 * Extendable interval ----+++++++++++++++++++++++++++-------------------
 * Requested timePeriod ----------------------[           ]--------------
 * Output ------------------------------------[******]-------------------
 *
 * Case 2: empty interval
 * eventTimes       s0             s2         s2          s3
 * ------------------------|--------------|----------|-----------|-------
 * timeTrajectory ---------------******************----------------------
 * Extendable interval ----+++++++++++++++++++++++++++-------------------
 * Requested timePeriod -----------------------------------[         ]---
 * Output ------------------------------------------------][-------------
 *
 * @param [in] timeTrajectory: Timestamp of the data to be interpolated.
 * @param [in] eventTimes: The event times array.
 * @param [in] timePeriod: The requested time period for possible interpolation.
 * @return The truncated time interval in which the interpolation is valid. In the case that the interpolation is not allowed it
 * returns and interval of negative length i.e., (timePeriod.first, timePeriod.first - 1e-4).
 */
std::pair<scalar_t, scalar_t> findIntersectionToExtendableInterval(const scalar_array_t& timeTrajectory, const scalar_array_t& eventTimes,
                                                                   const std::pair<scalar_t, scalar_t>& timePeriod);

}  // namespace ocs2
