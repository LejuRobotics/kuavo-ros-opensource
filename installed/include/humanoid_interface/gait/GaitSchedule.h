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

#include <mutex>
#include <iostream>
#include <string>
#include <deque>
#include <utility>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include "humanoid_interface/gait/ModeSequenceTemplate.h"
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2
{
  namespace humanoid
  {

      // 左脚腾空，右脚腾空，双脚站立，左脚脚尖着地，左脚脚跟着地，右脚脚尖着地，右脚脚跟着地
    enum class FootIdx { Left, Right, Stance, TS, HS, ST, SH, TH, HT};
    
    struct FootPoseSchedule
    {
      scalar_t startTime{-1.0};  // 新增：起始时间，默认为-1.0
      std::vector<scalar_t> eventTimes;  // event times
      std::vector<FootIdx> footIndices;  // foot indices
      std::vector<vector6_t> footPoseSequence;  // pose(xyz yaw) of each step at each event time
      std::vector<vector6_t> torsoPoseSequence;  // torso pose(xyz yaw) of each step at each event time
      std::vector<std::vector<vector6_t>> additionalFootPoseSequence;  // additional foot pose(xyz yaw) of each step at each event time
      std::vector<scalar_t> swingHeightSequence;  // swing height of each mode at swing phase
      void clear()
      {
        startTime = -1.0;  // 新增：重置起始时间
        eventTimes.clear();
        footIndices.clear();
        footPoseSequence.clear();
        torsoPoseSequence.clear();
        additionalFootPoseSequence.clear();
        swingHeightSequence.clear();
      }
    };

    std::ostream& operator<<(std::ostream& os, const FootPoseSchedule& schedule);

    struct FullBodySchedule
    {
      scalar_t startTime{-1.0};  // 新增：起始时间，默认为0
      std::vector<scalar_t> eventTimes; // event times
      std::vector<FootIdx> footIndices; // foot indices
      std::vector<TargetTrajectories> targetTrajectories;
      TargetTrajectories armTargetTrajectories;
      TargetTrajectories headTargetTrajectories;

      // std::vector<vector_t> fullBodyQTrajectory;
      // std::vector<vector_t> fullBodyVTrajectory;
      void clear()
      {
        startTime = -1.0;  // 新增：重置起始时间
        eventTimes.clear();
        footIndices.clear();
        // fullBodyQTrajectory.clear();
        // fullBodyVTrajectory.clear();
        targetTrajectories.clear();
        armTargetTrajectories.clear();
        headTargetTrajectories.clear();
      }
    };
    class GaitManager
    {
    public:
      GaitManager() {}
      GaitManager(size_t maxSize = 20) : maxSize_(maxSize) {}

      void add(const std::pair<scalar_t, std::string> &gait)
      {
        if (gaitTimeNames_.size() >= maxSize_)
        {
          gaitTimeNames_.pop_front(); // 移除最早的记录
        }
        gaitTimeNames_.emplace_back(gait);
      }
      void add(scalar_t startTime, const std::string &name)
      {

        if (gaitTimeNames_.size() >= maxSize_)
        {
          gaitTimeNames_.pop_front();
        }
        gaitTimeNames_.emplace_back(startTime, name);
      }

      std::pair<scalar_t, std::string> getGait(scalar_t currentTime) const
      {
        // 如果 gaitTimeNames_ 为空，返回默认值
        if (gaitTimeNames_.empty())
        {
          return {-1.0, "No_Gait"};
        }

        // 从末尾开始遍历 gaitTimeNames_
        for (auto it = gaitTimeNames_.rbegin(); it != gaitTimeNames_.rend(); ++it)
        {
          scalar_t startTime = it->first;
          if (currentTime >= startTime)
          {
            return *it; // 找到并返回第一个满足条件的记录
          }
        }

        // 如果所有 gait 的时间都比 currentTime 大，返回第一个 gait 记录
        return gaitTimeNames_.front();
      }

      std::string getGaitName(scalar_t currentTime) const
      {
        return getGait(currentTime).second;
      }

      std::string getLastGaitName() const
      {
        if (gaitTimeNames_.empty())
        {
          return "No_Gait";
        }
        return gaitTimeNames_.back().second; 
      }

    private:
      std::deque<std::pair<scalar_t, std::string>> gaitTimeNames_; 
      size_t maxSize_{20};
    };
    class GaitSchedule
    {
    public:
      GaitSchedule(const std::string &referenceFile ,const std::string &gaitFile , scalar_t phaseTransitionStanceTime);

      /**
       * Sets the mode schedule.
       *
       * @param [in] modeSchedule: The mode schedule to be used.
       */
      void setModeSchedule(const ModeSchedule &modeSchedule) { modeSchedule_ = modeSchedule;std::cout << "modeSchedule_ size: " << modeSchedule_.eventTimes.size() << std::endl; }

      /**
       * Gets the mode schedule.
       *
       * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
       * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
       */
      ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime, bool clean_up = false);

      /**
       * Gets the mode schedule.
       *
       * @return The mode schedule.
       */
      ModeSchedule getModeSchedule() { return currentModeSchedule_; }

      /**
       * Gets the full mode schedule.
       *
       * @return The full mode schedule.
       */
      ModeSchedule getFullModeSchedule() { return modeSchedule_; }
      ModeSchedule& getFullModeScheduleRef() { return modeSchedule_; }
      /**
       * Used to insert a new user defined logic in the given time period.
       *
       * @param [in] startTime: The initial time from which the new mode sequence template should start.
       * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
       */
      void insertModeSequenceTemplate(const ModeSequenceTemplate &modeSequenceTemplate, scalar_t startTime, scalar_t finalTime);

      /**
       * 将之前的模板序列的时间范围缩放scale倍.
       *
       * @param [in] scale: 缩放因子.
       * @param [in] startTime: The initial time from which the new mode sequence template should start.
       * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
       */
      void scaleModeSequenceTemplate(scalar_t scale, scalar_t startTime, scalar_t finalTime);
      scalar_t getModeEndTime(scalar_t& modeTime);
      /**
       * 对于walk模式的模板序列, 调节其后脚跟抬起幅度.
       *
       * @param [in] modeSequenceTemplate: walk模式的模板序列.
       * @param [in] scale: 缩放因子.0->最小抬起幅度, 1->最大抬起幅度.
       * @param [in] startTime: The initial time from which the new mode sequence template should start.
       * @param [in] timeHorizon: The final time until when the new mode sequence needs to be defined.
       */
      void modifyWalkModeSequenceTemplate(const ModeSequenceTemplate &newModeSequenceTemplate, scalar_t startTime, scalar_t timeHorizon, std::string gaitName);

      /**
       * 获取当前的gait开始时间和名称.
      */
      inline std::pair<scalar_t, std::string> getGgaitTimeName()const{ return gaitTimeName_; }

      inline std::string getGaitName(scalar_t currentTime)
      {
        return gaitManager_.getGaitName(currentTime);
      }

      inline std::string getLastGaitName() const
      {
        return gaitManager_.getLastGaitName();
      }

      inline bool isWalkingGait(const std::string& gaitName){ return gaitName == "walk" || gaitName == "trot" ; }
      inline bool isStanceGait(const std::string& gaitName){ return gaitName == "stance"; }

      inline void setGaitName(const ModeSequenceTemplate &newModeSequenceTemplate){ gaitTimeName_.second = getModeScheduleName(gaitMap_,newModeSequenceTemplate); }

     
      inline void setAutoGaitEnabled(bool enabled) { autoGaitEnabled_ = enabled; }
      inline bool isAutoGaitEnabled() const { return autoGaitEnabled_; }
      /**
       * Gets the mode sequence template.
       */
      ModeSequenceTemplate getModeSequenceTemplate() { return modeSequenceTemplate_; }

      /**
       * Count the steps.
       * @param [in] currentTime: 当前时间.
      */
      void countSteps(scalar_t currentTime);

      inline long int getSteps(){return total_steps_;}

      inline void setStopStepIdx(long int stop_step_idx){ stop_step_idx_ = stop_step_idx; }
      std::map<std::string, ModeSequenceTemplate> getGaitMap() { return gaitMap_; }


      ModeSchedule modifyModeFullBodySchedules(scalar_t currentTime, const Eigen::Vector4d &currentTorsoPose, const FullBodySchedule &fullBodySchedule, const feet_array_t<vector3_t> &foot_pos, scalar_t startInsertTime = -1);
      /**
       * 拓展足端位姿序列, 并更新模式位姿序列.
       */
      ModeSchedule modifyModePoseSchedules(scalar_t currentTime, const Eigen::Vector4d &currentTorsoPose, const FootPoseSchedule& footPoseSchedule, const feet_array_t<vector3_t> &foot_pos, scalar_t startInsertTime = -1, const TargetTrajectories &targetTrajectories=TargetTrajectories());
      ModeSchedule modifyModeWorldPoseSchedules(scalar_t currentTime, const Eigen::Vector4d &currentTorsoPose, const FootPoseSchedule& footPoseSchedule, const feet_array_t<vector3_t> &foot_pos, scalar_t startInsertTime = -1, const TargetTrajectories &targetTrajectories=TargetTrajectories());

      inline scalar_t getCustomGaitEndTime() const
      {
        const auto &eventTimes = modeSchedule_.eventTimes;
        const auto &enableFootSequence = modeSchedule_.enableFootSequence;
        size_t eventIdx = eventTimes.size()-1;
        for(size_t i=eventIdx; i>=0; i--)
        {
          if(enableFootSequence[i]){
           eventIdx = i;
           break;
          }
        }
        return eventTimes[eventIdx];
      }

      inline scalar_t getCustomGaitStartTime() const
      {
        const auto &eventTimes = modeSchedule_.eventTimes;
        const auto &enableFootSequence = modeSchedule_.enableFootSequence;
        const auto &isLastCommandSequence = modeSchedule_.isLastCommandSequence;
        size_t eventIdx = 0;
        for(size_t i=1; i<eventTimes.size(); i++)
        {
          if(enableFootSequence[i] && !isLastCommandSequence[i]){
            eventIdx = i - 1;
            break;
          }
        }
        return eventTimes[eventIdx];
      }

      inline scalar_t getFullBodyGaitEndTime() const
      {
        const auto &eventTimes = modeSchedule_.eventTimes;
        const auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
        size_t eventIdx = eventTimes.size()-1;
        bool find = false;
        for(size_t i=eventIdx; i>0; i--)
        {
          if(enableFullBodySequence[i]){
           eventIdx = i;
           find = true;
           break;
          }
        }
        if(!find)
          return -1;
        return eventTimes[eventIdx];
      }

      inline scalar_t getFullBodyGaitStartTime() const
      {
        const auto &eventTimes = modeSchedule_.eventTimes;
        const auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
        size_t eventIdx = 0;
        bool find = false;
        for(size_t i=1; i<eventTimes.size(); i++)
        {
          if(enableFullBodySequence[i]){
            eventIdx = i - 1;
            find = true;
            break;
          }
        }
        if(!find)
          return -1;
        return eventTimes[eventIdx];
      }

      inline void disableFoot(scalar_t currentTime)
      {
        const auto &eventTimes = modeSchedule_.eventTimes;
        const size_t index = 1 + std::lower_bound(eventTimes.begin(), eventTimes.end(), currentTime) - eventTimes.begin();
        for(size_t i = index; i < modeSchedule_.enableFootSequence.size(); ++i) modeSchedule_.enableFootSequence[i] = false;
        for(size_t i = index; i < modeSchedule_.modeSequence.size(); ++i) modeSchedule_.modeSequence[i] = ModeNumber::SS;
      }

    private:
      /**
       * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
       *
       * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
       * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
       */
      void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

    private:
      ModeSchedule modeSchedule_;
      ModeSchedule currentModeSchedule_;
      ModeSequenceTemplate modeSequenceTemplate_;
      const ModeSequenceTemplate defaultModeSequenceTemplate_;
      scalar_t phaseTransitionStanceTime_;
      std::pair<scalar_t, std::string> gaitTimeName_;
      GaitManager gaitManager_;
      long int total_steps_{0};
      long int stop_step_idx_{-1};
      char prev_left_mode_;
      char prev_right_mode_;
      bool autoGaitEnabled_ = true; 
      std::map<std::string, ModeSequenceTemplate> gaitMap_;
      std::vector<std::string> gaitList_;
      double default_stance_duration_ = 0.4;
      size_t erase_index = -1;
    };

  } // namespace humanoid
} // namespace ocs2
