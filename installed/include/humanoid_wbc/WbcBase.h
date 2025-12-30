//
// Created by qiayuan on 2022/7/1.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include "humanoid_interface/common/TopicLogger.h"

namespace ocs2
{
  namespace humanoid
  {
    // using namespace ocs2;
    // using namespace humanoid;

    // Ax -b = w
    // Dx - f <= v
    // w -> 0, v -> 0
    struct Wbc_weight_t {
    scalar_t weightComPos_;
    scalar_t weightBaseAccel_;// useless
    scalar_t weightBaseAccelXY_;
    scalar_t weightBaseAccelHeight_;
    scalar_t weightBaseAccelAngular_;
    scalar_t weightContactForce_;
    scalar_t weightStanceLeg_;
    scalar_t weightSwingLeg_;
    scalar_t weightArmAccel_;
    };

    // Decision Variables: x = [\dot u^T, 3*F(3)^T, \tau^T]^T , \dot u in ocal frame
    class WbcBase
    {
      using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
      using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
      using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

    public:
      WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
              const PinocchioEndEffectorKinematics &eeKinematics);

      virtual void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real);

      virtual vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                              size_t mode, scalar_t period, bool mpc_update = false);
      void setCmdBodyPosVel(const vector_t &cmd_body_pos, const vector_t &cmd_body_vel)
      {
        cmd_body_pos_ = cmd_body_pos;
        cmd_body_vel_ = cmd_body_vel;
      }

      void setEarlyLateContact(const std::array<contact_flag_t, 2> &early_late_contact)
      {
        earlyLatecontact_ = early_late_contact;
      }

      void setFootPosVelAccDesired(const std::array<vector_t, 3> &footPosVelAccDesired)
      {
        footPosVelAccDesired_ = footPosVelAccDesired;
      }
      void setJointAccDesired(const vector_t &jointAccDesired)
      {
        jointAccDesired_ = jointAccDesired;
      }
      void setKpKd(scalar_t swingKp, scalar_t swingKd)
      {
        swingKp_ = swingKp;
        swingKd_ = swingKd;
      }
      size_t getContactForceSize()
      {
        return contact_force_size_;
      }
      void setStanceMode(bool stance_mode)
      {
        stance_mode_ = stance_mode;
      }
      void setPullUpState(bool pull_up_state)
      {
        pull_up_state_ = pull_up_state;
      }
      void setArmNums(int arm_nums)
      {
        arm_nums_ = arm_nums;
      }

      inline Eigen::Vector3d getR() const { return r; }
      inline Eigen::Vector3d getRd() const { return rd; }
      inline Eigen::Vector3d getRdd() const { return rdd; }
      inline Eigen::Vector3d getRDes() const { return r_des; }
      inline Eigen::Vector3d getRdDes() const { return rd_des; }
      inline Vector6 getPoseDes() const { return basePoseDes_; }
      inline Vector6 getVelocityDes() const { return baseVelocityDes_; }
      inline Vector6 getAccelerationDes() const { return baseAccelerationDes_; }
      inline vector_t getQDes() const { return q_des_; }
      inline vector_t getQdDes() const { return qd_des_; }
      inline vector_t getQ() const { return qMeasured_; }
      inline vector_t getQd() const { return vMeasured_; }
      inline vector_t getEefVel() const { return j_ * vMeasured_; }
      inline vector_t getEefAcc() const { return j_ * vd_measured_ + dj_ * vMeasured_; }
      inline vector_t getEefVelDes() const { return j_des_ * qd_des_; }
      inline vector_t getEefAccDes() const { return j_des_ * vd_des_fd_ + dj_des_ * qd_des_; }
      inline vector_t getEefAccWbcOutput(const vector_t &wbc_planned_body_acc) const { return j_ * vd_measured_ + dj_ * wbc_planned_body_acc; }
      inline void updateVd(vector_t &vd)
      {
        vd_measured_.segment(6, 12) = vd.segment(0, 12);
        // vd_measured_ = ;
        std::cout << "vd.size: " << vd.size() << std::endl;
        std::cout << "vd_mes.size: " << vd_measured_.size() << std::endl;
        std::cout << "over: " << std::endl;
      }
      inline void setHalfBodyMode(bool half_body_mode)
      {
        half_body_mode_ = half_body_mode;
      }
    protected:
      TopicLogger* topic_logger_;
      scalar_t period_;
      size_t mode_;
      size_t after_change_index_{0};
      bool mpc_updated_ = true;
      vector_t jointAccelerations_;
      vector_t inputDesired_prev_;
      std::vector<vector3_t> eePosMeasured_;
      std::vector<vector3_t> eeVelMeasured_;
      std::vector<vector3_t> eePosDesired_ ;
      std::vector<vector3_t> eeVelDesired_ ;
      std::vector<vector3_t> velDesired_prev_;
      std::vector<vector3_t> eeAccDesired_;
      void updateMeasured(const vector_t &rbdStateMeasured);
      void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);
      size_t getNumDecisionVars() const
      {
        return numDecisionVars_;
      }
      void preComputation();
      Task formulateFloatingBaseEomTask(const vector_t &inputDesired);
      Task formulateTorqueLimitsTask();
      Task formulateNoContactMotionTask();
      Task formulateFrictionConeTask();
      Task formulateBaseHeightMotionTask();
      Task formulateBaseAngularMotionTask();
      Task formulateBaseXYLinearAccelTask();
      Task formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
      Task formulateSwingLegTask();
      Task formulateCenterOfMassTask(const vector_t &stateDesired,const vector_t &inputDesired,scalar_t period);
      Task formulateContactForceTask(const vector_t &inputDesired) const;
      Task formulateArmJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period);
      Task formulateStandUpJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period);

      void compensateFriction(vector_t &x);

      size_t numDecisionVars_;
      PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
      CentroidalModelInfo info_;

      std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
      CentroidalModelPinocchioMapping mapping_;
      CentroidalModelRbdConversions rbdConversions_;

      vector_t qMeasured_, vMeasured_, inputLast_;
      matrix_t j_, dj_;
      matrix_t j_hand_, dj_hand_;
      Matrix6x base_j_, base_dj_;
      contact_flag_t contactFlag_{};
      size_t numContacts_{};

      vector_t torqueLimits_;
      matrix_t Wbc_rdd_K_;
      matrix_t Wbc_rdd_K_stance_;
      matrix_t Wbc_rdd_K_walk_;
      Wbc_weight_t wbc_weight_stance_;
      Wbc_weight_t wbc_weight_walk_;
      scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{}, stanceKp_{}, stanceKd_{0};
      vector3_t swingKp3d_ = vector3_t::Zero(), swingKd3d_ = vector3_t::Zero();
      scalar_t baseHeightKp_{}, baseHeightKd_{};
      scalar_t baseAngularKp_{}, baseAngularKd_{};
      vector_t armJointKp_, armJointKd_;
      scalar_t standUp_legKp_{}, standUp_legKd_{}, standUp_armKp_{}, standUp_armKd_{};
      vector3_t baseAngular3dKp_, baseAngular3dKd_;

      vector_t cmd_body_pos_;
      vector_t cmd_body_vel_;
      scalar_t com_kp_{}, com_kd_{};
      Vector6 basePoseDes_, baseVelocityDes_, baseAccelerationDes_;

      std::array<contact_flag_t, 2> earlyLatecontact_;

      std::vector<vector3_t> footPosDesired_, footVelDesired_;
      std::array<vector_t, 3> footPosVelAccDesired_;

      vector_t jointAccDesired_;
      size_t contact_force_size_ = 0;
      bool stance_mode_ = false;
      Eigen::Vector3d r_des, rd_des,rdd_des, r, rd, rdd;
      Eigen::Vector3d Jq_dq_com_;
      Eigen::MatrixXd Js_v_Ccm_,A,ADot;
      vector_t q_mes_, qd_mes_, q_des_, qd_des_;
      vector_t vd_measured_ = vector_t::Zero(18);
      vector_t last_v_measured_ = vector_t::Zero(18);
      vector_t last_v_des_ = vector_t::Zero(18);
      vector_t vd_des_fd_ = vector_t::Zero(18);
      matrix_t j_des_, dj_des_;

      size_t arm_nums_{};
      matrix3_t rotationYawBaseMeasuredToWorld_;
      bool half_body_mode_ = false;
      bool pull_up_state_ = false;
    };

  } // namespace humanoid
} // namespace ocs2
