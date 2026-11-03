#pragma once

#include <Eigen/Dense>
#include <vector>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/solve.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "humanoid_interface/gait/MotionPhaseDefinition.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "kuavo_common/common/kuavo_settings.h"

namespace HighlyDynamic
{
  using namespace drake;
  Eigen::MatrixXd solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, int maxIterations = 1000, double tolerance = 1e-9);
  class JointFilter
  {
  public:
    JointFilter(multibody::MultibodyPlant<double> *plant, KuavoSettings *settings, int num_joints = 12, double dt = 0.002, ocs2::humanoid::TopicLogger *logger_ptr = nullptr)
        : num_joints_(num_joints),
          dt_(dt),
          plant_(plant),
          logger_ptr_(logger_ptr),
          settings_ptr_(settings)
    {
      end_frames_name_ = settings->model_settings.end_frames_name;
      plant_context_ = plant_->CreateDefaultContext();
      na_ = plant_->num_actuated_dofs();
      nq_ = plant_->num_positions();
      nv_ = plant_->num_velocities();

      J_old.setZero();
      J_old_1.setZero();

      q_jifen_v.setZero(12);

      q_jifen.setZero(12);
      bkf_old.setZero(30);
      bkf_old_1.setZero(24);
      bkf_old_2.setZero(18);
      joint_v_old.setZero(12);
      P.setIdentity();
      P_old.setIdentity();
      q_est_kf.setZero(12);
      q_est_kf_zy.setZero(12);
      q_est_pos_af_.setZero(12);
      q_est_pos_be_.setZero(12);

      joint_filter_Q_ = settings_ptr_->filter_settings.joint_vel_Q;
      joint_filter_R_ = settings_ptr_->filter_settings.joint_vel_R;
      joint_pos_filter_Q_ = settings_ptr_->filter_settings.joint_pos_Q;
      joint_pos_filter_R_ = settings_ptr_->filter_settings.joint_pos_R;
      l_joint_defor_K = settings_ptr_->filter_settings.l_joint_defor_K;
      r_joint_defor_K = settings_ptr_->filter_settings.r_joint_defor_K;
      joint_pos_filter_Kk_ = solvePosKk();

      velocity_base_pre << 0, 0, 0;
      pose_base_pre << 0, 0, 0.78;
      angular_velocity_base_pre << 0, 0, 0;
    };

    Eigen::MatrixXd solvePosKk();
    inline int mode2state(const size_t &mode)
    {
      // touch_down_state=0 双脚触地 1左脚触地 2右脚触地 3 腾空
      ocs2::humanoid::contact_flag_t stanceleg = ocs2::humanoid::modeNumber2StanceLeg(mode);
      bool lf_touch = std::any_of(stanceleg.begin(), stanceleg.begin() + stanceleg.size() / 2, [](bool val)
                                  { return val; });
      bool rf_touch = std::any_of(stanceleg.begin() + stanceleg.size() / 2, stanceleg.end(), [](bool val)
                                  { return val; });
      if (lf_touch && rf_touch)
      {
        return 0;
      }
      else if (lf_touch)
      {
        return 1;
      }
      else if (rf_touch)
      {
        return 2;
      }
      else
      {
        return 3;
      }
    }
    inline Eigen::VectorXd rbdState2drakeQV(const Eigen::VectorXd &rbdState)
    {
      size_t generalizedCoordinatesNum = rbdState.size() / 2;
      Eigen::Vector3d pos = rbdState.segment<3>(3);
      Eigen::Vector3d vel = rbdState.segment<3>(generalizedCoordinatesNum + 3);
      Eigen::Vector3d zyx = rbdState.segment<3>(0);
      Eigen::Vector3d angularVel = rbdState.segment<3>(generalizedCoordinatesNum);
      Eigen::VectorXd jointPos = rbdState.segment(6, na_);
      Eigen::VectorXd jointVel = rbdState.segment(generalizedCoordinatesNum + 6, na_);
      Eigen::Quaterniond quaternion;
      quaternion = Eigen::AngleAxisd(zyx[2], Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(zyx[1], Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(zyx[0], Eigen::Vector3d::UnitX());
      Eigen::Vector4d quatd(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
      Eigen::VectorXd qv(nq_ + nv_);
      qv << quatd, pos, jointPos, angularVel, vel, jointVel;
      return qv;
    }
    inline void drakeQV2rbdState(const Eigen::VectorXd &qv, Eigen::VectorXd &rbdState)
    {
      size_t generalizedCoordinatesNum = rbdState.size() / 2;

      // 提取四元数并转换为ZYX欧拉角
      Eigen::Quaterniond quaternion(qv[0], qv[1], qv[2], qv[3]);
      Eigen::Vector3d zyx = quaternion.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

      // 提取位置和速度向量
      Eigen::Vector3d pos = qv.segment<3>(4);
      Eigen::Vector3d vel = qv.segment<3>(nq_ + 3);
      Eigen::Vector3d angularVel = qv.segment<3>(nq_);
      Eigen::VectorXd jointPos = qv.segment(4 + 3, na_);
      Eigen::VectorXd jointVel = qv.tail(na_);

      // 组合为rbdState向量
      rbdState.segment<3>(0) = zyx;
      rbdState.segment<3>(3) = pos;
      rbdState.segment(6, na_) = jointPos;
      rbdState.segment<3>(generalizedCoordinatesNum) = angularVel;
      rbdState.segment<3>(3 + generalizedCoordinatesNum) = vel;
      rbdState.segment(6 + generalizedCoordinatesNum, na_) = jointVel;
    }
    /*
        @brief 进行滤波更新关节位置和速度测量值
        @param rbdState 机器人状态
        @param joint_pos 测量的关节位置，update之后更新
        @param joint_vel 测量的关节速度，update之后更新
        @param tau_input 输入力矩
        @param mode ocs2_mode
    */
    void update(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_pos, Eigen::VectorXd &joint_vel, const Eigen::VectorXd &joint_current, const Eigen::VectorXd &tau_input, const size_t &mode);
    void jointVelocityFilter(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_vel, const Eigen::VectorXd &tau_input, const size_t &mode);
    void jointPositionFilter(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_pos, const Eigen::VectorXd &joint_vel, const Eigen::VectorXd &tau_input, const size_t &mode);

    inline void logPublishVector(const std::string &channel, const Eigen::VectorXd &vec) const
    {
      if (logger_ptr_ != nullptr)
      {
        logger_ptr_->publishVector(channel, vec);
      }
    }

  private:
    int touch_down_state_prev_;
    int num_joints_;
    double dt_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    Eigen::VectorXd joint_filter_Q_;
    Eigen::VectorXd joint_filter_R_;
    Eigen::Vector3d velocity_base_pre;
    Eigen::Vector3d pose_base_pre;
    Eigen::Vector3d angular_velocity_base_pre;
    Eigen::Vector2d joint_pos_hip_offset_;
    double angular_velocity_hip_;

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;

    mutable Eigen::Matrix<double, 12, 18> J_old;
    mutable Eigen::Matrix<double, 6, 18> J_old_1;
    mutable Eigen::VectorXd q_jifen_v;
    mutable Eigen::VectorXd q_jifen_v_old;
    mutable Eigen::VectorXd q_jifen;
    mutable Eigen::VectorXd bkf_old;
    mutable Eigen::VectorXd bkf_old_1;
    mutable Eigen::VectorXd bkf_old_2;
    mutable Eigen::VectorXd joint_v_old;
    mutable Eigen::Matrix<double, 12, 12> P;
    mutable Eigen::Matrix<double, 12, 12> P_old;
    mutable Eigen::VectorXd q_est_kf;
    mutable Eigen::VectorXd q_est_kf_zy;

    // position filter
    Eigen::VectorXd joint_pos_filter_Q_;
    Eigen::VectorXd joint_pos_filter_R_;
    Eigen::VectorXd q_est_pos_be_;
    Eigen::VectorXd q_est_pos_af_; // after update
    double l_joint_defor_K;
    double r_joint_defor_K;

    Eigen::Matrix<double, 12, 12> joint_pos_filter_Kk_;

    std::vector<std::string> end_frames_name_;
    ocs2::humanoid::TopicLogger *logger_ptr_{nullptr};
    KuavoSettings *settings_ptr_{nullptr};
  };

}; // namespace HighlyDynamic
