#pragma once
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <vector>
#include <eigen3/Eigen/Dense>
#include "kuavo_estimation/base_filter/InEKF.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "kuavo_common/common/kuavo_settings.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "humanoid_estimation/StateEstimateBase.h"
#include "kuavo_common/common/json_config_reader.hpp"
#define END_FRAMES_TORSO 0
#define END_FRAMES_L_FOOT_SOLE 1
#define END_FRAMES_R_FOOT_SOLE 2

namespace ocs2
{
  namespace humanoid
  {
    // using namespace HighlyDynamic;
    // using namespace drake;
    class InEkfBaseFilter : public StateEstimateBase
    {
    public:
      InEkfBaseFilter(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                      const PinocchioEndEffectorKinematics &eeKinematics,
                      HighlyDynamic::HumanoidInterfaceDrake *drake_interface_ptr,
                      double dt = 0.002,
                      ocs2::humanoid::TopicLogger *logger_ptr = nullptr);
      // InEkfBaseFilter(multibody::MultibodyPlant<double> *plant, KuavoSettings *settings, double dt = 0.002, ocs2::humanoid::TopicLogger *logger_ptr = nullptr);
      void set_intial_state(const vector_t &state) override;

      void UpdateQvInekf(Eigen::VectorXd &rbdState, const size_t &mode);
      void UpdateContactState(bool contact_des, double diff_z, Eigen::Vector2d &contact_com_z_diff, bool &contact_est, int contact_id, std::vector<std::pair<int, bool>> &contacts);
      inline void logPublishVector(const std::string &channel, const Eigen::VectorXd &vec) const
      {
        if (logger_ptr_ != nullptr)
        {
          logger_ptr_->publishVector(channel, vec);
        }
      }
      vector_t update(const ros::Time &time, const ros::Duration &period) override;

    protected:
      inekf::RobotState robot_state;
      inekf::NoiseParams noise_params;
      inekf::InEKF inekf_filter;
      Eigen::MatrixXd imu_measurement_prev;
      Eigen::VectorXd kinematics_noise;
      Eigen::VectorXd stand_kinematics_noise;
      ocs2::humanoid::TopicLogger *logger_ptr_{nullptr};
      HighlyDynamic::HumanoidInterfaceDrake *drake_interface_ptr_{nullptr};
      double dt_{0.002};
      HighlyDynamic::JSONConfigReader *robot_config_;
      // drake
      drake::multibody::MultibodyPlant<double> *plant_;
      std::unique_ptr<drake::systems::Context<double>> plant_context_;
      int32_t na_;
      int32_t nq_;
      int32_t nv_;
      HighlyDynamic::KuavoSettings kuavo_settings_;
      Eigen::Vector2d contact_state_prev_des;
      Eigen::Vector2d contact_state_prev_est;
      Eigen::Vector2d contact_com_z_diff;
      bool use_contact_estimator;
      std::vector<std::string> end_frames_name_;
    };
  }; // namespace humanoid
}; // namespace ocs2
