#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "kuavo_common/common/json_config_reader.hpp"
#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/seat_config.h"  // SeatConfig = nlohmann::json

// x86 always builds with Drake (eee7f6b8 behavior); aarch64 stub omits KUAVO_HAS_DRAKE
#ifndef KUAVO_BUILD_FOR_AARCH64
#ifndef KUAVO_HAS_DRAKE
#define KUAVO_HAS_DRAKE 1
#endif
#endif

#if defined(KUAVO_HAS_DRAKE) && KUAVO_HAS_DRAKE
#include "humanoid_interface_drake/ankle_solver/arm_ankle_solver.h"
#include "humanoid_interface_drake/ankle_solver/arm_joint_controller.h"
#include "humanoid_interface_drake/ankle_solver/arm_tendon_controller.h"
#include "humanoid_interface_drake/planner/plantIK.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"
#endif

namespace HighlyDynamic
{

using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;

class HumanoidInterfaceDrake
{
public:
  static HumanoidInterfaceDrake& getInstance(RobotVersion rb_version, bool real, double dt = 0.002);
  static HumanoidInterfaceDrake* getInstancePtr(RobotVersion rb_version, bool real, double dt = 0.002);

  virtual ~HumanoidInterfaceDrake();

  HumanoidInterfaceDrake(const HumanoidInterfaceDrake&) = delete;
  HumanoidInterfaceDrake& operator=(const HumanoidInterfaceDrake&) = delete;

  inline const vector_t& getInitialState() const { return initial_state_; }
  inline const vector_t& getSquatInitialState() const { return squat_initial_state_; }
  inline const vector_t& getSitInitialState() const { return sit_initial_state_; }
  /** MPC sit_down 轨迹时长（秒），来自 kuavo.json sit_down_duration_seconds，缺省 2.0 */
  inline double getSitDownDurationSeconds() const { return sit_down_duration_seconds_; }
  /** 座椅共享配置（seat_config_v5.json），v5 专有。返回 nullptr 表示非 v5。 */
  const kuavo_common::SeatConfig* getSeatConfig() const;
  inline const vector_t& getDefaultJointState() const { return default_joint_state_; }
  inline const vector_t& getDrakeState() const { return q_initial_; }
  inline const vector_t& getDrakeSquatState() const { return q_squat_initial_; }
  inline double getIntialHeight() const {
    return initial_state_.size() > 8 ? initial_state_[8] : 0.0;
  }
  inline double getTimeStep() const { return dt_; }
  inline const KuavoSettings& getKuavoSettings() const { return kuavo_settings_; }
  JSONConfigReader* getRobotConfig() const { return robot_config_; }
  RobotVersion getRobotVersion() const { return rb_version_; }

#if defined(KUAVO_HAS_DRAKE) && KUAVO_HAS_DRAKE
  inline double getPlantMass() const { return plant_ptr_->CalcTotalMass(*plant_context_); }
  inline double getPlantWithArmMass() const { return plant_with_arm_ptr_->CalcTotalMass(*plant_with_arm_context_); }
  std::pair<drake::multibody::MultibodyPlant<double>&, drake::systems::Context<double>&> getPlantAndContext() const {
    return {*plant_ptr_, *plant_context_};
  }
  std::pair<drake::multibody::MultibodyPlant<double>&, drake::systems::Context<double>&> getPlantWithArmAndContext()
      const {
    return {*plant_with_arm_ptr_, *plant_with_arm_context_};
  }
  std::pair<lower_leg::ArmAnkleSolverSystem&, lower_leg::ArmAnkleSolverSystem&> getFootAnkleSolvers() const {
    return {*left_foot_ankle_solver_ptr_, *right_foot_ankle_solver_ptr_};
  }
  std::pair<lower_leg::ArmJointController&, lower_leg::ArmJointController&> getFootAnkleJointControllers() const {
    return {*left_foot_ankle_jcont_ptr_, *right_foot_ankle_jcont_ptr_};
  }
  std::pair<lower_leg::ArmTendonController&, lower_leg::ArmTendonController&> getFootAnkleTendonControllers() const {
    return {*left_foot_ankle_tendon_ptr_, *right_foot_ankle_tendon_ptr_};
  }
  std::pair<lower_leg::ArmAnkleSolverSystem&, lower_leg::ArmAnkleSolverSystem&> getArmAnkleSolvers() const {
    return {*left_arm_ankle_solver_ptr_, *right_arm_ankle_solver_ptr_};
  }
  std::pair<lower_leg::ArmJointController&, lower_leg::ArmJointController&> getArmAnkleJointControllers() const {
    return {*left_arm_ankle_jcont_ptr_, *right_arm_ankle_jcont_ptr_};
  }
  std::pair<lower_leg::ArmTendonController&, lower_leg::ArmTendonController&> getArmAnkleTendonControllers() const {
    return {*left_arm_ankle_tendon_ptr_, *right_arm_ankle_tendon_ptr_};
  }
  std::pair<drake::multibody::MultibodyPlant<double>&, drake::multibody::MultibodyPlant<double>&> getFootAnklePlants()
      const {
    return {*left_foot_ankle_plant_ptr_, *right_foot_ankle_plant_ptr_};
  }
  std::pair<drake::multibody::MultibodyPlant<double>&, drake::multibody::MultibodyPlant<double>&> getArmAnklePlants()
      const {
    return {*left_arm_ankle_plant_ptr_, *right_arm_ankle_plant_ptr_};
  }
  Eigen::VectorXd calInitialState(drake::multibody::MultibodyPlant<double>* plant,
                                  drake::systems::Context<double>* plant_context);
  void calcSquatState(drake::multibody::MultibodyPlant<double>* plant, drake::systems::Context<double>* plant_context);
  void calcSitState(drake::multibody::MultibodyPlant<double>* plant, drake::systems::Context<double>* plant_context);
#else
  inline double getPlantMass() const {
    throw std::runtime_error("HumanoidInterfaceDrake: getPlantMass requires Drake (not available on this platform)");
  }
  inline double getPlantWithArmMass() const {
    throw std::runtime_error(
        "HumanoidInterfaceDrake: getPlantWithArmMass requires Drake (not available on this platform)");
  }
#endif

private:
  HumanoidInterfaceDrake(RobotVersion rb_version, bool real, double dt = 0.001);
  void buildFallbackInitialState();

#if defined(KUAVO_HAS_DRAKE) && KUAVO_HAS_DRAKE
  void buildMultibodyPlant(bool real);
  void buildAnkleSolversMultibodyPlant(bool is_parallel_arm);
  void buildAnkleSolvers(bool is_parallel_arm);
#endif

  static std::shared_ptr<HumanoidInterfaceDrake> instance;
  RobotVersion rb_version_;
  double dt_;
  vector_t initial_state_, default_joint_state_, q_initial_, squat_initial_state_, q_squat_initial_,
      sit_initial_state_;
  KuavoSettings kuavo_settings_;
  JSONConfigReader* robot_config_;
  double sit_down_duration_seconds_{2.0};

#if defined(KUAVO_HAS_DRAKE) && KUAVO_HAS_DRAKE
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_with_arm_ptr_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<drake::systems::Context<double>> plant_with_arm_context_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  std::unique_ptr<drake::systems::lcm::LcmInterfaceSystem> lcm_ptr_;
  std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr_;
  drake::systems::DiagramBuilder<double> builder_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> left_foot_ankle_plant_ptr_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> right_foot_ankle_plant_ptr_;
  std::unique_ptr<lower_leg::ArmAnkleSolverSystem> left_foot_ankle_solver_ptr_;
  std::unique_ptr<lower_leg::ArmAnkleSolverSystem> right_foot_ankle_solver_ptr_;
  std::unique_ptr<lower_leg::ArmJointController> left_foot_ankle_jcont_ptr_;
  std::unique_ptr<lower_leg::ArmJointController> right_foot_ankle_jcont_ptr_;
  std::unique_ptr<lower_leg::ArmTendonController> left_foot_ankle_tendon_ptr_;
  std::unique_ptr<lower_leg::ArmTendonController> right_foot_ankle_tendon_ptr_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> left_arm_ankle_plant_ptr_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> right_arm_ankle_plant_ptr_;
  std::unique_ptr<lower_leg::ArmAnkleSolverSystem> left_arm_ankle_solver_ptr_;
  std::unique_ptr<lower_leg::ArmAnkleSolverSystem> right_arm_ankle_solver_ptr_;
  std::unique_ptr<lower_leg::ArmJointController> left_arm_ankle_jcont_ptr_;
  std::unique_ptr<lower_leg::ArmJointController> right_arm_ankle_jcont_ptr_;
  std::unique_ptr<lower_leg::ArmTendonController> left_arm_ankle_tendon_ptr_;
  std::unique_ptr<lower_leg::ArmTendonController> right_arm_ankle_tendon_ptr_;
#endif
};

}  // namespace HighlyDynamic
