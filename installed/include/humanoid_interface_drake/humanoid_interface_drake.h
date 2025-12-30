#pragma once
// #include "humanoid_interface_drake/common/json.hpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "kuavo_common/common/json_config_reader.hpp"
#include "humanoid_interface_drake/ankle_solver/arm_ankle_solver.h"
#include "humanoid_interface_drake/ankle_solver/arm_joint_controller.h"
#include "humanoid_interface_drake/ankle_solver/arm_tendon_controller.h"
#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/common/utils.h"
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

namespace HighlyDynamic
{

    using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    class HumanoidInterfaceDrake
    {
    public:
        static HumanoidInterfaceDrake &getInstance(RobotVersion robot_version, bool real, double dt = 0.002);
        static HumanoidInterfaceDrake *getInstancePtr(RobotVersion robot_version, bool real, double dt = 0.002);
        
        virtual ~HumanoidInterfaceDrake();

        HumanoidInterfaceDrake(const HumanoidInterfaceDrake &) = delete;
        HumanoidInterfaceDrake &operator=(const HumanoidInterfaceDrake &) = delete;
        inline const vector_t &getInitialState() const { return initial_state_; }
        inline const vector_t &getSquatInitialState() const { return squat_initial_state_; }
        inline const vector_t &getDefaultJointState() const { return default_joint_state_; }
        inline const vector_t &getDrakeState() const { return q_initial_; }
        inline const vector_t &getDrakeSquatState() const { return q_squat_initial_; }
        inline double getIntialHeight() const { return initial_state_[8]; }
        inline double getPlantMass() const { return plant_ptr_->CalcTotalMass(*plant_context_); }
        inline double getPlantWithArmMass() const { return plant_with_arm_ptr_->CalcTotalMass(*plant_with_arm_context_); }
        inline double getTimeStep() const { return dt_; }
        inline const KuavoSettings &getKuavoSettings() const { return kuavo_settings_; }
        std::pair<drake::multibody::MultibodyPlant<double> &, drake::systems::Context<double> &> getPlantAndContext() const
        {
            return {*plant_ptr_, *plant_context_};
        }
        std::pair<drake::multibody::MultibodyPlant<double> &, drake::systems::Context<double> &> getPlantWithArmAndContext() const
        {
            return {*plant_with_arm_ptr_, *plant_with_arm_context_};
        }
        std::pair<lower_leg::ArmAnkleSolverSystem &, lower_leg::ArmAnkleSolverSystem &> getFootAnkleSolvers() const
        {
            return {*left_foot_ankle_solver_ptr_, *right_foot_ankle_solver_ptr_};
        }
        std::pair<lower_leg::ArmJointController &, lower_leg::ArmJointController &> getFootAnkleJointControllers() const
        {
            return {*left_foot_ankle_jcont_ptr_, *right_foot_ankle_jcont_ptr_};
        }
        std::pair<lower_leg::ArmTendonController &, lower_leg::ArmTendonController &> getFootAnkleTendonControllers() const
        {
            return {*left_foot_ankle_tendon_ptr_, *right_foot_ankle_tendon_ptr_};
        }

        std::pair<lower_leg::ArmAnkleSolverSystem &, lower_leg::ArmAnkleSolverSystem &> getArmAnkleSolvers() const
        {
            return {*left_arm_ankle_solver_ptr_, *right_arm_ankle_solver_ptr_};
        }
        std::pair<lower_leg::ArmJointController &, lower_leg::ArmJointController &> getArmAnkleJointControllers() const
        {
            return {*left_arm_ankle_jcont_ptr_, *right_arm_ankle_jcont_ptr_};
        }
        std::pair<lower_leg::ArmTendonController &, lower_leg::ArmTendonController &> getArmAnkleTendonControllers() const
        {
            return {*left_arm_ankle_tendon_ptr_, *right_arm_ankle_tendon_ptr_};
        }
        std::pair<drake::multibody::MultibodyPlant<double> &, drake::multibody::MultibodyPlant<double> &> getFootAnklePlants() const
        {
            return {*left_foot_ankle_plant_ptr_, *right_foot_ankle_plant_ptr_};
        }
        std::pair<drake::multibody::MultibodyPlant<double> &, drake::multibody::MultibodyPlant<double> &> getArmAnklePlants() const
        {
            return {*left_arm_ankle_plant_ptr_, *right_arm_ankle_plant_ptr_};
        }
        JSONConfigReader *getRobotConfig() const { return robot_config_; }

        Eigen::VectorXd calInitialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context);
        void calcSquatState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context);
        
        RobotVersion getRobotVersion() const { return robot_version_; }
    private:
        // 单例模式，私有构造函数
        HumanoidInterfaceDrake(RobotVersion robot_version, bool real, double dt = 0.001);
        void buildMultibodyPlant(bool real);
        void buildAnkleSolversMultibodyPlant(bool is_parallel_arm);
        void buildAnkleSolvers(bool is_parallel_arm);
        static std::shared_ptr<HumanoidInterfaceDrake> instance;
        RobotVersion robot_version_;

    private:
        double dt_;
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr_;
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_with_arm_ptr_;
        std::unique_ptr<drake::systems::Context<double>> plant_context_;
        std::unique_ptr<drake::systems::Context<double>> plant_with_arm_context_;

        std::unique_ptr<drake::systems::Diagram<double>> diagram_;
        std::unique_ptr<drake::systems::Context<double>> diagram_context_;

        std::unique_ptr<drake::systems::lcm::LcmInterfaceSystem> lcm_ptr_;
        std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr_;
        drake::systems::DiagramBuilder<double> builder_;

        vector_t initial_state_, default_joint_state_, q_initial_, squat_initial_state_, q_squat_initial_;
        KuavoSettings kuavo_settings_;
        JSONConfigReader *robot_config_;

        // ankle solvers
        //  foot
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> left_foot_ankle_plant_ptr_;
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> right_foot_ankle_plant_ptr_;
        std::unique_ptr<lower_leg::ArmAnkleSolverSystem> left_foot_ankle_solver_ptr_;
        std::unique_ptr<lower_leg::ArmAnkleSolverSystem> right_foot_ankle_solver_ptr_;
        std::unique_ptr<lower_leg::ArmJointController> left_foot_ankle_jcont_ptr_;
        std::unique_ptr<lower_leg::ArmJointController> right_foot_ankle_jcont_ptr_;
        std::unique_ptr<lower_leg::ArmTendonController> left_foot_ankle_tendon_ptr_;
        std::unique_ptr<lower_leg::ArmTendonController> right_foot_ankle_tendon_ptr_;

        // arm
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> left_arm_ankle_plant_ptr_;
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> right_arm_ankle_plant_ptr_;
        std::unique_ptr<lower_leg::ArmAnkleSolverSystem> left_arm_ankle_solver_ptr_;
        std::unique_ptr<lower_leg::ArmAnkleSolverSystem> right_arm_ankle_solver_ptr_;
        std::unique_ptr<lower_leg::ArmJointController> left_arm_ankle_jcont_ptr_;
        std::unique_ptr<lower_leg::ArmJointController> right_arm_ankle_jcont_ptr_;
        std::unique_ptr<lower_leg::ArmTendonController> left_arm_ankle_tendon_ptr_;
        std::unique_ptr<lower_leg::ArmTendonController> right_arm_ankle_tendon_ptr_;
    };
}
