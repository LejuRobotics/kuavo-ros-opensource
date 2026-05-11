#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace mujoco_sim {

struct ActuatorDynamicsParams {
  double armature_;
  double damping_;
  double frictionLoss_;
  double frictionThreshold_;
  double frictionAlpha_;
  bool isValid_;

  ActuatorDynamicsParams() : isValid_(false) {}

  ActuatorDynamicsParams(double inertia,
                         double damping,
                         double friction,
                         double frictionThreshold,
                         double frictionAlpha)
      : armature_(inertia),
        damping_(damping),
        frictionLoss_(friction),
        frictionThreshold_(frictionThreshold),
        frictionAlpha_(frictionAlpha),
        isValid_(true) {}
};

inline double computeEffectiveVelocity(double velocity, double threshold) {
  return (std::abs(velocity) < threshold) ? 0.0 : velocity;
}

inline double computeFrictionTerm(double dqCmd, double dqMeas,
                                  const ActuatorDynamicsParams& params) {
  const double qSwitch = dqCmd * dqMeas < 0.0 ? -dqCmd : dqCmd;
  const double effectiveVelocity =
      computeEffectiveVelocity(qSwitch, params.frictionThreshold_);
  return params.frictionLoss_ * std::tanh(params.frictionAlpha_ * effectiveVelocity);
}

inline double computeTotalTorque(double tauRbd, double ddq, double dqCmd, double dqMeas,
                                 const ActuatorDynamicsParams& params) {
  const double frictionTerm = computeFrictionTerm(dqCmd, dqMeas, params);
  return tauRbd + params.armature_ * ddq + params.damping_ * dqMeas + frictionTerm;
}

class ActuatorDynamicsCompensator {
 public:
  static constexpr int kCompensationDof = 14;

  ActuatorDynamicsCompensator() {
    if (!loadParamsFromYaml()) {
      loadDefaultParams();
      printParamsTable();
    }
  }

  Eigen::VectorXd compute(const Eigen::VectorXd& tauRbds,
                          const Eigen::VectorXd& ddqs,
                          const Eigen::VectorXd& dqsCmd,
                          const Eigen::VectorXd& dqsMeas) const {
    if (tauRbds.size() != kCompensationDof || ddqs.size() != kCompensationDof ||
        dqsCmd.size() != kCompensationDof || dqsMeas.size() != kCompensationDof) {
      return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd totalTorques(kCompensationDof);
    for (int i = 0; i < kCompensationDof; ++i) {
      totalTorques[i] = computeTotalTorque(tauRbds[i], ddqs[i], dqsCmd[i], dqsMeas[i], params_[i]);
    }
    return totalTorques;
  }

 private:
  bool loadParamsFromYaml() {
    try {
      const std::filesystem::path currentFile(__FILE__);
      const std::filesystem::path configFile =
          currentFile.parent_path().parent_path().parent_path() /
          "config" / "actuator_dynamics_config.yaml";
      if (!std::filesystem::exists(configFile)) {
        std::cerr << "[MujocoActuatorDynamicsCompensator] Config not found: "
                  << configFile << ", using built-in defaults." << std::endl;
        return false;
      }

      YAML::Node config = YAML::LoadFile(configFile.string());
      if (!config["actuator_params"]) {
        std::cerr << "[MujocoActuatorDynamicsCompensator] Missing 'actuator_params' in "
                  << configFile << ", using built-in defaults." << std::endl;
        return false;
      }

      const YAML::Node& paramsNode = config["actuator_params"];
      if (!paramsNode.IsSequence() || paramsNode.size() != kCompensationDof) {
        std::cerr << "[MujocoActuatorDynamicsCompensator] Invalid 'actuator_params' size in "
                  << configFile << ", expected " << kCompensationDof
                  << ", using built-in defaults." << std::endl;
        return false;
      }

      for (int i = 0; i < kCompensationDof; ++i) {
        const YAML::Node& param = paramsNode[i];
        const double armature = param["armature"] ? param["armature"].as<double>() : 0.0;
        const double damping = param["damping"] ? param["damping"].as<double>() : 0.0;
        const double frictionLoss = param["frictionLoss"] ? param["frictionLoss"].as<double>() : 0.0;
        const double frictionThreshold =
            param["frictionThreshold"] ? param["frictionThreshold"].as<double>() : 1e-3;
        const double frictionAlpha =
            param["frictionAlpha"] ? param["frictionAlpha"].as<double>() : 100.0;
        params_[i] = ActuatorDynamicsParams(armature, damping, frictionLoss, frictionThreshold, frictionAlpha);
      }

      std::cout << "[MujocoActuatorDynamicsCompensator] Loaded params from "
                << configFile << std::endl;
      printParamsTable();
      return true;
    } catch (const YAML::Exception& e) {
      std::cerr << "[MujocoActuatorDynamicsCompensator] YAML parsing error: " << e.what()
                << ", using built-in defaults." << std::endl;
      return false;
    } catch (const std::exception& e) {
      std::cerr << "[MujocoActuatorDynamicsCompensator] Config loading error: " << e.what()
                << ", using built-in defaults." << std::endl;
      return false;
    }
  }

  void loadDefaultParams() {
    params_[0] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[1] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[2] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[3] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[4] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
    params_[5] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
    params_[6] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
    params_[7] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[8] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[9] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[10] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.85, 100.0);
    params_[11] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
    params_[12] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
    params_[13] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 0.085, 100.0);
  }

  void printParamsTable() const {
    std::cout << "\n[MujocoActuatorDynamicsCompensator] Parameters Table:" << std::endl;
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::left << std::setw(6) << "Joint"
              << std::setw(12) << "Armature"
              << std::setw(12) << "Damping"
              << std::setw(15) << "FrictionLoss"
              << std::setw(18) << "FrictionThreshold"
              << std::setw(15) << "FrictionAlpha" << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    for (int i = 0; i < kCompensationDof; ++i) {
      std::cout << std::left << std::setw(6) << (i + 1)
                << std::setw(12) << std::fixed << std::setprecision(6) << params_[i].armature_
                << std::setw(12) << std::fixed << std::setprecision(6) << params_[i].damping_
                << std::setw(15) << std::fixed << std::setprecision(6) << params_[i].frictionLoss_
                << std::setw(18) << std::fixed << std::setprecision(6) << params_[i].frictionThreshold_
                << std::setw(15) << std::fixed << std::setprecision(1) << params_[i].frictionAlpha_
                << std::endl;
    }
    std::cout << std::string(120, '=') << std::endl << std::endl;
  }

  std::array<ActuatorDynamicsParams, kCompensationDof> params_;
};

}  // namespace mujoco_sim
