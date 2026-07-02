#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>
#include <string>

#include "kuavo_solver/ankle/ankle_solver.h"
#include "kuavo_solver/arm/arm_solver.h"
#include "kuavo_solver/waist/waist_solver.h"
#include "kuavo_solver/ankle/axis_offset_ankle_solver.h"
#include "kuavo_solver/arm/parallel_linear_arm_solver.h"
#include "kuavo_solver/waist/parallel_rotate_waist_solver.h"

using namespace kuavo_solver;

namespace py = pybind11;

namespace {

// 必须与 src/ankle/ankle_solver.cpp PickAnkleJoint4 / PickAnkleMotor4 一致：
// joint  [L_pitch,L_roll,R_pitch,R_roll] ↔ leg indices (4,5,10,11)
// motor  内部 [p₀,p₁,p₂,p₃] ↔ leg：左交叉 p₀→leg[5], p₁→leg[4], p₂→leg[10], p₃→leg[11]

Eigen::VectorXd MakeQ12FromQ4(const Eigen::Vector4d& q4) {
  Eigen::VectorXd q12 = Eigen::VectorXd::Zero(12);
  q12(4) = q4(0);
  q12(5) = q4(1);
  q12(10) = q4(2);
  q12(11) = q4(3);
  return q12;
}

Eigen::VectorXd MakeP12FromP4(const Eigen::Vector4d& p4) {
  Eigen::VectorXd p12 = Eigen::VectorXd::Zero(12);
  p12(5) = p4(0);
  p12(4) = p4(1);
  p12(10) = p4(2);
  p12(11) = p4(3);
  return p12;
}

Eigen::Vector4d ExtractQ4FromQ12(const Eigen::VectorXd& q12) {
  Eigen::Vector4d q4;
  q4 << q12(4), q12(5), q12(10), q12(11);
  return q4;
}

Eigen::Vector4d ExtractP4FromP12(const Eigen::VectorXd& p12) {
  Eigen::Vector4d p4;
  p4 << p12(5), p12(4), p12(10), p12(11);
  return p4;
}

}  // namespace

class AxisOffsetAnkleSolverPy {
 public:
  explicit AxisOffsetAnkleSolverPy(const kuavo_solver::AxisOffsetAnkleParams& params) : solver_(params) {}

  Eigen::Vector4d joint_to_motor_position(const Eigen::Vector4d& q4) {
    Eigen::VectorXd q(4);
    q << q4(0), q4(1), q4(2), q4(3);
    const Eigen::VectorXd p = solver_.joint_to_motor_position(q);
    Eigen::Vector4d out;
    out << p(0), p(1), p(2), p(3);
    return out;
  }

  Eigen::Vector4d motor_to_joint_position(const Eigen::Vector4d& p4) {
    Eigen::VectorXd p(4);
    p << p4(0), p4(1), p4(2), p4(3);
    const Eigen::VectorXd q = solver_.motor_to_joint_position(p);
    Eigen::Vector4d out;
    out << q(0), q(1), q(2), q(3);
    return out;
  }

  Eigen::Vector4d joint_to_motor_velocity(const Eigen::Vector4d& q4,
                                          const Eigen::Vector4d& p4,
                                          const Eigen::Vector4d& dq4) {
    Eigen::VectorXd q(4), p(4), dq(4);
    q << q4(0), q4(1), q4(2), q4(3);
    p << p4(0), p4(1), p4(2), p4(3);
    dq << dq4(0), dq4(1), dq4(2), dq4(3);
    const Eigen::VectorXd dp = solver_.joint_to_motor_velocity(q, p, dq);
    Eigen::Vector4d out;
    out << dp(0), dp(1), dp(2), dp(3);
    return out;
  }

  Eigen::Vector4d motor_to_joint_velocity(const Eigen::Vector4d& q4,
                                          const Eigen::Vector4d& p4,
                                          const Eigen::Vector4d& dp4) {
    Eigen::VectorXd q(4), p(4), dp(4);
    q << q4(0), q4(1), q4(2), q4(3);
    p << p4(0), p4(1), p4(2), p4(3);
    dp << dp4(0), dp4(1), dp4(2), dp4(3);
    const Eigen::VectorXd dq = solver_.motor_to_joint_velocity(q, p, dp);
    Eigen::Vector4d out;
    out << dq(0), dq(1), dq(2), dq(3);
    return out;
  }

  Eigen::Vector4d joint_to_motor_current(const Eigen::Vector4d& q4,
                                         const Eigen::Vector4d& p4,
                                         const Eigen::Vector4d& tau4) {
    Eigen::VectorXd q(4), p(4), tau(4);
    q << q4(0), q4(1), q4(2), q4(3);
    p << p4(0), p4(1), p4(2), p4(3);
    tau << tau4(0), tau4(1), tau4(2), tau4(3);
    const Eigen::VectorXd i = solver_.joint_to_motor_current(q, p, tau);
    Eigen::Vector4d out;
    out << i(0), i(1), i(2), i(3);
    return out;
  }

  Eigen::Vector4d motor_to_joint_torque(const Eigen::Vector4d& q4,
                                        const Eigen::Vector4d& p4,
                                        const Eigen::Vector4d& i4) {
    Eigen::VectorXd q(4), p(4), i(4);
    q << q4(0), q4(1), q4(2), q4(3);
    p << p4(0), p4(1), p4(2), p4(3);
    i << i4(0), i4(1), i4(2), i4(3);
    const Eigen::VectorXd tau = solver_.motor_to_joint_torque(q, p, i);
    Eigen::Vector4d out;
    out << tau(0), tau(1), tau(2), tau(3);
    return out;
  }

  Eigen::Vector3d compute_tendon_vector(double pitch,
                                        double roll,
                                        double actuator_angle,
                                        kuavo_solver::AnkleSide ankle_side,
                                        kuavo_solver::TendonSide tendon_side) const {
    return solver_.compute_tendon_vector(pitch, roll, actuator_angle, ankle_side, tendon_side);
  }

  Eigen::Matrix<double, 3, 2> compute_jacobian_ankle(double pitch,
                                                      double roll,
                                                      kuavo_solver::AnkleSide ankle_side,
                                                      kuavo_solver::TendonSide tendon_side) const {
    return solver_.compute_jacobian_ankle(pitch, roll, ankle_side, tendon_side);
  }

  Eigen::Vector3d compute_jacobian_actuator(double pitch,
                                            double roll,
                                            double actuator_angle,
                                            kuavo_solver::AnkleSide ankle_side,
                                            kuavo_solver::TendonSide tendon_side) const {
    return solver_.compute_jacobian_actuator(pitch, roll, actuator_angle, ankle_side, tendon_side);
  }

 private:
  kuavo_solver::AxisOffsetAnkleSolver solver_;
};

class AnkleSolverAdapter {
 public:
  explicit AnkleSolverAdapter(const std::string& type_token, const std::string&) {
    solver_.getconfig(type_token);
  }

  Eigen::VectorXd joint_to_motor_position_leg12(const Eigen::VectorXd& q12) {
    if (q12.size() != 12) {
      throw std::invalid_argument("joint_to_motor_position_leg12 expects size-12 vector");
    }
    return solver_.joint_to_motor_position_leg12(q12);
  }

  Eigen::VectorXd motor_to_joint_position_leg12(const Eigen::VectorXd& p12) {
    if (p12.size() != 12) {
      throw std::invalid_argument("motor_to_joint_position_leg12 expects size-12 vector");
    }
    return solver_.motor_to_joint_position_leg12(p12);
  }

  Eigen::VectorXd joint_to_motor_velocity_leg12(const Eigen::VectorXd& q12, const Eigen::VectorXd& p12,
                                                const Eigen::VectorXd& dq12) {
    if (q12.size() != 12 || p12.size() != 12 || dq12.size() != 12) {
      throw std::invalid_argument("joint_to_motor_velocity_leg12 expects size-12 vectors");
    }
    return solver_.joint_to_motor_velocity(q12, p12, dq12);
  }

  Eigen::VectorXd motor_to_joint_velocity_leg12(const Eigen::VectorXd& q12, const Eigen::VectorXd& p12,
                                                const Eigen::VectorXd& dp12) {
    if (q12.size() != 12 || p12.size() != 12 || dp12.size() != 12) {
      throw std::invalid_argument("motor_to_joint_velocity_leg12 expects size-12 vectors");
    }
    return solver_.motor_to_joint_velocity(q12, p12, dp12);
  }

  Eigen::Vector4d joint_to_motor_position(const Eigen::Vector4d& q4) {
    return ExtractP4FromP12(solver_.joint_to_motor_position(MakeQ12FromQ4(q4)));
  }

  Eigen::Vector4d motor_to_joint_position(const Eigen::Vector4d& p4) {
    return ExtractQ4FromQ12(solver_.motor_to_joint_position(MakeP12FromP4(p4)));
  }

  Eigen::Vector4d joint_to_motor_velocity(const Eigen::Vector4d& q4,
                                          const Eigen::Vector4d& p4,
                                          const Eigen::Vector4d& dq4) {
    return ExtractP4FromP12(
        solver_.joint_to_motor_velocity(MakeQ12FromQ4(q4), MakeP12FromP4(p4), MakeQ12FromQ4(dq4)));
  }

  Eigen::Vector4d motor_to_joint_velocity(const Eigen::Vector4d& q4,
                                          const Eigen::Vector4d& p4,
                                          const Eigen::Vector4d& dp4) {
    return ExtractQ4FromQ12(
        solver_.motor_to_joint_velocity(MakeQ12FromQ4(q4), MakeP12FromP4(p4), MakeP12FromP4(dp4)));
  }

  Eigen::Vector4d joint_to_motor_current(const Eigen::Vector4d& q4,
                                         const Eigen::Vector4d& p4,
                                         const Eigen::Vector4d& tau4) {
    return ExtractP4FromP12(
        solver_.joint_to_motor_current(MakeQ12FromQ4(q4), MakeP12FromP4(p4), MakeQ12FromQ4(tau4)));
  }

  Eigen::Vector4d motor_to_joint_torque(const Eigen::Vector4d& q4,
                                        const Eigen::Vector4d& p4,
                                        const Eigen::Vector4d& i4) {
    return ExtractQ4FromQ12(
        solver_.motor_to_joint_torque(MakeQ12FromQ4(q4), MakeP12FromP4(p4), MakeP12FromP4(i4)));
  }

 protected:
  AnkleSolver solver_;
};

class ArmSolverAdapter {
 public:
  explicit ArmSolverAdapter(const std::string& type_token, const std::string&) {
    solver_.getconfig(type_token);
  }

  Eigen::VectorXd joint_to_motor_position_arm14(const Eigen::VectorXd& q14) { return solver_.joint_to_motor_position(q14); }
  Eigen::VectorXd motor_to_joint_position_arm14(const Eigen::VectorXd& p14) { return solver_.motor_to_joint_position(p14); }

  Eigen::VectorXd joint_to_motor_velocity_arm14(const Eigen::VectorXd& q14, const Eigen::VectorXd& p14, const Eigen::VectorXd& dq14) {
    return solver_.joint_to_motor_velocity(q14, p14, dq14);
  }
  Eigen::VectorXd motor_to_joint_velocity_arm14(const Eigen::VectorXd& q14, const Eigen::VectorXd& p14, const Eigen::VectorXd& dp14) {
    return solver_.motor_to_joint_velocity(q14, p14, dp14);
  }

  Eigen::VectorXd joint_to_motor_current_arm14(const Eigen::VectorXd& q14, const Eigen::VectorXd& p14, const Eigen::VectorXd& tau14) {
    return solver_.joint_to_motor_current(q14, p14, tau14);
  }
  Eigen::VectorXd motor_to_joint_torque_arm14(const Eigen::VectorXd& q14, const Eigen::VectorXd& p14, const Eigen::VectorXd& i14) {
    return solver_.motor_to_joint_torque(q14, p14, i14);
  }

 protected:
  ArmSolver solver_;
};

class WaistSolverAdapter {
 public:
  explicit WaistSolverAdapter(const std::string& type_token, const std::string&) {
    solver_.getconfig(type_token);
  }

  Eigen::Vector3d joint_to_motor_position(const Eigen::Vector3d& q3) {
    return solver_.joint_to_motor_position(q3);
  }

  Eigen::Vector3d motor_to_joint_position(const Eigen::Vector3d& p3) {
    return solver_.motor_to_joint_position(p3);
  }

  Eigen::Vector3d motor_to_joint_position_hint(const Eigen::Vector3d& p3, const Eigen::Vector3d& q_hint3) {
    return solver_.motor_to_joint_position(p3, q_hint3);
  }

  Eigen::Vector3d joint_to_motor_velocity(const Eigen::Vector3d& q3,
                                           const Eigen::Vector3d& p3,
                                           const Eigen::Vector3d& dq3) {
    return solver_.joint_to_motor_velocity(q3, p3, dq3);
  }

  Eigen::Vector3d motor_to_joint_velocity(const Eigen::Vector3d& q3,
                                           const Eigen::Vector3d& p3,
                                           const Eigen::Vector3d& dp3) {
    return solver_.motor_to_joint_velocity(q3, p3, dp3);
  }

  Eigen::Vector3d joint_to_motor_current(const Eigen::Vector3d& q3,
                                           const Eigen::Vector3d& p3,
                                           const Eigen::Vector3d& tau3) {
    return solver_.joint_to_motor_current(q3, p3, tau3);
  }

  Eigen::Vector3d motor_to_joint_torque(const Eigen::Vector3d& q3,
                                         const Eigen::Vector3d& p3,
                                         const Eigen::Vector3d& i3) {
    return solver_.motor_to_joint_torque(q3, p3, i3);
  }

  std::pair<Eigen::Matrix2d, Eigen::Matrix2d> parallel_jacobian_system(
      const Eigen::Vector3d& q3, const Eigen::Vector3d& p3) {
    return solver_.parallel_jacobian_system(q3, p3);
  }

 protected:
  WaistSolver solver_;
};

PYBIND11_MODULE(kuavo_solver_py, m) {
  m.doc() = "kuavo solver python bindings";

  py::enum_<kuavo_solver::AnkleSide>(m, "AnkleSide")
      .value("LEFT", kuavo_solver::AnkleSide::LEFT)
      .value("RIGHT", kuavo_solver::AnkleSide::RIGHT);

  py::enum_<kuavo_solver::TendonSide>(m, "TendonSide")
      .value("LEFT", kuavo_solver::TendonSide::LEFT)
      .value("RIGHT", kuavo_solver::TendonSide::RIGHT);

  py::class_<kuavo_solver::AxisOffsetAnkleParams>(m, "AxisOffsetAnkleParams")
      .def(py::init<>())
      .def_readwrite("z_pitch", &kuavo_solver::AxisOffsetAnkleParams::z_pitch)
      .def_readwrite("z_roll", &kuavo_solver::AxisOffsetAnkleParams::z_roll)
      .def_readwrite("x_pitch", &kuavo_solver::AxisOffsetAnkleParams::x_pitch)
      .def_readwrite("x_lleq", &kuavo_solver::AxisOffsetAnkleParams::x_lleq)
      .def_readwrite("y_lleq", &kuavo_solver::AxisOffsetAnkleParams::y_lleq)
      .def_readwrite("z_lleq", &kuavo_solver::AxisOffsetAnkleParams::z_lleq)
      .def_readwrite("x_lreq", &kuavo_solver::AxisOffsetAnkleParams::x_lreq)
      .def_readwrite("y_lreq", &kuavo_solver::AxisOffsetAnkleParams::y_lreq)
      .def_readwrite("z_lreq", &kuavo_solver::AxisOffsetAnkleParams::z_lreq)
      .def_readwrite("x_llbar", &kuavo_solver::AxisOffsetAnkleParams::x_llbar)
      .def_readwrite("z_llbar", &kuavo_solver::AxisOffsetAnkleParams::z_llbar)
      .def_readwrite("x_lrbar", &kuavo_solver::AxisOffsetAnkleParams::x_lrbar)
      .def_readwrite("z_lrbar", &kuavo_solver::AxisOffsetAnkleParams::z_lrbar)
      .def_readwrite("x_lltd", &kuavo_solver::AxisOffsetAnkleParams::x_lltd)
      .def_readwrite("y_lltd", &kuavo_solver::AxisOffsetAnkleParams::y_lltd)
      .def_readwrite("z_lltd", &kuavo_solver::AxisOffsetAnkleParams::z_lltd)
      .def_readwrite("x_lrtd", &kuavo_solver::AxisOffsetAnkleParams::x_lrtd)
      .def_readwrite("y_lrtd", &kuavo_solver::AxisOffsetAnkleParams::y_lrtd)
      .def_readwrite("z_lrtd", &kuavo_solver::AxisOffsetAnkleParams::z_lrtd)
      .def_readwrite("l0_ll_eqtd", &kuavo_solver::AxisOffsetAnkleParams::l0_ll_eqtd)
      .def_readwrite("l0_lr_eqtd", &kuavo_solver::AxisOffsetAnkleParams::l0_lr_eqtd)
      .def_readwrite("x_rleq", &kuavo_solver::AxisOffsetAnkleParams::x_rleq)
      .def_readwrite("y_rleq", &kuavo_solver::AxisOffsetAnkleParams::y_rleq)
      .def_readwrite("z_rleq", &kuavo_solver::AxisOffsetAnkleParams::z_rleq)
      .def_readwrite("x_rreq", &kuavo_solver::AxisOffsetAnkleParams::x_rreq)
      .def_readwrite("y_rreq", &kuavo_solver::AxisOffsetAnkleParams::y_rreq)
      .def_readwrite("z_rreq", &kuavo_solver::AxisOffsetAnkleParams::z_rreq)
      .def_readwrite("x_rlbar", &kuavo_solver::AxisOffsetAnkleParams::x_rlbar)
      .def_readwrite("z_rlbar", &kuavo_solver::AxisOffsetAnkleParams::z_rlbar)
      .def_readwrite("x_rrbar", &kuavo_solver::AxisOffsetAnkleParams::x_rrbar)
      .def_readwrite("z_rrbar", &kuavo_solver::AxisOffsetAnkleParams::z_rrbar)
      .def_readwrite("x_rltd", &kuavo_solver::AxisOffsetAnkleParams::x_rltd)
      .def_readwrite("y_rltd", &kuavo_solver::AxisOffsetAnkleParams::y_rltd)
      .def_readwrite("z_rltd", &kuavo_solver::AxisOffsetAnkleParams::z_rltd)
      .def_readwrite("x_rrtd", &kuavo_solver::AxisOffsetAnkleParams::x_rrtd)
      .def_readwrite("y_rrtd", &kuavo_solver::AxisOffsetAnkleParams::y_rrtd)
      .def_readwrite("z_rrtd", &kuavo_solver::AxisOffsetAnkleParams::z_rrtd)
      .def_readwrite("l0_rl_eqtd", &kuavo_solver::AxisOffsetAnkleParams::l0_rl_eqtd)
      .def_readwrite("l0_rr_eqtd", &kuavo_solver::AxisOffsetAnkleParams::l0_rr_eqtd)
      .def_readwrite("default_tolerance", &kuavo_solver::AxisOffsetAnkleParams::default_tolerance)
      .def_readwrite("max_iterations", &kuavo_solver::AxisOffsetAnkleParams::max_iterations);

  py::class_<AxisOffsetAnkleSolverPy>(m, "AxisOffsetAnkleSolver")
      .def(py::init<const kuavo_solver::AxisOffsetAnkleParams&>())
      .def("joint_to_motor_position", &AxisOffsetAnkleSolverPy::joint_to_motor_position)
      .def("motor_to_joint_position", &AxisOffsetAnkleSolverPy::motor_to_joint_position)
      .def("joint_to_motor_velocity", &AxisOffsetAnkleSolverPy::joint_to_motor_velocity)
      .def("motor_to_joint_velocity", &AxisOffsetAnkleSolverPy::motor_to_joint_velocity)
      .def("joint_to_motor_current", &AxisOffsetAnkleSolverPy::joint_to_motor_current)
      .def("motor_to_joint_torque", &AxisOffsetAnkleSolverPy::motor_to_joint_torque)
      .def("compute_tendon_vector", &AxisOffsetAnkleSolverPy::compute_tendon_vector)
      .def("compute_jacobian_ankle", &AxisOffsetAnkleSolverPy::compute_jacobian_ankle)
      .def("compute_jacobian_actuator", &AxisOffsetAnkleSolverPy::compute_jacobian_actuator);

  py::class_<AnkleSolverAdapter>(m, "AnkleSolver")
      .def(py::init<const std::string&, const std::string&>(),
           py::arg("ankle_solver_type"),
           py::arg("config_dir"))
      .def("joint_to_motor_position_leg12", &AnkleSolverAdapter::joint_to_motor_position_leg12)
      .def("motor_to_joint_position_leg12", &AnkleSolverAdapter::motor_to_joint_position_leg12)
      .def("joint_to_motor_velocity_leg12", &AnkleSolverAdapter::joint_to_motor_velocity_leg12)
      .def("motor_to_joint_velocity_leg12", &AnkleSolverAdapter::motor_to_joint_velocity_leg12)
      .def("joint_to_motor_position", &AnkleSolverAdapter::joint_to_motor_position)
      .def("motor_to_joint_position", &AnkleSolverAdapter::motor_to_joint_position)
      .def("joint_to_motor_velocity", &AnkleSolverAdapter::joint_to_motor_velocity)
      .def("motor_to_joint_velocity", &AnkleSolverAdapter::motor_to_joint_velocity)
      .def("joint_to_motor_current", &AnkleSolverAdapter::joint_to_motor_current)
      .def("motor_to_joint_torque", &AnkleSolverAdapter::motor_to_joint_torque);

  py::class_<kuavo_solver::ParallelLinearArmSolver>(m, "ParallelLinearArmSolver")
      .def(py::init([](const std::string& token, const std::string& config_dir) {
             const auto loaded = kuavo_solver::ParallelLinearArmSolver::loadParam(token, config_dir);
             return new kuavo_solver::ParallelLinearArmSolver(loaded.params);
           }),
           py::arg("arm_solver_type_token"),
           py::arg("config_dir"))
      .def("joint_to_motor_position", &kuavo_solver::ParallelLinearArmSolver::joint_to_motor_position)
      .def("motor_to_joint_position", &kuavo_solver::ParallelLinearArmSolver::motor_to_joint_position)
      .def("joint_to_motor_velocity", &kuavo_solver::ParallelLinearArmSolver::joint_to_motor_velocity)
      .def("motor_to_joint_velocity", &kuavo_solver::ParallelLinearArmSolver::motor_to_joint_velocity)
      .def("joint_to_motor_current", &kuavo_solver::ParallelLinearArmSolver::joint_to_motor_current)
      .def("motor_to_joint_torque", &kuavo_solver::ParallelLinearArmSolver::motor_to_joint_torque)
      .def("elbow_jacobian_system", &kuavo_solver::ParallelLinearArmSolver::elbow_jacobian_system)
      .def("wrist_jacobian_system", &kuavo_solver::ParallelLinearArmSolver::wrist_jacobian_system);

  py::class_<kuavo_solver::ParallelRotateWaistSolver>(m, "ParallelRotateWaistSolver")
      .def(py::init([](const std::string& token, const std::string& config_dir) {
             const auto loaded = kuavo_solver::ParallelRotateWaistSolver::loadParam(token, config_dir);
             return new kuavo_solver::ParallelRotateWaistSolver(loaded.params);
           }),
           py::arg("waist_solver_type_token"),
           py::arg("config_dir"))
      .def("joint_to_motor_position", &kuavo_solver::ParallelRotateWaistSolver::joint_to_motor_position)
      .def(
          "motor_to_joint_position",
          static_cast<Eigen::Vector2d (kuavo_solver::ParallelRotateWaistSolver::*)(const Eigen::Vector2d&) const>(
              &kuavo_solver::ParallelRotateWaistSolver::motor_to_joint_position))
      .def(
          "motor_to_joint_position_hint",
          static_cast<Eigen::Vector2d (kuavo_solver::ParallelRotateWaistSolver::*)(
              const Eigen::Vector2d&, const Eigen::Vector2d&) const>(
              &kuavo_solver::ParallelRotateWaistSolver::motor_to_joint_position));

  py::class_<ArmSolverAdapter>(m, "ArmSolver")
      .def(py::init<const std::string&, const std::string&>(),
           py::arg("arm_solver_type"),
           py::arg("config_dir"))
      .def("joint_to_motor_position_arm14", &ArmSolverAdapter::joint_to_motor_position_arm14)
      .def("motor_to_joint_position_arm14", &ArmSolverAdapter::motor_to_joint_position_arm14)
      .def("joint_to_motor_velocity_arm14", &ArmSolverAdapter::joint_to_motor_velocity_arm14)
      .def("motor_to_joint_velocity_arm14", &ArmSolverAdapter::motor_to_joint_velocity_arm14)
      .def("joint_to_motor_current_arm14", &ArmSolverAdapter::joint_to_motor_current_arm14)
      .def("motor_to_joint_torque_arm14", &ArmSolverAdapter::motor_to_joint_torque_arm14);

  py::class_<WaistSolverAdapter>(m, "WaistSolver")
      .def(py::init<const std::string&, const std::string&>(),
           py::arg("waist_solver_type_token"),
           py::arg("config_dir"))
      .def("joint_to_motor_position", &WaistSolverAdapter::joint_to_motor_position)
      .def("motor_to_joint_position", &WaistSolverAdapter::motor_to_joint_position)
      .def("motor_to_joint_position_hint", &WaistSolverAdapter::motor_to_joint_position_hint)
      .def("joint_to_motor_velocity", &WaistSolverAdapter::joint_to_motor_velocity)
      .def("motor_to_joint_velocity", &WaistSolverAdapter::motor_to_joint_velocity)
      .def("joint_to_motor_current", &WaistSolverAdapter::joint_to_motor_current)
      .def("motor_to_joint_torque", &WaistSolverAdapter::motor_to_joint_torque)
      .def("parallel_jacobian_system", &WaistSolverAdapter::parallel_jacobian_system);
}
