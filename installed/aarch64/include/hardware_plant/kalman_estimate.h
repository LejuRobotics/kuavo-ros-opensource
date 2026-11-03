#pragma once

#include "Kalman.h"
#include "memory"

class KalmanEstimate
{
public:
  KalmanEstimate(uint32_t na, Eigen::Vector3d b_a, Eigen::Vector3d imu_in_toros);
  ~KalmanEstimate();

  void setcomQR(double *Q, double *R);
  void setcomState(Eigen::VectorXd &r_vec);
  void setBaseQR(Eigen::VectorXd Q, Eigen::VectorXd R);
  void setBaseQR(double *Q, double *R);
  void setBaseState(Eigen::VectorXd &qv);
  void setBaseState_(Eigen::VectorXd &qv);
  bool setJointQR(uint8_t index, double *Q, double *R);
  void setJointState(Eigen::VectorXd &qv, Eigen::VectorXd &vdot,
                     uint32_t nq, uint32_t nv,
                     uint32_t nq_f, uint32_t nv_f);
  void jointFiltering(Eigen::VectorXd &qv, Eigen::VectorXd &vdot, double dt);
  Decoupled_states baseFiltering(Eigen::VectorXd &qv, Eigen::Vector3d &acc, double dt);
  void baseFiltering2(Eigen::VectorXd &qv, Eigen::Vector3d &acc, double dt);
  void comFiltering(Eigen::VectorXd &r_vec_est, Eigen::VectorXd &r_vec, Eigen::Vector3d &p_st, double dt);
  void HipRollFiltering(Eigen::VectorXd &mhiproll, Eigen::VectorXd &hiproll, double v_torso, double v_foot, double l, double dt);

private:
  Eigen::MatrixXd getBaseF(Eigen::Vector3d gyro_w, Eigen::Vector3d acc, Eigen::Vector3d b_a, Eigen::Vector4d pq, double dt);
  Eigen::MatrixXd getBaseF2(Eigen::Vector3d gyro_w, Eigen::Vector3d acc, Eigen::Vector3d b_a, Eigen::Quaterniond quat, double dt);

  const double gravity_ = 9.81;
  Eigen::Vector3d b_a_; // accelerometer bias
  Eigen::VectorXd estimate_base_;
  Eigen::VectorXd estimate_com_;
  Eigen::VectorXd estimate_roll_;
  uint32_t na_;

  std::unique_ptr<Kalman> hiproll_kf;
  std::unique_ptr<Kalman> com_kf;
  std::unique_ptr<Kalman> base_kf;
  std::vector<std::unique_ptr<Kalman>> joint_kf;
  Decoupled_states predict_states;
  Decoupled_states update_state_;
  Eigen::MatrixXd decoupled_P;
  int print_num=0;
  Eigen::Vector3d r_imu_in_toros;//449.5/14.75
};
