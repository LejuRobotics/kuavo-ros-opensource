#ifndef _Kalman_h_
#define _Kalman_h_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "orientation_tools.h"


typedef struct
{
  Eigen::Vector3d px;
  Eigen::Vector4d pq;
  Eigen::Vector3d pv;
  Eigen::Vector3d pw;
  Eigen::Vector3d b_a;
} Decoupled_states;

class Kalman
{
public:
  Kalman();
  Kalman(int state_dim, int measure_dim);
  Kalman(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &x_init, int measure_dim);
  
  void setQR(Eigen::VectorXd q, Eigen::VectorXd r);
  void setQR(double *Q, double *R);
  void setState(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &x_init);
  void update_F_H(Eigen::MatrixXd &F, Eigen::MatrixXd &H);
  Eigen::MatrixXd updateData(Eigen::MatrixXd z);
  Eigen::MatrixXd updateData(Eigen::MatrixXd x, Eigen::MatrixXd z);
  Eigen::VectorXd updateData_(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, Eigen::MatrixXd &P, const Eigen::VectorXd x, const Eigen::VectorXd z);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x;

private:
  int state_dim_, measure_dim_;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _F;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _P;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _H;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _K;
};

#endif
