#pragma once
#include <Eigen/Core>
#include <memory>
#include <limits>
#include <initializer_list>
#include <utility>
namespace velocity_ik {
struct ResultCheck {
  bool accepted=false;
  int status=-1102;
  double violation=std::numeric_limits<double>::infinity();
  double stationarity=std::numeric_limits<double>::infinity();
  double complementarity=std::numeric_limits<double>::infinity();
};
ResultCheck checkQpResult(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,
                         const Eigen::MatrixXd& C,const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper,const Eigen::VectorXd& x,
                         const Eigen::VectorXd& z);
class WeightedQp {
 public:
  struct Stats {int native_status=-1,iterations=0,acceptance_status=-1;bool accepted=false;
    double primal=0,dual=0,solve_us=0,update_us=0,check_us=0,total_us=0,begin_ns=0,end_ns=0;
    double lock_wait_us=0,prepare_us=0; bool workspace_created=false;
    double violation=std::numeric_limits<double>::infinity(),stationarity=std::numeric_limits<double>::infinity(),complementarity=std::numeric_limits<double>::infinity();};
  WeightedQp(); ~WeightedQp();
  // Startup only. Restrict this instance to known physical problem families.
  // Generic standalone callers may retain the default unrestricted behavior.
  void prepareWorkspaces(std::initializer_list<std::pair<int,int>> dimensions);
  Stats stats() const;
  bool solve(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,
             const Eigen::VectorXd& lo,const Eigen::VectorXd& hi,
             const Eigen::MatrixXd& C,const Eigen::VectorXd& l,const Eigen::VectorXd& u,
             Eigen::VectorXd* x,int* status, Stats* statistics=nullptr);
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
