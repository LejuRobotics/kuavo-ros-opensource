#pragma once
#include <mujoco/mujoco.h>
#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <cmath>
#include <stdexcept>
#include <vector>

// Private simulator only. SI joint-side gains, NOT EtherCAT PDO gain units.
// MuJoCo evaluates the affine actuator feedback at every physics step and its
// implicit integrator includes the velocity derivative. No delayed ROS PD loop.
class AbsoluteSimulationServo {
 public:
  void configure(mjModel* m, mjData* d, ros::NodeHandle& nh,
                 const std::vector<int>& actuators) {
    nh.param("/absolute_simulation/motor_servo", enabled_, false);
    if (!enabled_) return;
    if (!nh.getParam("/absolute_simulation/kp", kp_) ||
        !nh.getParam("/absolute_simulation/kd", kd_) ||
        kp_.size()!=actuators.size() || kd_.size()!=actuators.size())
      throw std::runtime_error("absolute simulation servo gain/layout mismatch");
    configureModel(m,d,actuators,kp_,kd_);
  }
  void configureModel(mjModel* m, mjData* d, const std::vector<int>& actuators,
                      const std::vector<double>& kp, const std::vector<double>& kd) {
    if (kp.size()!=actuators.size() || kd.size()!=actuators.size())
      throw std::runtime_error("absolute simulation servo gain/layout mismatch");
    kp_=kp; kd_=kd;
    for (size_t i=0;i<actuators.size();++i) {
      const int a=actuators[i];
      if (!std::isfinite(kp_[i]) || !std::isfinite(kd_[i]) || kp_[i]<=0 || kd_[i]<=0 ||
          a<0 || a>=m->nu || m->actuator_trntype[a]!=mjTRN_JOINT ||
          m->actuator_gear[6*a]!=1 || m->actuator_dyntype[a]!=mjDYN_NONE ||
          !m->actuator_ctrllimited[a])
        throw std::runtime_error("unsupported absolute simulation actuator/gain");
      // Keep the XML torque bounds on TOTAL force, after the feedback term.
      // Clipping the affine control input before feedback changes equilibrium.
      m->actuator_forcelimited[a]=1;
      m->actuator_forcerange[2*a]=m->actuator_ctrlrange[2*a];
      m->actuator_forcerange[2*a+1]=m->actuator_ctrlrange[2*a+1];
      m->actuator_ctrllimited[a]=0;
      m->actuator_gaintype[a]=mjGAIN_FIXED;
      m->actuator_gainprm[mjNGAIN*a]=1;
      m->actuator_biastype[a]=mjBIAS_AFFINE;
      m->actuator_biasprm[mjNBIAS*a]=0;
      m->actuator_biasprm[mjNBIAS*a+1]=-kp_[i];
      m->actuator_biasprm[mjNBIAS*a+2]=-kd_[i];
      const int joint=m->actuator_trnid[2*a];
      d->ctrl[a]=kp_[i]*d->qpos[m->jnt_qposadr[joint]];
    }
    enabled_=true;
    m->opt.integrator=mjINT_IMPLICITFAST;
    ROS_INFO("[AbsoluteSimulationServo] enabled: %zu joint-side PD servos, implicitfast, XML total-force limits",kp_.size());
  }
  bool encode(const kuavo_msgs::jointCmd& msg, std::vector<double>& control) const {
    if (!enabled_) return true;
    if (control.size()!=kp_.size() || msg.control_modes.size()!=kp_.size() ||
        msg.joint_q.size()!=kp_.size() || msg.joint_v.size()!=kp_.size()) return false;
    for (size_t i=0;i<control.size();++i) {
      if (msg.control_modes[i]!=2 || !std::isfinite(msg.joint_q[i]) ||
          !std::isfinite(msg.joint_v[i]) || !std::isfinite(control[i])) return false;
      control[i]+=kp_[i]*msg.joint_q[i]+kd_[i]*msg.joint_v[i];
      if (!std::isfinite(control[i])) return false;
    }
    return true;
  }
 private:
  bool enabled_=false;
  std::vector<double> kp_,kd_;
};
