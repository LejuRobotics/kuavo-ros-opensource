#pragma once
#include "Eigen/Core"
#include <iostream>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

using namespace Eigen;
using namespace drake;

namespace lower_leg
{

  class ArmJointController : public drake::systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArmJointController)

    ArmJointController(multibody::MultibodyPlant<double> &plant, const std::vector<std::string> &cfg_str);
    void CalcArmState(Eigen::Vector4d &lower_qv, Eigen::Vector4d &ankle_motor_qv) const;
    void CalcArmStateQv(Eigen::Vector4d &lower_qv, Eigen::VectorXd &ankle_motor_qv) const;

    /*********大迭代版本解算函数维护*********/
    inline double safe_acos(double x) {
        return acos(fmax(-1.0, fmin(1.0, x)));  // 使用std::fmax和std::fmin来确保值在[-1, 1]范围内
    }
    std::pair<double, double> pos_joint2actuator_left(double pitch, double roll);
    std::pair<double, double> vel_joint2actuator_left(double pitch, double roll, double llbar, double lrbar, double pitchDt, double rollDt);
    std::pair<double, double> pos_joint2actuator_right(double pitch, double roll);
    std::pair<double, double> vel_joint2actuator_right(double pitch, double roll, double llbar, double lrbar, double pitchDt, double rollDt);
    /* 膝关节解算 */
    double pos_joint2actuator_knee(double knee);
    double vel_joint2actuator_knee(double knee, double bar, double kneeDt);
    /************************************/

  private:
    int nq_;
    int nv_;
    int i_j_l_arm_;
    int i_j_r_arm_;
    int i_roll_;
    int i_pitch_;
    int i_j_r_link, i_j_l_link;
    double l_Lkleft_;
    double l_Lkright_;
    double l_Armleft_;
    double l_Armright_;
    double qO_Armleft_; // this is "capital ou", not "zero"
    double qO_Armright_;
    double init_l_link_xz, init_l_link_yz, init_r_link_xz, init_r_link_yz;
    multibody::MultibodyPlant<double> &plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    double calProjectAngleXZ(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) const;

    /*********大迭代版本解算数据维护*********/
    double z_pitch = -0.346;     // foot_pitch 或 foot_roll 到膝关节的z轴向量
    double x_lleq = 0.0385 , y_lleq = 0.0282796, z_lleq = -0.01;  // 左边连杆末端与脚掌连接的操作点socket，到踝关节roll,pitch 实际运动轴foot的矢量
    double x_lreq = 0.0385 , y_lreq = -0.0282796, z_lreq = -0.01;  // 右边连杆末端与脚掌连接的操作点socket，到踝关节roll,pitch 实际运动轴foot的矢量
    double z_llbar = -0.153; // 左电机到膝盖的z轴矢量
    double z_lrbar = -0.09; // 右电机到膝盖的z轴矢量
    double x_lltd = 0.044, y_lltd = 0.0282564, z_lltd = -0.0100784;    // l_tendon 到 电机正中心 l_l_bar 的矢量
    double x_lrtd = 0.044, y_lrtd = -0.0282633, z_lrtd = -0.0100591;    // r_tendon 到 电机正中心 l_r_bar 的矢量
    double l_lltd = 0.19299998519378184, l_lrtd = 0.2559999893720701;   // 左右长柄的长度
    double l_llbar, l_lrbar;

    double x_rleq = 0.0385 , y_rleq = 0.0282796, z_rleq = -0.01;  // 左边连杆末端与脚掌连接的操作点socket，到踝关节roll,pitch 实际运动轴foot的矢量
    double x_rreq = 0.0385 , y_rreq = -0.0282796, z_rreq = -0.01;  // 右边连杆末端与脚掌连接的操作点socket，到踝关节roll,pitch 实际运动轴foot的矢量
    double z_rlbar = -0.09; // 左电机到膝盖的z轴矢量
    double z_rrbar = -0.153; // 右电机到膝盖的z轴矢量
    double x_rltd = 0.044, y_rltd = 0.0282633, z_rltd = -0.0100591;    // l_tendon 到 电机正中心 l_l_bar 的矢量
    double x_rrtd = 0.044, y_rrtd = -0.0282564, z_rrtd = -0.0100784;    // r_tendon 到 电机正中心 l_r_bar 的矢量
    double l_rltd = 0.25599998937207, l_rrtd = 0.19299998519378178;   // 左右长柄的长度
    double l_rlbar, l_rrbar;

    /* 膝关节解算所需数据 */
    double AD = 0.166;
    double BC = 0.166;
    double AB = 0.035;
    double CD = 0.035;
    double offsetBar = 0.3986332011555049;
    double offsetKnee = 0.2617993877991494;
    /************************************/
    struct ArmAnkleConfig
    {
      ArmAnkleConfig() {}
      ArmAnkleConfig(const std::vector<std::string> &cfg_str)
      {
        if (cfg_str.size() != 19)
          std::cout << "Please check the size of ankle config.\n";
        end_link = cfg_str[0];   // l_hand_pitch
        l_link = cfg_str[1];     // l_l_arm_tendon
        r_link = cfg_str[2];     // l_r_arm_tendon
        l_bar_link = cfg_str[3]; // l_l_arm_bar
        r_bar_link = cfg_str[4]; // l_r_arm_bar

        roll_joint = cfg_str[5];     // l_hand_roll
        pitch_joint = cfg_str[6];    // l_hand_pitch
        l_link_joint = cfg_str[7];   // l_l_arm_tendon
        r_link_joint = cfg_str[8];   // l_r_arm_tendon
        l_motor_joint = cfg_str[9];  // l_l_arm_bar
        r_motor_joint = cfg_str[10]; // l_r_arm_bar

        l_motor_actuator = cfg_str[11]; // l_l_arm_bar_motor
        r_motor_actuator = cfg_str[12]; // l_r_arm_bar_motor

        l_end_frame = cfg_str[13];   // l_l_hand_socket
        r_end_frame = cfg_str[14];   // l_r_hand_socket
        l_frame = cfg_str[15];       // l_l_arm_tendon_socket
        r_frame = cfg_str[16];       // l_r_arm_tendon_socket
        l_motor_frame = cfg_str[17]; // l_l_arm_bar_frame
        r_motor_frame = cfg_str[18]; // l_r_arm_bar_frame
      }
      std::string end_link;
      std::string l_bar_link;
      std::string r_bar_link;
      std::string l_link;
      std::string r_link;

      std::string roll_joint; // first
      std::string pitch_joint;
      std::string l_link_joint;
      std::string r_link_joint;
      std::string l_motor_joint;
      std::string r_motor_joint;

      std::string l_motor_actuator;
      std::string r_motor_actuator;

      std::string l_frame;
      std::string r_frame;
      std::string l_end_frame;
      std::string r_end_frame;
      std::string l_motor_frame;
      std::string r_motor_frame; // 考虑了偏移量
    };
    ArmAnkleConfig cfg_;
  };
} // namespace lower_leg
