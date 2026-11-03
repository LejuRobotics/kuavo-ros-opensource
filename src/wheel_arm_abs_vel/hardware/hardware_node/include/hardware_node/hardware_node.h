#ifndef HARDWARE_NODE_H
#define HARDWARE_NODE_H
#pragma once

#include <memory>
#include <queue>
#include <set>
#include <atomic>
#include "hardware_command_gate.h"
#include <claw_types.h>
#include <gesture_types.h>
#include <lejuclaw_controller.h>
#include "hardware_plant.h"
#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "hardware_dds_plant.h"
#endif
#include "humanoid_interface/common/TopicLogger.h"

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "touch_dexhand_ros1.h"
#include "revo2_dexhand_ros.h"
#include "hardware_node/sg100_hand_driver.h"
#include "hardware_node/sg100_initialization_retry.h"

#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointData.h"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/jointMoveTo.h"
#include "kuavo_msgs/jointMoveToRequest.h"
#include "kuavo_msgs/jointMoveToResponse.h"
#include "kuavo_msgs/setHwIntialState.h"
#include "kuavo_msgs/setMotorEncoderRoundService.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/robotHeadMotionData.h"
#include "kuavo_msgs/gestureExecute.h"
#include "kuavo_msgs/gestureList.h"
#include "kuavo_msgs/gestureExecuteState.h"
#include "kuavo_msgs/gestureInfo.h"
#include "kuavo_msgs/controlLejuClaw.h"
#include "kuavo_msgs/lejuClawRecovery.h"
#include "kuavo_msgs/lejuClawState.h"
#include "kuavo_msgs/lejuClawCommand.h"
#include "kuavo_msgs/lejuClawTarget.h"
#include "kuavo_msgs/JoySticks.h"
#include "kuavo_msgs/changeMotorParam.h"
#include "kuavo_msgs/getMotorParam.h"
#include "kuavo_msgs/adjustZeroPoint.h"
#include "kuavo_msgs/getMotorZeroPoints.h"
#include "kuavo_msgs/goToZeroPoint.h"
#include "kuavo_msgs/robotSwitchPose.h"

namespace leju_utils {
class ActuatorDynamicsCompensator;
}

namespace HighlyDynamic
{
  class HardwareNode
  {

  public:
    enum PROTECT_ACTIVE_MODE : int
    {
      ENUM_ACTIVE_DECE = 0,
      ENUM_ACTIVE_DISABLE_MPC = 1,
      ENUM_ACTIVE_POS_LOCK = 2,
      ENUM_ACTIVE_BOT_RELEASE = 3,

      ENUM_ACTIVE_INIT = 99
    };

    enum PROTEST_MODE : int
    {
      ENUM_PEAR_PROTECTION = 0,
      ENUM_LOCKED_ROTOR_PROTECTION = 1,
      ENUM_SPEED_PROTECTION = 2
    };

  private:
      std::unique_ptr<HighlyDynamic::HardwarePlant> hardware_plant_;

      ros::NodeHandle nh_;
      std::thread sensor_thread_;
      std::atomic<bool> sensor_thread_running_{false};
      HardwareCommandGate command_gate_;
      std::mutex hardware_lifecycle_mutex_;
      bool readStartupLine(std::string& line);
      ros::Publisher sensor_data_pub_;
      ros::Subscriber joint_sub_;
      ros::Subscriber hand_sub_;
      ros::Subscriber head_sub_;
      ros::Subscriber stop_sub_;
      ros::Publisher stop_pub_;
      ros::Publisher robot_disabled_flag_pub_;
      ros::Publisher enable_wbc_pub_;
      ros::Publisher enable_mpc_pub_;
      ros::Subscriber re_start_sub_;
      ros::Subscriber bot_stand_up_sub_;
      ros::Publisher leju_claw_state_pub_;
      ros::Publisher leju_claw_debug_pub_;
      ros::Publisher leju_claw_target_pub_;
      ros::Publisher arm_zeros_pub_;
      ros::Publisher leg_offset_pub_;
      ros::Subscriber leju_claw_command_sub_;
      ros::Subscriber leju_claw_joy_sub_;
      ros::Timer     leju_claw_state_timer_;
      std::atomic<int64_t> last_vr_joystick_us_{0};
      std::atomic<bool> last_high_freq_{false};
      static constexpr int64_t kLejuClawVrStreamTimeoutMs = 1000;

      ros::ServiceServer joint_move_to_srv_;
      /** 座椅 stand_up 段0：运行中允许的 reverse prep（jointMoveToPrepGoal），与 use_sit_init phase2a/2b 同路径 */
      ros::ServiceServer seat_return_reverse_prep_srv_;
      ros::ServiceServer set_intial_state_srv_;
      ros::ServiceServer modify_motor_encoder_offset_srv_;
      ros::ServiceServer deintial_srv_;
      ros::ServiceServer leju_claw_ctrl_srv_;
      ros::ServiceServer leju_claw_recovery_srv_;
      /** jointMoveTo 与 jointCmd 互斥：reverse prep 期间忽略 /joint_cmd */
      std::atomic<bool> seat_reverse_prep_active_{false};

      ros::ServiceServer change_motor_param_srv_;
      ros::ServiceServer get_motor_param_srv_;

      ros::ServiceServer adjust_zero_point_srv_;
      ros::ServiceServer get_hardware_ready_srv_;
      ros::ServiceServer get_motor_zero_points_srv_;
      ros::ServiceServer switch_robot_state_srv_;

      /**
       * for kuavo-humanoid-websocket-sdk: 获取启动状态服务 
       * initing: 初始化中
       * calibrate: 准备下蹲缩腿 --> cali 全身绷直状态
       * ready_stance: 缩腿准备站立
       * launched: 启动完成, 站立 OK
       */
      ros::ServiceServer real_launch_status_srv_;

      ocs2::humanoid::TopicLogger *logger_ptr_;

      int8_t bot_stand_up_complete_{0};    //0:初始化  -1：站立失败   1：站立成功
      bool ready_to_feedback_{false};      // cali模式下先缩腿再 feedback；非cali直接为true
      HardwareParam hardware_param_;
      std::shared_ptr<eef_controller::DexhandRosNode> dexhand_ros_actuator_;
      std::shared_ptr<eef_controller::Revo2DexhandRosNode> revo2_dexhand_ros_actuator_;
      std::atomic<bool> sg100_shutdown_requested_{false};
      std::mutex end_effector_lifecycle_mutex_;
      std::shared_ptr<class SG100HandROSNode> sg100_driver_;  // heiman(SG100) 灵巧手驱动
      sg100::InitializationRetry sg100_init_retry_;

    bool joint_protect_enable_{false};
    bool cmd_truncation_enable_{false};
    bool leg_disable_mode_{false};  // 腿部失能模式：false=单个关节失能, true=腿部全失能
    bool cali_zero_point_mode{false};
    bool zero_point_status_swith{false};
    std::mutex enable_mpc_mtx_;
    std::mutex protect_mode_mtx_;
    Eigen::VectorXd his_cmd_;
    Eigen::VectorXd cmd_out_prev_;

    std::vector<double> curVel_;
    std::vector<double> curPos_;
    std::vector<int> armJointIndices_;
    std::unique_ptr<leju_utils::ActuatorDynamicsCompensator> actuatorDynamicsCompensatorPtr_;

    double c2t_coeff_[28];
    std::mutex disable_motor_mtx_;
    std::set<int> disableMotor_;
    std::mutex motor_status_map_mtx_;
    std::map<int, bool> motorStatusMap_; 
    std::vector<ros::Time> pre_vel_over_time_;
    std::vector<ros::Time> pre_cur_over_time_;
    std::vector<ros::Time> pre_peak_over_time_;
    ros::Time set_mpc_disable_time_;
    PROTECT_ACTIVE_MODE cur_protect_mode_{ENUM_ACTIVE_INIT};
    char stand_up_cmd_ = '\0';
    double dt_;
    int num_joint;
    double jointMoveSpeed_{60.0};
    

    // Dexterous hand state publish frequency in Hz
    // 真实反馈只有 5Hz/手（见 touch_hand_controller.cpp feedback_period_ms_），
    // 200Hz 发布只是把同一份快照重复发 200 次，纯 CPU/ROS 开销。
    // 50Hz 对现有消费端无感（joy_customize_config / visualizer / arm_trajectory 均 queue_size=1 只取最新）。
    static constexpr double kDexhandStatePublishFrequency = 50.0;

      bool tryInitializeSg100();
      void stopEndEffectorRosNodes();
      void deInitHardwarePlant();

  public:
      HardwareNode(ros::NodeHandle &nh, double dt);
      ~HardwareNode();
      void init();
      void requestShutdown() { command_gate_.requestStop(); }
      bool shutdownRequested() const { return command_gate_.stopping(); }
      void stopHardware();
      void jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr& msg);
      void handCmdCallback(const kuavo_msgs::robotHandPosition::ConstPtr &msg);
      bool jointMoveToServiceCallback(kuavo_msgs::jointMoveToRequest &req, kuavo_msgs::jointMoveToResponse &res);
      /** sitcomplete→stand_up 段0：从当前姿 reverse 回 sit_joint_pos（先腿后臂可选） */
      bool seatReturnReversePrepCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      bool setHwIntialStateCallback(kuavo_msgs::setHwIntialStateRequest &req, kuavo_msgs::setHwIntialStateResponse &res);
      bool setMotorEncoderOffsetCallback(kuavo_msgs::setMotorEncoderRoundServiceRequest &req, kuavo_msgs::setMotorEncoderRoundServiceResponse &res);
      bool deintialCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      void stopCallback(const std_msgs::Bool::ConstPtr &msg);
      bool controlLejuClawCallback(kuavo_msgs::controlLejuClawRequest &req, kuavo_msgs::controlLejuClawResponse &res);
      bool recoverLejuClawCallback(kuavo_msgs::lejuClawRecoveryRequest &req, kuavo_msgs::lejuClawRecoveryResponse &res);
      void lejuClawCommandCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg);
      void lejuClawJoystickCallback(const kuavo_msgs::JoySticks::ConstPtr &msg);
      void pubLejuClawStateTimerCallback(const ros::TimerEvent& event);
      void publishLejuClawDebug(const std::string& debug_msg);
      void publishLejuClawTarget(const std::vector<std::string>& name,
                                 const std::vector<double>& position,
                                 const std::vector<double>& velocity,
                                 const std::vector<double>& effort,
                                 const std::vector<int>& kp,
                                 const std::vector<int>& kd,
                                 bool zero_kp);
      bool getLejuClawHighFreq();
      bool notifyLejuClawModeSwitch(bool high_freq);
      bool changeMotorParamCallback(kuavo_msgs::changeMotorParamRequest &req, kuavo_msgs::changeMotorParamResponse &res);
      bool getMotorParamCallback(kuavo_msgs::getMotorParamRequest &req, kuavo_msgs::getMotorParamResponse &res);
      void reStartCallback(const std_msgs::Bool::ConstPtr &msg);
      void getBotStandUpCompleteCallback(const std_msgs::Int8::ConstPtr &msg);
      void StandUpLoop();
      void real_init_wait();
      void readyToRecvCmd();
      bool realLaunchStatusCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

      // 新增的三个服务回调函数
      bool adjustZeroPointCallback(kuavo_msgs::adjustZeroPoint::Request &req, kuavo_msgs::adjustZeroPoint::Response &res);
      bool getHardwareReadyCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      bool getMotorZeroPointsCallback(kuavo_msgs::getMotorZeroPoints::Request &req, kuavo_msgs::getMotorZeroPoints::Response &res);
      bool goToMotorZeroPointsCallback(kuavo_msgs::goToZeroPoint::Request &req,kuavo_msgs::goToZeroPoint::Response &res);
      bool switchRobotStateCallback(kuavo_msgs::robotSwitchPose::Request &req, kuavo_msgs::robotSwitchPose::Response &res);  // 新增的服务回调函数


      void sensorThreadFunc(ros::Publisher &sensor_data_pub);
      // void signalHandler(int signal);

      bool realIntialStartCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
      {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) {
          response.success = false;
          response.message = "Hardware has been stopped; restart the node to initialize";
          return true;
        }
        std::cout << "realIntialStartCallback" << std::endl;
        response.success = true;
        hardware_plant_->initial_input_cmd_ = 'o';
        stand_up_cmd_ = hardware_plant_->initial_input_cmd_;
        // 座椅重入等 start 时 StandUpLoop 已退出，需直接置 is_ready（对齐开机按 o）
        // await_start=false 时 StandUpLoop 还在跑，让它自己设 is_ready（已订阅 joint_cmd，时序安全）
        // cali 模式第一次 start 只缩腿（ready_to_feedback_=false），不唤醒控制器避免抢跑
        // 抬手途中按 C（ready_stance→stance）时硬件仍卡在 prep 的 jointMoveTo、尚未订阅 /joint_cmd，
        // 提前置 is_ready 会让控制器在硬件就绪前苏醒，pre-action 目标与手臂实位不连续导致抽动（issue 3978）。
        // 此时不直接置位，'o' 已入队，prep 完成后 StandUpLoop 会置 is_ready，时序安全。
        bool await_start = true;
        ros::param::get("/seat/await_start_after_p5", await_start);
        if (await_start && ready_to_feedback_ && hardware_plant_->hardware_ready_) {
          ros::param::set("/hardware/is_ready", 1);
        }
        return true;
      }

      inline size_t get_num_actuated() const {return hardware_plant_->get_num_actuated();}
      inline HardwareSettings get_motor_info() const {return hardware_plant_->get_motor_info();}
      inline void jointMoveTo(std::vector<double> goal_pos, double speed, double dt = 1e-3, double current_limit=-1){
        if (shutdownRequested()) return;
        hardware_plant_->jointMoveTo(goal_pos, speed, dt, current_limit);
        
      }

      void cmdTruncation(Eigen::VectorXd &cmd_out);

      inline int getDisableMotorId()
      {
        std::lock_guard<std::mutex> lk(disable_motor_mtx_);
        if (disableMotor_.empty())
        {
          return -1;
        }
        auto first_id = *disableMotor_.begin();
        disableMotor_.erase(first_id);
        return first_id;
      }

      void JointProtectionAction(Eigen::VectorXd &cmd_out, std::vector<int> &control_mode);
      
      int JointProtectionTrigger(PROTEST_MODE mode, std::vector<double> &values);  

      void zeroTorqueCalibrateLegs();
      /** 可选 /hardware_prep_moves；speed_index 选速度档位；无参数则用 jointMoveSpeed_ */
      void jointMoveToPrepGoal(const std::vector<double>& goal_pos, int speed_index = 1);
      void applyArmActuatorDynamicsCompensation(const kuavo_msgs::jointCmd::ConstPtr& msg, Eigen::VectorXd& cmd);

      void publishRobotDisabledFlag(bool disabled = true)
      {
          std_msgs::Bool robot_disabled_msg;
          robot_disabled_msg.data = disabled;
          robot_disabled_flag_pub_.publish(robot_disabled_msg);
      }

      void publishDisableWbcFlag()
      {
        std_msgs::Bool enable_wbc_msg;
        enable_wbc_msg.data = false;
        enable_wbc_pub_.publish(enable_wbc_msg);
      }

      void publishDisableMpcFlag()
      {
          std_msgs::Bool enable_mpc_msg;
          enable_mpc_msg.data = false;
          enable_mpc_pub_.publish(enable_mpc_msg);
      }

      void publishEnableMpcFlag()
      {
          std_msgs::Bool enable_mpc_msg;
          enable_mpc_msg.data = true;
          enable_mpc_pub_.publish(enable_mpc_msg);
      }
  };
}
#endif
