#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Core>
#include <Eigen/Geometry> // Required for Eigen::Quaterniond and transformations
#include <cmath>
#include <tuple>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <grab_box/utils/customIoPort.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/twoArmHandPoseCmdSrv.h"
#include "kuavo_msgs/ikSolveParam.h"
#include "kuavo_msgs/fkSrv.h"
#include "kuavo_msgs/headBodyPose.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/armTargetPoses.h"
#include "grab_box/common/ocs2_ros_interface.hpp"
#include "grab_box/common/drake_interface.hpp"
#include "grab_box/common/math.hpp"
#include "grab_box/utils/cubic_interpolator.hpp"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "ros/serialization.h"
#include "kuavo_msgs/setMmCtrlFrame.h"

namespace GrabBox
{
  using namespace std::chrono;
  // Example of Asynchronous node that uses StatefulActionNode as base class
  enum class ClawHandSide{
    Left = 0,
    Right = 1
  };
  // enum class FrameType{
  //   CurrentFrame = 0,    // keep current frame
  //   WorldFrame = 1,     
  //   LocalFrame = 2,
  //   VRFrame = 3,
  //   MmWorldFrame = 4
  // };
  enum class ClawPartType{
    Go = 0,
    Back = 1
  };
  class ClawPart : public BT::StatefulActionNode
  {
  public:
    ClawPart(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    {
      ros::NodeHandle nh;
      Manipulator_traj_pub_ = nh.advertise<kuavo_msgs::armTargetPoses>("/mm/end_effector_trajectory", 1);
      changeManipulatorControlFrame_srv_ = nh.serviceClient<kuavo_msgs::setMmCtrlFrame>("/set_mm_ctrl_frame");
      changeManipulatorControlFlow_srv_ = nh.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_mm_wbc_arm_trajectory_control");

      while(!nh.hasParam("/com_height"))
      {
        ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      ROS_INFO_STREAM("com_height parameter is founded.");
      nh.getParam("/com_height", com_height_);
      max_hand_dis_ = getParamsFromBlackboard<double>(config, "claw_part.max_hand_dis");
      hand_move_spd_ = getParamsFromBlackboard<double>(config, "claw_part.hand_move_spd");
      box_holdon_move_spd_ = getParamsFromBlackboard<double>(config, "claw_part.box_holdon_move_spd");
      claw_up_height_ = getParamsFromBlackboard<double>(config, "claw_part.claw_up_height");
      auto torso_move_spd = getParamsFromBlackboard<std::vector<double>>(config, "claw_part.torso_move_spd");
      torso_z_spd_ = torso_move_spd[0];
      torso_pitch_spd_ = torso_move_spd[1] * M_PI / 180.0;//rad/s
      auto go_traj = getParamsFromBlackboard<std::vector<double>>(config, "claw_part.go_traj");
      auto back_traj = getParamsFromBlackboard<std::vector<double>>(config, "claw_part.back_traj");
      int tick_rate = getParamsFromBlackboard<int>(config, "tick_rate");
      if(go_traj.size() % 6 != 0 || back_traj.size() % 6 != 0)
      {
        ROS_ERROR("Invalid go_traj or back_traj size.");
        abort();
      }
      for(auto g : go_traj)
        std::cout << "go_traj: " << g << std::endl;
      for(int i = 0; i < go_traj.size()/6; i++){
        go_trajectory_.push_back(Eigen::Map<Vector6d>(go_traj.data() + i*6, 6));
        std::cout << "go_trajectory_[" << i << "]: " << go_trajectory_[i].transpose() << std::endl;
      }
      for(int i = 0; i < back_traj.size()/6; i++){
        back_trajectory_.push_back(Eigen::Map<Vector6d>(back_traj.data() + i*6, 6));
        std::cout << "back_trajectory_[" << i << "]: " << back_trajectory_[i].transpose() << std::endl;
      }
      dt_ = 1 / static_cast<double>(tick_rate);
      std::cout << "dt: " << dt_ << std::endl;
    }

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<Eigen::Vector3d>("hand_pos"), BT::InputPort<Eigen::Vector4d>("hand_quat")
        , BT::InputPort<Eigen::Vector3d>("hand_offset"), BT::InputPort<int>("hand_side"), BT::InputPort<ClawPartType>("part_type")};
    }

    BT::NodeStatus onStart() override
    {
      changeArmCtrlModeSrv(2);//using external controller
      if(!changeManipulatorControlFlow(1)) // 手臂关节直接传递给wbc
      {
        ROS_ERROR("Failed to set enable_wbc_arm_trajectory control mode.");
        return BT::NodeStatus::FAILURE;
      }
      if(!changeManipulatorControlFrame(FrameType::WorldFrame))
      {
        ROS_ERROR("Failed to set mm_ctrl_frame to WorldFrame.");
        return BT::NodeStatus::FAILURE;
      }
      Eigen::Vector3d hand_pos, hand_offset;
      Eigen::Vector4d hand_quat;
      int part_type_int = 0;
      getInput("hand_pos", hand_pos);
      getInput("hand_offset", hand_offset);
      getInput("hand_quat", hand_quat);
      getInput("part_type", part_type_int);
      part_type_ = static_cast<ClawPartType>(part_type_int);
      if (!config().blackboard->get("torso_pose", initial_torso_pose_)) 
      {
        ROS_ERROR("Cannot get initial_torso_pose_ from blackboard");
        return BT::NodeStatus::FAILURE;
      }

      config().blackboard->get("real_hand_poses", initial_two_hand_pose_);
      if(abs(initial_two_hand_pose_.first.first(2)) < 0.1 && abs(initial_two_hand_pose_.second.first(2)) < 0.1) // check if both hands are near zero height. A more robust check might be needed.
      {
        ROS_ERROR("Failed to get valid real_hand_poses from blackboard, or poses are near zero.");
        return BT::NodeStatus::FAILURE;
      }
      twoHandPoseTrajectory_.clear();
      int hand_side_int = 0; 
      getInput("hand_side", hand_side_int);
      hand_side_ = static_cast<ClawHandSide>(hand_side_int);
      
      twoHandPoseTrajectory_ = generateTwoHandPoseTrajectory(hand_pos, hand_offset, hand_quat);
      twoHandPoseTrajectory_.insert(twoHandPoseTrajectory_.begin(), std::make_pair(initial_two_hand_pose_, Eigen::Vector3d::Zero()));
      end_time_ = interpolateTwoHandPoseTrajectoryByCubicSpline(twoHandPoseTrajectory_, hand_move_spd_);

      {
        std::vector<double> t_values = {0, end_time_};
        Vector6d torso_ref_dummy = Vector6d::Zero();
        std::vector<Eigen::VectorXd> pose_values = {torso_ref_dummy, torso_ref_dummy};
        cubic_interp_torso_ = CubicInterpolator(t_values, pose_values);
      }

      if(!resetMpcMrtService())
      {
        ROS_ERROR("Failed to Pre-reset MPC.");
        return BT::NodeStatus::FAILURE;
      }
      ros::Duration(0.5).sleep();
      
      is_runing_ = true;
      current_time_ = 0.0;
      sendArmTrajectory();

      return BT::NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus onRunning() override
    {
      if(current_time_ > end_time_)
      {
        {
          std::cout << "GraspBox finished." << std::endl;
          ros::Duration(1.0).sleep();
          if(!changeKinematicMpcControlMode(0))// 不接入运动学mpc控制
            return BT::NodeStatus::FAILURE;
          if(!changeManipulatorControlFlow(0)) // 关闭wbc手臂直接控制
          {
            ROS_ERROR("Failed to set enable_wbc_arm_trajectory control mode to false.");
            return BT::NodeStatus::FAILURE;
          }
          return BT::NodeStatus::SUCCESS;
        }
      }

      if(current_time_ <= 2*dt_)
      {
        std::cout << "[ClawPart] send!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
          if(!changeKinematicMpcControlMode(1))// 0:Do NOT control, 1:arm only control mode, 2:base only control mode, 3: control base and arm mode
            return BT::NodeStatus::FAILURE;
      }
      current_time_ += dt_;
      ros::Duration(dt_).sleep();
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "ClawPart interrupted" << std::endl;
      is_runing_ = false;
    }


  private:
    void sendArmTrajectory()
    {
      kuavo_msgs::armTargetPoses msg;
      double terrain_height = 0.0;
      config().blackboard->get("terrain_height", terrain_height);

      for (double t = 0; t <= end_time_; t += dt_)
      {
        msg.times.push_back(t);

        HandPose l_pose, r_pose;
        l_pose.first = cubic_interp_lh_.getPos(t);
        l_pose.second = cubic_interp_lh_.getQuat(t);
        r_pose.first = cubic_interp_rh_.getPos(t);
        r_pose.second = cubic_interp_rh_.getQuat(t);

        l_pose.first(2) -= terrain_height;
        r_pose.first(2) -= terrain_height;
        
        // left hand: [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
        msg.values.push_back(l_pose.first.x());
        msg.values.push_back(l_pose.first.y());
        msg.values.push_back(l_pose.first.z());
        msg.values.push_back(l_pose.second.x());
        msg.values.push_back(l_pose.second.y());
        msg.values.push_back(l_pose.second.z());
        msg.values.push_back(l_pose.second.w());

        // right hand: [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
        msg.values.push_back(r_pose.first.x());
        msg.values.push_back(r_pose.first.y());
        msg.values.push_back(r_pose.first.z());
        msg.values.push_back(r_pose.second.x());
        msg.values.push_back(r_pose.second.y());
        msg.values.push_back(r_pose.second.z());
        msg.values.push_back(r_pose.second.w());
      }
      Manipulator_traj_pub_.publish(msg);
    }

    TwoHandPoseWithForceTrajectory generateTwoHandPoseTrajectory(
        const Eigen::Vector3d& target_hand_base_pos_from_input,
        const Eigen::Vector3d& hand_offset_from_input,
        const Eigen::Vector4d& hand_quat_xyzw_from_input)
    {
      TwoHandPoseWithForceTrajectory traj;
      traj.reserve(4);
      HandPose l_hand, r_hand;

      auto add_trajectory_point = [&traj, this](const Eigen::Vector3d& target_hand_base_pos,
                                                const Eigen::Vector3d& hand_offset,
                                                const Eigen::Vector4d& hand_quat_xyzw,
                                                const Eigen::Vector3d& bias,
                                                const ClawHandSide& hand_side)
      {
        HandPose l_hand, r_hand;
        if(hand_side == ClawHandSide::Left)
        {
          l_hand.first = target_hand_base_pos + hand_offset + bias;
          l_hand.second = hand_quat_xyzw;
          // Use initial pose for the right (inactive) hand
          r_hand = initial_two_hand_pose_.second; 
        }
        else // hand_side == ClawHandSide::Right
        {
          r_hand.first = target_hand_base_pos + hand_offset + bias;
          r_hand.second = hand_quat_xyzw;
          // Use initial pose for the left (inactive) hand
          l_hand = initial_two_hand_pose_.first;
        }
        auto two_hand_pose = std::make_pair(l_hand, r_hand);
        traj.push_back(std::make_pair(two_hand_pose, Eigen::Vector3d::Zero()));
      };
      std::vector<Eigen::Vector3d> bias_vec;
      if (part_type_ == ClawPartType::Go)
      {
        for (const auto& bias : go_trajectory_)
        {
          bias_vec.push_back(bias.head<3>());
        }
  
      }
      else if (part_type_ == ClawPartType::Back)
      {
        for (const auto& bias : back_trajectory_)
        {
          bias_vec.push_back(bias.head<3>());
        }
      }
      else
      {
        ROS_ERROR("Invalid part_type.");
        abort();
      }
      for (const auto& bias : bias_vec)
      {
        add_trajectory_point(target_hand_base_pos_from_input, hand_offset_from_input, hand_quat_xyzw_from_input, bias, hand_side_);
      }
      return std::move(traj);
    }

    double interpolateTwoHandPoseTrajectoryByCubicSpline(const TwoHandPoseWithForceTrajectory& traj_with_force, double vel=0.5)
    {
      std::vector<double> t_values_l, t_values_r;
      std::vector<Eigen::VectorXd> pos_values_l, pos_values_r;
      std::vector<Eigen::Quaterniond> quat_values_l, quat_values_r;
      std::vector<Eigen::Vector3d> force_values_l, force_values_r;

      t_values_l.push_back(0);
      t_values_r.push_back(0);
      pos_values_l.push_back(traj_with_force[0].first.first.first);
      pos_values_r.push_back(traj_with_force[0].first.second.first);
      quat_values_l.push_back(traj_with_force[0].first.first.second);
      quat_values_r.push_back(traj_with_force[0].first.second.second);
      force_values_l.push_back(traj_with_force[0].second);
      force_values_r.push_back(traj_with_force[0].second);
      for(int i=0; i<traj_with_force.size()-1; i++)
      {
        const auto &traj = traj_with_force[i].first;
        const auto &traj_next = traj_with_force[i+1].first;
        const auto &force_next = traj_with_force[i+1].second;
        Eigen::Vector3d force_next_r = (Eigen::Vector3d() << force_next[0], -force_next[1], force_next[2]).finished();

        const auto &l_pose = traj.first;
        const auto &l_pose_next = traj_next.first;
        const auto &r_pose = traj.second;
        const auto &r_pose_next = traj_next.second;

        double time_cost_l = std::max(0.05, (l_pose_next.first - l_pose.first).norm() / vel);
        double time_cost_r = std::max(0.05, (r_pose_next.first - r_pose.first).norm() / vel);

        t_values_l.push_back(t_values_l.back() + time_cost_l);
        t_values_r.push_back(t_values_r.back() + time_cost_r);
        pos_values_l.push_back(l_pose_next.first);
        pos_values_r.push_back(r_pose_next.first);
        quat_values_l.push_back(l_pose_next.second);
        quat_values_r.push_back(r_pose_next.second);
        force_values_l.push_back(force_next);
        force_values_r.push_back(force_next_r);
      }
      cubic_interp_lh_ = CubicInterpolator(t_values_l, pos_values_l);
      cubic_interp_lh_.addQuaternion(quat_values_l);
      cubic_interp_lh_.addForce(force_values_l);
      cubic_interp_rh_ = CubicInterpolator(t_values_r, pos_values_r);
      cubic_interp_rh_.addQuaternion(quat_values_r);
      cubic_interp_rh_.addForce(force_values_r);
      return std::max(t_values_l.back(), t_values_r.back());
    }

    bool changeManipulatorControlFlow(int mode)
    {
      kuavo_msgs::changeArmCtrlMode srv;
      srv.request.control_mode = mode;
      if (changeManipulatorControlFlow_srv_.call(srv)) 
      {
        return true;
      }
      else
        ROS_ERROR("Failed to call service enable_mm_wbc_arm_trajectory_control");
      return false;
    }

    bool changeManipulatorControlFrame(FrameType frame_type)
    {
      kuavo_msgs::setMmCtrlFrame srv;
      srv.request.frame = static_cast<int>(frame_type);
      if (changeManipulatorControlFrame_srv_.call(srv))   
      {
        return true;
      }
      else
        ROS_ERROR("Failed to call service set_mm_ctrl_frame");
      return false;
    }


    ros::Publisher Manipulator_traj_pub_;
    ros::ServiceClient changeManipulatorMode_srv_;
    ros::ServiceClient changeArmCtrlModeSrv_;
    ros::ServiceClient changeManipulatorControlFlow_srv_;
    ros::ServiceClient changeManipulatorControlFrame_srv_;

    bool is_runing_;
    kuavo_msgs::ikSolveParam ikParam_;
    double com_height_;
    double max_hand_dis_;
    double hand_move_spd_;
    double box_holdon_move_spd_;
    double claw_up_height_;
    double torso_z_spd_;
    double torso_pitch_spd_;
    Eigen::Vector4d initial_torso_pose_;
    TwoHandPoseWithForceTrajectory twoHandPoseTrajectory_;
    TwoHandPose initial_two_hand_pose_;
    std::vector<Vector6d> go_trajectory_;
    std::vector<Vector6d> back_trajectory_;
    double dt_ = 0.01;// s
    double current_time_ = 0.0;
    double end_time_ = 0.0;
    ClawPartType part_type_;

    ClawHandSide hand_side_;

    HandPose inactive_hand_pose_world_;
    Eigen::Vector3d configured_grasp_force_local_ = {5.0, 0.0, 0.0};

    CubicInterpolator cubic_interp_lh_, cubic_interp_rh_, cubic_interp_torso_;

  };
} // namespace GrabBox