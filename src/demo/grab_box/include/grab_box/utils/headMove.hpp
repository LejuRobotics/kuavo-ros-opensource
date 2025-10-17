#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>

#include <kuavo_msgs/setTagId.h>
#include <kuavo_msgs/robotHeadMotionData.h>
#include <kuavo_msgs/jointData.h>
#include <kuavo_msgs/sensorsData.h>

namespace GrabBox
{
  class HeadMove : public BT::StatefulActionNode
  {
  public:
    HeadMove(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config)
    {
      // 节点句柄，这里使用私有命名空间
      ros::NodeHandle nh("~");

      // 服务客户端：关闭/开启连续跟踪
      continuousTrackClient_ = nh.serviceClient<std_srvs::SetBool>("/continuous_track");

      // 发布头部运动数据
      head_orientation_pub_ = nh.advertise<kuavo_msgs::robotHeadMotionData>(
                                "/robot_head_motion_data", 10);

      // 订阅头部实际关节角度等传感器数据（弧度）
      // 注意这里需要根据自己项目中真实的话题名称替换 "/sensors_data_raw"
      joint_data_sub_ = nh.subscribe("/sensors_data_raw", 
                                     10, 
                                     &HeadMove::jointDataCallback, 
                                     this);
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<int>("control_mode"), // 0: Sweep, 1: Waypoints
          // 如果需要从外部读取其它端口，可以在这里补充
      };
    }

    // 行为开始时调用一次
    BT::NodeStatus onStart() override
    {
      // 先关掉连续跟踪
      if (!setContinuousTracking(false))
      {
        ROS_ERROR("[HeadMove] Failed to stop continuous tracking");
        return BT::NodeStatus::FAILURE;
      }
      else
      {
        config().blackboard->set<bool>("continuous_tracking_started", false);
      }

      // 从黑板读取 headMove.head_traj（单位：度）
      int control_mode = 0;
      getInput("control_mode", control_mode);
      if (control_mode == 0)
        {
          head_move_traj_ = getParamsFromBlackboard<std::vector<double>>(
              config(), "headMove.head_traj");
        }
        else if (control_mode == 1)
        {
          head_move_traj_ = getParamsFromBlackboard<std::vector<double>>(
              config(), "headMove.head_part_detect");
        }
        else
        {
          ROS_ERROR("[HeadMove] Invalid control mode: %d", control_mode);
          return BT::NodeStatus::FAILURE;
        } 
        

      head_move_err_ = getParamsFromBlackboard<double>(
          config(), "headMove.head_move_err");

      // 额外增加一个从黑板读取的单点超时（单位：秒），如没设置就默认给 3.0
      max_time_for_waypoint_ = getParamsFromBlackboard<double>(
          config(), "headMove.head_move_timeout");

      // head_move_traj_ 形如 [yaw1, pitch1, yaw2, pitch2, ...] （度）
      // 因此它的长度必须是 2 的倍数
      if (head_move_traj_.empty() || head_move_traj_.size() % 2 != 0)
      {
        ROS_ERROR("[HeadMove] Invalid head_move_traj size.");
        return BT::NodeStatus::FAILURE;
      }

      // 索引从 0 开始
      current_waypoint_idx_ = 0;

      // 初始化当前点的起始时间
      waypoint_start_time_ = ros::Time::now();

      ROS_INFO("[HeadMove] Start head movement, total points: %lu",
               head_move_traj_.size()/2);

      return BT::NodeStatus::RUNNING;
    }

    // 正在执行时每次都会调用
    BT::NodeStatus onRunning() override
    {
      // 如果所有目标点都执行完，返回 SUCCESS
      if (current_waypoint_idx_ >= head_move_traj_.size() / 2)
      {
        return BT::NodeStatus::SUCCESS;
      }

      // 当前目标角度（度）
      double target_yaw_deg   = head_move_traj_[current_waypoint_idx_ * 2 + 0];
      double target_pitch_deg = head_move_traj_[current_waypoint_idx_ * 2 + 1];

      // 发布目标角度指令
      publishHeadOrientationCommand(target_pitch_deg, target_yaw_deg);

      // 判断是否到达目标角度（允许误差为 head_move_err_，单位：度）
      double pitch_err = fabs(current_pitch_deg_ - target_pitch_deg);
      double yaw_err   = fabs(current_yaw_deg_   - target_yaw_deg);

      bool reached = false;

      // 先看角度误差是否满足
      if (pitch_err <= head_move_err_ && yaw_err <= head_move_err_)
      {
        reached = true;
      }
      else
      {
        // 如果还没到误差范围，则查看是否已经超时
        double elapsed = (ros::Time::now() - waypoint_start_time_).toSec();
        if (elapsed >= max_time_for_waypoint_)
        {
          ROS_WARN("[HeadMove] Waypoint %lu not reached in %.2fs, forcing next. (pitch_err=%.2f, yaw_err=%.2f)",
                   current_waypoint_idx_, elapsed, pitch_err, yaw_err);
          reached = true;
        }
      }

      if (reached)
      {
        // 已到达(或者超时强制认为到达)当前目标点，则切换至下一个点
        ROS_INFO("[HeadMove] Reached or forced waypoint %lu", current_waypoint_idx_);
        current_waypoint_idx_++;

        // 更新下一目标点的起始时间
        waypoint_start_time_ = ros::Time::now();

        // 如果已经是最后一个点，也可以在这里再判断一次是否结束
        if (current_waypoint_idx_ >= head_move_traj_.size() / 2)
        {
          ROS_INFO("[HeadMove] All waypoints reached");
          return BT::NodeStatus::SUCCESS;
        }
      }

      // 如果还没到最后一个点，返回 RUNNING，下一帧继续执行
      return BT::NodeStatus::RUNNING;
    }

    // 当外部强行打断本节点时调用
    void onHalted() override
    {
      ROS_WARN("[HeadMove] interrupted");
    }

  private:

    /// @brief 关闭或开启连续追踪的服务调用
    bool setContinuousTracking(bool enable)
    {
      std_srvs::SetBool srv;
      srv.request.data = enable;
      if (!continuousTrackClient_.call(srv) || !srv.response.success)
      {
        ROS_WARN("Failed to set continuous tracking = %s.",
                 enable ? "true" : "false");
        return false;
      }
      return true;
    }

    /// @brief 订阅 /sensors_data_raw 的回调
    ///        获取头部实际关节角度（joint_q 的倒数 2 个值），其单位是弧度
    void jointDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg)
    {
      // 确保 joint_q 长度足够
      if (msg->joint_data.joint_q.size() < 2)
      {
        ROS_WARN_THROTTLE(1.0, "[HeadMove] joint_q size < 2.");
        return;
      }

      // 头部俯仰 pitch = joint_q 的倒数第 1 个
      double pitch_rad = msg->joint_data.joint_q[msg->joint_data.joint_q.size() - 1];
      // 头部偏航 yaw   = joint_q 的倒数第 2 个
      double yaw_rad   = msg->joint_data.joint_q[msg->joint_data.joint_q.size() - 2];

      // 弧度转角度
      current_pitch_deg_ = pitch_rad * 180.0 / M_PI;
      current_yaw_deg_   = yaw_rad   * 180.0 / M_PI;
    }

    /// @brief 发布头部运动指令
    ///        这里 pitch, yaw 都是“度”！！
    void publishHeadOrientationCommand(double pitch_deg, double yaw_deg)
    {
      // 限幅
      yaw_deg   = std::max(-30.0, std::min(30.0, yaw_deg));
      pitch_deg = std::max(-35.0, std::min(35.0, pitch_deg));

      kuavo_msgs::robotHeadMotionData head_cmd;
      head_cmd.joint_data.resize(2);

      //  joint_data[0] = yaw,  joint_data[1] = pitch
      head_cmd.joint_data[0] = yaw_deg;
      head_cmd.joint_data[1] = pitch_deg;

      head_orientation_pub_.publish(head_cmd);
    }

  private:
    ros::ServiceClient continuousTrackClient_;   // 控制连续追踪的服务客户端
    ros::Subscriber joint_data_sub_;             // 订阅实际头部关节角度
    ros::Publisher head_orientation_pub_;        // 发布头部运动指令

    std::vector<double> head_move_traj_;         // 从黑板读取到的目标轨迹（度）
    size_t current_waypoint_idx_{0};             // 当前目标索引
    double head_move_err_{3.0};                  // 允许误差（度）

    // 为了给每个 waypoint 限时，增加以下成员
    double max_time_for_waypoint_{3.0};          // 到达一个点的超时时间（秒）
    ros::Time waypoint_start_time_;              // 当前点开始执行的时间戳

    // 实时头部关节角度（度）
    double current_pitch_deg_{0.0};
    double current_yaw_deg_{0.0};
  };
} // namespace GrabBox
