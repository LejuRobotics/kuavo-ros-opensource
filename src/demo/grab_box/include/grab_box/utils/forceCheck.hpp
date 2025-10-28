#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <kuavo_msgs/setTagId.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>  // 确保包含Eigen头文件

namespace GrabBox
{
  class ForceCheck : public BT::ConditionNode
  {
  public:
    ForceCheck(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::ConditionNode(name, config),
        failure_count_(0)
    {
      ros::NodeHandle nh("~");
      est_force_sub_ = nh.subscribe("/state_estimate/est_arm_contact_force", 10, &ForceCheck::forceCallback, this);
    }

    static BT::PortsList providedPorts()
    {
      return {
              BT::InputPort<Eigen::Vector3d>("force_threshold"), 

            //   BT::OutputPort<Eigen::Vector4d>("box_quat")
      };
    }

    BT::NodeStatus tick() override final
    {
      Eigen::Vector3d force_threshold;
      if (!getInput("force_threshold", force_threshold))
      {
        ROS_ERROR("[ForceCheck] Missing input port: force_threshold");
        return BT::NodeStatus::FAILURE;
      }
      
      // ROS_INFO_STREAM("[ForceCheck] Force threshold: " << force_threshold.transpose());

      if (est_force_.empty())
      {
        ROS_WARN("[ForceCheck] No force data received");
        return BT::NodeStatus::RUNNING;
      }

      bool threshold_exceeded = false;
    //   for (size_t i = 0; i < force_threshold.size(); ++i)
    //   {
    //     double current_force_l = std::abs(est_force_[est_force_leg_num_ + i]);
    //     double current_force_r = std::abs(est_force_[est_force_leg_num_ + 6 + i]);
    //     if (force_threshold(i) >= current_force_l || force_threshold(i) >= current_force_r) 
    //     {   
    //         ROS_WARN("[ForceCheck] Force threshold exceeded");
    //         std::cout << "[ForceCheck] Force[" << est_force_leg_num_ + i << "] : " << est_force_[est_force_leg_num_ + i] << std::endl;
    //         std::cout << "[ForceCheck] Force[" << est_force_leg_num_ + 6 + i << "] : " << est_force_[est_force_leg_num_ + 6 + i] << std::endl;
    //         threshold_exceeded = true;
    //         break;
    //     }
    //   }


      double current_force_l = std::sqrt(est_force_[est_force_leg_num_ + 0] * est_force_[est_force_leg_num_ + 0] + est_force_[est_force_leg_num_ + 1] * est_force_[est_force_leg_num_ + 1] + est_force_[est_force_leg_num_ + 2] * est_force_[est_force_leg_num_ + 2]);
      double current_force_r = std::sqrt(est_force_[est_force_leg_num_ + 6] * est_force_[est_force_leg_num_ + 6] + est_force_[est_force_leg_num_ + 7] * est_force_[est_force_leg_num_ + 7] + est_force_[est_force_leg_num_ + 8] * est_force_[est_force_leg_num_ + 8]);
      if (force_threshold.norm() >= current_force_l || force_threshold.norm() >= current_force_r) 
        {   
            ROS_WARN("[ForceCheck] Force threshold exceeded");

            std::cout << "[ForceCheck] Force [ current_force_l ] : " << current_force_l << std::endl;
            std::cout << "[ForceCheck] Force [ current_force_r ] : " << current_force_r << std::endl;
            threshold_exceeded = true;
        }
      

      if (threshold_exceeded)
      {
        failure_count_++;
        ROS_WARN_STREAM("[ForceCheck] Failure count: " << failure_count_);
        ros::Duration(0.01).sleep();

        // for(size_t i = 0; i < est_force_.size(); ++i)
        // {
        //     ROS_INFO_STREAM("[ForceCheck] Force[" << i << "] : " << est_force_[i]);
        // }
        if (failure_count_ >= required_failures_)
        {
          ROS_ERROR("[ForceCheck] Force threshold exceeded consecutively. Returning FAILURE.");
          failure_count_ = 0;  // 重置计数器
          return BT::NodeStatus::FAILURE;
        }
      }
      else
      {
        if (failure_count_ > 0)
        {
          ROS_INFO("[ForceCheck] Force within threshold. Resetting failure count.");
        }
        failure_count_ = 0;  // 重置计数器
      }
      return BT::NodeStatus::SUCCESS;
    }


  private:

    void forceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        // ROS_DEBUG_STREAM("[ForceCheck] Force Size : " << msg->data.size());
        // for(size_t i = 0; i < msg->data.size(); ++i)
        // {
        //     ROS_DEBUG_STREAM("[ForceCheck] Force[" << i << "] : " << msg->data[i]);
        // }
        est_force_ = msg->data;
    }

    ros::Subscriber est_force_sub_;
    std::vector<double> est_force_;
    int est_force_leg_num_ = 12;

    
    int failure_count_;
    int required_failures_ = 30;
    
  };
} // namespace GrabBox
