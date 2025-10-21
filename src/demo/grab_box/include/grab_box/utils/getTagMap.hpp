#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <kuavo_msgs/setTagId.h>
#include <kuavo_msgs/tagDataArray.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <unordered_map>

using TagStatusMap = std::unordered_map<int, int>; // tag_id -> 状态

namespace GrabBox
{
  class GetTagMap : public BT::ConditionNode
  {
  public:
    GetTagMap(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::ConditionNode(name, config)
    {
      ros::NodeHandle nh = ros::NodeHandle("~");
      tag_data_suber_ = nh.subscribe("/tag_data", 10, &GetTagMap::tagDataCallBack, this);
      sorted_tags_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/test/sorted_tag_ids", 10);
    }

    static BT::PortsList providedPorts()
    {
      return {
              BT::InputPort<int>("put_tag_id"),               // 输入：目标放置位置的tag_id
              BT::OutputPort<TagStatusMap>("tag_status_map"),  // 输出：处理后的TagStatusMap
              BT::OutputPort<std::vector<int>>("sorted_tag_ids")
      };
    }

    BT::NodeStatus tick() override
    {
      // 检查是否收到数据
      if (!data_received_) {
        ROS_WARN_THROTTLE(1, "Waiting for /tag_data...");
        return BT::NodeStatus::RUNNING;
      }

      // 获取输入的 put_tag_id
      if (!getInput("put_tag_id", put_tag_id_)) {
        throw BT::RuntimeError("Missing required input [put_tag_id]");
      }

      // 构建 tag_status_map
      TagStatusMap tag_status_map;
      
      // 清空之前的排序数据
      sortable_tags_.clear();

      {
        std::lock_guard<std::mutex> lock(data_mutex_); // 保护共享数据
        for (const auto& [tag_id, pose] : tag_data_) {
          // 初始化状态为0（未操作），跳过与 put_tag_id 相同的tag
          if (tag_id != put_tag_id_) {
            tag_status_map[tag_id] = 0;

            double height = pose(2);
            sortable_tags_.emplace_back(tag_id, height);
          }
        }
      }

      // 根据高度(Y)降序排序
      std::sort(sortable_tags_.begin(), sortable_tags_.end(),
      [](const auto &a, const auto &b) {
          return a.second > b.second;
      });

      // 生成sorted_tag_ids
      std::vector<int> sorted_tag_ids;
      for (const auto &item : sortable_tags_) {
      sorted_tag_ids.push_back(item.first);
      std::cout << "tag_id: " << item.first << " height: " << item.second << std::endl;
      }

      // 输出 tag_status_map
      setOutput("tag_status_map", tag_status_map);
      setOutput("sorted_tag_ids", sorted_tag_ids);
      
      // 发布排序后的标签ID
      std_msgs::Int32MultiArray sorted_tags_msg;
      sorted_tags_msg.data = sorted_tag_ids;
      sorted_tags_pub_.publish(sorted_tags_msg);
      
      ROS_INFO("Generated TagStatusMap with %lu entries", tag_status_map.size());

      return BT::NodeStatus::SUCCESS;
    }

    void tagDataCallBack(const kuavo_msgs::tagDataArray::ConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(data_mutex_); // 保护共享数据
      tag_data_.clear(); // 清除旧数据

      for (size_t i = 0; i < msg->tag_ids.size(); ++i) {
        int tag_id = msg->tag_ids[i];
        const auto& pose = msg->tag_poses[i];

        // 转换为 Eigen::VectorXd 并存储
        Eigen::VectorXd tag_pose(7);
        tag_pose(0) = pose.position.x;
        tag_pose(1) = pose.position.y;
        tag_pose(2) = pose.position.z;
        tag_pose(3) = pose.orientation.x;
        tag_pose(4) = pose.orientation.y;
        tag_pose(5) = pose.orientation.z;
        tag_pose(6) = pose.orientation.w;

        tag_data_[tag_id] = tag_pose;
      }

      data_received_ = true; // 标记为已收到数据
      // ROS_INFO("Received /tag_data with %lu tags", tag_data_.size());
    }

  private:
    ros::Subscriber tag_data_suber_;          // 订阅 /tag_data
    ros::Publisher sorted_tags_pub_;          // 发布排序后的标签ID
    bool data_received_ = false;              // 是否接收到数据的标志
    int put_tag_id_ = -1;                     // 放置位置的目标 tag_id
    std::map<int, Eigen::VectorXd> tag_data_; // 存储收到的标签数据
    std::vector<std::pair<int, double>> sortable_tags_; 
    std::mutex data_mutex_;                   // 保护 tag_data_ 的互斥锁
  };

} // namespace GrabBox
