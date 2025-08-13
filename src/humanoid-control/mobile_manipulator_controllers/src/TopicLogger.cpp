#include "mobile_manipulator_controllers/TopicLogger.h"
namespace ocs2
{
  namespace humanoid
  {
    TopicLogger::TopicLogger(ros::NodeHandle& nh) : nh_(nh)
    {
      predefinedTopics();
    }
    TopicLogger::TopicLogger()
    {
      nh_ = ros::NodeHandle();
      predefinedTopics();
    }
    void TopicLogger::predefinedTopics()
    {
      // 预定义一些常用的log记录topic列表, 加快初始发布速度
      // tips: 运行程序，通过以下命令查找筛选需要发布的topic(仅支持Float64MultiArray)：
      // `rostopic list | while read topic; do if [ "$(rostopic type $topic)" == "std_msgs/Float64MultiArray" ]; then echo "\"$topic\""; fi; done`

      const std::vector<std::string> predefinedTopics_ = {
          "/mm/external_state",
          "/mm/next_state",
          "/mm/optimized_input"
      };

      for (const auto &topic : predefinedTopics_)
      {
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64MultiArray>(topic, 10);
        publishedTopics_[topic] = newPublisher;
      }
    }
    void TopicLogger::publishVector(const std::string &topic_name, const ocs2::vector_t &data, const std::string &label)
    {
      std::vector<double> stdVector(data.data(), data.data() + data.size());
      publishVector(topic_name, stdVector, label);
    }
    void TopicLogger::publishVector(const std::string &topic_name, const std::vector<double> &data, const std::string &label)
    {
      if (publishedTopics_.count(topic_name) == 0)
      {
        // 如果该topic尚未发布过，则创建新的发布者
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
        publishedTopics_[topic_name] = newPublisher;
      }

      // 发布数据到对应的topic
      std_msgs::Float64MultiArray arrayMsg;
      arrayMsg.data = data;
      // arrayMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      // arrayMsg.layout.dim[0].size = arrayMsg.data.size();
      // arrayMsg.layout.dim[0].stride = 1;
      // arrayMsg.layout.dim[0].label = label;

      publishedTopics_[topic_name].publish(arrayMsg);
    }
    void TopicLogger::publishValue(const std::string &topic_name, const double &data)
    {
      if (publishedValueTopics_.count(topic_name) == 0)
      {
        // 如果该topic尚未发布过，则创建新的发布者
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64>(topic_name, 10);
        publishedValueTopics_[topic_name] = newPublisher;
      }

      // 发布数据到对应的topic
      std_msgs::Float64 arrayMsg;
      arrayMsg.data = data;
      publishedValueTopics_[topic_name].publish(arrayMsg);
    }
  }
}
