#pragma once

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <unordered_map>
#include <string>
#include <ros/master.h>
#include <vector>
#include <string>
#include <rosbag/recorder.h>
#include <ros/callback_queue.h>
#include <ros/topic.h>
#include <ocs2_core/Types.h>
#include <thread>
#include <mutex>
#include <condition_variable>
namespace ocs2
{
  namespace humanoid
  {
    class TopicLogger
    {
    public:
      TopicLogger();
      TopicLogger(ros::NodeHandle& input_nh);
      ~TopicLogger() {stopThread();};
      void predefinedTopics();

      void startThread() {
        std::cout << "start a logger thread...\n";
        if (!thread_running_) {
            thread_running_ = true;
            worker_thread_ = std::thread(&TopicLogger::processQueue, this);
        }
      }

      void stopThread() {
          thread_running_ = false;
          cv_.notify_all();
          if (worker_thread_.joinable()) {
              worker_thread_.join();
          }
      }
      
      void publishVector(const std::string &topic_name, const ocs2::vector_t &data, const std::string &label = "");

      void publishVector(const std::string &topic_name, const std::vector<double> &data, const std::string &label = "");
      void publishValue(const std::string &topic_name, const double &data);
      // asynchronous callback
      void publishVectorCallback(const std::string &topic_name, const ocs2::vector_t &data)
      {
        std::vector<double> stdVector(data.data(), data.data() + data.size());
        publishStdVectorCallback(topic_name, stdVector);
      }

      // asynchronous callback
      void publishStdVectorCallback(const std::string &topic_name, const std::vector<double> &data)
      {
        if (!thread_running_)
          return;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          queue_.emplace(topic_name, data);
        }
        cv_.notify_one();
      }


    private:
      void processQueue() {
          while (thread_running_) {
              std::pair<std::string, std::vector<double>> item;
              {
                  std::unique_lock<std::mutex> lock(mutex_);
                  cv_.wait(lock, [this]() { return !queue_.empty() || !thread_running_; });
                  if (!thread_running_) break;
                  item = queue_.front();
                  queue_.pop();
              }
              publishVector(item.first, item.second);
          }
      }
      ros::NodeHandle nh_;
      std::unordered_map<std::string, ros::Publisher> publishedTopics_;
      std::unordered_map<std::string, ros::Publisher> publishedValueTopics_;
      std::queue<std::pair<std::string, std::vector<double>>> queue_;
      std::thread worker_thread_;
      std::mutex mutex_;
      std::condition_variable cv_;

      bool thread_running_ = false;
    };

  }
}
