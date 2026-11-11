#pragma once

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h" //waao
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
#include <array>
#include <atomic>
#include <memory>
#include <cstring>
namespace ocs2
{
  namespace humanoid
  {
    class TopicLogger
    {
    public:
      TopicLogger();
      TopicLogger(ros::NodeHandle& input_nh);
      ~TopicLogger() {stopAsync(); stopThread();};
      void startAsync();
      void stopAsync();
      uint64_t asyncDropped() const {return async_dropped_.load();}
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
      void publishText(const std::string &topic_name, const std::string &data); //waao
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
      // Observation-only MPSC ring. Producers never wait for ROS, the writer,
      // another producer, allocation or queue space. Saturation drops logs.
      struct AsyncRow {
        char topic[192]{};
        char text[1024]{};
        std::array<double,256> data{};
        size_t size=0; int kind=0;
      };
      static constexpr size_t kAsyncCapacity=1024;
      std::unique_ptr<std::array<AsyncRow,kAsyncCapacity>> async_rows_;
      std::mutex async_mutex_;
      size_t async_head_=0, async_tail_=0;
      std::atomic<bool> async_running_{false};
      std::atomic<uint64_t> async_dropped_{0};
      std::thread async_thread_;
      bool enqueue(int kind,const std::string&,const double*,size_t,const std::string* = nullptr);
      static thread_local const TopicLogger* draining_;
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
      std::unordered_map<std::string, ros::Publisher> publishedTextTopics_; //waao
      std::queue<std::pair<std::string, std::vector<double>>> queue_;
      std::thread worker_thread_;
      std::mutex mutex_;
      std::mutex publish_mutex_;
      std::condition_variable cv_;

      std::atomic<bool> thread_running_{false};
    };

  }
}
