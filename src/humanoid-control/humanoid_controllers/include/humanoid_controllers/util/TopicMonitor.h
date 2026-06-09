#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace humanoid_controller
{
  /**
   * @brief 通用话题监测器（独立 CallbackQueue + AsyncSpinner）
   *
   * 当前能力：
   * - 常驻订阅某个 topic，在回调里仅记录到达时间并估计频率
   * - 对外提供线程安全的快照读取，避免控制路径阻塞等待消息
   *
   * 说明：
   * - 目前实现等价于“到达频率/活性监测”，不关心消息内容
   * - 为了通用性，start() 使用模板参数指定消息类型
   */
  class TopicMonitor
  {
  public:
    enum class CheckResult
    {
      Ok = 0,
      NotPublished,
      NoSamples,
      TooOld,
      NotEnoughSamples,
      HzTooLow
    };

    struct Snapshot
    {
      double hz = 0.0;
      size_t samples = 0;
      ros::WallTime last_msg_walltime{0.0};
      double last_interval_sec = 0.0;
    };

    struct Requirements
    {
      // Require the topic to appear in rosmaster topic list.
      bool must_be_published = true;

      // Minimum cached frequency. <=0 means "don't check".
      double min_hz = 0.0;

      // Maximum allowed age since last message. <=0 means "don't check".
      double max_age_sec = 0.0;

      // Minimum cached samples to consider frequency valid.
      size_t min_samples = 2;
    };

    struct CheckReport
    {
      CheckResult result = CheckResult::NoSamples;
      Snapshot snapshot;
      double age_sec = 0.0;
      std::string reason;
    };

    TopicMonitor() = default;
    ~TopicMonitor();

    TopicMonitor(const TopicMonitor&) = delete;
    TopicMonitor& operator=(const TopicMonitor&) = delete;
    TopicMonitor(TopicMonitor&&) = delete;
    TopicMonitor& operator=(TopicMonitor&&) = delete;

    void setMaxSamples(int max_samples);

    template <typename MsgT>
    void start(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)
    {
      if (spinner_)
      {
        if (!topic_.empty() && topic_ != topic)
        {
          ROS_WARN("[TopicMonitor] start() ignored: already monitoring '%s', requested '%s'",
                   topic_.c_str(), topic.c_str());
        }
        return;  // already started
      }

      topic_ = topic;

      ros::SubscribeOptions opts;
      opts.init<MsgT>(topic, queue_size, boost::bind(&TopicMonitor::onMsg<MsgT>, this, _1));
      opts.callback_queue = &callback_queue_;
      subscriber_ = nh.subscribe(opts);

      spinner_ = std::make_unique<ros::AsyncSpinner>(1, &callback_queue_);
      spinner_->start();
    }

    template <typename MsgT>
    void restart(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)
    {
      stop();
      start<MsgT>(nh, topic, queue_size);
    }

    void stop();

    bool isStarted() const;

    Snapshot snapshot() const;

    bool isAlive(double max_age_sec) const;

    bool meets(double min_hz, double max_age_sec, size_t min_samples = 2) const;

    CheckResult check(double min_hz,
                      double max_age_sec,
                      size_t min_samples,
                      Snapshot* out_snapshot = nullptr,
                      double* out_age_sec = nullptr) const;

    CheckResult check(const Requirements& req, CheckReport* out_report = nullptr) const;

    const std::string& topic() const
    {
      return topic_;
    }

    static bool isTopicPublished(const std::string& topic);

    bool isTopicPublished() const;

    static int statusCode(CheckResult result);

    static void ensurePositiveParam(const char* log_tag, const char* param_name, double& value, double fallback);
    static void ensureMinIntParam(const char* log_tag, const char* param_name, int& value, int min_value, int fallback);

  private:
    static std::string formatReason(const std::string& topic,
                                    CheckResult result,
                                    const Snapshot& snapshot,
                                    double age_sec,
                                    const Requirements* req);

    template <typename MsgT>
    void onMsg(const typename MsgT::ConstPtr& /*msg*/)
    {
      const double now_sec = ros::Time::now().toSec();
      const ros::WallTime now_wall = ros::WallTime::now();

      std::lock_guard<std::mutex> lock(mutex_);
      if (!timestamps_sec_.empty())
      {
        last_interval_sec_ = (now_wall - last_msg_walltime_).toSec();
      }
      last_msg_walltime_ = now_wall;
      timestamps_sec_.push_back(now_sec);

      while (static_cast<int>(timestamps_sec_.size()) > max_samples_)
      {
        timestamps_sec_.pop_front();
      }

      if (timestamps_sec_.size() >= 2)
      {
        const double duration = timestamps_sec_.back() - timestamps_sec_.front();
        cached_hz_ = duration > 0.0 ? static_cast<double>(timestamps_sec_.size() - 1) / duration : 0.0;
      }
    }

  private:
    std::string topic_;

    // 独立回调队列与 spinner，避免与主控制路径抢回调线程
    mutable std::mutex mutex_;
    ros::CallbackQueue callback_queue_;
    ros::Subscriber subscriber_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    // 频率统计缓存
    int max_samples_ = 10;
    std::deque<double> timestamps_sec_;
    ros::WallTime last_msg_walltime_{0.0};
    double last_interval_sec_ = 0.0;
    double cached_hz_ = 0.0;
  };

}  // namespace humanoid_controller
