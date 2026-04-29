#include "humanoid_controllers/util/TopicMonitor.h"

#include <ros/master.h>

namespace humanoid_controller
{
  namespace
  {
    inline const char* normalizeTag(const char* tag)
    {
      return (tag && tag[0] != '\0') ? tag : "[TopicMonitor]";
    }
  }  // namespace

  TopicMonitor::~TopicMonitor()
  {
    stop();
  }

  void TopicMonitor::setMaxSamples(int max_samples)
  {
    const int clamped = std::max(2, max_samples);
    std::lock_guard<std::mutex> lock(mutex_);
    max_samples_ = clamped;
    while (static_cast<int>(timestamps_sec_.size()) > max_samples_)
    {
      timestamps_sec_.pop_front();
    }
  }

  void TopicMonitor::stop()
  {
    if (spinner_)
    {
      spinner_->stop();
    }
    if (subscriber_)
    {
      subscriber_.shutdown();
    }
    spinner_.reset();
  }

  bool TopicMonitor::isStarted() const
  {
    return static_cast<bool>(spinner_);
  }

  TopicMonitor::Snapshot TopicMonitor::snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot s;
    s.hz = cached_hz_;
    s.samples = timestamps_sec_.size();
    s.last_msg_walltime = last_msg_walltime_;
    return s;
  }

  bool TopicMonitor::isAlive(double max_age_sec) const
  {
    const auto s = snapshot();
    if (s.samples == 0)
    {
      return false;
    }
    if (max_age_sec <= 0.0)
    {
      return true;
    }
    const double age_sec = (ros::WallTime::now() - s.last_msg_walltime).toSec();
    return age_sec <= max_age_sec;
  }

  bool TopicMonitor::meets(double min_hz, double max_age_sec, size_t min_samples) const
  {
    return check(min_hz, max_age_sec, min_samples) == CheckResult::Ok;
  }

  TopicMonitor::CheckResult TopicMonitor::check(double min_hz,
                                                double max_age_sec,
                                                size_t min_samples,
                                                Snapshot* out_snapshot,
                                                double* out_age_sec) const
  {
    const auto s = snapshot();
    if (out_snapshot)
    {
      *out_snapshot = s;
    }

    if (s.samples == 0)
    {
      if (out_age_sec)
      {
        *out_age_sec = 0.0;
      }
      return CheckResult::NoSamples;
    }

    const double age_sec = (ros::WallTime::now() - s.last_msg_walltime).toSec();
    if (out_age_sec)
    {
      *out_age_sec = age_sec;
    }

    if (max_age_sec > 0.0 && age_sec > max_age_sec)
    {
      return CheckResult::TooOld;
    }

    if (s.samples < min_samples)
    {
      return CheckResult::NotEnoughSamples;
    }

    if (min_hz > 0.0 && s.hz < min_hz)
    {
      return CheckResult::HzTooLow;
    }

    return CheckResult::Ok;
  }

  TopicMonitor::CheckResult TopicMonitor::check(const Requirements& req, CheckReport* out_report) const
  {
    // Optional rosmaster publication check
    if (req.must_be_published)
    {
      if (topic_.empty() || !isTopicPublished(topic_))
      {
        if (out_report)
        {
          out_report->result = CheckResult::NotPublished;
          out_report->snapshot = snapshot();
          out_report->age_sec = 0.0;
          out_report->reason = "Required topic '" + topic_ + "' is not published.";
        }
        return CheckResult::NotPublished;
      }
    }

    Snapshot s;
    double age_sec = 0.0;
    const auto res = check(req.min_hz, req.max_age_sec, req.min_samples, &s, &age_sec);

    if (out_report)
    {
      out_report->result = res;
      out_report->snapshot = s;
      out_report->age_sec = age_sec;
      out_report->reason = formatReason(topic_, res, s, age_sec, &req);
    }

    return res;
  }

  std::string TopicMonitor::formatReason(const std::string& topic,
                                         CheckResult result,
                                         const Snapshot& snapshot,
                                         double age_sec,
                                         const Requirements* req)
  {
    std::ostringstream oss;
    const std::string t = topic.empty() ? "<unknown>" : topic;

    switch (result)
    {
      case CheckResult::Ok:
        oss << "Topic '" << t << "' is available (cached " << snapshot.hz << " Hz).";
        break;
      case CheckResult::NotPublished:
        oss << "Required topic '" << t << "' is not published.";
        break;
      case CheckResult::NoSamples:
        oss << "Topic '" << t << "' has no cached samples yet.";
        break;
      case CheckResult::TooOld:
        oss << "Topic '" << t << "' has no recent messages (age=" << age_sec << "s).";
        break;
      case CheckResult::NotEnoughSamples:
        oss << "Topic '" << t << "' does not have enough cached samples to estimate frequency.";
        break;
      case CheckResult::HzTooLow:
        oss << "Topic '" << t << "' frequency too low: cached " << snapshot.hz << " Hz.";
        if (req && req->min_hz > 0.0)
        {
          oss << " Require at least " << req->min_hz << " Hz.";
        }
        break;
      default:
        oss << "Topic '" << t << "' check failed.";
        break;
    }

    return oss.str();
  }

  bool TopicMonitor::isTopicPublished(const std::string& topic)
  {
    ros::master::V_TopicInfo published_topics;
    if (!ros::master::getTopics(published_topics))
    {
      return false;
    }
    return std::any_of(published_topics.begin(),
                       published_topics.end(),
                       [&](const ros::master::TopicInfo& info)
                       {
                         return info.name == topic;
                       });
  }

  bool TopicMonitor::isTopicPublished() const
  {
    if (topic_.empty())
    {
      return false;
    }
    return isTopicPublished(topic_);
  }

  void TopicMonitor::ensurePositiveParam(const char* log_tag, const char* param_name, double& value, double fallback)
  {
    if (value <= 0.0)
    {
      ROS_WARN("%s Invalid param '%s'=%.3f, fallback to %.3f", normalizeTag(log_tag), param_name, value, fallback);
      value = fallback;
    }
  }

  void TopicMonitor::ensureMinIntParam(const char* log_tag, const char* param_name, int& value, int min_value, int fallback)
  {
    if (value < min_value)
    {
      ROS_WARN("%s Invalid param '%s'=%d, fallback to %d", normalizeTag(log_tag), param_name, value, fallback);
      value = fallback;
    }
  }

}  // namespace humanoid_controller

