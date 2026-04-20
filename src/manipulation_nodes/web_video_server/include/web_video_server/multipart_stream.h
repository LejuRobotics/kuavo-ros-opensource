#ifndef MULTIPART_STREAM_H_
#define MULTIPART_STREAM_H_

#include <ros/ros.h>
#include <async_web_server_cpp/http_connection.hpp>

#include <queue>
#include <string>
#include <vector>

namespace web_video_server
{

struct PendingFooter {
  std::chrono::steady_clock::time_point timestamp;
  boost::weak_ptr<const void> contents;
};

class MultipartStream {
public:
  struct StatusSnapshot {
    bool found = false;
    std::string level = "unknown";
    uint64_t busy_count = 0;
    uint64_t replace_count = 0;
    uint64_t timeout_drop_count = 0;
    bool has_pending = false;
    size_t pending_size = 0;
  };

  MultipartStream(async_web_server_cpp::HttpConnectionPtr& connection,
                  const std::string& boundry="boundarydonotcross",
                  std::size_t max_queue_size=1,
                  double pending_frame_timeout_seconds=0.1,
                  const std::string& status_key="");

  static std::string makeStatusKey(const std::string& topic, const std::string& stream_type);
  static StatusSnapshot getStatusSnapshot(const std::string& status_key);

  void sendInitialHeader();
  void sendPartHeader(const std::chrono::steady_clock::time_point &time, const std::string& type, size_t payload_size);
  void sendPartFooter(const std::chrono::steady_clock::time_point &time);
  void sendPartAndClear(const std::chrono::steady_clock::time_point &time, const std::string& type, std::vector<unsigned char> &data);
  void sendPart(const std::chrono::steady_clock::time_point &time, const std::string& type, const boost::asio::const_buffer &buffer,
		async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  enum class NetworkHealthLevel {
    kOk,
    kCongested,
    kPoor
  };

  struct PendingPart {
    std::chrono::steady_clock::time_point timestamp;
    std::string type;
    std::vector<unsigned char> data;
  };

  void updateNetworkHealth(const std::chrono::steady_clock::time_point& now);
  void publishStatus() const;
  static const char* networkHealthToString(NetworkHealthLevel level);
  void sendEncodedPartAndClear(const std::chrono::steady_clock::time_point &time, const std::string& type,
                               std::vector<unsigned char> &data);
  bool isBusy();

private:
  const std::size_t max_queue_size_;
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
  std::queue<PendingFooter> pending_footers_;
  bool has_pending_part_ = false;
  PendingPart pending_part_;
  const double pending_frame_timeout_seconds_;
  std::string status_key_;
  uint64_t busy_store_count_ = 0;
  uint64_t busy_replace_count_ = 0;
  uint64_t pending_send_count_ = 0;
  uint64_t pending_timeout_drop_count_ = 0;
  NetworkHealthLevel network_health_level_ = NetworkHealthLevel::kOk;
  std::chrono::steady_clock::time_point network_health_window_start_ = std::chrono::steady_clock::time_point::min();
  uint64_t network_health_busy_store_baseline_ = 0;
  uint64_t network_health_replace_baseline_ = 0;
  uint64_t network_health_timeout_drop_baseline_ = 0;
};

}

#endif
