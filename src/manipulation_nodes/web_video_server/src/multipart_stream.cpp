#include "web_video_server/multipart_stream.h"
#include "async_web_server_cpp/http_reply.hpp"

#include <map>
#include <sstream>

namespace web_video_server
{

namespace
{
boost::mutex g_status_mutex;
std::map<std::string, MultipartStream::StatusSnapshot> g_status_by_key;

struct MultipartWriteResource
{
  std::string headers;
  std::string footer;
  std::vector<unsigned char> owned_payload;
  boost::asio::const_buffer payload_buffer;
  async_web_server_cpp::HttpConnection::ResourcePtr payload_resource;
};

std::string buildMultipartHeaders(const std::chrono::steady_clock::time_point &time,
                                  const std::string &type,
                                  size_t payload_size)
{
  char stamp[20];
  snprintf(stamp, sizeof(stamp), "%.06lf",
      std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());

  std::ostringstream stream;
  stream << "Content-type: " << type << "\r\n"
         << "X-Timestamp: " << stamp << "\r\n"
         << "Access-Control-Allow-Origin: *\r\n"
         << "Access-Control-Allow-Methods: GET,POST,PUT,DELETE,HEAD,OPTIONS\r\n"
         << "Access-Control-Allow-Headers: Origin, Authorization, Accept, Content-Type\r\n"
         << "Access-Control-Max-Age: 3600\r\n"
         << "Content-Length: " << payload_size << "\r\n\r\n";
  return stream.str();
}
}

MultipartStream::MultipartStream(
    async_web_server_cpp::HttpConnectionPtr& connection,
    const std::string& boundry,
    std::size_t max_queue_size,
    double pending_frame_timeout_seconds,
    const std::string& status_key)
  : max_queue_size_(max_queue_size),
    connection_(connection),
    boundry_(boundry),
    pending_frame_timeout_seconds_(pending_frame_timeout_seconds),
    status_key_(status_key)
{
  publishStatus();
}

std::string MultipartStream::makeStatusKey(const std::string& topic, const std::string& stream_type)
{
  return stream_type + ":" + topic;
}

MultipartStream::StatusSnapshot MultipartStream::getStatusSnapshot(const std::string& status_key)
{
  boost::mutex::scoped_lock lock(g_status_mutex);
  auto it = g_status_by_key.find(status_key);
  if (it == g_status_by_key.end()) {
    return StatusSnapshot();
  }
  StatusSnapshot snapshot = it->second;
  snapshot.found = true;
  return snapshot;
}

void MultipartStream::sendInitialHeader() {
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Content-type", "multipart/x-mixed-replace;boundary="+boundry_).header(
      "Access-Control-Allow-Origin", "*").header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,HEAD,OPTIONS").header(
      "Access-Control-Allow-Headers", "Origin, Authorization, Accept, Content-Type").header("Access-Control-Max-Age", "3600").write(connection_);
  connection_->write("--"+boundry_+"\r\n");
}

void MultipartStream::sendPartHeader(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  size_t payload_size)
{
  char stamp[20];
  snprintf(stamp, sizeof(stamp), "%.06lf",
      std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());
  boost::shared_ptr<std::vector<async_web_server_cpp::HttpHeader> > headers(
      new std::vector<async_web_server_cpp::HttpHeader>());
  headers->push_back(async_web_server_cpp::HttpHeader("Content-type", type));
  headers->push_back(async_web_server_cpp::HttpHeader("X-Timestamp", stamp));
  headers->push_back(async_web_server_cpp::HttpHeader("Access-Control-Allow-Origin", "*"));
  headers->push_back(async_web_server_cpp::HttpHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,HEAD,OPTIONS"));
  headers->push_back(async_web_server_cpp::HttpHeader("Access-Control-Allow-Headers", "Origin, Authorization, Accept, Content-Type"));
  headers->push_back(async_web_server_cpp::HttpHeader("Access-Control-Max-Age", "3600"));
  headers->push_back(
      async_web_server_cpp::HttpHeader("Content-Length", boost::lexical_cast<std::string>(payload_size)));
  connection_->write(async_web_server_cpp::HttpReply::to_buffers(*headers), headers);
}

void MultipartStream::sendPartFooter(const std::chrono::steady_clock::time_point & time)
{
  boost::shared_ptr<std::string> str(new std::string("\r\n--"+boundry_+"\r\n"));
  PendingFooter pf;
  pf.timestamp = time;
  pf.contents = str;
  connection_->write(boost::asio::buffer(*str), str);
  if (max_queue_size_ > 0) pending_footers_.push(pf);
}

void MultipartStream::sendPartAndClear(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  std::vector<unsigned char> & data)
{
  const auto now = std::chrono::steady_clock::now();
  if (has_pending_part_ && pending_frame_timeout_seconds_ > 0.0) {
    const double pending_age = std::chrono::duration_cast<std::chrono::duration<double>>(now - pending_part_.timestamp).count();
    if (pending_age > pending_frame_timeout_seconds_) {
      ++pending_timeout_drop_count_;
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "[multipart_stream] drop_pending_timeout pending_age_s=" << pending_age
          << " drop_count=" << pending_timeout_drop_count_
          << " store_count=" << busy_store_count_
          << " replace_count=" << busy_replace_count_
          << " sent_pending=" << pending_send_count_);
      has_pending_part_ = false;
      pending_part_.data.clear();
      publishStatus();
    }
  }

  updateNetworkHealth(now);

  if (isBusy()) {
    ++busy_store_count_;
    if (has_pending_part_) {
      ++busy_replace_count_;
    }
    pending_part_.timestamp = time;
    pending_part_.type = type;
    pending_part_.data.swap(data);
    has_pending_part_ = true;
    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "[multipart_stream] busy pending=" << (has_pending_part_ ? 1 : 0)
        << " pending_size=" << pending_part_.data.size()
        << " store_count=" << busy_store_count_
        << " replace_count=" << busy_replace_count_
        << " sent_pending=" << pending_send_count_
        << " timeout_drop=" << pending_timeout_drop_count_);
    publishStatus();
    return;
  }

  if (has_pending_part_) {
    if (pending_part_.timestamp >= time) {
      ++pending_send_count_;
      ROS_INFO_STREAM_THROTTLE(
          1.0,
          "[multipart_stream] flush_latest pending_size=" << pending_part_.data.size()
          << " store_count=" << busy_store_count_
          << " replace_count=" << busy_replace_count_
          << " sent_pending=" << pending_send_count_
          << " timeout_drop=" << pending_timeout_drop_count_);
      sendEncodedPartAndClear(pending_part_.timestamp, pending_part_.type, pending_part_.data);
      has_pending_part_ = false;
      publishStatus();
      return;
    }
    ROS_DEBUG_STREAM("Dropping stale pending multipart frame in favor of current frame");
    has_pending_part_ = false;
    pending_part_.data.clear();
    publishStatus();
  }

  sendEncodedPartAndClear(time, type, data);
}

void MultipartStream::sendPart(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  const boost::asio::const_buffer & buffer,
  async_web_server_cpp::HttpConnection::ResourcePtr resource)
{
  if (!isBusy()) {
    boost::shared_ptr<MultipartWriteResource> part(new MultipartWriteResource());
    part->headers = buildMultipartHeaders(time, type, boost::asio::buffer_size(buffer));
    part->payload_buffer = buffer;
    part->payload_resource = resource;
    part->footer = "\r\n--" + boundry_ + "\r\n";

    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(part->headers));
    buffers.push_back(part->payload_buffer);
    buffers.push_back(boost::asio::buffer(part->footer));
    connection_->write_replaceable(buffers, part);

    PendingFooter pf;
    pf.timestamp = time;
    pf.contents = part;
    if (max_queue_size_ > 0) pending_footers_.push(pf);
    publishStatus();
  }
}

void MultipartStream::sendEncodedPartAndClear(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  std::vector<unsigned char> & data)
{
  boost::shared_ptr<MultipartWriteResource> part(new MultipartWriteResource());
  part->headers = buildMultipartHeaders(time, type, data.size());
  part->owned_payload.swap(data);
  part->payload_buffer = boost::asio::buffer(part->owned_payload);
  part->footer = "\r\n--" + boundry_ + "\r\n";

  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(part->headers));
  buffers.push_back(part->payload_buffer);
  buffers.push_back(boost::asio::buffer(part->footer));
  connection_->write_replaceable(buffers, part);

  PendingFooter pf;
  pf.timestamp = time;
  pf.contents = part;
  if (max_queue_size_ > 0) pending_footers_.push(pf);
  publishStatus();
}

void MultipartStream::updateNetworkHealth(const std::chrono::steady_clock::time_point& now)
{
  if (network_health_window_start_ == std::chrono::steady_clock::time_point::min()) {
    network_health_window_start_ = now;
    network_health_busy_store_baseline_ = busy_store_count_;
    network_health_replace_baseline_ = busy_replace_count_;
    network_health_timeout_drop_baseline_ = pending_timeout_drop_count_;
    return;
  }

  const double window_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - network_health_window_start_).count();
  if (window_seconds < 2.0) {
    return;
  }

  const uint64_t busy_delta = busy_store_count_ - network_health_busy_store_baseline_;
  const uint64_t replace_delta = busy_replace_count_ - network_health_replace_baseline_;
  const uint64_t timeout_drop_delta = pending_timeout_drop_count_ - network_health_timeout_drop_baseline_;

  if (busy_delta >= 15 && (replace_delta >= 8 || timeout_drop_delta >= 2)) {
    network_health_level_ = NetworkHealthLevel::kPoor;
  } else if (busy_delta >= 6 || replace_delta >= 3 || timeout_drop_delta >= 2) {
    network_health_level_ = NetworkHealthLevel::kCongested;
  } else {
    network_health_level_ = NetworkHealthLevel::kOk;
  }

  network_health_window_start_ = now;
  network_health_busy_store_baseline_ = busy_store_count_;
  network_health_replace_baseline_ = busy_replace_count_;
  network_health_timeout_drop_baseline_ = pending_timeout_drop_count_;
  publishStatus();
}

void MultipartStream::publishStatus() const
{
  if (status_key_.empty()) {
    return;
  }

  StatusSnapshot snapshot;
  snapshot.found = true;
  snapshot.level = networkHealthToString(network_health_level_);
  snapshot.busy_count = busy_store_count_;
  snapshot.replace_count = busy_replace_count_;
  snapshot.timeout_drop_count = pending_timeout_drop_count_;
  snapshot.has_pending = has_pending_part_;
  snapshot.pending_size = pending_part_.data.size();

  boost::mutex::scoped_lock lock(g_status_mutex);
  g_status_by_key[status_key_] = snapshot;
}

const char* MultipartStream::networkHealthToString(NetworkHealthLevel level)
{
  switch (level) {
    case NetworkHealthLevel::kPoor:
      return "poor";
    case NetworkHealthLevel::kCongested:
      return "congested";
    case NetworkHealthLevel::kOk:
    default:
      return "ok";
  }
}

bool MultipartStream::isBusy()
{
  auto current_time = std::chrono::steady_clock::now();
  while (!pending_footers_.empty()) {
    if (pending_footers_.front().contents.expired()) {
      pending_footers_.pop();
    } else {
      auto footer_time = pending_footers_.front().timestamp;
      if (std::chrono::duration_cast<std::chrono::duration<double>>((current_time -
        footer_time)).count() > 0.5)
      {
        pending_footers_.pop();
      } else {
        break;
      }
    }
  }
  return !(max_queue_size_ == 0 || pending_footers_.size() < max_queue_size_);
}

}
