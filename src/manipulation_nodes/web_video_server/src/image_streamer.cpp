#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <limits>
#include <atomic>
#include <iomanip>

// 添加FaceBoundingBox的包含
#include <kuavo_msgs/FaceBoundingBox.h>
#include <sensor_msgs/Image.h>

namespace web_video_server
{

namespace
{
std::atomic<uint64_t> g_streamer_transport_id(0);

bool computeComparableRosAge(const ros::Time& stamp, double& age_seconds)
{
  if (stamp.isZero()) {
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (now.isZero()) {
    return false;
  }

  const double age = (now - stamp).toSec();
  if (age < -1.0 || age > 60.0) {
    return false;
  }

  age_seconds = age;
  return true;
}
}

ImageStreamer::ImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
    request_(request), connection_(connection), nh_(nh), inactive_(false)
{
  topic_ = request.get_query_param_value_or_default("topic", "");
  // 支持ROS remap机制
  if (!topic_.empty()) {
    topic_ = nh_.resolveName(topic_);
  }
  // 订阅人脸检测边界框话题
  face_bounding_box_sub_ = nh_.subscribe("/face_detection/bounding_box", 1, &ImageStreamer::faceBoundingBoxCallback, this);
  last_face_box_time_ = ros::Time(0);
  timestamp_tolerance_ = request.get_query_param_value_or_default<double>("sync_tolerance", 0.06);
  face_box_state_timeout_seconds_ =
      request.get_query_param_value_or_default<double>("face_box_state_timeout", 0.1);
  frame_timeout_seconds_ = request.get_query_param_value_or_default<double>("frame_timeout", 0.08);
  strict_sync_hard_timeout_seconds_ =
      request.get_query_param_value_or_default<double>("strict_sync_hard_timeout", 5.0);
  detection_mode_timeout_seconds_ =
      request.get_query_param_value_or_default<double>("detection_mode_timeout", 0.5);
  last_face_box_arrival_time_ = std::chrono::steady_clock::time_point::min();

  const std::string strict_sync_param = request.get_query_param_value_or_default("strict_sync", "");
  strict_sync_ = request.has_query_param("strict_sync") &&
                 strict_sync_param != "0" &&
                 strict_sync_param != "false" &&
                 strict_sync_param != "False";
}

ImageStreamer::~ImageStreamer()
{
}

static bool isValidFaceBox(const kuavo_msgs::FaceBoundingBox& face_box)
{
  return face_box.confidence > 0.0f &&
         face_box.x1 < face_box.x2 &&
         face_box.y1 < face_box.y2;
}

void ImageStreamer::faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg)
{
  {
    boost::mutex::scoped_lock queue_lock(queue_mutex_);
    face_box_queue_.emplace_back(*msg, msg->header.stamp);
    ++face_boxes_enqueued_;
    last_face_box_arrival_time_ = std::chrono::steady_clock::now();
    trimFaceBoxQueueLocked();
    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "[face_box_queue] topic=" << topic_
        << " detection_mode=1"
        << " enqueued=" << face_boxes_enqueued_
        << " size=" << face_box_queue_.size()
        << " stamp=" << std::fixed << std::setprecision(3) << msg->header.stamp.toSec());
  }

  // 检测框作为“显示状态更新事件”，后续图像会一直沿用该状态直到下一条框到来
  processPendingFaceBoxesInQueue();
}

ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh),
  transport_nh_(nh_, "streamer_" + std::to_string(g_streamer_transport_id.fetch_add(1))),
  it_(transport_nh_),
  initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "compressed");
}

ImageTransportImageStreamer::~ImageTransportImageStreamer()
{
}

void ImageTransportImageStreamer::start()
{
  image_transport::TransportHints hints(default_transport_, ros::TransportHints(), transport_nh_);
  ros::master::V_TopicInfo available_topics;
  ros::master::getTopics(available_topics);
  inactive_ = true;
  
  // 检查是否请求了默认的/camera/color/image_raw话题，优先使用/cam_h/color/image_raw
  if (topic_ == "/camera/color/image_raw") {
    for (const auto& topic_info : available_topics) {
      if (topic_info.name == "/cam_h/color/image_raw" && 
          topic_info.datatype == "sensor_msgs/Image") {
        topic_ = "/cam_h/color/image_raw";
        ROS_INFO("Found /cam_h/color/image_raw topic. Redirecting subscription from /camera/color/image_raw to /cam_h/color/image_raw");
        break;
      }
    }
  }
  
  // 话题匹配逻辑：检查话题是否存在
  for (const auto& topic_info : available_topics) {
    const std::string& available_topic_name = topic_info.name;
    
    // 完全匹配
    if (available_topic_name == topic_) {
      inactive_ = false;
      ROS_INFO("Found exact match for topic: %s", topic_.c_str());
      break;
    }
    
    // 处理前导斜杠的差异：统一规范化比较
    if (!topic_.empty() && !available_topic_name.empty()) {
      std::string normalized_topic = (topic_[0] == '/') ? topic_ : "/" + topic_;
      std::string normalized_available = (available_topic_name[0] == '/') ? available_topic_name : "/" + available_topic_name;
      
      if (normalized_topic == normalized_available) {
        inactive_ = false;
        ROS_INFO("Found match for topic (normalized): %s", topic_.c_str());
        break;
      }
    }
  }
  
  if (inactive_) {
    ROS_WARN("Topic %s is not available.", topic_.c_str());
  }
  
  ROS_INFO("Subscribing to topic: %s with transport hint: %s", topic_.c_str(), default_transport_.c_str());
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, hints);
  ROS_INFO("Actually subscribed to topic: %s (transport: %s)", image_sub_.getTopic().c_str(), default_transport_.c_str());
}

void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

void ImageTransportImageStreamer::restreamFrame(std::chrono::duration<double> max_age)
{
  if (inactive_ || !initialized_ )
    return;
  try {
    if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
      boost::mutex::scoped_lock lock(send_mutex_);
      // don't update last_frame, it may remain an old value.
      sendImage(output_size_image, std::chrono::steady_clock::now());
    }
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}

cv::Mat ImageTransportImageStreamer::decodeImage(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->encoding.find("F") != std::string::npos)
  {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if (max_val > 0)
    {
      float_image *= (255 / max_val);
    }
    return float_image;
  }
  else
  {
    // Convert to OpenCV native BGR color
    return cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
}

void ImageTransportImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (inactive_)
    return;

  // 打印实际接收消息的话题（只打印一次）
  static bool first_frame = true;
  if (first_frame) {
    ROS_INFO("Received first image from topic: %s (requested topic: %s, transport: %s)", 
             image_sub_.getTopic().c_str(), topic_.c_str(), default_transport_.c_str());
    first_frame = false;
  }

  cv::Mat img;
  try
  {
    img = decodeImage(msg);

    int input_width = img.cols;
    int input_height = img.rows;

    if (output_width_ == -1)
      output_width_ = input_width;
    if (output_height_ == -1)
      output_height_ = input_height;

    if (invert_)
    {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    // 调整图像大小（如果需要）并存入队列
    {
      boost::mutex::scoped_lock queue_lock(queue_mutex_);
      
      // 如果队列已满，移除最旧的图像
      if (image_queue_.size() >= MAX_QUEUE_SIZE) {
        ++image_frames_dropped_queue_full_;
        image_queue_.pop_front();
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[image_queue] topic=" << topic_
            << " dropped=queue_full"
            << " dropped_total=" << image_frames_dropped_queue_full_
            << " max_size=" << MAX_QUEUE_SIZE);
      }
      
      // 创建队列项
      QueuedImage queued_img;
      queued_img.timestamp = msg->header.stamp;
      queued_img.processed = false;
      queued_img.queue_time = std::chrono::steady_clock::now();
      queued_img.original_width = input_width;   // 保存原始图像尺寸
      queued_img.original_height = input_height; // 保存原始图像尺寸
      
      // 调整图像大小（如果需要），直接存入队列避免额外拷贝
      if (output_width_ != input_width || output_height_ != input_height)
      {
        cv::resize(img, queued_img.image, cv::Size(output_width_, output_height_));
      }
      else
      {
        queued_img.image = img.clone();  // 深拷贝，因为要存入队列
      }
      
      image_queue_.push_back(queued_img);
      ++image_frames_enqueued_;
      const double newest_age =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::steady_clock::now() - image_queue_.front().queue_time).count();
      double source_age = 0.0;
      const bool has_comparable_source_age = computeComparableRosAge(msg->header.stamp, source_age);
      ROS_INFO_STREAM_THROTTLE(
          1.0,
          "[image_queue] topic=" << topic_
          << " detection_mode=" << (isDetectionModeActiveLocked(std::chrono::steady_clock::now()) ? 1 : 0)
          << " enqueued=" << image_frames_enqueued_
          << " sent=" << image_frames_sent_
          << " queue_full_drop=" << image_frames_dropped_queue_full_
          << " timeout_drop=" << image_frames_dropped_timeout_
          << " image_q=" << image_queue_.size()
          << " face_q=" << face_box_queue_.size()
          << " source_age_s="
          << (has_comparable_source_age ? (static_cast<std::ostringstream&&>(std::ostringstream()
                 << std::fixed << std::setprecision(3) << source_age)).str()
                                        : std::string("n/a(clock_mismatch)"))
          << " oldest_age_s=" << std::fixed << std::setprecision(3) << newest_age
          << " img_stamp=" << msg->header.stamp.toSec());
    }

    processPendingFaceBoxesInQueue();

    // 处理超时的图像（即使未匹配到人脸框也发送，避免延迟过大）
    processTimeoutImagesInQueue();
    
    // 发送队列中已处理的图像
    sendProcessedImagesFromQueue();
    
    // 更新初始化状态（使用队列中的图像）
    if (!initialized_)
    {
      cv::Mat init_img;
      {
        boost::mutex::scoped_lock queue_lock(queue_mutex_);
        if (!image_queue_.empty()) {
          init_img = image_queue_.back().image.clone();  // 在锁内拷贝
        } else {
          return;  // 队列为空，下次再初始化
        }
      }
      // 在锁外进行初始化，避免长时间持有锁
      boost::mutex::scoped_lock send_lock(send_mutex_);
      initialize(init_img);
      initialized_ = true;
    }
    
    last_frame_ = std::chrono::steady_clock::now();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "OpenCV exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}

// 辅助函数：将人脸框坐标从原图坐标系转换到resize后的图像坐标系
static void transformFaceBoxCoordinates(const kuavo_msgs::FaceBoundingBox& face_box,
                                       int img_width, int img_height,
                                       int original_width, int original_height,
                                       int& x1, int& y1, int& x2, int& y2)
{
  // 计算缩放比例
  double scale_x = static_cast<double>(img_width) / original_width;
  double scale_y = static_cast<double>(img_height) / original_height;
  
  // 转换坐标
  x1 = static_cast<int>(face_box.x1 * scale_x);
  y1 = static_cast<int>(face_box.y1 * scale_y);
  x2 = static_cast<int>(face_box.x2 * scale_x);
  y2 = static_cast<int>(face_box.y2 * scale_y);
  
  // 限制在图像范围内
  x1 = std::max(0, std::min(x1, img_width - 1));
  y1 = std::max(0, std::min(y1, img_height - 1));
  x2 = std::max(0, std::min(x2, img_width - 1));
  y2 = std::max(0, std::min(y2, img_height - 1));
}

// 辅助函数：在图像上绘制人脸框
static bool drawFaceBoxOnImage(cv::Mat& img, const kuavo_msgs::FaceBoundingBox& face_box,
                               int original_width, int original_height)
{
  if (!isValidFaceBox(face_box)) {
    return false;
  }
  
  int x1, y1, x2, y2;
  transformFaceBoxCoordinates(face_box, img.cols, img.rows, 
                             original_width, original_height,
                             x1, y1, x2, y2);
  
  // 只有当坐标有效时才绘制
  if (x2 > x1 && y2 > y1) {
    cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
    return true;
  }
  
  return false;
}

bool ImageStreamer::processFaceBoxInQueue(const kuavo_msgs::FaceBoundingBox& face_box, const ros::Time& face_box_time)
{
  face_bounding_box_ = face_box;
  face_detected_ = isValidFaceBox(face_box);
  has_received_face_box_ = true;
  last_face_box_time_ = face_box_time;

  return true;
}

bool ImageStreamer::isDetectionModeActiveLocked(const std::chrono::steady_clock::time_point& now) const
{
  if (last_face_box_arrival_time_ == std::chrono::steady_clock::time_point::min()) {
    return false;
  }

  if (detection_mode_timeout_seconds_ <= 0.0) {
    return true;
  }

  return std::chrono::duration_cast<std::chrono::duration<double>>(now - last_face_box_arrival_time_).count()
         <= detection_mode_timeout_seconds_;
}

void ImageStreamer::processPendingFaceBoxesInQueue()
{
  boost::mutex::scoped_lock queue_lock(queue_mutex_);
  const bool detection_mode_active = isDetectionModeActiveLocked(std::chrono::steady_clock::now());

  while (!image_queue_.empty() && !image_queue_.front().processed) {
    QueuedImage& front_img = image_queue_.front();

    if (!detection_mode_active) {
      // 非检测模式下直接推流，不等待检测框，也不沿用旧框状态绘制。
      front_img.processed = true;
      front_img.should_send = true;
      continue;
    }

    size_t consumed_boxes = 0;

    while (!face_box_queue_.empty() &&
           face_box_queue_.front().timestamp <= front_img.timestamp + ros::Duration(timestamp_tolerance_)) {
      processFaceBoxInQueue(face_box_queue_.front().face_box, face_box_queue_.front().timestamp);
      face_box_queue_.pop_front();
      consumed_boxes++;
    }

    if (consumed_boxes == 0) {
      // 检测模式下只等待“真正被检测线程处理过”的帧，未匹配到检测结果的帧先不推进。
      break;
    }

    front_img.should_send = true;

    if (face_detected_) {
      if (drawFaceBoxOnImage(front_img.image, face_bounding_box_,
                             front_img.original_width, front_img.original_height)) {
        ROS_DEBUG("检测模式绘制匹配到的人脸框: 图像时间戳=%.3f, 检测框时间戳=%.3f, 原图坐标=(%d,%d)-(%d,%d)",
                  front_img.timestamp.toSec(),
                  last_face_box_time_.toSec(),
                  face_bounding_box_.x1, face_bounding_box_.y1,
                  face_bounding_box_.x2, face_bounding_box_.y2);
      }
    } else {
      ROS_DEBUG_THROTTLE(5, "检测模式匹配到无脸结果，发送检测帧但不绘制: 图像时间戳=%.3f, 检测框时间戳=%.3f",
                         front_img.timestamp.toSec(),
                         last_face_box_time_.toSec());
    }

    front_img.processed = true;
  }
  
  while (!face_box_queue_.empty() && image_queue_.empty()) {
    processFaceBoxInQueue(face_box_queue_.front().face_box, face_box_queue_.front().timestamp);
    face_box_queue_.pop_front();
  }

  while (!face_box_queue_.empty() && !image_queue_.empty() &&
         face_box_queue_.front().timestamp + ros::Duration(timestamp_tolerance_) < image_queue_.front().timestamp) {
    processFaceBoxInQueue(face_box_queue_.front().face_box, face_box_queue_.front().timestamp);
    ROS_DEBUG_THROTTLE(5, "提前切换到更新后的人脸框状态: 检测框时间戳=%.3f, 队列头图像时间戳=%.3f",
                      face_box_queue_.front().timestamp.toSec(),
                      image_queue_.front().timestamp.toSec());
    face_box_queue_.pop_front();
  }
}

void ImageStreamer::trimFaceBoxQueueLocked()
{
  while (face_box_queue_.size() > MAX_FACE_BOX_QUEUE_SIZE) {
    face_box_queue_.pop_front();
  }
}

void ImageTransportImageStreamer::sendProcessedImagesFromQueue()
{
  if (inactive_)
    return;

  while (true) {
    QueuedImage front_img;

    {
      boost::mutex::scoped_lock queue_lock(queue_mutex_);

      if (image_queue_.empty() || !image_queue_.front().processed) {
        break;
      }

      front_img = image_queue_.front();
      image_queue_.pop_front();
    }

    if (!front_img.should_send) {
      ROS_DEBUG_THROTTLE(5, "丢弃非检测帧，不进行推流: 图像时间戳=%.3f",
                         front_img.timestamp.toSec());
      continue;
    }

    try {
      const double queue_latency =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::steady_clock::now() - front_img.queue_time).count();
      double send_age = 0.0;
      const bool has_comparable_send_age = computeComparableRosAge(front_img.timestamp, send_age);
      boost::mutex::scoped_lock send_lock(send_mutex_);
      // 更新 output_size_image 用于 restreamFrame
      front_img.image.copyTo(output_size_image);
      sendImage(front_img.image, front_img.queue_time);
      last_frame_ = front_img.queue_time;
      ++image_frames_sent_;
      ROS_INFO_STREAM_THROTTLE(
          1.0,
          "[stream_send] topic=" << topic_
          << " sent=" << image_frames_sent_
          << " enqueued=" << image_frames_enqueued_
          << " send_age_s="
          << (has_comparable_send_age ? (static_cast<std::ostringstream&&>(std::ostringstream()
                 << std::fixed << std::setprecision(3) << send_age)).str()
                                      : std::string("n/a(clock_mismatch)"))
          << " queue_latency_s=" << std::fixed << std::setprecision(3) << queue_latency
          << " img_stamp=" << front_img.timestamp.toSec());
    }
    catch (boost::system::system_error &e)
    {
      // happens when client disconnects
      ROS_DEBUG("system_error exception in sendProcessedImagesFromQueue: %s", e.what());
      inactive_ = true;
      boost::mutex::scoped_lock queue_lock(queue_mutex_);
      image_queue_.clear();
      return;
    }
    catch (std::exception &e)
    {
      ROS_ERROR_THROTTLE(30, "exception in sendProcessedImagesFromQueue: %s", e.what());
      inactive_ = true;
      boost::mutex::scoped_lock queue_lock(queue_mutex_);
      image_queue_.clear();
      return;
    }
  }
}

void ImageTransportImageStreamer::processTimeoutImagesInQueue()
{
  if (inactive_)
    return;
    
  boost::mutex::scoped_lock queue_lock(queue_mutex_);
  
  // 提前计算当前时间，避免在循环中重复计算
  const auto now = std::chrono::steady_clock::now();
  const bool detection_mode_active = isDetectionModeActiveLocked(now);
  
  // 从队列头部开始，处理所有超时的未处理图像
  // 注意：只处理队列头部的连续超时图像，保持FIFO顺序
  while (!image_queue_.empty() && !image_queue_.front().processed) {
    const auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - image_queue_.front().queue_time).count() / 1000.0;

    QueuedImage& front_img = image_queue_.front();
    if (!detection_mode_active) {
      front_img.processed = true;
      front_img.should_send = true;
      continue;
    }

    if (wait_duration > frame_timeout_seconds_) {
      front_img.processed = true;
      front_img.should_send = false;
      ++image_frames_dropped_timeout_;
      ROS_DEBUG_THROTTLE(5, "检测模式丢弃未匹配到检测结果的图像: 等待时间=%.3fs, 图像时间戳=%.3f",
                         wait_duration, front_img.timestamp.toSec());
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "[image_queue] topic=" << topic_
          << " dropped=timeout_waiting_face_box"
          << " dropped_total=" << image_frames_dropped_timeout_
          << " wait_s=" << std::fixed << std::setprecision(3) << wait_duration
          << " image_q=" << image_queue_.size()
          << " face_q=" << face_box_queue_.size()
          << " img_stamp=" << front_img.timestamp.toSec());
      continue;
    }

    // 检测模式下，队头图像只等匹配到检测结果；没到超时就继续等。
    break;
  }
}

}
