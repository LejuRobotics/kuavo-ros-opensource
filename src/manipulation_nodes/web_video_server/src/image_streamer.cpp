#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <limits>

// 添加FaceBoundingBox的包含
#include <kuavo_msgs/FaceBoundingBox.h>
#include <sensor_msgs/Image.h>

namespace web_video_server
{

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
  face_box_retention_time_ = 0.1; // 默认保留0.1秒
}

ImageStreamer::~ImageStreamer()
{
}

void ImageStreamer::faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg)
{
  // 在队列中查找匹配的图像并处理（无论是否有效）
  // 这样可以确保即使没有检测到人脸，也能标记对应图像为已处理
  processFaceBoxInQueue(*msg, msg->header.stamp);
  
  // 检查消息是否有效（坐标是否合理）
  // 如果坐标无效（x1>=x2 或 y1>=y2），则认为检测不到人脸
  if (msg->x1 >= msg->x2 || msg->y1 >= msg->y2) {
    face_detected_ = false;
    return;
  }
  
  // 更新人脸边界框
  face_bounding_box_ = *msg;
  last_valid_face_box_ = *msg;  // 保存最近的有效人脸框（用于保留显示）
  face_detected_ = true;
  // 使用消息中的时间戳而不是当前时间
  last_face_box_time_ = msg->header.stamp;
}

ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), it_(nh), initialized_(false)
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
  image_transport::TransportHints hints(default_transport_);
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
        image_queue_.pop_front();
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
    }

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
  // 检查坐标是否有效
  if (face_box.x1 >= face_box.x2 || face_box.y1 >= face_box.y2) {
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

void ImageStreamer::processFaceBoxInQueue(const kuavo_msgs::FaceBoundingBox& face_box, const ros::Time& face_box_time)
{
  boost::mutex::scoped_lock queue_lock(queue_mutex_);
  
  // 在队列中查找时间戳最接近的图像
  // 允许的时间戳误差（秒）- 100ms以应对网络延迟和处理延迟
  const double timestamp_tolerance = 0.1;  // 100ms
  const double perfect_match_threshold = 0.01;  // 10ms内认为是完美匹配，可以提前退出
  
  double min_time_diff = std::numeric_limits<double>::max();
  std::deque<QueuedImage>::iterator best_match = image_queue_.end();
  
  // 遍历队列查找最佳匹配
  for (auto it = image_queue_.begin(); it != image_queue_.end(); ++it) {
    if (it->processed) {
      continue;  // 跳过已处理的图像
    }
    
    double time_diff = fabs((it->timestamp - face_box_time).toSec());
    
    // 如果找到完美匹配（时间戳差小于10ms），直接使用，提前退出
    if (time_diff < perfect_match_threshold) {
      best_match = it;
      min_time_diff = time_diff;
      break;
    }
    
    // 否则查找容差范围内的最佳匹配
    if (time_diff < timestamp_tolerance && time_diff < min_time_diff) {
      min_time_diff = time_diff;
      best_match = it;
    }
  }
  
  // 如果找到匹配的图像，绘制人脸框
  if (best_match != image_queue_.end()) {
    // 尝试绘制人脸框
    if (drawFaceBoxOnImage(best_match->image, face_box, 
                          best_match->original_width, best_match->original_height)) {
      ROS_DEBUG("绘制人脸框: 时间戳差=%.3fs, 原图坐标=(%d,%d)-(%d,%d)", 
               min_time_diff, 
               face_box.x1, face_box.y1, face_box.x2, face_box.y2);
    }
    // 无论是否绘制了人脸框，都标记为已处理（即使坐标无效，也表示已收到对应的人脸框消息）
    best_match->processed = true;
  } else {
    // 未找到匹配的图像，不绘制人脸框，只记录调试信息
    if (!image_queue_.empty()) {
      double oldest_diff = fabs((image_queue_.front().timestamp - face_box_time).toSec());
      double newest_diff = fabs((image_queue_.back().timestamp - face_box_time).toSec());
      ROS_DEBUG_THROTTLE(5, "未找到匹配图像，不绘制: 人脸框时间戳=%.3f, 队列最旧=%.3f(差%.3fs), 队列最新=%.3f(差%.3fs), 队列大小=%zu",
                        face_box_time.toSec(), 
                        image_queue_.front().timestamp.toSec(), oldest_diff,
                        image_queue_.back().timestamp.toSec(), newest_diff,
                        image_queue_.size());
    }
  }
}

void ImageTransportImageStreamer::sendProcessedImagesFromQueue()
{
  if (inactive_)
    return;
    
  boost::mutex::scoped_lock queue_lock(queue_mutex_);
  
  // 从队列头部开始，发送所有已处理的图像
  while (!image_queue_.empty() && image_queue_.front().processed) {
    QueuedImage& front_img = image_queue_.front();
    
    try {
      boost::mutex::scoped_lock send_lock(send_mutex_);
      // 更新 output_size_image 用于 restreamFrame
      front_img.image.copyTo(output_size_image);
      sendImage(front_img.image, front_img.queue_time);
      last_frame_ = front_img.queue_time;
    }
    catch (boost::system::system_error &e)
    {
      // happens when client disconnects
      ROS_DEBUG("system_error exception in sendProcessedImagesFromQueue: %s", e.what());
      inactive_ = true;
      image_queue_.clear();
      return;
    }
    catch (std::exception &e)
    {
      ROS_ERROR_THROTTLE(30, "exception in sendProcessedImagesFromQueue: %s", e.what());
      inactive_ = true;
      image_queue_.clear();
      return;
    }
    
    // 移除已发送的图像
    image_queue_.pop_front();
  }
}

void ImageTransportImageStreamer::processTimeoutImagesInQueue()
{
  if (inactive_)
    return;
    
  boost::mutex::scoped_lock queue_lock(queue_mutex_);
  
  // 处理所有超时的未处理图像
  // 增加超时时间到300ms，给人脸框消息更多时间到达
  const double timeout_seconds = 0.3;  // 300ms超时
  
  // 提前计算当前时间，避免在循环中重复计算
  const auto now = std::chrono::steady_clock::now();
  
  // 从队列头部开始，处理所有超时的未处理图像
  // 注意：只处理队列头部的连续超时图像，保持FIFO顺序
  while (!image_queue_.empty() && !image_queue_.front().processed) {
    const auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - image_queue_.front().queue_time).count() / 1000.0;
    
    if (wait_duration > timeout_seconds) {
      // 超时，检查是否可以使用保留的人脸框
      QueuedImage& front_img = image_queue_.front();
      bool should_draw_face_box = false;
      
      // 检查是否有有效的保留人脸框，且时间在保留期内
      if (face_detected_) {
        // 计算图像时间戳与最后人脸框时间戳的差值
        double time_since_last_face_box = fabs((front_img.timestamp - last_face_box_time_).toSec());
        
        // 如果在保留时间内，使用保留的人脸框绘制
        if (time_since_last_face_box < face_box_retention_time_) {
          should_draw_face_box = drawFaceBoxOnImage(front_img.image, last_valid_face_box_,
                                                   front_img.original_width, front_img.original_height);
          if (should_draw_face_box) {
            ROS_DEBUG_THROTTLE(5, "使用保留人脸框绘制: 等待时间=%.3fs, 时间戳=%.3f, 距离最后人脸框=%.3fs",
                              wait_duration, front_img.timestamp.toSec(), time_since_last_face_box);
          }
        }
      }
      
      // 标记为已处理
      front_img.processed = true;
      
      if (!should_draw_face_box) {
        ROS_DEBUG_THROTTLE(5, "图像超时未匹配到人脸框，直接发送（不绘制）: 等待时间=%.3fs, 时间戳=%.3f",
                          wait_duration, front_img.timestamp.toSec());
      }
    } else {
      // 队列头部图像还未超时，停止处理（保持FIFO顺序）
      break;
    }
  }
}

}