#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <deque>
#include <algorithm>
#include <ros/topic.h>

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
  // 初始化图像缓存队列和相关参数
  image_cache_.clear();
  cache_max_size_ = 20;  // 缓存最大大小
  time_tolerance_ = 0.03;  // 时间戳匹配容差，单位秒
}

ImageStreamer::~ImageStreamer()
{
}

void ImageStreamer::faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg)
{
  // 更新人脸边界框
  face_bounding_box_ = *msg;
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
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
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
  
  // 检查是否请求了默认的/camera/color/image_raw话题
  std::string original_topic = topic_;
  if (topic_ == "/camera/color/image_raw") {
    // 检查/cam_h/color/image_raw是否存在
    for (size_t it = 0; it < available_topics.size(); it++) {
      if (available_topics[it].name == "/cam_h/color/image_raw" && 
          available_topics[it].datatype == "sensor_msgs/Image") {
        // 如果存在，则订阅/cam_h/color/image_raw
        topic_ = "/cam_h/color/image_raw";
        ROS_INFO("Found /cam_h/color/image_raw topic. Redirecting subscription from /camera/color/image_raw to /cam_h/color/image_raw");
        break;
      }
    }
  }
  
  // 更全面的话题匹配逻辑
  for (size_t it = 0; it < available_topics.size(); it++) {
    std::string available_topic_name = available_topics[it].name;
    
    // 检查完全匹配
    if (available_topic_name == topic_) {
      inactive_ = false;
      ROS_INFO("Found exact match for topic: %s", topic_.c_str());
      break;
    }
    
    // 检查去除前导斜杠的匹配
    if (available_topic_name.length() > 0 && available_topic_name[0] == '/' &&
        available_topic_name.substr(1) == topic_) {
      inactive_ = false;
      ROS_INFO("Found match for topic (without leading slash): %s", topic_.c_str());
      break;
    }
    
    // 检查topic_是否没有前导斜杠但available_topic_name有
    if (topic_.length() > 0 && topic_[0] != '/' &&
        available_topic_name == "/" + topic_) {
      inactive_ = false;
      ROS_INFO("Found match for topic (added leading slash): %s", topic_.c_str());
      break;
    }
  }
  
  if (inactive_) {
    ROS_WARN("Topic %s is not available. Available topics will be checked.", topic_.c_str());
  }
  
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, hints);
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

  cv::Mat img;
  try
  {
    img = decodeImage(msg);

    // 将当前图像添加到缓存队列
    CachedImage cached_img;
    cached_img.image = img.clone();
    cached_img.timestamp = msg->header.stamp;
    image_cache_.push_back(cached_img);

    // 保持缓存队列大小在限制范围内
    if (image_cache_.size() > cache_max_size_) {
      image_cache_.pop_front();
    }

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

    // 检查人脸检测框是否超时，如果检测到人脸，在图像上绘制检测框
    if (face_detected_) {
      ros::Time current_time = msg->header.stamp;
      double time_diff = (current_time - last_face_box_time_).toSec();
      time_diff = fabs(time_diff);  // 取绝对值

      
      // 如果超过设定时间没有收到新的人脸框，则认为话题已关闭
      if (time_diff > face_box_timeout_) {
        // ROS_INFO("Face detection topic closed.");
        face_detected_ = false;
      }
      
      // 如果人脸检测框仍然有效，则尝试在缓存图像中找到匹配的帧进行绘制
      if (face_detected_) {
        // 在缓存中查找与检测框时间戳最接近的图像
        CachedImage* matched_image = nullptr;
        double min_time_diff = time_tolerance_;

        // 打印
        ROS_INFO("Searching for matching image with timestamp %f", last_face_box_time_.toSec());
        
        for (auto it = image_cache_.begin(); it != image_cache_.end(); ++it) {
          double diff = fabs((it->timestamp - last_face_box_time_).toSec());
          if (diff-min_time_diff<= 0.03) {
            // 打印插值
            ROS_INFO("Found matching image with timestamp %f and time difference %f", it->timestamp.toSec(), diff);
            min_time_diff = diff;
            matched_image = &(*it);
          }
        }
        
        // 如果找到了匹配的图像，则在该图像上绘制检测框
        if (matched_image != nullptr) {
          ROS_INFO("Drawing face bounding box on image with timestamp %f", matched_image->timestamp.toSec());
          // 使用接收到的边界框坐标
          int x1 = static_cast<int>(face_bounding_box_.x1);
          int y1 = static_cast<int>(face_bounding_box_.y1);
          int x2 = static_cast<int>(face_bounding_box_.x2);
          int y2 = static_cast<int>(face_bounding_box_.y2);
          
          // 确保坐标在图像范围内
          x1 = std::max(0, std::min(x1, matched_image->image.cols - 1));
          x2 = std::max(0, std::min(x2, matched_image->image.cols - 1));
          y1 = std::max(0, std::min(y1, matched_image->image.rows - 1));
          y2 = std::max(0, std::min(y2, matched_image->image.rows - 1));

          img = matched_image->image;
          
          // 绘制矩形框
          cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        }
      }
    }

    boost::mutex::scoped_lock lock(send_mutex_); // protects output_size_image
    if (output_width_ != input_width || output_height_ != input_height)
    {
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    }
    else
    {
      output_size_image = img;
    }

    if (!initialized_)
    {
      initialize(output_size_image);
      initialized_ = true;
    }

    last_frame_ = std::chrono::steady_clock::now();
    sendImage(output_size_image, last_frame_);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
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

}