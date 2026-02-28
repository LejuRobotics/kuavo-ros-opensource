#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <chrono>
#include <deque>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"

// 添加FaceBoundingBox的包含
#include <kuavo_msgs/FaceBoundingBox.h>
#include <sensor_msgs/Image.h>

namespace web_video_server
{

class ImageStreamer
{
public:
  ImageStreamer(const async_web_server_cpp::HttpRequest &request,
		async_web_server_cpp::HttpConnectionPtr connection,
		ros::NodeHandle& nh);

  virtual void start() = 0;
  virtual ~ImageStreamer();

  bool isInactive()
  {
    return inactive_;
  }
  ;

  /**
   * Restreams the last received image frame if older than max_age.
   */
  virtual void restreamFrame(std::chrono::duration<double> max_age) = 0;

  std::string getTopic()
  {
    return topic_;
  }
  ;

protected:
  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  ros::NodeHandle nh_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
  
  // 添加人脸检测边界框订阅者和相关变量
  ros::Subscriber face_bounding_box_sub_;
  kuavo_msgs::FaceBoundingBox face_bounding_box_;
  kuavo_msgs::FaceBoundingBox last_valid_face_box_;  // 保存最近的有效人脸框（用于保留显示）
  bool face_detected_ = false;
  ros::Time last_face_box_time_;
  double face_box_timeout_ = 0.1; // 0.1秒超时，超过此时间未收到新的人脸框消息则认为检测不到人脸，清除显示
  double face_box_retention_time_ = 0.1; // 0.1秒保留时间，即使没有新的人脸框消息，也继续显示之前的人脸框
  
  // 图像队列结构
  struct QueuedImage {
    cv::Mat image;
    ros::Time timestamp;
    bool processed;  // 是否已处理（已匹配人脸框或确认无需处理）
    std::chrono::steady_clock::time_point queue_time;
    int original_width;   // 原始图像宽度（用于坐标转换）
    int original_height;  // 原始图像高度（用于坐标转换）
    
    QueuedImage() : processed(false), original_width(0), original_height(0) {}
  };
  
  // 图像队列，最多存储10帧
  std::deque<QueuedImage> image_queue_;
  static const size_t MAX_QUEUE_SIZE = 10;
  boost::mutex queue_mutex_;  // 保护图像队列的互斥锁
  
  // 人脸边界框回调函数
  void faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg);
  
  // 根据时间戳在队列中查找并处理匹配的图像
  void processFaceBoxInQueue(const kuavo_msgs::FaceBoundingBox& face_box, const ros::Time& face_box_time);

};


class ImageTransportImageStreamer : public ImageStreamer
{
public:
  ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
			      ros::NodeHandle& nh);
  virtual ~ImageTransportImageStreamer();
  virtual void start();

protected:
  virtual cv::Mat decodeImage(const sensor_msgs::ImageConstPtr& msg);
  virtual void sendImage(const cv::Mat &, const std::chrono::steady_clock::time_point &time) = 0;
  virtual void restreamFrame(std::chrono::duration<double> max_age);
  virtual void initialize(const cv::Mat &);

  int output_width_;
  int output_height_;
  bool invert_;
  std::string default_transport_;

  std::chrono::steady_clock::time_point last_frame_;
  cv::Mat output_size_image;
  boost::mutex send_mutex_;

private:
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  
  // 从队列中发送已处理的图像
  void sendProcessedImagesFromQueue();
  
  // 处理队列中等待时间过长的图像（即使未匹配到人脸框也发送）
  void processTimeoutImagesInQueue();
};

class ImageStreamerType
{
public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                           async_web_server_cpp::HttpConnectionPtr connection,
                                                           ros::NodeHandle& nh) = 0;

  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest &request) = 0;
};

}

#endif