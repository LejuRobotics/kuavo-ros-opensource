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

// 图像缓存结构体
struct CachedImage {
  cv::Mat image;
  ros::Time timestamp;
};

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
  bool face_detected_ = false;
  ros::Time last_face_box_time_;
  double face_box_timeout_ = 1; // 0.05秒超时
  
  // 图像缓存队列和相关参数
  std::deque<CachedImage> image_cache_;
  size_t cache_max_size_;
  double time_tolerance_;
  
  // 人脸边界框回调函数
  void faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg);

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