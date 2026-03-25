#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <chrono>
#include <deque>
#include <memory>

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
  bool face_detected_ = false;
  bool has_received_face_box_ = false;
  ros::Time last_face_box_time_;
  double timestamp_tolerance_ = 0.06; // 图像与检测框的时间戳匹配容差
  double face_box_state_timeout_seconds_ = 0.1; // 超过该时间未收到新框，则清除当前框状态
  bool strict_sync_ = false; // 严格同步模式：没有足够的新框推进队列时尽量等待
  double frame_timeout_seconds_ = 0.08; // 实时优先模式下，图像等待检测框的超时时间
  double strict_sync_hard_timeout_seconds_ = 5.0; // 严格同步模式下的硬超时，避免永久卡死
  double detection_mode_timeout_seconds_ = 0.5; // 超过该时间未收到检测结果，退出检测模式
  std::chrono::steady_clock::time_point last_face_box_arrival_time_;
  
  // 图像队列结构
  struct QueuedImage {
    cv::Mat image;
    ros::Time timestamp;
    bool processed;  // 是否已处理（已匹配人脸框或确认无需处理）
    bool should_send;  // 是否需要推流发送
    std::chrono::steady_clock::time_point queue_time;
    int original_width;   // 原始图像宽度（用于坐标转换）
    int original_height;  // 原始图像高度（用于坐标转换）
    
    QueuedImage() : processed(false), should_send(true), original_width(0), original_height(0) {}
  };

  struct QueuedFaceBox {
    kuavo_msgs::FaceBoundingBox face_box;
    ros::Time timestamp;

    QueuedFaceBox(const kuavo_msgs::FaceBoundingBox& box, const ros::Time& stamp)
        : face_box(box), timestamp(stamp) {}
  };
  
  // 图像队列，最多存储10帧
  std::deque<QueuedImage> image_queue_;
  std::deque<QueuedFaceBox> face_box_queue_;
  static const size_t MAX_QUEUE_SIZE = 4;
  static const size_t MAX_FACE_BOX_QUEUE_SIZE = 20;
  boost::mutex queue_mutex_;  // 保护图像队列的互斥锁

  uint64_t image_frames_enqueued_ = 0;
  uint64_t image_frames_sent_ = 0;
  uint64_t image_frames_dropped_queue_full_ = 0;
  uint64_t image_frames_dropped_timeout_ = 0;
  uint64_t face_boxes_enqueued_ = 0;
  
  // 人脸边界框回调函数
  void faceBoundingBoxCallback(const kuavo_msgs::FaceBoundingBox::ConstPtr& msg);
  
  // 更新当前生效的人脸框状态
  bool processFaceBoxInQueue(const kuavo_msgs::FaceBoundingBox& face_box, const ros::Time& face_box_time);
  // 根据当前生效的人脸框状态处理图像队列
  void processPendingFaceBoxesInQueue();
  void trimFaceBoxQueueLocked();
  bool isDetectionModeActiveLocked(const std::chrono::steady_clock::time_point& now) const;

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
  ros::NodeHandle transport_nh_;
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  
  // 从队列中发送已处理的图像
  void sendProcessedImagesFromQueue();
  
  // 处理队列中等待时间过长的图像
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
