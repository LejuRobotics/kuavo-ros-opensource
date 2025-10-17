#include "grab_box/tagTracker.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <kuavo_msgs/setTagId.h>
#include "grab_box/package_path.h"

namespace autoHeadChase {

TagTracker::TagTracker()
    : is_continuous_tracking_(false),
      robot_pose_world_(Eigen::VectorXd::Zero(7)),
      tag_pose_robot_(Eigen::VectorXd::Zero(7)),
      tag_pose_world_(Eigen::VectorXd::Zero(7)),
      target_tag_id_(-1) {  // 初始未指定目标ID

    // Initialize subscribers
    odom_sub_ = nh_.subscribe("/odom", 10, &TagTracker::odomCallback, this);
    tag_info_sub_ = nh_.subscribe("/robot_tag_info", 10, &TagTracker::tagInfoCallback, this);

    // Initialize publishers
    head_orientation_pub_ = nh_.advertise<kuavo_msgs::robotHeadMotionData>("/robot_head_motion_data", 10);
    tag_world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_world_pose", 10);
    tag_data_pub_ = nh_.advertise<kuavo_msgs::tagDataArray>("/tag_data", 10);

    // Initialize services
    one_time_track_srv_ = nh_.advertiseService("one_time_track", &TagTracker::oneTimeTrackService, this);
    continuous_track_srv_ = nh_.advertiseService("continuous_track", &TagTracker::continuousTrackService, this);
    set_target_tag_id_srv_ = nh_.advertiseService("set_target_tag_id", &TagTracker::setTargetTagIdService, this);
    del_target_tag_id_srv_ = nh_.advertiseService("del_target_tag_id", &TagTracker::delTargetTagIdService, this);

    ros::param::get("/robot_version", robot_version_);
    std::cout << "Robot version : " << robot_version_ << std::endl;

    const std::string version_int_str = "kuavo_v" + std::to_string(robot_version_);
    YAML::Node config = YAML::LoadFile(GrabBox::getPath() + "/cfg/" + version_int_str + "/bt_config.yaml");
    safe_space_ = config["safe_space"].as<vector<double>>();
}

void TagTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护共享数据
    robot_pose_world_(0) = msg->pose.pose.position.x;
    robot_pose_world_(1) = msg->pose.pose.position.y;
    robot_pose_world_(2) = msg->pose.pose.position.z;
    robot_pose_world_(3) = msg->pose.pose.orientation.x;
    robot_pose_world_(4) = msg->pose.pose.orientation.y;
    robot_pose_world_(5) = msg->pose.pose.orientation.z;
    robot_pose_world_(6) = msg->pose.pose.orientation.w;

    odom_received_ = true;
}

void TagTracker::tagInfoCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    
    if(!odom_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << "[Tag Tracker]: Wait for odom or tag msg " << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护共享数据

    for (const auto& detection : msg->detections) {
        int tag_id = detection.id[0];  // 假设每个标签只有一个ID
        const auto& tag_pose = detection.pose.pose.pose;

        Eigen::VectorXd tag_pose_robot(7);
        tag_pose_robot(0) = tag_pose.position.x;
        tag_pose_robot(1) = tag_pose.position.y;
        tag_pose_robot(2) = tag_pose.position.z;
        tag_pose_robot(3) = tag_pose.orientation.x;
        tag_pose_robot(4) = tag_pose.orientation.y;
        tag_pose_robot(5) = tag_pose.orientation.z;
        tag_pose_robot(6) = tag_pose.orientation.w;

        // 转换到世界坐标系
        Eigen::VectorXd tag_pose_world = PoseTransformer::transformPoseToWorld(tag_pose_robot, robot_pose_world_);

        if (checkSafty(tag_pose_world, safe_space_))        
            tag_data_[tag_id] = tag_pose_world;  // 存储到字典中
        else
        {
            ROS_INFO("Tag %d is out of safe space", tag_id);
        }
    }

    tag_info_received_ = true;
    data_mutex_.unlock();  // 解锁
    updateTagWorldPose();
    publishTagData();

}

bool TagTracker::checkSafty(const Eigen::VectorXd& tag_pose_world, const vector<double>& safe_space) {
    double x_max = safe_space[0];
    double x_min = safe_space[1];
    double y_max = safe_space[2];
    double y_min = safe_space[3];

    if (tag_pose_world(0) < x_min || tag_pose_world(0) > x_max) {
        return false;
    }
    if (tag_pose_world(1) < y_min || tag_pose_world(1) > y_max) {
        return false;
    }

    return true;
}

void TagTracker::publishTagData() {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 保护 tag_data_ 访问

    kuavo_msgs::tagDataArray msg;
    for (const auto& [tag_id, tag_pose] : tag_data_) {
        msg.tag_ids.push_back(tag_id);

        geometry_msgs::Pose pose;
        pose.position.x = tag_pose(0);
        pose.position.y = tag_pose(1);
        pose.position.z = tag_pose(2);
        pose.orientation.x = tag_pose(3);
        pose.orientation.y = tag_pose(4);
        pose.orientation.z = tag_pose(5);
        pose.orientation.w = tag_pose(6);

        msg.tag_poses.push_back(pose);
    }

    tag_data_pub_.publish(msg);

}


bool TagTracker::setTargetTagIdService(kuavo_msgs::setTagId::Request& req, kuavo_msgs::setTagId::Response& res) {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护目标ID访问

    if (req.tag_id >= 0) {  // 检查有效的ID
        target_tag_id_ = req.tag_id;
        res.success = true;
        res.message = "Target tag ID set to " + std::to_string(target_tag_id_);
        res.tag_id = target_tag_id_;
        data_mutex_.unlock();  // 解锁
        updateTagWorldPose();

        std::cout << "Target tag ID set to: " << target_tag_id_ << std::endl;
    } else {
        res.success = false;
        res.message = "Invalid target tag ID.";
        res.tag_id = target_tag_id_;
    }
    return true;
}

bool TagTracker::delTargetTagIdService(kuavo_msgs::setTagId::Request& req, kuavo_msgs::setTagId::Response& res) {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护目标ID访问

    if (req.tag_id >= 0) {  // 检查有效的ID
        res.success = true;
        res.message = "Target tag ID : " + std::to_string(req.tag_id) +  " deleted.";
        res.tag_id = req.tag_id;
        tag_data_.erase(req.tag_id);  // 删除字典中的数据
        data_mutex_.unlock();  // 解锁
        std::cout << "Target tag ID set to: " << target_tag_id_ << std::endl;
    } else {
        res.success = false;
        res.message = "Invalid target tag ID.";
        res.tag_id = target_tag_id_;
    }
    return true;
}

void TagTracker::updateTagWorldPose() {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护共享数据

    if (tag_data_.find(target_tag_id_) != tag_data_.end()) {
        const auto& tag_pose_world = tag_data_[target_tag_id_];

        geometry_msgs::PoseStamped tag_world_pose;
        tag_world_pose.header.frame_id = "odom";
        tag_world_pose.header.stamp = ros::Time::now();

        tag_world_pose.pose.position.x = tag_pose_world(0);
        tag_world_pose.pose.position.y = tag_pose_world(1);
        tag_world_pose.pose.position.z = tag_pose_world(2);
        tag_world_pose.pose.orientation.x = tag_pose_world(3);
        tag_world_pose.pose.orientation.y = tag_pose_world(4);
        tag_world_pose.pose.orientation.z = tag_pose_world(5);
        tag_world_pose.pose.orientation.w = tag_pose_world(6);

        tag_world_pose_pub_.publish(tag_world_pose);
    }
}

void TagTracker::calculateHeadOrientation() {
    if(!odom_received_ || !tag_info_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << "[Tag Tracker]: Wait for odom or tag msg " << std::endl;
        return;
    }

    if (ros::ok()) {
        std::lock_guard<std::mutex> lock(data_mutex_);  // 加锁保护共享数据

        if (tag_data_.find(target_tag_id_) != tag_data_.end()) {
            const auto& tag_pose_world = tag_data_[target_tag_id_];

            double dx = tag_pose_world(0) - robot_pose_world_(0);
            double dy = tag_pose_world(1) - robot_pose_world_(1);
            double dz = tag_pose_world(2) - robot_pose_world_(2);

            double yaw = atan2(dy, dx);
            Eigen::Quaterniond robot_orientation(
                robot_pose_world_(6),  // w
                robot_pose_world_(3),  // x
                robot_pose_world_(4),  // y
                robot_pose_world_(5)   // z
            );
            double robot_yaw = atan2(
                2.0 * (robot_orientation.w() * robot_orientation.z() + robot_orientation.x() * robot_orientation.y()),
                1.0 - 2.0 * (robot_orientation.y() * robot_orientation.y() + robot_orientation.z() * robot_orientation.z())
            );
            yaw -= robot_yaw;

        if (robot_version_ == 42) 
        {

            pitch0 = 0.488692;
            heigjht_neck_motor_offset_ = 0.6014;
            head_link_ = 0.1584;
        }

            double height = heigjht_neck_motor_offset_ - dz;
            double dl = sqrt(dx * dx + dy * dy + height * height);
            double dl_xy = sqrt(dx * dx + dy * dy);
            double alpha1 = acos(head_link_ / dl);
            double alpha2 = asin(dl_xy / dl);
            double pitch = (height < 0) ? -(alpha2 - alpha1 - pitch0) : -(M_PI - pitch0 - alpha1 - alpha2);

            yaw = std::fmod(yaw + M_PI, 2 * M_PI) - M_PI;
            pitch = - std::fmod(pitch + M_PI, 2 * M_PI) - M_PI;

            if (yaw < -M_PI) yaw += 2 * M_PI;
            if (yaw > M_PI) yaw -= 2 * M_PI;
            if (pitch < -M_PI) pitch += 2 * M_PI;
            if (pitch > M_PI) pitch -= 2 * M_PI;


            publishHeadOrientationCommand(pitch, yaw);
        }
    }
}

void TagTracker::publishHeadOrientationCommand(double pitch, double yaw) {
    //
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    // std::cout << "[Tag] Head pitch : " << pitch << std::endl;
    // std::cout << "[Tag] Head yaw : " << yaw << std::endl;

    yaw = std::max(-30.0, std::min(30.0, yaw));

    // Ensure pitch is within the range [-25, 25]
    pitch = std::max(-25.0, std::min(25.0, pitch));

    // 
    kuavo_msgs::robotHeadMotionData head_cmd;
    head_cmd.joint_data.resize(2);  
    head_cmd.joint_data[0] = yaw;   // 
    head_cmd.joint_data[1] = pitch; // 

    // Publish 
    head_orientation_pub_.publish(head_cmd);
}

bool TagTracker::oneTimeTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (req.data) {
        calculateHeadOrientation();
        res.success = true;
        res.message = "One-time tracking executed.";
    } else {
        res.success = false;
        res.message = "Invalid request for one-time tracking.";
    }
    return true;
}

bool TagTracker::continuousTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    std::cout << "Continuous tracking request received." << std::endl;
    if (req.data) {
        if(!is_continuous_tracking_) {
          tracking_timer_ = nh_.createTimer(ros::Duration(0.02), [this](const ros::TimerEvent&) {
              calculateHeadOrientation();
          });
          is_continuous_tracking_ = true;
        }
        std::cout << "Continuous tracking started." << std::endl;
        res.success = true;
        res.message = "Continuous tracking started.";
    } else if (!req.data) {
        std::cout << "Continuous tracking stopped." << std::endl;
        is_continuous_tracking_ = false;
        tracking_timer_.stop();
        res.success = true;
        res.message = "Continuous tracking stopped.";
    } else {
        res.success = false;
        res.message = "Invalid request for continuous tracking.";
    }
    return true;
}

// Run the ROS node
void TagTracker::run() {
    ros::spin();
}

}  // namespace autoHeadChase

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_tracker_node");
    autoHeadChase::TagTracker tag_tracker;
    tag_tracker.run();
    return 0;
}
