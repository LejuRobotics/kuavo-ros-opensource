#pragma once

#include <ros/ros.h>

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <map>
#include <string>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <kuavo_msgs/armCollisionCheckInfo.h>
#include <kuavo_msgs/armTargetPoses.h>
#include <kuavo_msgs/sensorsData.h>
#include <std_msgs/Int32.h>
#include <deque>
#include <chrono>
#include <std_srvs/SetBool.h>

namespace kuavo_arm_collision_check {

struct CollisionPair {
    fcl::CollisionObjectd* first;
    fcl::CollisionObjectd* second;
};

struct Triangle { float v0[3], v1[3], v2[3]; };


struct CollisionCheckUserData {

    std::string parent_parent_link_name_;
    std::string parent_link_name_;
    std::string link_name;
    bool is_collision_enabled_;
};

std::vector<std::string> enable_link_list = {
    "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_l5_link", "zarm_l6_link", "zarm_l7_link", "zarm_l7_end_effector",
    "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", "zarm_r5_link", "zarm_r6_link", "zarm_r7_link", "zarm_r7_end_effector"
};

class ArmCollisionChecker {
public:
    ArmCollisionChecker(ros::NodeHandle& nh);
    ~ArmCollisionChecker();

private:
    // ROS related members
    ros::NodeHandle& nh_;
    ros::Subscriber sensors_data_sub_;  // 订阅传感器数据
    ros::Subscriber arm_mode_sub_;  // 订阅手臂模式
    ros::Publisher arm_pose_pub_;
    ros::Publisher arm_traj_forward_pub_;  // 转发手臂轨迹数据的发布者
    ros::Publisher arm_traj_debug_pub_;  // 调试用的手臂轨迹发布者
    ros::Publisher collision_info_pub_;
    ros::Publisher collision_marker_pub_;
    ros::Publisher collision_check_duration_pub_;
    ros::Publisher delay_sensors_data_pub_;
    // 新增的碰撞控制话题订阅者
    ros::Subscriber kuavo_arm_traj_sub_;
    ros::Subscriber kuavo_arm_target_poses_sub_;
    ros::Subscriber kuavo_sensors_data_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::ServiceServer wait_complete_srv_;
    ros::ServiceServer set_arm_moving_check_srv_;

    ros::ServiceClient fk_client_;

    int head_joint_num = 2;
    int arm_joint_num = 14;

    bool is_collision_moving_ = false;
    bool is_control_enabled_ = true;
    int current_arm_mode_ = 2;

    std::string kuavo_asset_path = "";
    std::string robot_version = "";
    // URDF related members
    urdf::Model robot_model_;
    KDL::Tree kdl_tree_;
    std::map<std::string, KDL::Chain> chains_;
    std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> fk_solvers_;
    
    // FCL related members
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> link_collision_objects_;
    std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> link_name_to_collision_object_;
    std::map<std::string, urdf::LinkConstSharedPtr> link_name_to_link_;
    std::map<fcl::CollisionObjectd*, std::string> collision_object_to_parent_name_;
    std::map<std::string, Eigen::Vector3d> collision_object_center_offset_;
    std::map<std::string, std::vector<Triangle>> link_name_to_mesh_triangles_;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerd> collision_manager_;

    std::map<std::string, CollisionCheckUserData> link_name_to_collision_check_user_data_;
    
    bool publish_markers_ = false;
    bool enable_arm_moving_check_ = false;

    double record_duration_ = 3.0;  // 记录3秒

    std::deque<sensor_msgs::JointState> recorded_arm_pose_data_;
    std::deque<kuavo_msgs::sensorsData> recorded_sensors_data_;
    std::mutex recorded_arm_pose_data_mutex_;
    std::mutex recorded_sensors_data_mutex_;
    std::vector<double> back_to_safe_arm_pose_data_;
    double arm_move_diff_ = 0.1;
    int stable_count_ = 0;

    // Helper functions
    bool loadURDF(const std::string& urdf_file_path);
    void initializeCollisionObjects();
    bool checkCollision(std::vector<CollisionPair>& collision_pairs);
    void updateCollisionObjects();
    
    std::shared_ptr<fcl::CollisionObjectd> createCollisionObject(const urdf::LinkConstSharedPtr& link);
    fcl::Transform3d getLinkTransform(const std::string& link_name);
    void createFKChain(const std::string& base_link, const std::string& tip_link);
    void publishCollisionMarkers(const std::vector<CollisionPair>& collision_pairs);
    void saveCollisionMesh(const std::string& link_name, const std::vector<Triangle>& triangles, const std::string& output_path);

    bool loadSTL(const std::string& filename, std::vector<Triangle>& triangles);
    
    // 手臂姿态记录相关函数
    void armModeCallback(const std_msgs::Int32::ConstPtr& msg);

    // 碰撞控制话题回调函数
    void kuavoArmTrajCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void kuavoArmTargetPosesCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg);
    void sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg);

    void playArmTrajBack();

    void tryToKeepArmPose();
    void stopArmCollisionControl();

    bool waitCompleteCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool setArmMovingEnableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    bool internal_change_arm_mode = false;
    
public:
    void triggerCollisionCheck();

    double trigger_frequency = 5.0;
};

} // namespace kuavo_arm_collision_check 