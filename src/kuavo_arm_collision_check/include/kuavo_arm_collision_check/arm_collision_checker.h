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
#include <kuavo_msgs/armCollisionCheckInfo.h>

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
    ros::Subscriber joint_state_sub_;
    ros::Publisher collision_info_pub_;
    ros::Publisher collision_marker_pub_;
    ros::Publisher collision_check_duration_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
public:
    void triggerCollisionCheck();
};

} // namespace kuavo_arm_collision_check 