#include "kuavo_arm_collision_check/arm_collision_checker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <limits>
#include <set>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <chrono>
#include "kuavo_common/common/json_config_reader.hpp"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include <kuavo_msgs/fkSrv.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> KuavoMesh;

using BVHModel = fcl::BVHModel<fcl::OBBRSSd>;

namespace kuavo_arm_collision_check {

ArmCollisionChecker::ArmCollisionChecker(ros::NodeHandle& nh) 
    : nh_(nh), tf_listener_(tf_buffer_) {
    
    // Get kuavo_asset package path
    kuavo_asset_path = ros::package::getPath("kuavo_assets");
    if (!kuavo_asset_path.empty()) {
        ROS_INFO_STREAM("kuavo_asset package path: " << kuavo_asset_path);
    } else {
        ROS_ERROR("Failed to get kuavo_asset package path");
        return;
    }

    // Get ROBOT_VERSION environment variable
    char* robot_version_ = std::getenv("ROBOT_VERSION");
    if (robot_version_) {
        ROS_INFO_STREAM("ROBOT_VERSION: " << robot_version_);
    } else {
        ROS_ERROR("ROBOT_VERSION environment variable not set");
        return;
    }

    robot_version = std::string(robot_version_);

    if(robot_version != "45"){

        ROS_ERROR("current ROBOT_VERSION != 45, might cause stl model mismatch!");
    }

    // Read parameter for publishing markers
    nh_.param("/arm_collision/publish_collision_markers", publish_markers_, false);
    if (publish_markers_) {
        ROS_INFO("Collision markers will be published.");
    } else {
        ROS_INFO("Collision markers will not be published.");
    }

    nh_.param("/arm_collision/enable_arm_moving", enable_arm_moving_check_, false);
    if (enable_arm_moving_check_) {
        ROS_INFO("Arm moving is enabled, will send arm pose when collision detected.");
    } else {
        ROS_INFO("Arm moving is disabled.");
    }

    nh_.param("/arm_collision/arm_move_diff", arm_move_diff_, 0.01);

    // Get URDF file path
    std::string urdf_file_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/urdf/biped_s" + robot_version + ".urdf";
    std::ifstream f(urdf_file_path.c_str());
    if (!f.good()) {
        ROS_ERROR("URDF file not found at path: %s", urdf_file_path.c_str());
        return;
    }
    
    // Get arm joint number parameter
    std::string kuavo_json_path = kuavo_asset_path + "/config/kuavo_v" + robot_version + "/kuavo.json";
    std::ifstream kuavo_json_file(kuavo_json_path.c_str());
    if (!kuavo_json_file.good()) {
        ROS_ERROR("kuavo.json file not found at path: %s", kuavo_json_path.c_str());
        return;
    }
    HighlyDynamic::JSONConfigReader kuavo_json(kuavo_json_path);
    arm_joint_num = kuavo_json.getValue<int>("NUM_ARM_JOINT");
    head_joint_num = kuavo_json.getValue<int>("NUM_HEAD_JOINT");
    ROS_INFO("head_joint_num: %d, arm_joint_num: %d", head_joint_num, arm_joint_num);
    kuavo_json_file.close();

    // Load URDF
    if (!loadURDF(urdf_file_path)) {
        ROS_ERROR("Failed to load URDF model");
        return;
    }

    // 订阅手臂轨迹、传感器数据、手臂模式 
    arm_pose_pub_ = nh_.advertise<kuavo_msgs::armTargetPoses>("/kuavo_arm_target_poses", 1);
    arm_traj_forward_pub_ = nh_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 1);
    arm_traj_debug_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm_collision/debug_kuavo_arm_traj", 1);
    arm_mode_sub_ = nh_.subscribe("/quest3/triger_arm_mode", 1, 
        &ArmCollisionChecker::armModeCallback, this);

    // auto get_arm_control_mode_client_ = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");
    // kuavo_msgs::changeArmCtrlMode srv;
    // srv.request.control_mode = 0;
    // if (get_arm_control_mode_client_.call(srv)) {
    //     current_arm_mode_ = srv.response.mode;
    // } else {
    //     ROS_ERROR("Failed to call get arm control mode service");
    // }
    wait_complete_srv_ = nh_.advertiseService("/arm_collision/wait_complete", &ArmCollisionChecker::waitCompleteCallback, this);
    set_arm_moving_check_srv_ = nh_.advertiseService("/arm_collision/set_arm_moving_enable", &ArmCollisionChecker::setArmMovingEnableCallback, this);


    collision_info_pub_ = nh_.advertise<kuavo_msgs::armCollisionCheckInfo>("/arm_collision/info", 1);
    collision_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/arm_collision/markers", 1);
    collision_check_duration_pub_ = nh_.advertise<std_msgs::Float64>("/arm_collision/check_duration", 1);
    delay_sensors_data_pub_ = nh_.advertise<kuavo_msgs::sensorsData>("/arm_collision/delay_sensors_data", 1);
    
    // 初始化碰撞控制话题订阅者
    kuavo_arm_traj_sub_ = nh_.subscribe("/arm_collision/kuavo_arm_traj", 1, 
        &ArmCollisionChecker::kuavoArmTrajCallback, this);
    kuavo_arm_target_poses_sub_ = nh_.subscribe("/arm_collision/kuavo_arm_target_poses", 1, 
        &ArmCollisionChecker::kuavoArmTargetPosesCallback, this);
    kuavo_sensors_data_sub_ = nh_.subscribe("/sensors_data_raw", 10, 
        &ArmCollisionChecker::sensorsDataCallback, this);

    fk_client_ = nh_.serviceClient<kuavo_msgs::fkSrv>("/ik/fk_srv");

    // Initialize collision manager
    collision_manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    
    // Initialize collision objects
    initializeCollisionObjects();
}

ArmCollisionChecker::~ArmCollisionChecker() {
}

bool ArmCollisionChecker::loadSTL(const std::string& filename, std::vector<Triangle>& triangles) {

    // 构造缓存文件路径
    std::string cache_dir = ros::package::getPath("kuavo_arm_collision_check") + "/cache";
    std::string base_name = filename.substr(0, filename.find_last_of('.'));
    std::string cached_path = cache_dir + "/" + base_name + "_processed.stl";

    // 检查缓存文件是否存在
    std::ifstream cached_file(cached_path.c_str());
    if (cached_file.good()) {
        // 直接加载缓存STL
        KuavoMesh mesh;
        OpenMesh::IO::Options opt;
        opt += OpenMesh::IO::Options::VertexNormal;
        std::streambuf* old_buf = std::cout.rdbuf();
        std::ofstream of("/dev/null");
        std::cout.rdbuf(of.rdbuf());
        std::cerr.rdbuf(of.rdbuf());
        bool read_ok = OpenMesh::IO::read_mesh(mesh, cached_path, opt);
        std::cout.rdbuf(old_buf);
        std::cerr.rdbuf(old_buf);
        if (!read_ok) {
            ROS_ERROR("Failed to open cached STL file: %s", cached_path.c_str());
            return false;
        }
        triangles.clear();
        triangles.reserve(mesh.n_faces());
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            std::vector<OpenMesh::Vec3f> verts;
            for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
                auto pt = mesh.point(*fv_it);
                verts.push_back(pt);
            }
            if (verts.size() == 3) {
                Triangle tri;
                for (int i = 0; i < 3; ++i) {
                    tri.v0[i] = verts[0][i];
                    tri.v1[i] = verts[1][i];
                    tri.v2[i] = verts[2][i];
                }
                triangles.push_back(tri);
            }
        }
        ROS_INFO("Loaded mesh from cache: %s", cached_path.c_str());
        return true;
    }

    std::string meshes_dir = ros::package::getPath("kuavo_arm_collision_check") + "/meshes";
    std::string path = meshes_dir + "/" + filename;

    std::ifstream mesh_file(path.c_str());
    if(!mesh_file.good()){

        ROS_ERROR("Failed to load STL mesh: %s", path.c_str());

        std::string mesh_kuavo_assets_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/meshes/" + filename;
        
        ROS_INFO_STREAM("try to load " << filename << "from kuavo_assets instead!");
        // 使用 kuavo_assets 的 stl 模型
        std::ifstream mesh_kuavo_assets_file(mesh_kuavo_assets_path.c_str());
        if(!mesh_kuavo_assets_file.good()){

            ROS_ERROR("Failed to load STL mesh: %s", mesh_kuavo_assets_path.c_str());
            return false;
        }
        else path = mesh_kuavo_assets_path;
    }

    KuavoMesh mesh;
    OpenMesh::IO::Options opt;
    opt += OpenMesh::IO::Options::VertexNormal;
    // 临时重定向stdout到/dev/null
    std::streambuf* old_buf = std::cout.rdbuf();
    std::ofstream of("/dev/null");
    std::cout.rdbuf(of.rdbuf());
    std::cerr.rdbuf(of.rdbuf());
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cout.rdbuf(old_buf);  // 恢复stdout
        ROS_ERROR("Failed to open STL file: %s", path.c_str());
        return false;
    }
    std::cout.rdbuf(old_buf);  // 恢复stdout
    std::cerr.rdbuf(old_buf);  // 恢复stderr
    
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    mesh.request_face_status();
    mesh.request_vertex_status();
    mesh.request_edge_status();
    
    // 计算法线
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    
    // ROS_INFO("mesh has %u vertices", mesh.n_vertices());
    // ROS_INFO("mesh has %u faces", mesh.n_faces());

    mesh.garbage_collection();

    std::vector<KuavoMesh::FaceHandle> faces_to_delete;

    for (KuavoMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            KuavoMesh::Normal face_normal = mesh.calc_face_normal(*f_it); // 计算面的法线
        if (face_normal.sqrnorm() < 1e-12) { // 如果法线平方范数接近零，则认为是简并面
            // std::cout << "Warning: Degenerate face detected at index " << f_it->idx() << std::endl;
            faces_to_delete.push_back(*f_it);
        }
    }
    for (KuavoMesh::FaceHandle fh : faces_to_delete) {
        mesh.delete_face(fh, true); // true 表示不清理顶点，只删除面
    }
    mesh.garbage_collection(); // 彻底移除未被使用的顶点

    // 检查mesh是否还有面和顶点
    if (mesh.n_faces() == 0 || mesh.n_vertices() == 0) {
        ROS_ERROR("膨胀前mesh没有有效面或顶点，无法保存STL！");
        return false;
    }

    // 顶点膨胀，先缓存所有新坐标，最后统一写回
    double offset = 0.02; 
    int vaild_count = 0;
    std::vector<OpenMesh::Vec3f> new_points(mesh.n_vertices());
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        auto n = mesh.normal(*v_it);
        if (n.norm() < 1e-8) {
            new_points[v_it->idx()] = mesh.point(*v_it); // 不动
            continue;
        }
        n.normalize();
        new_points[v_it->idx()] = mesh.point(*v_it) + offset * n;
        vaild_count++;
    }
    // ROS_INFO("膨胀后有效顶点数: %d, 膨胀前有效顶点数: %d", vaild_count, mesh.n_vertices());
    for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        mesh.set_point(*v_it, new_points[v_it->idx()]);
    }

    // 再次检查mesh有效性
    if (mesh.n_faces() == 0 || mesh.n_vertices() == 0) {
        ROS_ERROR("膨胀后mesh没有有效面或顶点，无法保存STL！");
        return false;
    }

    mesh.garbage_collection();

    // 创建cache目录（如果不存在）
    std::string mkdir_cmd = "mkdir -p " + cache_dir;
    system(mkdir_cmd.c_str());
    
    // 设置写入选项为二进制格式
    OpenMesh::IO::Options write_opt;
    write_opt += OpenMesh::IO::Options::Binary;
    
    if (OpenMesh::IO::write_mesh(mesh, cached_path, write_opt)) {
        // ROS_INFO("Processed mesh saved to: %s (binary format)", cached_path.c_str());
    } else {
        ROS_WARN("Failed to save processed mesh to: %s", cached_path.c_str());
    }

    triangles.clear();
    triangles.reserve(mesh.n_faces());
    for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        std::vector<OpenMesh::Vec3f> verts;
        for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
            auto pt = mesh.point(*fv_it);
            verts.push_back(pt);
        }
        if (verts.size() == 3) {
            Triangle tri;
            for (int i = 0; i < 3; ++i) {
                tri.v0[i] = verts[0][i];
                tri.v1[i] = verts[1][i];
                tri.v2[i] = verts[2][i];
            }
            triangles.push_back(tri);
        }
    }
    return true;
}

std::shared_ptr<BVHModel> createCollisionMesh(const std::vector<Triangle>& tris) {
    try {
        auto mesh = std::make_shared<BVHModel>();
        
        // 开始构建模型
        if (!mesh->beginModel(tris.size(), tris.size() * 3)) {
            ROS_ERROR("Failed to begin BVH model");
            return nullptr;
        }
        
        // 添加所有三角面片
        for (const auto& tri : tris) {
            Eigen::Vector3d p0(tri.v0[0], tri.v0[1], tri.v0[2]);
            Eigen::Vector3d p1(tri.v1[0], tri.v1[1], tri.v1[2]);
            Eigen::Vector3d p2(tri.v2[0], tri.v2[1], tri.v2[2]);
            
            if (!mesh->addTriangle(p0, p1, p2)) {
                ROS_ERROR("Failed to add triangle to BVH model");
                return nullptr;
            }
        }
        
        // 结束构建模型
        if (!mesh->endModel()) {
            ROS_ERROR("Failed to end BVH model");
            return nullptr;
        }
        
        return mesh;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception while creating collision mesh: %s", e.what());
        return nullptr;
    }
    catch (...) {
        ROS_ERROR("Unknown exception while creating collision mesh");
        return nullptr;
    }
}

std::string extractFilenameFromPackageURL(const std::string& package_url) {
    // 找到最后一个斜杠的位置
    size_t last_slash = package_url.find_last_of("/");
    if (last_slash != std::string::npos) {
        // 返回最后一个斜杠后面的部分
        return package_url.substr(last_slash + 1);
    }
    return package_url;  // 如果没有斜杠，返回原始字符串
}

bool ArmCollisionChecker::loadURDF(const std::string& urdf_file_path) {

    if (!robot_model_.initFile(urdf_file_path)) {
        ROS_ERROR("Failed to parse URDF model");
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(robot_model_, kdl_tree_)) {
        ROS_ERROR("Failed to extract KDL tree from URDF model");
        return false;
    }

    // Create FK chains for each link
    for (const auto& link_pair : robot_model_.links_) {
        const auto& link = link_pair.second;
        if (link->parent_joint) {
            createFKChain(link->parent_joint->parent_link_name, link->name);
        }
    }

    return true;
}

void ArmCollisionChecker::createFKChain(const std::string& base_link, const std::string& tip_link) {
    KDL::Chain chain;
    if (kdl_tree_.getChain(base_link, tip_link, chain)) {
        chains_[tip_link] = chain;
        fk_solvers_[tip_link] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    }
}

void ArmCollisionChecker::initializeCollisionObjects() {
    // Create collision objects for each link
    for (const auto& link_pair : robot_model_.links_) {
        const auto& link = link_pair.second;
        if (link->visual) {
            auto collision_object = createCollisionObject(link);
            if (collision_object) {

                link_collision_objects_.push_back(collision_object);
                link_name_to_collision_object_[link->name] = collision_object;
                link_name_to_link_[link->name] = link;
                // Register object with collision manager

                bool is_collision_enabled = false;
                if (std::find(enable_link_list.begin(), enable_link_list.end(), link->name) != enable_link_list.end()) {
                    is_collision_enabled = true;
                }

                std::string parent_link_name = "";
                std::string parent_parent_link_name = "";
                if(link->parent_joint)
                {
                    parent_link_name = link->parent_joint->parent_link_name;
                    urdf::LinkConstSharedPtr parent_link = robot_model_.getLink(parent_link_name);
                    if(parent_link && parent_link->parent_joint)
                    {
                        parent_parent_link_name = parent_link->parent_joint->parent_link_name;
                    }
                }

                // robot_model_.links_
                link_name_to_collision_check_user_data_[link->name] = CollisionCheckUserData{parent_parent_link_name, parent_link_name, link->name, is_collision_enabled};

                collision_object->setUserData(&link_name_to_collision_check_user_data_[link->name]);

                // ROS_INFO_STREAM("Registering collision object for link: " << link->name);
                collision_manager_->registerObject(collision_object.get());
            }
        }
    }
}

std::shared_ptr<fcl::CollisionObjectd> ArmCollisionChecker::createCollisionObject(const urdf::LinkConstSharedPtr& link) {
    if (!link->visual || !link->visual->geometry) {
        ROS_INFO_STREAM(link->name << " visual or geometry is nullptr");
        return nullptr;
    }

    std::shared_ptr<fcl::CollisionGeometryd> geometry;

    // link->visual->geometry->type;
    
    // Create collision geometry based on the type
    switch (link->visual->geometry->type) {
        case urdf::Geometry::BOX: {
            auto box = std::static_pointer_cast<urdf::Box>(link->visual->geometry);
            geometry = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
            break;
        }
        case urdf::Geometry::CYLINDER: {
            auto cylinder = std::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);
            geometry = std::make_shared<fcl::Cylinderd>(cylinder->radius, cylinder->length);
            break;
        }
        case urdf::Geometry::SPHERE: {
            auto sphere = std::static_pointer_cast<urdf::Sphere>(link->visual->geometry);
            geometry = std::make_shared<fcl::Sphered>(sphere->radius);
            break;
        }
        case urdf::Geometry::MESH: {
            
            if (!link->visual) {
                ROS_ERROR("Link visual is nullptr");
                return nullptr;
            }
            auto mesh = std::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

            if(!mesh) {
                ROS_ERROR("Mesh is nullptr");
                return nullptr;
            }

            if(mesh->filename.empty()) {
                ROS_ERROR("Mesh filename is empty");
                return nullptr;
            }

            // ROS_INFO_STREAM("Mesh filename: " << mesh->filename);
            // Extract the filename from the package URL
            std::string filename = extractFilenameFromPackageURL(mesh->filename);
            // ROS_INFO_STREAM("Extracted filename: " << filename);

            std::string mesh_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/meshes/" + filename;
            
            
            // 读取 STL 文件
            std::vector<Triangle> triangles;
            if (!loadSTL(filename, triangles)) {
                ROS_ERROR("Failed to load STL mesh: %s", filename.c_str());
            }

            // 构建 FCL mesh
            auto bvh = std::make_shared<BVHModel>();
            bvh->beginModel(triangles.size(), triangles.size() * 3);
            for (const auto& tri : triangles) {
                Eigen::Vector3d v0(tri.v0[0], tri.v0[1], tri.v0[2]);
                Eigen::Vector3d v1(tri.v1[0], tri.v1[1], tri.v1[2]);
                Eigen::Vector3d v2(tri.v2[0], tri.v2[1], tri.v2[2]);
                bvh->addTriangle(v0, v1, v2);
            }
            bvh->endModel();

            geometry = bvh;
            // mesh 原点就是 mesh 文件原点，不需要额外平移
            collision_object_center_offset_[link->name] = Eigen::Vector3d(0, 0, 0);
            link_name_to_mesh_triangles_[link->name] = triangles;
            break;
        }
        default:
            ROS_WARN("Unsupported collision geometry type");
            return nullptr;
    }

    auto collision_object = std::make_shared<fcl::CollisionObjectd>(geometry);
    
    return collision_object;
}

void ArmCollisionChecker::triggerCollisionCheck() {
    // 记录开始时间
    auto start = std::chrono::steady_clock::now();

    // Update collision objects based on current joint states
    updateCollisionObjects();
    
    std::vector<CollisionPair> collision_pairs;
    // Check for collisions
    bool is_collision = checkCollision(collision_pairs);

    for(const auto& pair : collision_pairs) {
        CollisionCheckUserData* user_data1 = static_cast<CollisionCheckUserData*>(pair.first->getUserData());
        CollisionCheckUserData* user_data2 = static_cast<CollisionCheckUserData*>(pair.second->getUserData());
        
        kuavo_msgs::armCollisionCheckInfo collision_msg;
        collision_msg.link1_name = user_data1->link_name;
        collision_msg.link2_name = user_data2->link_name;
        collision_info_pub_.publish(collision_msg);
    }

    // current_arm_mode_ 为1时，不进行碰撞归位
    if(enable_arm_moving_check_ && is_collision && !is_collision_moving_ && current_arm_mode_ != 1) {
        
        is_collision_moving_ = true;
        ROS_WARN_STREAM("Back to safe arm pose");
        playArmTrajBack();
        
        tryToKeepArmPose();
        is_collision_moving_ = false;
    }

    // 记录结束时间并发布
    auto end = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std_msgs::Float64 duration_msg;
    duration_msg.data = duration;
    collision_check_duration_pub_.publish(duration_msg);
}

void ArmCollisionChecker::updateCollisionObjects() {

    // Update transforms for each link
    for (const auto& link_pair : link_name_to_collision_object_) {
        const auto& link_name = link_pair.first;
        const auto& collision_object = link_pair.second;
        
        // Get transform from forward kinematics
        fcl::Transform3d transform = getLinkTransform(link_name);
        fcl::Transform3d transform_center_offset;
        transform_center_offset.setIdentity();
        transform_center_offset.translation() = collision_object_center_offset_[link_name];
        transform = transform_center_offset * transform;
        collision_object->setTransform(transform);
    }

    // Update collision manager
    collision_manager_->update();
}

fcl::Transform3d ArmCollisionChecker::getLinkTransform(const std::string& link_name) {
    // Try to get transform from TF first
    try {
        geometry_msgs::TransformStamped transform_stamped = 
            tf_buffer_.lookupTransform("base_link", link_name, ros::Time(0));
        
        // Convert to FCL transform
        Eigen::Isometry3d transform_eigen;
        transform_eigen = tf2::transformToEigen(transform_stamped.transform);
              
        fcl::Transform3d transform;
        transform.matrix() = transform_eigen.matrix();
        return transform;
    }
    catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Could not get transform from TF: from %s to %s, %s", "base_link", link_name.c_str(), ex.what());
        return fcl::Transform3d::Identity();
    }
}

bool ArmCollisionChecker::checkCollision(std::vector<CollisionPair>& collision_pairs) {
    // Create collision request and result
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
      
    // 定义回调函数
    auto collision_callback = [](fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* cdata) -> bool {

        CollisionCheckUserData* user_data1 = static_cast<CollisionCheckUserData*>(o1->getUserData());
        CollisionCheckUserData* user_data2 = static_cast<CollisionCheckUserData*>(o2->getUserData());
        
        // 去掉相邻关节的碰撞
        if(user_data1->parent_link_name_ == user_data2->link_name || user_data2->parent_link_name_ == user_data1->link_name) {
            return false;
        }
        // 去掉相邻两个关节的碰撞
        if(user_data1->parent_parent_link_name_ == user_data2->link_name || user_data2->parent_parent_link_name_ == user_data1->link_name) {
            return false;
        }
        // 去掉碰撞检测关闭的关节
        if(!user_data1->is_collision_enabled_ && !user_data2->is_collision_enabled_) {
            return false;
        }

        fcl::CollisionRequestd req;
        
        fcl::CollisionResultd res;
        fcl::collide(o1, o2, req, res);
        if (res.isCollision()) {
            auto* pairs = static_cast<std::vector<CollisionPair>*>(cdata);
            pairs->push_back({o1, o2});
        }

        // fcl::DistanceRequestd dist_req;
        // fcl::DistanceResultd dist_res;
        // fcl::distance(o1, o2, dist_req, dist_res);

        // if (dist_res.min_distance < 0.1) { // 阈值0.1米
        //     auto* pairs = static_cast<std::vector<CollisionPair>*>(cdata);
        //     pairs->push_back({o1, o2});
        // }
        return false;  // 继续检测
    };
    
    // Check for collisions
    collision_manager_->collide(static_cast<void*>(&collision_pairs), collision_callback);
    
    // Publish collision markers
    if (publish_markers_)
    {
        publishCollisionMarkers(collision_pairs);
    }
    
    return !collision_pairs.empty();
}

void ArmCollisionChecker::publishCollisionMarkers(const std::vector<CollisionPair>& collision_pairs) {
    visualization_msgs::MarkerArray marker_array;
    
    // Create a set of colliding objects for quick lookup
    std::set<fcl::CollisionObjectd*> colliding_objects;
    for (const auto& pair : collision_pairs) {
        colliding_objects.insert(pair.first);
        colliding_objects.insert(pair.second);
    }
    
    // Publish all collision objects
    int marker_id = 0;
    for (const auto& link_pair : link_name_to_collision_object_) {
        const auto& link_name = link_pair.first;
        const auto& collision_object = link_pair.second;
        
        // Get collision geometry and transform
        auto geom = collision_object->getCollisionGeometry();
        auto transform = collision_object->getTransform();
        
        // Create marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_objects";
        marker.id = marker_id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1 / trigger_frequency);
        
        // Set color based on collision status
        if (colliding_objects.find(collision_object.get()) != colliding_objects.end()) {
            // Red for colliding objects
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        } else {
            // Green for non-colliding objects
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.1;
            continue;
        }
        
        // Set position and orientation
        marker.pose.position.x = transform.translation().x();
        marker.pose.position.y = transform.translation().y();
        marker.pose.position.z = transform.translation().z();
        
        // From rotation matrix get quaternion
        Eigen::Quaterniond q(transform.linear());
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        // Set marker type and scale based on geometry
        if (auto box = dynamic_cast<const fcl::Boxd*>(geom)) {
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = box->side[0];
            marker.scale.y = box->side[1];
            marker.scale.z = box->side[2];
        } else if (auto sphere = dynamic_cast<const fcl::Sphered*>(geom)) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = sphere->radius * 2;
            marker.scale.y = sphere->radius * 2;
            marker.scale.z = sphere->radius * 2;
        } else if (auto cylinder = dynamic_cast<const fcl::Cylinderd*>(geom)) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = cylinder->radius * 2;
            marker.scale.y = cylinder->radius * 2;
            marker.scale.z = cylinder->lz;
        } else if (auto mesh = dynamic_cast<const BVHModel*>(geom)) {
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.points.clear();

            // 获取 mesh 顶点
            const auto& tris = link_name_to_mesh_triangles_[link_name];
            for (const auto& tri : tris) {
                geometry_msgs::Point p;
                p.x = tri.v0[0]; p.y = tri.v0[1]; p.z = tri.v0[2];
                marker.points.push_back(p);
                p.x = tri.v1[0]; p.y = tri.v1[1]; p.z = tri.v1[2];
                marker.points.push_back(p);
                p.x = tri.v2[0]; p.y = tri.v2[1]; p.z = tri.v2[2];
                marker.points.push_back(p);
            }
        } else {
            // Default to sphere for unknown types
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
        }
        
        marker_array.markers.push_back(marker);
    }

    // === 新增：画左右手轨迹 ===
    visualization_msgs::Marker left_traj, right_traj;
    left_traj.header.frame_id = right_traj.header.frame_id = "base_link";
    left_traj.header.stamp = right_traj.header.stamp = ros::Time::now();
    left_traj.ns = "left_hand_traj";
    right_traj.ns = "right_hand_traj";
    left_traj.id = 1000;
    right_traj.id = 1001;
    left_traj.type = right_traj.type = visualization_msgs::Marker::LINE_STRIP;
    left_traj.action = right_traj.action = visualization_msgs::Marker::ADD;
    left_traj.scale.x = right_traj.scale.x = 0.01;
    left_traj.color.r = 1.0; left_traj.color.a = 1.0;
    right_traj.color.b = 1.0; right_traj.color.a = 1.0;
    
    for (int i = 0; i < recorded_sensors_data_.size(); ++i) {
        
        if(i % 100 != 0) continue;
        auto msg = recorded_sensors_data_[i];
        std::vector<double> q(msg.joint_data.joint_q.end() - arm_joint_num - head_joint_num, msg.joint_data.joint_q.end() - head_joint_num);

        kuavo_msgs::fkSrv fk_srv;
        fk_srv.request.q = q; // 如果FK服务需要全量joint_q
        if (fk_client_.call(fk_srv) && fk_srv.response.success) {
            geometry_msgs::Point p_left, p_right;
            p_left.x = fk_srv.response.hand_poses.left_pose.pos_xyz[0];
            p_left.y = fk_srv.response.hand_poses.left_pose.pos_xyz[1];
            p_left.z = fk_srv.response.hand_poses.left_pose.pos_xyz[2];
            left_traj.points.push_back(p_left);

            p_right.x = fk_srv.response.hand_poses.right_pose.pos_xyz[0];
            p_right.y = fk_srv.response.hand_poses.right_pose.pos_xyz[1];
            p_right.z = fk_srv.response.hand_poses.right_pose.pos_xyz[2];
            right_traj.points.push_back(p_right);
        }
    }
    marker_array.markers.push_back(left_traj);
    marker_array.markers.push_back(right_traj);


    // Publish markers
    collision_marker_pub_.publish(marker_array);
}


void ArmCollisionChecker::armModeCallback(const std_msgs::Int32::ConstPtr& msg) {
    
    if (internal_change_arm_mode) return;

    current_arm_mode_ = msg->data;
    ROS_INFO("Arm mode callback: %d", msg->data);
    if (msg->data == 1 || msg->data == 2) {

        if(is_collision_moving_) {
            is_collision_moving_ = false;
            stopArmCollisionControl();

            auto arm_collision_control_client = nh_.serviceClient<std_srvs::SetBool>("/quest3/set_arm_collision_control");
            std_srvs::SetBool arm_collision_control_request;
            arm_collision_control_request.request.data = false;
            if (arm_collision_control_client.call(arm_collision_control_request)) {
                ROS_INFO("Set arm collision control service call successful");
            } else {
                ROS_ERROR("Failed to call set arm collision control service");
            }

            is_control_enabled_ = true;
        }
    } 
}

void ArmCollisionChecker::tryToKeepArmPose() {

    // 检查IK节点是否存在
    ros::NodeHandle nh;
    std::vector<std::string> node_list;
    ros::master::getNodes(node_list);
    bool ik_node_exists = false;
    for (const auto& node : node_list) {
        if (node == "/ik_ros_uni") {
            ik_node_exists = true;
            break;
        }
    }

    // VR 模式下，碰撞手臂回归后，保持手臂姿态
    if(ik_node_exists) {
        auto arm_collision_control_client = nh_.serviceClient<std_srvs::SetBool>("/quest3/set_arm_collision_control");
        std_srvs::SetBool arm_collision_control_request;
        arm_collision_control_request.request.data = true;
        if (arm_collision_control_client.call(arm_collision_control_request)) {
            ROS_INFO("Set arm collision control service call successful");
        } else {
            ROS_ERROR("Failed to call set arm collision control service");
        }
    }
}

void ArmCollisionChecker::stopArmCollisionControl() {

    // 调用服务设置collision_check_control
    std_srvs::SetBool set_collision_check_request;
    set_collision_check_request.request.data = false;
    auto set_collision_check_client = nh_.serviceClient<std_srvs::SetBool>("/quest3/set_collision_check_control");
    if (set_collision_check_client.call(set_collision_check_request)) {
        ROS_INFO("Set collision check control service call successful");
    } else {
        ROS_ERROR("Failed to call set collision check control service");
    }
}

void ArmCollisionChecker::kuavoArmTrajCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    if(!is_collision_moving_) {
        // 转发到原有的 /kuavo_arm_traj 话题
        arm_traj_forward_pub_.publish(*msg);
    }
}

void ArmCollisionChecker::kuavoArmTargetPosesCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg) {
    
    if(!is_collision_moving_) {
        // 转发到原有的 /kuavo_arm_target_poses 话题
        arm_pose_pub_.publish(*msg);
    }
}

void ArmCollisionChecker::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg) {
    
    if(!is_collision_moving_) {
        double current_time = ros::Time::now().toSec();
        // std::lock_guard<std::mutex> lock(recorded_arm_pose_data_mutex_);
        recorded_sensors_data_.push_back(*msg);
        while(recorded_sensors_data_.size() > 0) {
            if (recorded_sensors_data_.front().header.stamp.toSec() < (current_time - record_duration_)) {
                // ROS_INFO("pop arm pose data: %f, %f", recorded_arm_pose_data_.front().header.stamp.toSec(), current_time);
                delay_sensors_data_pub_.publish(recorded_sensors_data_.front());
                recorded_sensors_data_.pop_front();
            }
            else break;
        }

        // ROS_INFO_THROTTLE(1.0, "Recorded sensors data size: %d", recorded_sensors_data_.size());
    }

}

void ArmCollisionChecker::playArmTrajBack() {
    if (recorded_sensors_data_.empty()) return;

    internal_change_arm_mode = true;
    auto change_arm_mode_client = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
    kuavo_msgs::changeArmCtrlMode srv;
    srv.request.control_mode = 2; // 切换到模式2
    change_arm_mode_client.call(srv);

    // 倒序回放
    ros::Time last_stamp = recorded_sensors_data_.back().header.stamp;
    // 倒序遍历,从最近的数据开始回放
    for (auto it = recorded_sensors_data_.rbegin(); it != recorded_sensors_data_.rend(); ++it) {
        if (it != recorded_sensors_data_.rbegin()) {
            ros::Time current_stamp = it->header.stamp;
            ros::Duration sleep_time = last_stamp - current_stamp;
            if (sleep_time.toNSec() > 0) {
                sleep_time.sleep(); 
            }
            last_stamp = current_stamp;
        }

        // 转换为JointState消息
        sensor_msgs::JointState js_msg;
        js_msg.header.stamp = ros::Time::now();
        js_msg.header.frame_id = it->header.frame_id;
        js_msg.name = {"arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6", 
                        "arm_joint_7", "arm_joint_8", "arm_joint_9", "arm_joint_10", "arm_joint_11", "arm_joint_12", "arm_joint_13"};
        // 只拷贝手臂关节
        js_msg.position.assign(
                it->joint_data.joint_q.end() - arm_joint_num - head_joint_num,
                it->joint_data.joint_q.end() - head_joint_num
            );

        for (int i = 0; i < js_msg.position.size(); ++i) {
            js_msg.position[i] = js_msg.position[i] * 180.0 / M_PI;
        }

        if (is_collision_moving_) arm_traj_forward_pub_.publish(js_msg);
        else {
            ROS_INFO("Arm collision check control is interrupted.");
            return;
        }
        arm_traj_debug_pub_.publish(js_msg);
    }    
    recorded_sensors_data_.clear();
    ROS_INFO("Arm collision check control is completed.");

    srv.request.control_mode = 0; // 切换到模式0
    change_arm_mode_client.call(srv);
    internal_change_arm_mode = false;
}

bool ArmCollisionChecker::waitCompleteCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    ROS_INFO("Wait complete callback");
    while(ros::ok()) {
        if(!is_collision_moving_) break;
        ros::Duration(0.1).sleep();
    }
    res.success = true;
    res.message = "Wait complete callback";
    return true;
}

bool ArmCollisionChecker::setArmMovingEnableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    enable_arm_moving_check_ = req.data;
    ROS_INFO("Arm moving check set to: %s", enable_arm_moving_check_ ? "enabled" : "disabled");
    res.success = true;
    res.message = enable_arm_moving_check_ ? "Arm moving check enabled" : "Arm moving check disabled";
    return true;
}

} // namespace kuavo_arm_collision_check

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_collision_checker");
    ros::NodeHandle nh;
    
    // 读取触发频率参数
    double trigger_frequency = 5.0;
    nh.param("/arm_collision/check_freq", trigger_frequency, 5.0);
    ROS_INFO("Arm collision check trigger frequency: %.2f Hz", trigger_frequency);

    kuavo_arm_collision_check::ArmCollisionChecker checker(nh);
    checker.trigger_frequency = trigger_frequency;
    // Create timer to trigger collision check at specified frequency
    ros::Timer collision_check_timer = nh.createTimer(
        ros::Duration(1.0/trigger_frequency),
        [&checker](const ros::TimerEvent&) {
            checker.triggerCollisionCheck();
        }
    );

    ros::spin();
    return 0;
} 
 