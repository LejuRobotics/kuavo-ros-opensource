// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <ostream>
#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include "mujoco_cpp/task2_score_observation.h"
#include "mujoco_cpp/task13_score_observation.h"
#include "ros/ros.h"
#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointData.h"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/FTsensorData.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <csignal>
#include <atomic>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "kuavo_msgs/lejuClawCommand.h"
#include "kuavo_msgs/SetObjectPosition.h"
#include "kuavo_msgs/SetJointPosition.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <kuavo_common/common/common.h>

#include "mujoco_cpp/depth_camera_config.h"
#include "joint_address.hpp"
#include "dexhand_mujoco_node.h"
#include "dexhand/json.hpp"

#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "mujoco_dds.h"
#endif

//  ******************* raycaster camera *********************

#include "RayCasterCamera.h"
#include "OffscreenCameraRenderer.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cmath>

//  ************************* lcm ****************************

#include "lcm_interface/LcmInterface.h"

// *****************************************************

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}
namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;
  std::shared_ptr<mj::Simulate> sim;
  std::mutex simulation_lifecycle_mutex;
  bool simulation_stop_requested = false;

  // 机器人版本号
  int robotVersion_ = 60;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length
  double frequency = 1000.0;             // simulation frequency (Hz)
  ros::Publisher sensorsPub;
  ros::Publisher pubGroundTruth;  // 重命名原来的pubOdom
  ros::Publisher pubOdom;          // 新增odom发布者
  ros::Publisher pubTimeDiff;
  ros::Publisher pubLeftArmFT;   // 左手臂末端力/扭矩
  ros::Publisher pubRightArmFT;  // 右手臂末端力/扭矩
  std::vector<std::string> task_body_names;
  std::unordered_map<std::string, ros::Publisher> task_body_pose_publishers;
  std::mt19937 object_random_generator(std::random_device{}());

  // Palm seating followed by an opposing finger grip releases the lever hinge.
  struct Task1LeverFingerLatchState {
    int handle_geom_id = -1;
    int palm_geom_id = -1;
    int wrist_body_id = -1;
    mjtNum palm_contact_duration = 0;
    bool palm_seated = false;
    ros::Publisher palm_publisher;
    int lever_lock_equality_id = -1;
    std::array<int, 3> finger_body_ids{{-1, -1, -1}};
    mjtNum grip_duration = 0;
    mjtNum grip_gap_duration = 0;
    bool lever_unlocked = false;
    ros::Publisher grip_publisher;
    ros::Publisher grip_contacts_publisher;
    std::uint8_t grip_contacts = 0;
  } task1_lever_latch;

  // Scene 2 observes physical contacts; it never attaches boxes to hands.
  struct BimanualContactFollowerState {
    std::string name;
    int body_id = -1;
    int qpos_addr = -1;
    int dof_addr = -1;
    bool held = false;  // Both hands have load-bearing contacts, not a latch.
    std::array<mjtNum, 3> conveyor_force{{0, 0, 0}};
    std::vector<int> collision_geom_ids;
  };
  std::vector<BimanualContactFollowerState> bimanual_contact_followers;
  std::unordered_set<int> bimanual_left_hand_geom_ids;
  std::unordered_set<int> bimanual_right_hand_geom_ids;
  std::unordered_map<std::string, ros::Publisher> bimanual_contact_state_publishers;
  std::unordered_map<std::string, ros::Publisher> task2_score_publishers;
  struct Task13ScoreBody {
    int body, source, target;
    bool bin;
    ros::Publisher publisher;
  };
  std::vector<Task13ScoreBody> task13_score_bodies;
  ros::Publisher source_bin_score_publisher;
  int source_bin_slide_id = -1;
  mjtNum source_bin_still_since = -1;
  int task2_table_geom_id = -1;
  mjtNum task2_score_last_time = -1;
  int task2_conveyor_body_id = -1;
  int task2_conveyor_belt_geom_id = -1;
  bool task2_conveyor_started = false;
  bool task2_conveyor_completed = false;
  std::atomic<bool> task2_conveyor_enabled{false};

  // Task 3 contact observations only. Objects and fingers remain physical.
  struct InternalContactFollowerState {
    std::string name;
    int body_id = -1;
    bool held = false;
    bool supported = false;
    int support_geom = -1;
    std::array<bool, 3> finger_latched{{false, false, false}};
    std::uint8_t last_published_latch_mask = 0;
    std::array<int, 3> finger_body_ids{{-1, -1, -1}};
  };
  std::vector<InternalContactFollowerState> internal_contact_followers;
  std::array<int, 3> internal_expansion_fingertip_geom_ids{{-1, -1, -1}};
  std::unordered_map<std::string, ros::Publisher>
      internal_contact_state_publishers;
  std::unordered_map<std::string, ros::Publisher>
      internal_finger_latch_publishers;
  std::unordered_map<std::string, ros::Publisher> physical_support_publishers;
  ros::Publisher internal_grasp_armed_publisher;
  bool internal_grasp_armed_published = false;
  ros::Publisher internal_fingertip_collision_suppressed_publisher;
  bool internal_fingertip_collision_suppressed_published = false;
  bool updateContactFollowers();
  void applyTask2ConveyorForce();
  mjtNum numericScalarOrDefault(const char *name, mjtNum fallback);
  bool pure_sim = false;

  // raycaster camera
  ros::Publisher depthImagePub;
  ros::Publisher depthImageArrayPub;
  std::unique_ptr<RayCasterCamera> g_depth_camera;
  const int DEPTH_CAMERA_WIDTH = 64;  // 64
  const int DEPTH_CAMERA_HEIGHT = 36;  // 36
  const mjtNum FOCAL_LENGTH = 2.12;
  const mjtNum HORIZONTAL_APERTURE = 4.24;  // 4.24
  const mjtNum VERTICAL_APERTURE = 2.4480;  // 2.4480
  const mjtNum DEPTH_CAMERA_MIN_RANGE = 0.17;
  const mjtNum DEPTH_CAMERA_MAX_RANGE = 2.5;
  const mjtNum DEPTH_CAMERA_H_PIXEL_SIZE = HORIZONTAL_APERTURE / DEPTH_CAMERA_WIDTH;
  const mjtNum DEPTH_CAMERA_V_PIXEL_SIZE = VERTICAL_APERTURE / DEPTH_CAMERA_HEIGHT;
  // raycaster camera thread
  std::thread depth_thread;
  std::atomic<bool> depth_thread_running{true};
  std::mutex mujoco_data_mutex;  // Protects access to m and d
  constexpr double kDefaultDepthFrequency = 30.0;
  double depth_frequency = kDefaultDepthFrequency;  // Hz
  bool isRunCamera_{false};

  // Data-challenge RGB cameras.  The upstream simulator already provides
  // RayCasterCamera for a single waist depth stream.  Keep that path intact
  // and add three opt-in named cameras for rosbag collection.  Only the color
  // image is published: depth is still rendered every frame by
  // OffscreenCameraRenderer (the render call fills both buffers in one pass),
  // but nothing subscribes to or publishes it.
  struct TaskRgbdCamera {
    std::string model_name;
    std::string color_frame_id;
    std::string color_topic;
    int camera_id = -1;
    image_transport::Publisher color_publisher;
  };
  std::unique_ptr<image_transport::ImageTransport> task_camera_transport;
  std::vector<TaskRgbdCamera> task_rgbd_cameras;
  mjData *task_camera_data = nullptr;
  // recursive: the model-reload path holds this across ConfigureTaskRgbdCameras...
  // while the camera thread may already hold it for the current frame.
  std::recursive_mutex task_camera_mutex;
  std::thread task_camera_thread;
  std::atomic<bool> task_camera_thread_running{false};
  // Owned and used exclusively by task_camera_thread: GL contexts are thread
  // private, so this must never be touched from the physics thread.
  std::unique_ptr<mujoco_cpp::OffscreenCameraRenderer> task_offscreen_renderer;
  // 渲染目标缓冲，同样只由 task_camera_thread 使用；放在成员上是为了每帧复用，
  // 不在 30 Hz 的热路径上反复分配。
  std::vector<std::uint8_t> task_rgbd_camera_rgb;
  // 深度缓冲仍在渲染时按帧填充（见 Render 的签名），只是不再发给任何人。
  std::vector<float> task_rgbd_camera_depth;
  // Bumped on every model (re)load; the camera thread rebuilds the GL context
  // when it sees a new value, because mjrContext holds display lists and
  // textures bound to the old mjModel.
  std::atomic<int> task_camera_model_epoch{0};
  constexpr int kTaskCameraWidth = 640;
  constexpr int kTaskCameraHeight = 480;
  constexpr double kTaskCameraFrequency = 30.0;
  constexpr mjtNum kTaskCameraHorizontalAperture = 4.24;

  // Depth image history buffer (3*7+1=22 frames)
  struct DepthImageFrame {
      std::vector<float> data;
      ros::Time timestamp;
  };
  static const int DEPTH_BUFFER_SIZE = 22;
  std::array<DepthImageFrame, DEPTH_BUFFER_SIZE> depth_buffer;
  size_t current_buffer_index = 0;
  bool depth_buffer_filled = false;  // Track if buffer has been filled once
  std::mutex depth_buffer_mutex;
  ros::Publisher depthHistoryPub;

#ifdef USE_DDS
  std::unique_ptr<MujocoDdsClient<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>> dds_client;
#elif defined(USE_LEJU_DDS)
  std::unique_ptr<MujocoDdsClient<leju::msgs::JointCmd, leju::msgs::SensorsData>> dds_client;
#endif
  std::queue<std::vector<double>> controlCommands;
  std::vector<double> joint_tau_cmd;
  bool cmd_updated = false;
  bool is_chassic_cmd_changed = false;
  bool is_chassic_cmd_vel_changed = false;

  geometry_msgs::Wrench external_wrench_;
  bool external_wrench_updated_ = false;

  std::vector<double> claw_cmd;
  bool claw_cmd_updated = false;
  size_t numClawJoints = 2; // 夹抓的自由度

  // 手臂外力（持续施加）
  geometry_msgs::Wrench left_hand_wrench_;
  geometry_msgs::Wrench right_hand_wrench_;
  bool left_hand_active_ = false;
  bool right_hand_active_ = false;
  int left_arm_link_id_ = -1;   // 缓存左手link ID
  int right_arm_link_id_ = -1;  // 缓存右手link ID
  // 躯干吊绳：三模块独立叠加（绞盘回锚 + 姿态回正 + 六轴阻尼）
  bool torso_rope_active_ = false;
  bool torso_rope_got_anchor_ = false;
  // 按直立 torso 高度估算：锚点 Z=1.8m， torso 质心约 0.65~0.7m，
  // 距离约 1.1m；绳长设 1.1m，站着激活时松弛，倒了激活时吊起。
  double torso_rope_length_ = 1.1;      // m，绳长；仅允许 ±5cm 步长调整
  double torso_rope_speed_ = 0.3;       // m/s，绞盘恒定回拉速度
  double torso_rope_kv_ = 5000.0;       // N/(m/s)，绞盘速度追踪增益
  constexpr double torso_rope_max_force_ = 2500.0; // N，绞盘最大拉力
  const double torso_rope_anchor_z_ = 1.8; // 锚点 Z，固定不可调
  double torso_rope_anchor_[3] = {0, 0, 0};
  // 模块 2：姿态回正（速度-位置控制），目标为世界坐标系直立
  double torso_upright_kp_ = 100.0;     // Nm/(rad/s)，姿态角速度追踪增益
  double torso_upright_speed_ = 0.35;   // rad/s，超出死区后三轴恒定回正速度（≈20°/s）
  double torso_upright_deadzone_ = 0.043633; // rad，≈2.5°，死区内无力矩
  // 模块 3：六轴阻尼（让躯干趋于静止）
  double torso_rope_lin_damp_ = 100.0;  // N/(m/s)，线速度阻尼
  double torso_rope_ang_damp_ = 15.0;   // Nm/(rad/s)，角速度阻尼
  int torso_body_id_ = -1;

  std::mutex queueMutex;
  ros::NodeHandle *g_nh_ptr;
  int robot_type = -1;
  size_t numJoints = 12;  // 默认值，将从配置文件中读取
  size_t waistNum = 0;
  size_t numWheels = 8;   /* LF + LB + RF + RB wheel */
  double is_spin_thread = true;
  ros::Time sim_time;
  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;
  std::vector<double> qpos_init;

  Eigen::Vector3d cmd_vel_chassis;

  // ******
  low_cmd_t recvCmd;
  // ******

  // 全局手臂末端关节名称变量
  std::string left_arm_end_joint = "zarm_l7_joint";   // 默认值
  std::string right_arm_end_joint = "zarm_r7_joint";  // 默认值
  
  // 躯干约束相关变量
  bool torso_constrained = false;
  double fixed_torso_pos[3] = {0, 0, 0};
  double fixed_torso_quat[4] = {1, 0, 0, 0};
  
  // 腿部关节约束相关变量
  bool leg_joints_constrained = false;
  std::vector<double> fixed_leg_l_qpos;  // 左腿关节固定位置
  std::vector<double> fixed_leg_r_qpos;  // 右腿关节固定位置
  
  double RayDistanceToZDepth(double ray_distance, double pixel_x, double pixel_y,
                                    double focal_length) {
    const double ray_norm = std::sqrt(pixel_x * pixel_x + pixel_y * pixel_y +
                                      focal_length * focal_length);
    if (ray_norm <= 0.0) {
      return 0.0;
    }
    return ray_distance * focal_length / ray_norm;
  }

  void ResetDepthBufferState()
  {
    std::unique_lock<std::mutex> buffer_lock(depth_buffer_mutex);
    current_buffer_index = 0;
    depth_buffer_filled = false;
    for (DepthImageFrame &frame : depth_buffer)
    {
      frame.data.clear();
      frame.timestamp = ros::Time();
    }
  }

  bool ConfigureDepthCameraForCurrentModel()
  {
    if(isRunCamera_ == false) return false;
    std::unique_lock<std::mutex> data_lock(mujoco_data_mutex);
    g_depth_camera.reset();

    if (!mujoco_cpp::ModelHasTargetDepthCamera(m))
    {
      data_lock.unlock();
      ResetDepthBufferState();
      ROS_INFO("[RayCasterCamera] Target camera '%s' not found in model, depth camera disabled.",
               mujoco_cpp::kDepthCameraName);
      return false;
    }

    try
    {
      auto depth_camera = std::make_unique<RayCasterCamera>(
          m, d,
          mujoco_cpp::kDepthCameraName,
          FOCAL_LENGTH,
          HORIZONTAL_APERTURE,
          DEPTH_CAMERA_WIDTH,
          DEPTH_CAMERA_HEIGHT,
          std::array<mjtNum, 2>{DEPTH_CAMERA_MIN_RANGE, DEPTH_CAMERA_MAX_RANGE},
          VERTICAL_APERTURE);
      depth_camera->set_num_thread(16);
      g_depth_camera = std::move(depth_camera);
      data_lock.unlock();
      ResetDepthBufferState();
      ROS_INFO("[RayCasterCamera] Depth camera initialized successfully at %s",
               mujoco_cpp::kDepthCameraName);
      return true;
    }
    catch (const std::exception &e)
    {
      data_lock.unlock();
      ResetDepthBufferState();
      ROS_WARN("[RayCasterCamera] Initialization failed for %s: %s",
               mujoco_cpp::kDepthCameraName, e.what());
      return false;
    }
  }

  bool ConfigureTaskRgbdCamerasForCurrentModel()
  {
    const std::unique_lock<std::recursive_mutex> camera_lock(task_camera_mutex);
    task_rgbd_cameras.clear();
    if (task_camera_data != nullptr)
    {
      mj_deleteData(task_camera_data);
      task_camera_data = nullptr;
    }
    if (!isRunCamera_ || !task_camera_transport || m == nullptr || d == nullptr)
    {
      return false;
    }
    task_camera_data = mj_makeData(m);
    if (task_camera_data == nullptr)
    {
      ROS_ERROR("[TaskCamera] Failed to allocate MuJoCo data snapshot");
      return false;
    }
    {
      const std::unique_lock<std::recursive_mutex> simulation_lock(sim->mtx);
      mj_copyData(task_camera_data, m, d);
    }

    struct CameraSpec {
      const char *model_name;
      const char *color_frame_id;
      const char *color_topic;
    };
    static constexpr CameraSpec kCameraSpecs[] = {
        {"cam_h", "cam_h_color_optical_frame", "/cam_h/color/image_raw"},
        {"cam_l", "cam_l_color_optical_frame", "/cam_l/color/image_raw"},
        {"cam_r", "cam_r_color_optical_frame", "/cam_r/color/image_raw"},
    };

    for (const CameraSpec &spec : kCameraSpecs)
    {
      const int camera_id = mj_name2id(m, mjOBJ_CAMERA, spec.model_name);
      if (camera_id < 0)
      {
        ROS_WARN("[TaskCamera] Named camera '%s' is absent", spec.model_name);
        continue;
      }
      TaskRgbdCamera camera;
      camera.model_name = spec.model_name;
      camera.color_frame_id = spec.color_frame_id;
      camera.color_topic = spec.color_topic;
      camera.camera_id = camera_id;
      camera.color_publisher = task_camera_transport->advertise(camera.color_topic, 2);
      task_rgbd_cameras.emplace_back(std::move(camera));
    }

    // 模型换了，GL 侧的一切（显示列表、纹理、离屏 FBO 尺寸）都要重建。
    // 真正重建发生在 task_camera_thread 里，因为 GL context 是线程私有的。
    task_camera_model_epoch.fetch_add(1);

    ROS_INFO("[TaskCamera] Configured %zu RGB cameras at %dx%d, %.1f Hz",
             task_rgbd_cameras.size(), kTaskCameraWidth, kTaskCameraHeight,
             kTaskCameraFrequency);
    return task_rgbd_cameras.size() == 3;
  }

  // 从光栅化的颜色缓冲直接拷成 RGB8。不再按 geom 上色：颜色、光照、材质、
  // 阴影都由 mjr_render 给出。
  sensor_msgs::Image BuildTaskColorImage(
      const TaskRgbdCamera &camera, const ros::Time &stamp,
      const std::uint8_t *rgb)
  {
    sensor_msgs::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = camera.color_frame_id;
    msg.height = kTaskCameraHeight;
    msg.width = kTaskCameraWidth;
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.is_bigendian = 0;
    msg.step = kTaskCameraWidth * 3;
    const std::size_t size =
        static_cast<std::size_t>(kTaskCameraWidth) * kTaskCameraHeight * 3;
    msg.data.assign(rgb, rgb + size);
    return msg;
  }

  // control noise variables
  // mjtNum* ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  //---------------------------------- depth history publisher -----------------------------------
  
  void publish_depth_history()
  {
    std::unique_lock<std::mutex> lock(depth_buffer_mutex);

    // From 6*7+1=43 frames, take frame indices: 0, 3, 6, 9, 12, 15, 18, 21 (8 frames total)
    std::vector<int> selected_indices;
    for (int i = 0; i < 7; ++i) {
      selected_indices.push_back(i * 3); // 1st frame of each group
    }
    selected_indices.push_back(DEPTH_BUFFER_SIZE - 1);  // Last remaining frame
    
    std::vector<float> first_frame_data;
    if (!depth_buffer[0].data.empty()) {
      first_frame_data = depth_buffer[0].data;
    }

    // if (!depth_buffer_filled) {
    //   printf("depth buffer filled: %d; cur ids: %d \n", depth_buffer_filled, current_buffer_index);
    // }
    
    ros::Time start_time = ros::Time::now();
    std_msgs::Float64MultiArray history_array_msg;
    for (int i = 0; i < selected_indices.size(); ++i) {
    // for (int i = selected_indices.size() - 1; i >= 0; --i) {
      int idx = selected_indices[i];
      // go backward to find history frame
      // int buffer_pos = (current_buffer_index - 1 - (DEPTH_BUFFER_SIZE - 1 - idx) + DEPTH_BUFFER_SIZE * 100) % DEPTH_BUFFER_SIZE;
      int buffer_pos = ((current_buffer_index - 1) - idx + DEPTH_BUFFER_SIZE * 100) % DEPTH_BUFFER_SIZE;
      
      // If buffer is not yet full and this position is beyond the current write point, use first frame
      if (!depth_buffer_filled && buffer_pos >= current_buffer_index) {
        for (float val : first_frame_data) {
          history_array_msg.data.push_back(val);
        }
        if (current_buffer_index < 3){
          printf("%d ", buffer_pos);
        }
      } else if (!depth_buffer[buffer_pos].data.empty()) {
        for (float val : depth_buffer[buffer_pos].data) {
          history_array_msg.data.push_back(val);
        }
      } else if (!first_frame_data.empty()) {
        // If this position is empty but buffer is full, use first frame as fallback
        for (float val : first_frame_data) {
          history_array_msg.data.push_back(val);
        }
      }
    }
    // if (!depth_buffer_filled && current_buffer_index < 3){
    //   printf("\n");
    // }
    lock.unlock();
    depthHistoryPub.publish(history_array_msg);
  }
  

  /************************************* Joint Address******************************************/
  // This section defines the joint addresses for various body parts of the robot.
  using namespace mujoco_node;
  JointGroupAddress LegJointsAddr("leg_joints");
  JointGroupAddress WheelJointsAddr("wheel_yaw_joint");
  JointGroupAddress LLegJointsAddr("l_leg_joints");
  JointGroupAddress RLegJointsAddr("r_leg_joints");
  JointGroupAddress WaistJointsAddr("waist_yaw_joint");
  JointGroupAddress LArmJointsAddr("l_arm_joints");
  JointGroupAddress RArmJointsAddr("r_arm_joints");
  JointGroupAddress HeadJointsAddr("head_joints");
  JointGroupAddress LHandJointsAddr("l_hand_joints");
  JointGroupAddress RHandJointsAddr("r_hand_joints");
  /*********************************************************************************************/
  // Mujoco Dexhand
  std::shared_ptr<mujoco_node::DexHandMujocoRosNode> g_dexhand_node = nullptr;
  /*********************************************************************************************/

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {

    constexpr char kPathSep = '/';
    const char *path = "/proc/self/exe";

    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }
    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }
    const std::string sep = "/";

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  void init_joint_address(mjModel* model, JointGroupAddress &jga, const std::string& joint0, const std::string& joint1)
  {
    // 获取关节 ID
    auto id0 = mj_name2id(model, mjOBJ_JOINT, joint0.c_str());
    auto id1 = mj_name2id(model, mjOBJ_JOINT, joint1.c_str());
    if (!(id0 >= 0 && id1 >= 0) || !(id1 < model->njnt)) {
        std::cout << "\033[31mWarning: Invalid joint index for joints " << joint0 << " (id=" << id0 
                  << ") and " << joint1 << " (id=" << id1 << ")\033[0m" << std::endl;
        return;
    }

    // 获取 qpos 地址
    auto qpos0 = model->jnt_qposadr[id0];
    auto qpos1 = model->jnt_qposadr[id1];
    if (qpos0 == -1 || qpos1 == -1) {
        std::cout << "\033[31mWarning: Invalid qpos address for joints " << joint0 << " (addr=" << qpos0 
                  << ") and " << joint1 << " (addr=" << qpos1 << ")\033[0m" << std::endl;
        return;
    }

    // 获取自由度（dof）地址
    auto dof0 = model->jnt_dofadr[id0];
    auto dof1 = model->jnt_dofadr[id1];
    if (dof0 == -1 || dof1 == -1) {
        std::cout << "\033[31mWarning: Invalid dof address for joints " << joint0 << " (addr=" << dof0 
                  << ") and " << joint1 << " (addr=" << dof1 << ")\033[0m" << std::endl;
        return;
    }

    std::string actuator0 = joint0 + "_motor";
    std::string actuator1 = joint1 + "_motor";
    auto ctrl0 = mj_name2id(model, mjOBJ_ACTUATOR, actuator0.c_str());
    auto ctrl1 = mj_name2id(model, mjOBJ_ACTUATOR, actuator1.c_str());
    if (!(ctrl0 >= 0 && ctrl1 >= 0) || !(ctrl1 < model->nu)) {
        std::cout << "\033[31mWarning: Invalid actuator index for actuators " << actuator0 << " (id=" << ctrl0 
                  << ") and " << actuator1 << " (id=" << ctrl1 << ")\033[0m" << std::endl;
        return;
    }

    // Set joint addresses
    jga.set_ctrladr(ctrl0, ctrl1)
        .set_qposadr(qpos0, qpos1)
        .set_qdofadr(dof0, dof1);

    std::cout << jga <<std::endl;
  }

  //------------------------------------------- simulation -------------------------------------------
  void signalHandler(int signum)
  {
    if (signum == SIGINT) // 捕获Ctrl+C信号
    {
      sim->exitrequest.store(1);
      if(g_dexhand_node) {
        g_dexhand_node->stop();
      }
      
      std::cout << "Ctrl+C pressed, exit request sent." << std::endl;
    }
  }
  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      if (!mnew){
        std::cerr << "[Mujoco]: load mode error:" << loadError <<std::endl;
        return nullptr;
      }
      
      /* Init Joint Address 初始化关节组的数据地址 */

      // 通过rosparam robot_type (int) 区分结构
      if (robot_type == 2) 
      {
        std::cout << "[mujoco_node] Using ROS param: biped (双足) robot structure." << std::endl;
        init_joint_address(mnew, LLegJointsAddr, "leg_l1_joint", "leg_l6_joint");
        init_joint_address(mnew, RLegJointsAddr, "leg_r1_joint", "leg_r6_joint");
      } 
      else if (robot_type == 1) 
      {
        std::cout << "[mujoco_node] Using ROS param: wheel-arm (轮臂) robot structure." << std::endl;
        init_joint_address(mnew, WheelJointsAddr, "LF_wheel_yaw_joint", "RB_wheel_pitch_joint");
        init_joint_address(mnew, LegJointsAddr, "knee_joint", "waist_yaw_joint");
      } 
      else 
      {
        std::cout << "[mujoco_node] Unknown robot_type param, please set to 1 (轮臂) or 2 (双足)!" << std::endl;
        return nullptr;
      }
      
      // 其余分组
      init_joint_address(mnew, WaistJointsAddr, "waist_yaw_joint", "waist_yaw_joint");
      std::cout << "left_arm_end_joint: " << left_arm_end_joint << std::endl;
      std::cout << "right_arm_end_joint: " << right_arm_end_joint << std::endl;
      init_joint_address(mnew, LArmJointsAddr, "zarm_l1_joint", left_arm_end_joint.c_str());
      init_joint_address(mnew, RArmJointsAddr, "zarm_r1_joint", right_arm_end_joint.c_str());
      init_joint_address(mnew, HeadJointsAddr, "zhead_1_joint", "zhead_2_joint");

      /* dexhand joint address - 根据URDF自定义元数据hand_type区分手型号 */
      int hand_type_id = mj_name2id(mnew, mjOBJ_NUMERIC, "hand_type");
      if (hand_type_id != -1) {
          int data_adr = mnew->numeric_adr[hand_type_id];
          int hand_type_value = static_cast<int>(mnew->numeric_data[data_adr]);
          if (hand_type_value == 1) {
              // LinkerL6灵巧手关节命名
              std::cout << "[mujoco_node]: Initialize LinkerL6 dexhand joint addresses" << std::endl;
              init_joint_address(mnew, LHandJointsAddr, "l_thumb_cmc_yaw", "l_pinky_dip");
              init_joint_address(mnew, RHandJointsAddr, "r_thumb_cmc_yaw", "r_pinky_dip");
          } else if (hand_type_value == 2) {
              // LinkerO6灵巧手关节命名
              std::cout << "[mujoco_node]: Initialize LinkerO6 dexhand joint addresses" << std::endl;
              init_joint_address(mnew, LHandJointsAddr, "l_thumb_cmc_yaw", "l_pinky_dip");
              init_joint_address(mnew, RHandJointsAddr, "r_thumb_cmc_yaw", "r_pinky_dip");
          } else if (hand_type_value == 3) {
              std::cout << "[mujoco_node]: Initialize Heiman SG100 joint addresses" << std::endl;
              init_joint_address(mnew, LHandJointsAddr, "l_thumb_j1", "l_little_j3");
              init_joint_address(mnew, RHandJointsAddr, "r_thumb_j1", "r_little_j3");
          }
      } else {
          // 旧版无hand_type元数据时，默认使用强脑手关节命名
          std::cout << "[mujoco_node]: No hand_type metadata found, default to Qiangnao hand joint addresses" << std::endl;
          init_joint_address(mnew, LHandJointsAddr, "l_thumbCMC", "l_littlePIP");
          init_joint_address(mnew, RHandJointsAddr, "r_thumbCMC", "r_littlePIP");
      }

      // 遍历所有的物体
      double totalMass = 0.0;
      for (int i = 0; i < mnew->nbody; i++)
      {
        totalMass += mnew->body_mass[i];
      }
      std::cout << "mujoco totalMass: " << totalMass << std::endl;

      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }
    sim.run = 0;

    return mnew;
  }

  // *****************************************************

  void InitRobotState(mjData *d)
  {
    // init qpos
    
//0.99863, -0.00000, 0.05233, -0.00000, -0.01767, 0.00000, 0.77337, -0.01871, -0.00197, -0.63345, 0.88205, -0.35329, 0.01882, 0.01871, 0.00197, -0.63345, 0.88204, -0.35329, -0.01882, 
    for (int i = 0; i < m->nq; i++)
    {
      d->qpos[i] = qpos_init[i];
    }
  }

  void mycontroller(const mjModel *m, mjData *d)
  {
    // 10
    for (size_t i = 0; i < m->nu; i++)
      d->ctrl[i] = recvCmd.ff_tau[i] + recvCmd.kp[i] * (recvCmd.joint_pos[i] - d->qpos[7 + i]) + recvCmd.kd[i] * (recvCmd.joint_vel[i] - d->qvel[6 + i]);
  }

  void init_cmd(mjData *d)
  {
    for (size_t i = 0; i < m->nu; i++)
    {
      recvCmd.ff_tau[i] = 0;
      recvCmd.kp[i] = 0;
      recvCmd.kd[i] = 0;
      recvCmd.joint_pos[i] = 0;
      recvCmd.joint_vel[i] = 0;
      d->ctrl[i] = 0;
    }
  }
  Eigen::Vector3d removeGravity(const Eigen::Vector3d &rawAccel, const Eigen::Quaterniond &orientation)
  {
    // 设置重力向量在全局坐标系中的方向，假设重力向量的方向为 (0, 0, -9.81)
    Eigen::Vector3d gravity(0.0, 0.0, 9.785); // TODO: 安装方向

    // 将重力向量转换到局部坐标系中
    Eigen::Vector3d localGravity = orientation.conjugate() * gravity;

    // 计算去除重力影响的加速度
    Eigen::Vector3d accelNoGravity = rawAccel - localGravity;

    return accelNoGravity;
  }
  // *****************************************************
  void publish_ros_data(const mjData *d, bool is_running)
  {
    static double last_ros_time = ros::Time::now().toSec();
    std_msgs::Float64 time_diff;
    time_diff.data = (ros::Time::now().toSec() - last_ros_time) * 1000;
    pubTimeDiff.publish(time_diff);
    last_ros_time = ros::Time::now().toSec();
    // publish joint data
    kuavo_msgs::sensorsData sensors_data;
    sensors_data.header.stamp = ros::Time::now();
    sensors_data.sensor_time = sim_time;
    sensors_data.header.frame_id = "world";
    kuavo_msgs::jointData joint_data;

    auto updateJointData = [&](const JointGroupAddress& jointAddr) {
        for (auto iter = jointAddr.qposadr().begin(); iter != jointAddr.qposadr().end(); iter++) {
            // add joint position
            joint_data.joint_q.push_back(d->qpos[*iter]);
        }
        for (auto iter = jointAddr.qdofadr().begin(); iter != jointAddr.qdofadr().end(); iter++) {
            // add joint velocity, acceleration, force
            joint_data.joint_v.push_back(d->qvel[*iter]);
            joint_data.joint_vd.push_back(d->qacc[*iter]);
            joint_data.joint_torque.push_back(d->qfrc_actuator[*iter]);
        }
    };
    // Joint Data: LLeg, RLeg, LArm, RArm, Head
    if (robot_type == 2) 
    {
      updateJointData(LLegJointsAddr);
      updateJointData(RLegJointsAddr);
      updateJointData(WaistJointsAddr);
    } 
    else if (robot_type == 1) 
    {
      updateJointData(LegJointsAddr);
    }
    else 
    {
      std::cout << "[mujoco_node] Unknown robot_type param, please set to 1 (轮臂) or 2 (双足)!" << std::endl;
      return;
    }
    updateJointData(LArmJointsAddr);
    updateJointData(RArmJointsAddr);
    updateJointData(HeadJointsAddr);
    
    // Dexhand: read state
    if(g_dexhand_node) {
      g_dexhand_node->readCallback(d);
    }
    
    kuavo_msgs::imuData imu_data;
    nav_msgs::Odometry bodyOdom;
    int pos_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyPos")];
    int ori_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyQuat")];
    int vel_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyVel")];
    int gyro_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyGyro")];
    int acc_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyAcc")];
    // std::cout << "pos_addr: " << pos_addr << std::endl;
    // std::cout << "ori_addr: " << ori_addr << std::endl;
    // std::cout << "gyro_addr: " << gyro_addr << std::endl;
    // std::cout << "acc_addr: " << acc_addr << std::endl;
    mjtNum *pos = d->sensordata + pos_addr;
    mjtNum *ori = d->sensordata + ori_addr;
    mjtNum *vel = d->sensordata + vel_addr;

    mjtNum *angVel = d->sensordata + gyro_addr;
    mjtNum *acc = d->sensordata + acc_addr;
    Eigen::Vector3d free_acc;
    Eigen::Vector3d acc_eigen;
    Eigen::Quaterniond quat_eigen(ori[0], ori[1], ori[2], ori[3]);
    if (is_running)
    {
      acc_eigen << acc[0], acc[1], acc[2];
      free_acc = removeGravity(acc_eigen, quat_eigen);
    }
    else
    {
      acc_eigen << 0, 0, 9.81;
      free_acc << 0, 0, 0;
    }
    // mjtNum *free_acc = remove_gravity(acc, [ ori[0], ori[1], ori[2], ori[3] ]);
    imu_data.acc.x = acc_eigen[0];
    imu_data.acc.y = acc_eigen[1];
    imu_data.acc.z = acc_eigen[2];
    imu_data.gyro.x = angVel[0];
    imu_data.gyro.y = angVel[1];
    imu_data.gyro.z = angVel[2];
    imu_data.free_acc.x = free_acc[0];
    imu_data.free_acc.y = free_acc[1];
    imu_data.free_acc.z = free_acc[2];
    imu_data.quat.x = ori[1];
    imu_data.quat.y = ori[2];
    imu_data.quat.z = ori[3];
    imu_data.quat.w = ori[0];

    sensors_data.joint_data = joint_data;
    sensors_data.imu_data = imu_data;

    // Read and publish FT sensor data (force/torque sensors)
    kuavo_msgs::FTsensorData FTsensor_data;
    int l_force_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "l_force")];
    int l_torque_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "l_torque")];
    int r_force_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "r_force")];
    int r_torque_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "r_torque")];
    
    // Check if FT sensors exist in the model
    if (l_force_addr >= 0 && l_torque_addr >= 0 && r_force_addr >= 0 && r_torque_addr >= 0) {
      mjtNum *l_force = d->sensordata + l_force_addr;
      mjtNum *l_torque = d->sensordata + l_torque_addr;
      mjtNum *r_force = d->sensordata + r_force_addr;
      mjtNum *r_torque = d->sensordata + r_torque_addr;
      FTsensor_data.Fx.push_back(l_force[0]);
      FTsensor_data.Fx.push_back(r_force[0]);
      FTsensor_data.Fy.push_back(l_force[1]);
      FTsensor_data.Fy.push_back(r_force[1]);
      FTsensor_data.Fz.push_back(l_force[2]);
      FTsensor_data.Fz.push_back(r_force[2]);
      FTsensor_data.Mx.push_back(l_torque[0]);
      FTsensor_data.Mx.push_back(r_torque[0]);
      FTsensor_data.My.push_back(l_torque[1]);
      FTsensor_data.My.push_back(r_torque[1]);
      FTsensor_data.Mz.push_back(l_torque[2]);
      FTsensor_data.Mz.push_back(r_torque[2]);
    }
    sensors_data.FTsensor_data = FTsensor_data;

#ifdef USE_DDS
    // Publish DDS LowState via DDS (instead of ROS when DDS is enabled)
    if (dds_client) {
      unitree_hg::msg::dds_::LowState_ dds_state;
      Eigen::Vector3d angVel_eigen(angVel[0], angVel[1], angVel[2]);
      Eigen::Vector4d ori_eigen(ori[0], ori[1], ori[2], ori[3]);
      ConvertMujocoToDdsState(joint_data.joint_q, joint_data.joint_v, joint_data.joint_vd, joint_data.joint_torque, acc_eigen, angVel_eigen, free_acc, ori_eigen, dds_state);
      dds_client->publishLowState(dds_state);
    }
    else
    {
      std::cout << "NOT PUB" << std::endl;
    }
#elif defined(USE_LEJU_DDS)
    // Publish Leju DDS SensorsData when LEJU_DDS is enabled
    if (dds_client) {
      leju::msgs::SensorsData leju_sensors_data;
      Eigen::Vector3d angVel_eigen(angVel[0], angVel[1], angVel[2]);
      Eigen::Vector4d ori_eigen(ori[0], ori[1], ori[2], ori[3]);
      ConvertMujocoToDdsState(joint_data.joint_q, joint_data.joint_v, joint_data.joint_vd, joint_data.joint_torque, acc_eigen, angVel_eigen, free_acc, ori_eigen, leju_sensors_data);
      dds_client->publishLowState(leju_sensors_data);
    }
    else
    {
      std::cout << "Leju NOT PUB" << std::endl;
    }
#else
    // Publish ROS sensor data only when DDS is disabled
    sensorsPub.publish(sensors_data);
#endif

    // bodyOdom = Odometry();
    bodyOdom.header.stamp = sim_time;
    bodyOdom.pose.pose.position.x = pos[0];
    bodyOdom.pose.pose.position.y = pos[1];
    bodyOdom.pose.pose.position.z = pos[2];
    bodyOdom.pose.pose.orientation.x = ori[1];
    bodyOdom.pose.pose.orientation.y = ori[2];
    bodyOdom.pose.pose.orientation.z = ori[3];
    bodyOdom.pose.pose.orientation.w = ori[0];
    bodyOdom.twist.twist.linear.x = vel[0];
    bodyOdom.twist.twist.linear.y = vel[1];
    bodyOdom.twist.twist.linear.z = vel[2];

    bodyOdom.twist.twist.angular.x = angVel[0];
    bodyOdom.twist.twist.angular.y = angVel[1];
    bodyOdom.twist.twist.angular.z = angVel[2];
    pubGroundTruth.publish(bodyOdom);  // 发布到/ground_truth/state
    pubOdom.publish(bodyOdom);         // 发布到/odom

    for (const std::string &name : task_body_names) {
      const int body_id = mj_name2id(m, mjOBJ_BODY, name.c_str());
      if (body_id < 0) {
        continue;
      }
      const mjtNum *body_pos = d->xpos + 3 * body_id;
      const mjtNum *body_quat = d->xquat + 4 * body_id;
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = sim_time;
      pose.header.frame_id = "world";
      pose.pose.position.x = body_pos[0];
      pose.pose.position.y = body_pos[1];
      pose.pose.position.z = body_pos[2];
      pose.pose.orientation.w = body_quat[0];
      pose.pose.orientation.x = body_quat[1];
      pose.pose.orientation.y = body_quat[2];
      pose.pose.orientation.z = body_quat[3];
      task_body_pose_publishers.at(name).publish(pose);
    }

    // 读取并发布手臂末端力/扭矩传感器数据
    int l_arm_force_id = mj_name2id(m, mjOBJ_SENSOR, "l_arm_force");
    int l_arm_torque_id = mj_name2id(m, mjOBJ_SENSOR, "l_arm_torque");
    int r_arm_force_id = mj_name2id(m, mjOBJ_SENSOR, "r_arm_force");
    int r_arm_torque_id = mj_name2id(m, mjOBJ_SENSOR, "r_arm_torque");

    // 检查左手臂传感器是否存在
    if (l_arm_force_id >= 0 && l_arm_torque_id >= 0) {
      int l_arm_force_addr = m->sensor_adr[l_arm_force_id];
      int l_arm_torque_addr = m->sensor_adr[l_arm_torque_id];
      mjtNum *l_arm_force = d->sensordata + l_arm_force_addr;
      mjtNum *l_arm_torque = d->sensordata + l_arm_torque_addr;

      geometry_msgs::WrenchStamped left_arm_ft;
      left_arm_ft.header.stamp = sim_time;
      left_arm_ft.header.frame_id = "l_arm_ft_frame";
      left_arm_ft.wrench.force.x = l_arm_force[0];
      left_arm_ft.wrench.force.y = l_arm_force[1];
      left_arm_ft.wrench.force.z = l_arm_force[2];
      left_arm_ft.wrench.torque.x = l_arm_torque[0];
      left_arm_ft.wrench.torque.y = l_arm_torque[1];
      left_arm_ft.wrench.torque.z = l_arm_torque[2];
      pubLeftArmFT.publish(left_arm_ft);
    }

    // 检查右手臂传感器是否存在
    if (r_arm_force_id >= 0 && r_arm_torque_id >= 0) {
      int r_arm_force_addr = m->sensor_adr[r_arm_force_id];
      int r_arm_torque_addr = m->sensor_adr[r_arm_torque_id];
      mjtNum *r_arm_force = d->sensordata + r_arm_force_addr;
      mjtNum *r_arm_torque = d->sensordata + r_arm_torque_addr;

      geometry_msgs::WrenchStamped right_arm_ft;
      right_arm_ft.header.stamp = sim_time;
      right_arm_ft.header.frame_id = "r_arm_ft_frame";
      right_arm_ft.wrench.force.x = r_arm_force[0];
      right_arm_ft.wrench.force.y = r_arm_force[1];
      right_arm_ft.wrench.force.z = r_arm_force[2];
      right_arm_ft.wrench.torque.x = r_arm_torque[0];
      right_arm_ft.wrench.torque.y = r_arm_torque[1];
      right_arm_ft.wrench.torque.z = r_arm_torque[2];
      pubRightArmFT.publish(right_arm_ft);
    }
  }

  double velocity_pid_func(int i, double target_vel)
  {
    double cur_vel = d->qvel[6 + i];
    // std::cout << "i: " << i << "  cur_vel:  " << cur_vel << std::endl;
    double error = target_vel - cur_vel;

    double torque = 120 * error;
    // std::cout << "i: " << i << "  torque:  " << torque << std::endl;
    return torque;
  }

  void updateWheelVel_VectorContorl(Eigen::Vector3d& cmd_vel)
  {
    const double wheel_radius = 0.075;  // 底盘轮子半径
    const double robot_x_dis = 0.253; // 机器人中心到轮子的距离
    const double robot_y_dis = 0.1785; // 机器人中心到轮子的距离

    // 四个轮子的位置（相对于底盘中心）
    std::vector<Eigen::Vector2d> wheel_positions = {
        Eigen::Vector2d( robot_x_dis,  robot_y_dis), // 左前轮 (x+, y+)
        Eigen::Vector2d( robot_x_dis, -robot_y_dis), // 右前轮 (x+, y-)
        Eigen::Vector2d(-robot_x_dis,  robot_y_dis), // 左后轮 (x-, y+)
        Eigen::Vector2d(-robot_x_dis, -robot_y_dis)  // 右后轮 (x-, y-)
    };
    
    for(int i = 0; i < 4; i++)
    {
      // 计算对应轮子的旋转速度，
      Eigen::Vector2d rotational_vel(-wheel_positions[i].y() * cmd_vel[2], 
                                      wheel_positions[i].x() * cmd_vel[2]);
      
      // 计算对应轮子的总速度矢量， x 总指向机器人的正前方
      Eigen::Vector2d wheel_vel(cmd_vel[0] + rotational_vel.x(), 
                                 cmd_vel[1] + rotational_vel.y());

      // 3. 计算轮子的转向角度（yaw）
      double wheel_yaw = std::atan2(wheel_vel.y(), wheel_vel.x());
      
      // 4. 计算轮子的转速（模长）
      double wheel_speed = wheel_vel.norm();
      
      // 5. 设置电机控制
      d->qpos[7 + i*2] = wheel_yaw;                    // 设置转向角度
      d->ctrl[i*2 + 1] = velocity_pid_func(i*2 + 1, wheel_speed / wheel_radius);
    }
  }

  void updateWheelVel_VectorContorl_omniWheel(Eigen::Vector3d& cmd_vel)
  {
    const double wheel_radius = 0.13035;  // 底盘轮子半径
    // s63 底盘轮距更大
    const double robot_x_dis = (robotVersion_ == 63) ? 0.23865 : 0.232489;  // 机器人中心到轮子的距离
    const double robot_y_dis = (robotVersion_ == 63) ? 0.23865 : 0.232489;  // 机器人中心到轮子的距离

    // cmd_vel: [vx, vy, omega] - 机器人本体系速度和角速度
    // 四个轮子的位置（相对于底盘中心）
    std::vector<Eigen::Vector2d> wheel_positions = {
        Eigen::Vector2d( robot_x_dis,  robot_y_dis), // 左前轮 (x+, y+)
        Eigen::Vector2d( robot_x_dis, -robot_y_dis), // 右前轮 (x+, y-)
        Eigen::Vector2d(-robot_x_dis,  robot_y_dis), // 左后轮 (x-, y+)
        Eigen::Vector2d(-robot_x_dis, -robot_y_dis)  // 右后轮 (x-, y-)
    };

    for(int i = 0; i < 4; i++)
    {
      // 计算对应轮子的旋转速度，
      Eigen::Vector2d rotational_vel(-wheel_positions[i].y() * cmd_vel[2], 
                                      wheel_positions[i].x() * cmd_vel[2]);
      
      // 计算对应轮子的总速度矢量， x 总指向机器人的正前方
      Eigen::Vector2d wheel_vel(cmd_vel[0] + rotational_vel.x(), 
                                 cmd_vel[1] + rotational_vel.y());

      // 3. 计算轮子的转向角度（yaw）
      double wheel_yaw = std::atan2(wheel_vel.y(), wheel_vel.x());
      
      // 4. 计算轮子的转速（模长）
      double wheel_speed = wheel_vel.norm();
      
      // 5. 设置电机控制
      d->qpos[7 + i*2] = wheel_yaw;                    // 设置转向角度
      d->ctrl[i*2 + 1] = velocity_pid_func(i*2 + 1, wheel_speed / wheel_radius);
    }

  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    MujocoLcm mujocolcm;
    mujocolcm.startLCMThread();
    // mjcb_control = mycontroller;

    std::vector<double> tau_cmd(numJoints);
    std::cout << "loop started." << std::endl;
    queueMutex.lock();
    while (!controlCommands.empty())
    {
      controlCommands.pop();
    }
    joint_tau_cmd = std::vector<double>(numJoints, 0);
    claw_cmd = std::vector<double>(numClawJoints, 0);
    queueMutex.unlock();
    uint64_t step_count = 0;
    sim_time = ros::Time::now();

    // Depth history publishing timer (60Hz)
    ros::Time last_depth_history_pub_time = ros::Time::now();
    double depth_history_interval = 1.0 / 60.0;  // 60Hz

    // run until asked to exit
    ros::Rate loop_rate(frequency);
    while (!sim.exitrequest.load())
    {
      if (sim.uiloadrequest.load())
      {
        std::cout << "uiloadrequest" << std::endl;
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        // 相机线程在 task_camera_mutex 之外读 m 做离屏渲染，所以整个换模型窗口
        // （删旧、赋新、重建相机列表）都必须把它挡住，否则会读到已释放的 mjModel，
        // 或者拿着旧模型的 camera_id 去渲染新模型。
        // ConfigureTaskRgbdCamerasForCurrentModel 内部也拿这把锁，故用 recursive_mutex。
        std::unique_lock<std::recursive_mutex> camera_reload_lock(task_camera_mutex);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);
          const std::unique_lock<std::recursive_mutex> simulation_lock(sim.mtx);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          // ********************************
          init_cmd(d);
          InitRobotState(d);
          // ********************************

          mj_forward(m, d);

        }
        else
        {
          sim.LoadMessageClear();
        }
        queueMutex.lock();
        while (!controlCommands.empty())
        {
          controlCommands.pop();
        }
        cmd_updated = false;
        is_chassic_cmd_changed = false;
        is_chassic_cmd_vel_changed = false;
        joint_tau_cmd = std::vector<double>(numJoints, 0);

        if (depth_thread.joinable() && mnew && dnew)
        {
          ConfigureDepthCameraForCurrentModel();
        }
        else
        {
          ResetDepthBufferState();
        }
        if (task_camera_thread.joinable() && mnew && dnew)
        {
          ConfigureTaskRgbdCamerasForCurrentModel();
        }

        claw_cmd_updated = false;
        claw_cmd = std::vector<double>(numClawJoints, 0);
        queueMutex.unlock();
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        // std::this_thread::yield();
      }
      else
      {
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      loop_rate.sleep();

      { // todo 控制变量的生命周期
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (!sim.run)
          {
            mj_forward(m, d);
            sim.speed_changed = true;
            ROS_WARN_STREAM_THROTTLE(1.0, "Sim is not running, forward and publish data");
            publish_ros_data(d, sim.run);
            continue;
          }

          // ************ test ****************
          // mujocolcm.SetSend(d);
          // mujocolcm.Send();
          // mujocolcm.GetRecv(recvCmd);

          // ****************************
          // external wrench
          if (external_wrench_updated_)
          {
            std::cout << "Applying external wrench!\n";
            d->xfrc_applied[6 + 0] = external_wrench_.force.x;
            d->xfrc_applied[6 + 1] = external_wrench_.force.y;
            d->xfrc_applied[6 + 2] = external_wrench_.force.z;
            d->xfrc_applied[6 + 3] = external_wrench_.torque.x;
            d->xfrc_applied[6 + 4] = external_wrench_.torque.y;
            d->xfrc_applied[6 + 5] = external_wrench_.torque.z;
            external_wrench_updated_ = false;
          }
          // ****************************
          // control
          bool updated = false;
          bool claw_updated = false;
          queueMutex.lock();
          const bool dexhand_cmd_updated =
              g_dexhand_node && g_dexhand_node->consumeCommandUpdate();
          if (cmd_updated || is_chassic_cmd_changed || is_chassic_cmd_vel_changed ||
              claw_cmd_updated || dexhand_cmd_updated || pure_sim)
          {
            updated = true;
          }
          cmd_updated = false;
          is_chassic_cmd_changed = false;
          is_chassic_cmd_vel_changed = false;
          claw_updated = claw_cmd_updated;
          claw_cmd_updated = false;
          tau_cmd = joint_tau_cmd;
          queueMutex.unlock();

          if (claw_updated)
          {
            for (size_t i = 0; i < numClawJoints; i++)
            {
              d->ctrl[i + numJoints] = claw_cmd[i];
              // std::cout << "claw_cmd: " << claw_cmd[i] << std::endl;
            }
          }

          if (updated)
          {
            // update actuators/controls
            auto updateControl = [&](const JointGroupAddress &jointAddr, int &i)
            {
              for (auto iter = jointAddr.ctrladr().begin(); iter != jointAddr.ctrladr().end(); iter++)
              {
                d->ctrl[*iter] = tau_cmd[i++];
              }
            };
            int i = 0;
            // 在半身模式下跳过腿部关节的控制
            if (!leg_joints_constrained)
            {
              if (robot_type == 2)
              {
                updateControl(LLegJointsAddr, i);
                updateControl(RLegJointsAddr, i);
                updateControl(WaistJointsAddr, i);
              }
              else if (robot_type == 1)
              {
                if(robotVersion_ == 60)
                {
                  updateWheelVel_VectorContorl(cmd_vel_chassis);
                  updateControl(LegJointsAddr, i);
                }
                else if(robotVersion_ == 61 || robotVersion_ == 62 || robotVersion_ == 63 || robotVersion_ == 200062 || robotVersion_ == 300062 || robotVersion_ == 400062)
                {
                  updateWheelVel_VectorContorl_omniWheel(cmd_vel_chassis);
                  updateControl(LegJointsAddr, i);
                }
              }
              else
              {
                std::cout << "[mujoco_node] Unknown robot_type param, please set to 1 (轮臂) or 2 (双足)!" << std::endl;
                return;
              }
            }
            else
            {
              // 跳过腿部关节的控制输入，但需要更新索引
              // 在半身模式下，对于人形机器人版本，还需跳过腰部关节
              i += LLegJointsAddr.ctrladr().size() + RLegJointsAddr.ctrladr().size();
              if (robot_type == 2)
              {
                // 跳过腰部关节
                i += WaistJointsAddr.ctrladr().size();
              }
            }

            updateControl(LArmJointsAddr, i);
            updateControl(RArmJointsAddr, i);
            updateControl(HeadJointsAddr, i);

            // Dexhand: ctrl/command
            if (g_dexhand_node)
            {
              g_dexhand_node->writeCallback(d);
            }

            // 如果躯干被约束，在每步后强制固定躯干位置和姿态
            if (torso_constrained)
            {
              d->qpos[0] = fixed_torso_pos[0];  // x position
              d->qpos[1] = fixed_torso_pos[1];  // y position
              d->qpos[2] = fixed_torso_pos[2];  // z position
              d->qpos[3] = fixed_torso_quat[0]; // w quaternion
              d->qpos[4] = fixed_torso_quat[1]; // x quaternion
              d->qpos[5] = fixed_torso_quat[2]; // y quaternion
              d->qpos[6] = fixed_torso_quat[3]; // z quaternion

              // 同时固定躯干的速度为0
              d->qvel[0] = 0; // x velocity
              d->qvel[1] = 0; // y velocity
              d->qvel[2] = 0; // z velocity
              d->qvel[3] = 0; // x angular velocity
              d->qvel[4] = 0; // y angular velocity
              d->qvel[5] = 0; // z angular velocity
            }

            // 如果腿部关节被约束，在每步后强制固定腿部关节位置并停止控制
            if (leg_joints_constrained)
            {
              // 固定左腿关节位置
              if (!fixed_leg_l_qpos.empty() && !LLegJointsAddr.qposadr().invalid())
              {
                size_t idx = 0;
                for (auto iter = LLegJointsAddr.qposadr().begin();
                     iter != LLegJointsAddr.qposadr().end() && idx < fixed_leg_l_qpos.size();
                     iter++, idx++)
                {
                  d->qpos[*iter] = fixed_leg_l_qpos[idx];
                }

                // 固定左腿关节速度为0
                for (auto iter = LLegJointsAddr.qdofadr().begin(); iter != LLegJointsAddr.qdofadr().end(); iter++)
                {
                  d->qvel[*iter] = 0;
                }

                // 停止左腿关节控制输入
                for (auto iter = LLegJointsAddr.ctrladr().begin(); iter != LLegJointsAddr.ctrladr().end(); iter++)
                {
                  d->ctrl[*iter] = 0;
                }
              }

              // 固定右腿关节位置
              if (!fixed_leg_r_qpos.empty() && !RLegJointsAddr.qposadr().invalid())
              {
                size_t idx = 0;
                for (auto iter = RLegJointsAddr.qposadr().begin();
                     iter != RLegJointsAddr.qposadr().end() && idx < fixed_leg_r_qpos.size();
                     iter++, idx++)
                {
                  d->qpos[*iter] = fixed_leg_r_qpos[idx];
                }

                // 固定右腿关节速度为0
                for (auto iter = RLegJointsAddr.qdofadr().begin(); iter != RLegJointsAddr.qdofadr().end(); iter++)
                {
                  d->qvel[*iter] = 0;
                }

                // 停止右腿关节控制输入
                for (auto iter = RLegJointsAddr.ctrladr().begin(); iter != RLegJointsAddr.ctrladr().end(); iter++)
                {
                  d->ctrl[*iter] = 0;
                }
              }
            }
            // 躯干吊绳：三模块独立叠加（绞盘回锚 + 姿态回正 + 六轴阻尼）
            if (torso_rope_active_ && torso_body_id_ >= 0)
            {
              int bid = torso_body_id_;
              double *pos = d->xpos + 3 * bid;
              double *quat = d->xquat + 4 * bid;

              // 激活首帧记锚点 XY
              if (!torso_rope_got_anchor_)
              {
                torso_rope_anchor_[0] = pos[0];
                torso_rope_anchor_[1] = pos[1];
                torso_rope_got_anchor_ = true;
              }

              double dx = torso_rope_anchor_[0] - pos[0];
              double dy = torso_rope_anchor_[1] - pos[1];
              double dz = torso_rope_anchor_z_ - pos[2];
              double dist_sq = dx*dx + dy*dy + dz*dz;
              double rope_len_sq = torso_rope_length_ * torso_rope_length_;

              double *v_lin = d->cvel + 6 * bid + 3;
              double *omega = d->cvel + 6 * bid;
              Eigen::Map<Eigen::Vector3d> v_lin_eig(v_lin);
              Eigen::Map<Eigen::Vector3d> omega_eig(omega);

              // 模块 1：绞盘——绳长外按恒定速度拉回锚点
              Eigen::Vector3d F_winch(0.0, 0.0, 0.0);
              if (dist_sq > rope_len_sq)
              {
                double dist = sqrt(dist_sq);
                double v_radial = (v_lin[0]*dx + v_lin[1]*dy + v_lin[2]*dz) / dist;
                double F = std::clamp(torso_rope_kv_ * (torso_rope_speed_ - v_radial),
                                      0.0, torso_rope_max_force_);
                double scale = F / dist;
                F_winch << scale * dx, scale * dy, scale * dz;
              }

              // 模块 2：姿态回正——目标为世界坐标系直立，三轴速度-位置控制，死区 ±2.5°
              double err_vec[3];
              mju_quat2Vel(err_vec, quat, 1.0);
              Eigen::Vector3d err(err_vec);
              Eigen::Vector3d mask = (err.array().abs() > torso_upright_deadzone_).cast<double>();
              Eigen::Vector3d omega_des = -torso_upright_speed_ * err.cwiseSign();
              Eigen::Vector3d tau_upright = torso_upright_kp_ * mask.cwiseProduct(omega_des - omega_eig);

              // 模块 3：六轴阻尼——让躯干趋于静止
              Eigen::Vector3d F_damp = -torso_rope_lin_damp_ * v_lin_eig;
              Eigen::Vector3d tau_damp = -torso_rope_ang_damp_ * omega_eig;

              // 合成外力/力矩
              double *xfrc = &d->xfrc_applied[6 * bid];
              Eigen::Map<Eigen::Vector3d> force_applied(xfrc);
              Eigen::Map<Eigen::Vector3d> torque_applied(xfrc + 3);
              force_applied  = F_winch + F_damp;
              torque_applied = tau_upright + tau_damp;
            }
            else
            {
              torso_rope_got_anchor_ = false;
            }

            // 每次step前应用手臂外力（因为xfrc_applied会在mj_step后自动清零）
            if (left_hand_active_ && left_arm_link_id_ != -1) {
              d->xfrc_applied[6 * left_arm_link_id_ + 0] = left_hand_wrench_.force.x;
              d->xfrc_applied[6 * left_arm_link_id_ + 1] = left_hand_wrench_.force.y;
              d->xfrc_applied[6 * left_arm_link_id_ + 2] = left_hand_wrench_.force.z;
              d->xfrc_applied[6 * left_arm_link_id_ + 3] = left_hand_wrench_.torque.x;
              d->xfrc_applied[6 * left_arm_link_id_ + 4] = left_hand_wrench_.torque.y;
              d->xfrc_applied[6 * left_arm_link_id_ + 5] = left_hand_wrench_.torque.z;
            }
            if (right_hand_active_ && right_arm_link_id_ != -1) {
              d->xfrc_applied[6 * right_arm_link_id_ + 0] = right_hand_wrench_.force.x;
              d->xfrc_applied[6 * right_arm_link_id_ + 1] = right_hand_wrench_.force.y;
              d->xfrc_applied[6 * right_arm_link_id_ + 2] = right_hand_wrench_.force.z;
              d->xfrc_applied[6 * right_arm_link_id_ + 3] = right_hand_wrench_.torque.x;
              d->xfrc_applied[6 * right_arm_link_id_ + 4] = right_hand_wrench_.torque.y;
              d->xfrc_applied[6 * right_arm_link_id_ + 5] = right_hand_wrench_.torque.z;
            }
            applyTask2ConveyorForce();
            mj_step(m, d);
            if (updateContactFollowers()) {
              mj_forward(m, d);
            }
            step_count++;
            sim_time += ros::Duration(1 / frequency);
            sim.AddToHistory();

            // // record cpu time at start of iteration
            // const auto startCPU = mj::Simulate::Clock::now();

            // // elapsed CPU and simulation time since last sync
            // const auto elapsedCPU = startCPU - syncCPU;
            // double elapsedSim = d->time - syncSim;

            // // inject noise
            // // if (sim.ctrl_noise_std) {
            // //   // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            // //   mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
            // //   mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

            // //   for (int i=0; i<m->nu; i++) {
            // //     // update noise
            // //     ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

            // //     // apply noise
            // //     d->ctrl[i] = ctrlnoise[i];
            // //   }
            // // }

            // // requested slow-down factor
            // double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // // misalignment condition: distance from target sim time is bigger than syncmisalign
            // bool misaligned =
            //     mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // // out-of-sync (for any reason): reset sync times, step
            // if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
            //     misaligned || sim.speed_changed)
            // {
            //   // re-sync
            //   syncCPU = startCPU;
            //   syncSim = d->time;
            //   sim.speed_changed = false;

            //   // run single step, let next iteration deal with timing
            //   mj_step(m, d);
            //   std::cout << "step" << std::endl;
            //   bool stepped = true;
            // }

            // // in-sync: step until ahead of cpu
            // else
            // {
            //   bool measured = false;
            //   mjtNum prevSim = d->time;

            //   double refreshTime = simRefreshFraction / sim.refresh_rate;

            //   // step while sim lags behind cpu and within refreshTime
            //   while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
            //          mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
            //   {
            //     // measure slowdown before first step
            //     if (!measured && elapsedSim)
            //     {
            //       sim.measured_slowdown =
            //           std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
            //       measured = true;
            //     }

            //     // call mj_step
            //     mj_step(m, d);
            //     stepped = true;

            //     // break if reset
            //     if (d->time < prevSim)
            //     {
            //       break;
            //     }
            //   }
            // }

            // save current state to history buffer
            // if (stepped)
            // {
            //   sim.AddToHistory();
            // }
          }
          publish_ros_data(d, sim.run);

          // ros::Time current_time = ros::Time::now();
          // if ((current_time - last_depth_history_pub_time).toSec() >= depth_history_interval)
          // {
          //   publish_depth_history();
          //   last_depth_history_pub_time = current_time;
          // }
        }
      } // release std::lock_guard<std::mutex>
    }
    std::cout << "Physics thread exited." << std::endl;
    // Reap the LCM worker before this scope ends and ~MujocoLcm() destroys
    // lcm_.  Without this the worker keeps calling lcm_.handle() on a freed
    // LCM instance and corrupts the heap (malloc(): mismatching next->prev_size).
    mujocolcm.joinLCMThread();
  }
} // namespace

namespace
{

void initializeBimanualContactFollowers(
    const std::vector<std::string> &body_names)
{
  bimanual_contact_followers.clear();
  task2_score_publishers.clear();
  task2_score_last_time = -1;
  task2_table_geom_id = mj_name2id(m, mjOBJ_GEOM, "table_top");
  bimanual_contact_state_publishers.clear();
  bimanual_left_hand_geom_ids.clear();
  bimanual_right_hand_geom_ids.clear();
  for (const auto &side : {std::string("l"), std::string("r")}) {
    const int hand = mj_name2id(m, mjOBJ_BODY, (side + "_hand_base").c_str());
    if (hand < 0) continue;
    auto &geoms = side == "l" ? bimanual_left_hand_geom_ids
                              : bimanual_right_hand_geom_ids;
    for (int geom = 0; geom < m->ngeom; ++geom) {
      if (!(m->geom_contype[geom] || m->geom_conaffinity[geom])) continue;
      int body = m->geom_bodyid[geom];
      while (body > 0 && body != hand) body = m->body_parentid[body];
      if (body == hand) geoms.insert(geom);
    }
  }
  task2_conveyor_body_id = mj_name2id(m, mjOBJ_BODY, "destination_conveyor");
  task2_conveyor_belt_geom_id = mj_name2id(m, mjOBJ_GEOM, "destination_conveyor_belt");
  task2_conveyor_started = false;
  task2_conveyor_completed = false;
  for (const std::string &name : body_names) {
    const int body_id = mj_name2id(m, mjOBJ_BODY, name.c_str());
    if (body_id < 0 || m->body_jntnum[body_id] != 1) continue;
    const int joint_id = m->body_jntadr[body_id];
    if (m->jnt_type[joint_id] != mjJNT_FREE) continue;
    BimanualContactFollowerState state;
    state.name = name;
    state.body_id = body_id;
    state.qpos_addr = m->jnt_qposadr[joint_id];
    state.dof_addr = m->jnt_dofadr[joint_id];
    for (int id = m->body_geomadr[body_id];
         id < m->body_geomadr[body_id] + m->body_geomnum[body_id]; ++id) {
      state.collision_geom_ids.push_back(id);
    }
    bimanual_contact_followers.push_back(state);
    auto publisher = g_nh_ptr->advertise<std_msgs::Bool>(
        "/mujoco/" + name + "/bimanual_grasped", 1, true);
    std_msgs::Bool initial;
    initial.data = false;
    publisher.publish(initial);
    bimanual_contact_state_publishers.emplace(name, publisher);
    auto score_publisher = g_nh_ptr->advertise<std_msgs::UInt8>(
        "/mujoco/" + name + "/scoring_state", 1, true);
    std_msgs::UInt8 empty_score;
    empty_score.data = 0;
    score_publisher.publish(empty_score);
    task2_score_publishers.emplace(name, score_publisher);
  }
  ROS_INFO("[Task2Contact] observing %zu boxes; grasping uses physics only",
           bimanual_contact_followers.size());
}

void publishBimanualContactState(
    const BimanualContactFollowerState &state)
{
  const auto publisher = bimanual_contact_state_publishers.find(state.name);
  if (publisher == bimanual_contact_state_publishers.end()) {
    return;
  }
  std_msgs::Bool message;
  message.data = state.held;
  publisher->second.publish(message);
}

void task2ConveyorEnabledCallback(const std_msgs::Bool::ConstPtr &message)
{
  task2_conveyor_enabled.store(message->data, std::memory_order_release);
}

void initializeTask1LeverFingerLatch()
{
  task1_lever_latch = Task1LeverFingerLatchState{};
  task1_lever_latch.handle_geom_id = mj_name2id(m, mjOBJ_GEOM, "lever_handle_collision");
  task1_lever_latch.lever_lock_equality_id = mj_name2id(m, mjOBJ_EQUALITY, "task1_lever_lock");
  task1_lever_latch.palm_geom_id = mj_name2id(m, mjOBJ_GEOM, "task2_r_hand_base_collision");
  task1_lever_latch.wrist_body_id = mj_name2id(m, mjOBJ_BODY, "zarm_r7_link");
  const std::array<const char *, 3> names{{
      "r_index_link1", "r_middle_link1", "r_little_link1"}};
  for (std::size_t i = 0; i < names.size(); ++i)
    task1_lever_latch.finger_body_ids[i] = mj_name2id(m, mjOBJ_BODY, names[i]);
  if (task1_lever_latch.handle_geom_id >= 0) {
    task1_lever_latch.grip_publisher = g_nh_ptr->advertise<std_msgs::Bool>(
        "/mujoco/lever/grasp_confirmed", 1, true);
    task1_lever_latch.grip_publisher.publish(std_msgs::Bool{});
    task1_lever_latch.palm_publisher = g_nh_ptr->advertise<std_msgs::Bool>(
        "/mujoco/lever/palm_seated", 1, true);
    task1_lever_latch.palm_publisher.publish(std_msgs::Bool{});
    task1_lever_latch.grip_contacts_publisher = g_nh_ptr->advertise<std_msgs::UInt8>(
        "/mujoco/lever/grasp_contacts", 1, true);
    task1_lever_latch.grip_contacts_publisher.publish(std_msgs::UInt8{});
  }
}

bool updateTask1LeverFingerLatch()
{
  if (task1_lever_latch.handle_geom_id < 0 ||
      task1_lever_latch.lever_lock_equality_id < 0) return false;
  mjtNum normals[4][3] = {};
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact &contact = d->contact[i];
    const int other = contact.geom1 == task1_lever_latch.handle_geom_id
        ? contact.geom2 : (contact.geom2 == task1_lever_latch.handle_geom_id
        ? contact.geom1 : -1);
    if (other < 0 || contact.dist > -numericScalarOrDefault(
        "task1_lever_unlock_contact_depth", 0)) continue;
    mjtNum force[6];
    mj_contactForce(m, d, i, force);
    if (force[0] <= 0) continue;
    int finger = other == task1_lever_latch.palm_geom_id ? 0 : -1;
    for (int body = m->geom_bodyid[other]; body > 0; body = m->body_parentid[body]) {
      for (int j = 0; j < 3; ++j)
        if (body == task1_lever_latch.finger_body_ids[j]) finger = j + 1;
    }
    if (finger < 0) continue;
    const mjtNum direction = contact.geom2 == task1_lever_latch.handle_geom_id ? 1 : -1;
    if (finger == 0) {
      const mjtNum *wrist_rotation = d->xmat + 9 * task1_lever_latch.wrist_body_id;
      const mjtNum front_dot = direction * (contact.frame[0] * wrist_rotation[1] +
          contact.frame[1] * wrist_rotation[4] + contact.frame[2] * wrist_rotation[7]);
      if (front_dot < 0.5) continue;  // Palm face, never the back of the hand.
    }
    for (int axis = 0; axis < 3; ++axis)
      normals[finger][axis] += direction * force[0] * contact.frame[axis];
  }
  const mjtNum palm_force = mju_norm3(normals[0]);
  task1_lever_latch.palm_contact_duration = palm_force > 0
      ? task1_lever_latch.palm_contact_duration + m->opt.timestep : 0;
  if (!task1_lever_latch.palm_seated && task1_lever_latch.palm_contact_duration >= 0.05) {
    task1_lever_latch.palm_seated = true;
    std_msgs::Bool message;
    message.data = true;
    task1_lever_latch.palm_publisher.publish(message);
  }
  int opposing_fingers = 0;
  for (int finger = 1; finger < 4; ++finger) {
    const mjtNum finger_force = mju_norm3(normals[finger]);
    if (palm_force > 0 && finger_force > 0 &&
        mju_dot3(normals[0], normals[finger]) < -0.5 * palm_force * finger_force)
      ++opposing_fingers;
  }
  const bool opposing_grip = task1_lever_latch.palm_seated && opposing_fingers >= 2;
  std::uint8_t contacts = opposing_grip ? 16 : 0;
  for (int finger = 0; finger < 4; ++finger)
    if (mju_norm3(normals[finger]) > 0) contacts |= 1u << finger;
  if (contacts != task1_lever_latch.grip_contacts) {
    task1_lever_latch.grip_contacts = contacts;
    std_msgs::UInt8 message;
    message.data = contacts;
    task1_lever_latch.grip_contacts_publisher.publish(message);
  }
  if (task1_lever_latch.lever_unlocked) return false;
  if (opposing_grip) {
    task1_lever_latch.grip_duration += m->opt.timestep;
    task1_lever_latch.grip_gap_duration = 0;
  } else {
    // Solver contact forces can briefly unload between opposing contacts.
    // Missing contact never counts toward the required grip duration.
    task1_lever_latch.grip_gap_duration += m->opt.timestep;
    if (task1_lever_latch.grip_gap_duration > 0.01)
      task1_lever_latch.grip_duration = 0;
    return false;
  }
  if (task1_lever_latch.grip_duration < numericScalarOrDefault(
      "task1_lever_unlock_contact_duration", 0.05)) return false;
  d->eq_active[task1_lever_latch.lever_lock_equality_id] = 0;
  task1_lever_latch.lever_unlocked = true;
  std_msgs::Bool message;
  message.data = true;
  task1_lever_latch.grip_publisher.publish(message);
  ROS_INFO("[Task1LeverLatch] opposing physical grip confirmed; hinge unlocked");
  return true;
}

mjtNum numericScalarOrDefault(const char *name, mjtNum fallback)
{
  const int numeric_id = mj_name2id(m, mjOBJ_NUMERIC, name);
  if (numeric_id < 0 || m->numeric_size[numeric_id] < 1) {
    return fallback;
  }
  return m->numeric_data[m->numeric_adr[numeric_id]];
}

void initializeInternalContactFollowers(
    const std::vector<std::string> &body_names)
{
  internal_contact_followers.clear();
  internal_contact_state_publishers.clear();
  internal_finger_latch_publishers.clear();
  physical_support_publishers.clear();
  const std::array<const char *, 3> names{{
      "r_index_fingertip_collision", "r_middle_fingertip_collision",
      "r_little_fingertip_collision"}};
  for (std::size_t i = 0; i < names.size(); ++i)
    internal_expansion_fingertip_geom_ids[i] = mj_name2id(m, mjOBJ_GEOM, names[i]);
  if (body_names.empty()) return;
  for (const auto &name : body_names) {
    const int id = mj_name2id(m, mjOBJ_BODY, name.c_str());
    if (id < 0) continue;
    InternalContactFollowerState state;
    state.name = name;
    state.body_id = id;
    state.support_geom = mj_name2id(m, mjOBJ_GEOM,
        name.find("cylinder_") == 0 ? "target_bin_floor" : "task3_destination_table_top");
    state.finger_body_ids = {{
        mj_name2id(m, mjOBJ_BODY, "r_index_link1"),
        mj_name2id(m, mjOBJ_BODY, "r_middle_link1"),
        mj_name2id(m, mjOBJ_BODY, "r_little_link1")}};
    if (name.find("cylinder_") == 0) {
      state.finger_body_ids = {{
          mj_name2id(m, mjOBJ_BODY, "r_thumb_link1"),
          mj_name2id(m, mjOBJ_BODY, "r_index_link1"), -1}};
    }
    internal_contact_followers.push_back(state);
    auto held = g_nh_ptr->advertise<std_msgs::Bool>(
        "/mujoco/" + name + "/internal_grasped", 1, true);
    auto mask = g_nh_ptr->advertise<std_msgs::UInt8>(
        "/mujoco/" + name + "/internal_finger_latch_mask", 1, true);
    held.publish(std_msgs::Bool{});
    mask.publish(std_msgs::UInt8{});
    internal_contact_state_publishers.emplace(name, held);
    internal_finger_latch_publishers.emplace(name, mask);
    auto supported = g_nh_ptr->advertise<std_msgs::Bool>(
        "/mujoco/" + name + "/destination_supported", 1, true);
    supported.publish(std_msgs::Bool{});
    physical_support_publishers.emplace(name, supported);
  }
  internal_grasp_armed_publisher = g_nh_ptr->advertise<std_msgs::Bool>(
      "/mujoco/task3_grasp_armed", 1, true);
  internal_grasp_armed_published = false;
  internal_grasp_armed_publisher.publish(std_msgs::Bool{});
  internal_fingertip_collision_suppressed_publisher =
      g_nh_ptr->advertise<std_msgs::Bool>(
          "/mujoco/task3_fingertip_collision_suppressed", 1, true);
  internal_fingertip_collision_suppressed_published = false;
  internal_fingertip_collision_suppressed_publisher.publish(std_msgs::Bool{});
}

void publishInternalContactState(
    const InternalContactFollowerState &state)
{
  const auto publisher = internal_contact_state_publishers.find(state.name);
  if (publisher == internal_contact_state_publishers.end()) {
    return;
  }
  std_msgs::Bool message;
  message.data = state.held;
  publisher->second.publish(message);
}

std::uint8_t internalFingerLatchMask(
    const InternalContactFollowerState &state)
{
  std::uint8_t mask = 0;
  for (std::size_t finger = 0; finger < state.finger_latched.size(); ++finger) {
    if (state.finger_latched[finger]) {
      mask |= static_cast<std::uint8_t>(1u << finger);
    }
  }
  return mask;
}

void publishInternalFingerLatchMask(InternalContactFollowerState &state)
{
  const auto publisher = internal_finger_latch_publishers.find(state.name);
  if (publisher == internal_finger_latch_publishers.end()) {
    return;
  }
  state.last_published_latch_mask = internalFingerLatchMask(state);
  std_msgs::UInt8 message;
  message.data = state.last_published_latch_mask;
  publisher->second.publish(message);
}

void setInternalGraspArmedPublished(bool armed)
{
  if (armed == internal_grasp_armed_published ||
      !internal_grasp_armed_publisher) {
    return;
  }
  internal_grasp_armed_published = armed;
  std_msgs::Bool message;
  message.data = armed;
  internal_grasp_armed_publisher.publish(message);
}

void setInternalFingertipCollisionSuppressedPublished(bool suppressed)
{
  if (suppressed == internal_fingertip_collision_suppressed_published ||
      !internal_fingertip_collision_suppressed_publisher) {
    return;
  }
  internal_fingertip_collision_suppressed_published = suppressed;
  std_msgs::Bool message;
  message.data = suppressed;
  internal_fingertip_collision_suppressed_publisher.publish(message);
}

void resetContactFollower(const std::string &body_name)
{
  for (auto &state : bimanual_contact_followers) {
    if (state.name == body_name) {
      state.held = false;
      publishBimanualContactState(state);
    }
  }
  for (auto &state : internal_contact_followers) {
    if (state.name == body_name) {
      state.held = false;
      state.supported = false;
      physical_support_publishers.at(state.name).publish(std_msgs::Bool{});
      state.finger_latched.fill(false);
      publishInternalContactState(state);
      publishInternalFingerLatchMask(state);
    }
  }
}

Eigen::Matrix3d contactFollowerBodyRotation(int body_id)
{
  Eigen::Matrix3d rotation;
  const mjtNum *raw = d->xmat + 9 * body_id;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = raw[3 * row + col];
    }
  }
  return rotation;
}

void updateBimanualContactFollowers()
{

  for (auto &state : bimanual_contact_followers) {
    bool left = false, right = false;
    for (int i = 0; i < d->ncon; ++i) {
      const mjContact &contact = d->contact[i];
      const int other = m->geom_bodyid[contact.geom1] == state.body_id
          ? contact.geom2 : (m->geom_bodyid[contact.geom2] == state.body_id
          ? contact.geom1 : -1);
      if (other < 0) continue;
      mjtNum force[6];
      mj_contactForce(m, d, i, force);
      if (force[0] <= 0) continue;
      left = left || bimanual_left_hand_geom_ids.count(other);
      right = right || bimanual_right_hand_geom_ids.count(other);
    }
    const bool touching = left && right;
    if (touching != state.held) {
      state.held = touching;
      publishBimanualContactState(state);
    }
  }
}

void applyTask2ConveyorForce()
{
  // Remove only our previous force. Never overwrite object pose, velocity
  // or external forces contributed by another source.
  for (auto &state : bimanual_contact_followers) {
    for (int axis = 0; axis < 3; ++axis) {
      d->qfrc_applied[state.dof_addr + axis] -= state.conveyor_force[axis];
      state.conveyor_force[axis] = 0;
    }
  }
  if (!task2_conveyor_enabled.load(std::memory_order_acquire) ||
      task2_conveyor_body_id < 0 || task2_conveyor_belt_geom_id < 0 ||
      task2_conveyor_completed || bimanual_contact_followers.empty()) return;
  std::string conveyor_box;
  if (!g_nh_ptr->getParam("/task2_conveyor_box", conveyor_box) ||
      conveyor_box.empty()) return;

  const mjtNum speed = numericScalarOrDefault("task2_conveyor_speed", 0.10);
  const mjtNum stop_local_x =
      numericScalarOrDefault("task2_conveyor_stop_local_x", 0.45);
  const mjtNum *conveyor_position = d->xpos + 3 * task2_conveyor_body_id;
  const Eigen::Vector3d direction =
      contactFollowerBodyRotation(task2_conveyor_body_id).col(0);
  for (auto &state : bimanual_contact_followers) {
    if (state.name != conveyor_box || state.held) continue;
    mjtNum friction_limit = 0;
    for (int i = 0; i < d->ncon; ++i) {
      const mjContact &contact = d->contact[i];
      const int other =
          contact.geom1 == task2_conveyor_belt_geom_id ? contact.geom2 :
          contact.geom2 == task2_conveyor_belt_geom_id ? contact.geom1 : -1;
      if (other < 0 || m->geom_bodyid[other] != state.body_id) continue;
      mjtNum force[6];
      mj_contactForce(m, d, i, force);
      friction_limit += std::max<mjtNum>(0, force[0]) * contact.friction[0];
    }
    // Traction exists only while the box is actually supported by the belt.
    if (friction_limit <= 0) return;
    if (!task2_conveyor_started) {
      task2_conveyor_started = true;
      ROS_INFO("[Task2Conveyor] contact traction started for '%s'", state.name.c_str());
    }
    Eigen::Vector3d offset, velocity;
    for (int axis = 0; axis < 3; ++axis) {
      offset[axis] = d->qpos[state.qpos_addr + axis] - conveyor_position[axis];
      velocity[axis] = d->qvel[state.dof_addr + axis];
    }
    const mjtNum remaining = stop_local_x - direction.dot(offset);
    if (remaining <= 0.005) {
      task2_conveyor_completed = true;
      task2_conveyor_enabled.store(false, std::memory_order_release);
      g_nh_ptr->setParam("/task2_conveyor_complete", true);
      ROS_INFO("[Task2Conveyor] '%s' reached stop", state.name.c_str());
      return;
    }
    const mjtNum target_speed = std::min(speed, remaining / 0.2);
    const mjtNum requested =
        m->body_mass[state.body_id] * (target_speed - direction.dot(velocity)) / 0.1;
    // The displayed belt is stationary geometry: the bounded drive also
    // overcomes its sliding resistance. Contacts still resolve all motion,
    // including blocking obstacles; no position/velocity is prescribed.
    const mjtNum drive_limit = 2 * friction_limit;
    const mjtNum traction = std::max(-drive_limit,
        std::min(drive_limit, friction_limit + requested));
    for (int axis = 0; axis < 3; ++axis) {
      state.conveyor_force[axis] = direction[axis] * traction;
      d->qfrc_applied[state.dof_addr + axis] += state.conveyor_force[axis];
    }
    return;
  }
}

bool updateInternalContactFollowers()
{
  if (internal_contact_followers.empty()) return false;
  // Compatibility topics now report current load-bearing contacts. No ROS
  // command can disable collisions, freeze fingers or move a task object.
  setInternalGraspArmedPublished(true);
  setInternalFingertipCollisionSuppressedPublished(false);
  for (auto &state : internal_contact_followers) {
    state.finger_latched.fill(false);
    bool supported = false;
    for (int i = 0; i < d->ncon; ++i) {
      const mjContact &contact = d->contact[i];
      const int other = m->geom_bodyid[contact.geom1] == state.body_id
          ? contact.geom2 : (m->geom_bodyid[contact.geom2] == state.body_id
          ? contact.geom1 : -1);
      if (other < 0) continue;
      mjtNum force[6];
      mj_contactForce(m, d, i, force);
      if (force[0] <= 0) continue;
      const mjtNum upward_normal = m->geom_bodyid[contact.geom2] == state.body_id
          ? contact.frame[2] : -contact.frame[2];
      supported = supported || (other == state.support_geom && upward_normal > 0.5);
      for (std::size_t finger = 0; finger < state.finger_latched.size(); ++finger) {
        for (int body = m->geom_bodyid[other]; body > 0;
             body = m->body_parentid[body]) {
          if (body == state.finger_body_ids[finger]) {
            state.finger_latched[finger] = true;
            break;
          }
        }
      }
    }
    if (supported != state.supported) {
      state.supported = supported;
      std_msgs::Bool message;
      message.data = supported;
      physical_support_publishers.at(state.name).publish(message);
    }
    const auto mask = internalFingerLatchMask(state);
    if (mask != state.last_published_latch_mask)
      publishInternalFingerLatchMask(state);
    const bool held = __builtin_popcount(static_cast<unsigned int>(mask)) >= 2;
    if (held != state.held) {
      state.held = held;
      publishInternalContactState(state);
    }
  }
  return false;
}

bool updateContactFollowers()
{
  const bool task1_lever_changed = updateTask1LeverFingerLatch();
  updateBimanualContactFollowers();
  if (d->time < task2_score_last_time || d->time - task2_score_last_time >= 0.02) {
    for (const auto& state : bimanual_contact_followers) {
      std_msgs::UInt8 message;
      message.data = task2_score::observe(
          m, d, state.body_id, task2_table_geom_id, task2_conveyor_belt_geom_id);
      task2_score_publishers.at(state.name).publish(message);
    }
    for (const auto& state : task13_score_bodies) {
      std_msgs::UInt8 message;
      message.data = task13_score::observe(m, d, state.body, state.source, state.target, state.bin);
      state.publisher.publish(message);
    }
    if (source_bin_slide_id >= 0) {
      int joint = source_bin_slide_id;
      int body = m->jnt_bodyid[joint];
      mjtNum velocity[6];
      mj_objectVelocity(m, d, mjOBJ_BODY, body, velocity, 0);
      mjtNum angular = mju_norm3(velocity), linear = mju_norm3(velocity+3);
      // The scene conveyor stops at 0.45 m; the safety joint limit is 0.60 m.
      bool at_end = d->qpos[m->jnt_qposadr[joint]] >= .44;
      if (!at_end || linear > .01 || angular > .05 || d->time < source_bin_still_since)
        source_bin_still_since = -1;
      else if (source_bin_still_since < 0)
        source_bin_still_since = d->time;
      std_msgs::UInt8 message;
      message.data = source_bin_still_since >= 0 && d->time-source_bin_still_since >= .3 ? 16 : 0;
      source_bin_score_publisher.publish(message);
    }
    task2_score_last_time = d->time;
  }
  const bool internal_changed = updateInternalContactFollowers();
  return task1_lever_changed ||
      internal_changed;
}
} // namespace

bool handleSimStart(std_srvs::SetBool::Request &req,
                    std_srvs::SetBool::Response &res)
{
  if (req.data)
  {
    ROS_INFO("Received sim_start request: true");
  }
  else
  {
    ROS_INFO("Received sim_start request: false");
  }
  res.success = true;
  res.message = "Received sim_start request";
  sim->run = req.data;
  return true;
}

bool setObjectPositionCallback(kuavo_msgs::SetObjectPosition::Request &req,
                               kuavo_msgs::SetObjectPosition::Response &res)
{
  if (!m || !d || !sim) {
    res.success = false;
    res.message = "MuJoCo model is not initialized";
    return true;
  }

  const int body_id = mj_name2id(m, mjOBJ_BODY, req.object_name.c_str());
  if (body_id < 0 || m->body_jntnum[body_id] != 1) {
    res.success = false;
    res.message = "Object '" + req.object_name + "' must have exactly one joint";
    return true;
  }
  const int joint_id = m->body_jntadr[body_id];
  if (m->jnt_type[joint_id] != mjJNT_FREE) {
    res.success = false;
    res.message = "Object '" + req.object_name + "' does not have a free joint";
    return true;
  }
  if (req.randomize &&
      (req.x_min > req.x_max || req.y_min > req.y_max || req.z_min > req.z_max)) {
    res.success = false;
    res.message = "Invalid randomization bounds";
    return true;
  }

  const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
  const int qpos_addr = m->jnt_qposadr[joint_id];
  const int dof_addr = m->jnt_dofadr[joint_id];
  double x = req.position.x;
  double y = req.position.y;
  double z = req.position.z;
  if (req.randomize) {
    x = std::uniform_real_distribution<double>(req.x_min, req.x_max)(object_random_generator);
    y = std::uniform_real_distribution<double>(req.y_min, req.y_max)(object_random_generator);
    z = std::uniform_real_distribution<double>(req.z_min, req.z_max)(object_random_generator);
  }

  const double quat_norm = std::sqrt(
      req.orientation.w * req.orientation.w + req.orientation.x * req.orientation.x +
      req.orientation.y * req.orientation.y + req.orientation.z * req.orientation.z);
  int base_weld_id = -1;
  if (req.object_name == "base_link") {
    const double qw = quat_norm > 1e-12
        ? req.orientation.w / quat_norm : d->qpos[qpos_addr + 3];
    const double qx = quat_norm > 1e-12
        ? req.orientation.x / quat_norm : d->qpos[qpos_addr + 4];
    const double qy = quat_norm > 1e-12
        ? req.orientation.y / quat_norm : d->qpos[qpos_addr + 5];
    const double qz = quat_norm > 1e-12
        ? req.orientation.z / quat_norm : d->qpos[qpos_addr + 6];
    if (std::abs(std::abs(qw) - 1.0) > 1e-9 ||
        std::abs(qx) > 1e-9 || std::abs(qy) > 1e-9 ||
        std::abs(qz) > 1e-9) {
      res.success = false;
      res.message = "Task base randomization supports translation only";
      return true;
    }

    base_weld_id = mj_name2id(m, mjOBJ_EQUALITY, "scene_base_lock");
    if (base_weld_id < 0 || m->eq_type[base_weld_id] != mjEQ_WELD) {
      res.success = false;
      res.message = "scene_base_lock weld is unavailable";
      return true;
    }
  }

  d->qpos[qpos_addr] = x;
  d->qpos[qpos_addr + 1] = y;
  d->qpos[qpos_addr + 2] = z;
  if (quat_norm > 1e-12) {
    d->qpos[qpos_addr + 3] = req.orientation.w / quat_norm;
    d->qpos[qpos_addr + 4] = req.orientation.x / quat_norm;
    d->qpos[qpos_addr + 5] = req.orientation.y / quat_norm;
    d->qpos[qpos_addr + 6] = req.orientation.z / quat_norm;
  }

  if (req.object_name == "base_link") {
    mjtNum *weld_data = m->eq_data + mjNEQDATA * base_weld_id;
    weld_data[3] = -x;
    weld_data[4] = -y;
    weld_data[5] = -z;
    weld_data[6] = 1.0;
    weld_data[7] = 0.0;
    weld_data[8] = 0.0;
    weld_data[9] = 0.0;

    if (qpos_init.size() >= static_cast<size_t>(qpos_addr + 7)) {
      std::copy(d->qpos + qpos_addr, d->qpos + qpos_addr + 7,
                qpos_init.begin() + qpos_addr);
    }
  }
  mju_zero(d->qvel + dof_addr, 6);
  resetContactFollower(req.object_name);
  mj_forward(m, d);

  res.success = true;
  res.message = "Object '" + req.object_name + "' pose updated";
  res.final_position.x = x;
  res.final_position.y = y;
  res.final_position.z = z;
  return true;
}

bool setJointPositionCallback(kuavo_msgs::SetJointPosition::Request &req,
                              kuavo_msgs::SetJointPosition::Response &res)
{
  if (!m || !d || !sim) {
    res.success = false;
    res.message = "MuJoCo model is not initialized";
    return true;
  }
  const int joint_id = mj_name2id(m, mjOBJ_JOINT, req.joint_name.c_str());
  if (joint_id < 0 || (m->jnt_type[joint_id] != mjJNT_HINGE &&
                       m->jnt_type[joint_id] != mjJNT_SLIDE)) {
    res.success = false;
    res.message = "Joint '" + req.joint_name + "' must be a hinge or slide joint";
    return true;
  }

  double position = req.position;
  if (m->jnt_limited[joint_id]) {
    position = std::clamp(position, m->jnt_range[2 * joint_id],
                          m->jnt_range[2 * joint_id + 1]);
  }
  const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
  const int source_bin_lock_id =
      mj_name2id(m, mjOBJ_EQUALITY, "task1_source_bin_lock");
  if (source_bin_lock_id >= 0 &&
      m->eq_type[source_bin_lock_id] == mjEQ_JOINT &&
      m->eq_obj1id[source_bin_lock_id] == joint_id) {
    // A one-joint equality constrains qpos to polycoef[0].  Keep the scene
    // lock active while moving its target so only this explicit service can
    // advance the source bin along the rail.
    m->eq_data[mjNEQDATA * source_bin_lock_id] = position;
    d->eq_active[source_bin_lock_id] = 1;
  }
  d->qpos[m->jnt_qposadr[joint_id]] = position;
  d->qvel[m->jnt_dofadr[joint_id]] = 0.0;
  mj_forward(m, d);
  res.success = true;
  res.message = "Joint '" + req.joint_name + "' position updated";
  return true;
}
#ifdef USE_DDS
void ddsLowCmdCallback(const unitree_hg::msg::dds_::LowCmd_& cmd)
{
  // Convert DDS LowCmd to MuJoCo joint commands
  std::vector<double> tau(numJoints, 0.0);
  
  // Map motor commands to joint torques (first 28 motors)
  size_t joint_count = std::min((size_t)numJoints, KUAVO_JOINT_COUNT);
  for (size_t i = 0; i < joint_count && i < cmd.motor_cmd().size(); ++i) {
    const auto& motor_cmd = cmd.motor_cmd()[i];
    tau[i] = static_cast<double>(motor_cmd.tau());
  }
  
  std::lock_guard<std::mutex> lock(queueMutex);
  joint_tau_cmd = tau;
  cmd_updated = true;
  
}
#elif defined(USE_LEJU_DDS)
void lejuDdsLowCmdCallback(const leju::msgs::JointCmd& cmd)
{
  // Convert LEJU DDS JointCmd to MuJoCo joint commands
  std::vector<double> tau(numJoints, 0.0);

  // Map joint commands to joint torques
  size_t joint_count = std::min((size_t)numJoints, cmd.tau().size());
  for (size_t i = 0; i < joint_count; ++i) {
    tau[i] = static_cast<double>(cmd.tau()[i]);
  }

  std::lock_guard<std::mutex> lock(queueMutex);
  joint_tau_cmd = tau;
  cmd_updated = true;
}
#endif

void jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr &msg)
{
   auto is_match_size = [&](size_t size)
  {
      if (msg->joint_q.size() != size || msg->joint_v.size() != size ||
          msg->tau.size() != size || msg->tau_ratio.size() != size ||
          msg->control_modes.size() != size || msg->tau_max.size() != size ||
          msg->joint_kd.size() != size || msg->joint_kp.size() != size)
      {
          return false;
      }
      return true;
  };

  if (!is_match_size(numJoints))
  {
      ROS_WARN_STREAM_THROTTLE(1.0, "jointCmdCallback Error: joint_q, joint_v, tau, tau_ratio, control_modes, joint_kp, joint_kd size not match!");
      ROS_WARN_STREAM_THROTTLE(1.0, "desired size: " << numJoints);
      ROS_WARN_STREAM_THROTTLE(1.0, "joint_q size: " << msg->joint_q.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "joint_v size: " << msg->joint_v.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "tau size: " << msg->tau.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "tau_ratio size: " << msg->tau_ratio.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "control_modes size: " << msg->control_modes.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "tau_max size: " << msg->tau_max.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "joint_kp size: " << msg->joint_kp.size());
      ROS_WARN_STREAM_THROTTLE(1.0, "joint_kd size: " << msg->joint_kd.size());
      return;
  }
  
  // std::cout << "Received jointCmd: " << msg->tau[0] << std::endl;
  std::vector<double> tau(numJoints);
  for (size_t i = 0; i < numJoints; i++)
  {
    tau[i] = msg->tau[i];
  }
  std::lock_guard<std::mutex> lock(queueMutex);
  // controlCommands.push(tau);
  joint_tau_cmd = tau;
  cmd_updated = true;
}

void clawCmdCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg)
{
  //std::cout << "Received lejuClawCommand: " << msg->data.position[0] << std::endl;
  
  // Check if the message has the expected size
  if (msg->data.position.size() < numClawJoints) {
    std::cerr << "Error: lejuClawCommand position size (" << msg->data.position.size() 
              << ") is less than expected numClawJoints (" << numClawJoints << ")" << std::endl;
    return;
  }
  
  std::vector<double> tem(numClawJoints);
  for (size_t i = 0; i < numClawJoints; i++)
  {
    // Convert position from percentage (0-100) to appropriate range for MuJoCo
    // Assuming 0 = fully closed, 100 = fully open
    // Map to range [0, 1] for MuJoCo control
    // Convert position from percentage (0-100) to appropriate range for MuJoCo
    // Assuming 0 = fully closed, 100 = fully open
    // Map to range [-100, 0] for MuJoCo control
    double raw_value = msg->data.position[i] - 100.0;
    // Clamp to valid range to prevent issues
    tem[i] = std::max(-100.0, std::min(0.0, raw_value));
    
    // Debug output for first few iterations
    static int debug_count = 0;
    if (debug_count < 10) {
      std::cout << "Claw cmd[" << i << "]: input=" << msg->data.position[i] 
                << ", raw=" << raw_value << ", clamped=" << tem[i] << std::endl;
    }
    debug_count++;
  }

  std::lock_guard<std::mutex> lock(queueMutex);
  claw_cmd = tem;
  claw_cmd_updated = true;
}

void extWrenchCallback(const geometry_msgs::Wrench::ConstPtr &msg)
{
  // std::cout << "Received jointCmd: " << msg->tau[0] << std::endl;
  std::cout << "in ext wrench callback!\n";
  external_wrench_ = *msg;
  external_wrench_updated_ = true;
}
void apply_wrench_to_link(mjModel* m, mjData* d, const char* link_name, const mjtNum* force, const mjtNum* torque) {
  // 获取 link 的索引
  int link_index = mj_name2id(m, mjOBJ_BODY, link_name);
  
  // 检查链接索引是否有效
  if (link_index == -1) {
      printf("Error: Link named '%s' not found.\n", link_name);
      return;
  }

  // 根据 link_index 设置 wrench
  d->xfrc_applied[6 * link_index + 0] = force[0]; // 力 x
  d->xfrc_applied[6 * link_index + 1] = force[1]; // 力 y
  d->xfrc_applied[6 * link_index + 2] = force[2]; // 力 z
  d->xfrc_applied[6 * link_index + 3] = torque[0]; // 力矩 x
  d->xfrc_applied[6 * link_index + 4] = torque[1]; // 力矩 y
  d->xfrc_applied[6 * link_index + 5] = torque[2]; // 力矩 z
}

void chassicPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  // 获取chassic_link的qpos地址（前7个元素：3个位置 + 4个四元数）
  // 注意：freejoint的qpos格式是 [x, y, z, qw, qx, qy, qz]
  
  std::lock_guard<std::mutex> lock(queueMutex);
  
  // 设置位置 (x, y, z)
  d->qpos[0] = msg->position.x;
  d->qpos[1] = msg->position.y; 
  d->qpos[2] = msg->position.z;
  
  // 设置姿态四元数 (qw, qx, qy, qz)
  d->qpos[3] = msg->orientation.w;
  d->qpos[4] = msg->orientation.x;
  d->qpos[5] = msg->orientation.y;
  d->qpos[6] = msg->orientation.z;
  
  // 调用前向动力学更新物理状态
  // mj_step(m, d);
  is_chassic_cmd_changed = true;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(queueMutex);

  cmd_vel_chassis[0] = msg->linear.x; // 线速度 x
  cmd_vel_chassis[1] = msg->linear.y; // 线速度 y
  cmd_vel_chassis[2] = msg->angular.z; // 角速度 z
  
  // 调用前向动力学更新物理状态
  // mj_step(m, d);
  // is_chassic_cmd_vel_changed = true;
  
  // std::cout << "Set chassic_link vel to: vel(" 
  //           << msg->linear.x << ", " << msg->linear.y << ", " << msg->angular.z << ")" << std::endl;
}

void chassicPoseForceCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  // 通过施加外力来控制chassic_link的位置
  // 这种方法更平滑，不会造成突然的位置跳变
  
  // 计算当前位置和目标位置的差异
  double pos_error_x = msg->position.x - d->qpos[0];
  double pos_error_y = msg->position.y - d->qpos[1];
  double pos_error_z = msg->position.z - d->qpos[2];
  
  // 简单的PD控制器参数 - 增加控制力
  double kp_pos = 5000.0;  // 位置增益 - 增加5倍
  double kd_pos = 500.0;   // 速度增益 - 增加5倍
  
  // 计算控制力
  double force_x = kp_pos * pos_error_x - kd_pos * d->qvel[0];
  double force_y = kp_pos * pos_error_y - kd_pos * d->qvel[1];
  double force_z = kp_pos * pos_error_z - kd_pos * d->qvel[2];
  
  // 施加外力到chassic_link (body index = 0)
  d->xfrc_applied[6 * 0 + 0] = force_x;  // 力 x
  d->xfrc_applied[6 * 0 + 1] = force_y;  // 力 y
  d->xfrc_applied[6 * 0 + 2] = force_z;  // 力 z
  d->xfrc_applied[6 * 0 + 3] = 0.0;      // 力矩 x
  d->xfrc_applied[6 * 0 + 4] = 0.0;      // 力矩 y
  d->xfrc_applied[6 * 0 + 5] = 0.0;      // 力矩 z
  
  std::cout << "Applied force to chassic_link: (" 
            << force_x << ", " << force_y << ", " << force_z 
            << ") for target pos(" << msg->position.x << ", " 
            << msg->position.y << ", " << msg->position.z << ")" << std::endl;
}

//-----------------------m--------------- physics_thread --------------------------------------------

void randomizeTaskLighting(mjModel *model)
{
  int seed = -1;
  if (!g_nh_ptr->getParam("task_light_seed", seed) || seed < 0) {
    return;
  }

  int profile = 0;
  if (!g_nh_ptr->getParam("task_light_profile", profile) ||
      profile < 1 || profile > 3) {
    ROS_WARN("[TaskLighting] invalid or missing task_light_profile; "
             "keeping scene lighting unchanged");
    return;
  }

  // The task entries generate lighting with the shared Task2 rules.
  // Both the GUI and policy cameras use this same model. Do not overwrite it.
  int scene_seed_id = mj_name2id(model, mjOBJ_NUMERIC, "task_lighting_seed");
  if (scene_seed_id < 0) {
    scene_seed_id = mj_name2id(model, mjOBJ_NUMERIC, "task2_scene_seed");
  }
  if (scene_seed_id >= 0) {
    const int scene_seed = static_cast<int>(
        model->numeric_data[model->numeric_adr[scene_seed_id]]);
    ROS_INFO("[TaskLighting] task=%d seed=%d source=scene_xml "
             "diffuse=(%.3f, %.3f, %.3f) ambient=(%.3f, %.3f, %.3f) "
             "specular=(%.3f, %.3f, %.3f) overhead_diffuse=(%.3f, %.3f, %.3f)",
             profile, scene_seed,
             model->vis.headlight.diffuse[0], model->vis.headlight.diffuse[1],
             model->vis.headlight.diffuse[2],
             model->vis.headlight.ambient[0], model->vis.headlight.ambient[1],
             model->vis.headlight.ambient[2],
             model->vis.headlight.specular[0], model->vis.headlight.specular[1],
             model->vis.headlight.specular[2],
             model->nlight ? model->light_diffuse[0] : 0.0,
             model->nlight ? model->light_diffuse[1] : 0.0,
             model->nlight ? model->light_diffuse[2] : 0.0);
    return;
  }

  // Bright pastel headlight only; scene lights and object layouts stay intact.
  std::seed_seq seed_sequence{seed, profile};
  std::mt19937 generator(seed_sequence);
  const int tint = std::uniform_int_distribution<int>(0, 2)(generator);
  for (int channel = 0; channel < 3; ++channel) {
    model->vis.headlight.diffuse[channel] = channel == tint ? 0.9f : 0.7f;
    model->vis.headlight.ambient[channel] = 0.25f;
    model->vis.headlight.specular[channel] = 0.0f;
  }
  ROS_INFO("[TaskLighting] task=%d seed=%d diffuse=(%.3f, %.3f, %.3f) "
           "ambient=(%.3f, %.3f, %.3f) specular=(%.3f, %.3f, %.3f)",
           profile, seed,
           model->vis.headlight.diffuse[0], model->vis.headlight.diffuse[1],
           model->vis.headlight.diffuse[2],
           model->vis.headlight.ambient[0], model->vis.headlight.ambient[1],
           model->vis.headlight.ambient[2],
           model->vis.headlight.specular[0], model->vis.headlight.specular[1],
           model->vis.headlight.specular[2]);
}

void PhysicsThread(mj::Simulate *sim, const char *filename, bool only_half_up_body = false)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      randomizeTaskLighting(m);
      d = mj_makeData(m);
    }
    m->opt.timestep = 1 / frequency;
  
    if (robot_type == 2) 
    {
      std::cout << "LLeg joints size: " << LLegJointsAddr.qdofadr().size() << std::endl;
      std::cout << "RLeg joints size: " << RLegJointsAddr.qdofadr().size() << std::endl;
      std::cout << "Waist joints size: " << WaistJointsAddr.qdofadr().size() << std::endl;
      std::cout << "LArm joints size: " << LArmJointsAddr.qdofadr().size() << std::endl;
      std::cout << "RArm joints size: " << RArmJointsAddr.qdofadr().size() << std::endl;
      std::cout << "Head joints size: " << HeadJointsAddr.qdofadr().size() << std::endl;
    } 
    else if (robot_type == 1) 
    {
      numWheels += WheelJointsAddr.qdofadr().size();
      std::cout << "Leg joints size: " << LegJointsAddr.qdofadr().size() << std::endl;
      std::cout << "Wheel joints size: " << WheelJointsAddr.qdofadr().size() << std::endl;
    }
    else 
    {
      std::cout << "[mujoco_node] Unknown robot_type param, please set to 1 (轮臂) or 2 (双足)!" << std::endl;
      return;
    }
    
    std::cout << "\033[32mnumJoints: " << (m->nq - 7) << "\033[0m" << std::endl;
    std::cout << "\033[32mnumJoints(without dexhand): " << numJoints << "\033[0m" << std::endl;
    std::cout << "\033[32mtotal qpos (m->nq): " << m->nq << "\033[0m" << std::endl;

    if (d)
    {
      // ********************************
      init_cmd(d);
      // Preserve the MJCF qpos0 for every joint that is not part of the robot
      // initialization message.  In particular, task objects use free joints;
      // zero-filling those seven qpos values moves every object to the world
      // origin and leaves it with an invalid zero quaternion.
      qpos_init.assign(m->qpos0, m->qpos0 + m->nq);
      if (robot_type == 1)
      {
        qpos_init[2] = 0.0;// 初始化轮臂位置 - 设置在地面
      }
      else
      {
        qpos_init[2] = 0.99;// 初始化双足位置
      }
      InitRobotState(d);
      // ********************************
      // Finish initialization before exposing this data to the render thread.
      mj_forward(m, d);
      sim->Load(m, d, filename);

      // 如果启用半身模式，固定躯干位置和腿部关节
      if (only_half_up_body)
      {
        // 获取躯干(body)的ID
        int torso_id = mj_name2id(m, mjOBJ_BODY, "base_link");
        if (torso_id != -1)
        {
          // 记录初始位置和姿态
          fixed_torso_pos[0] = d->qpos[0];
          fixed_torso_pos[1] = d->qpos[1]; 
          fixed_torso_pos[2] = d->qpos[2];
          fixed_torso_quat[0] = d->qpos[3];
          fixed_torso_quat[1] = d->qpos[4];
          fixed_torso_quat[2] = d->qpos[5];
          fixed_torso_quat[3] = d->qpos[6];
          
          torso_constrained = true;
          
          ROS_INFO("Torso position fixed at: [%.3f, %.3f, %.3f]", 
                   fixed_torso_pos[0], fixed_torso_pos[1], fixed_torso_pos[2]);
          ROS_INFO("Torso orientation fixed at quaternion: [%.3f, %.3f, %.3f, %.3f]", 
                   fixed_torso_quat[0], fixed_torso_quat[1], fixed_torso_quat[2], fixed_torso_quat[3]);
        }
        else
        {
          ROS_WARN("Could not find 'base_link' body for torso constraint");
        }
        
        // 固定腿部关节位置
        if (!LLegJointsAddr.qposadr().invalid() && !RLegJointsAddr.qposadr().invalid())
        {
          // 初始化左腿关节固定位置
          fixed_leg_l_qpos.clear();
          for (auto iter = LLegJointsAddr.qposadr().begin(); iter != LLegJointsAddr.qposadr().end(); iter++) {
            fixed_leg_l_qpos.push_back(d->qpos[*iter]);
          }
          
          // 初始化右腿关节固定位置
          fixed_leg_r_qpos.clear();
          for (auto iter = RLegJointsAddr.qposadr().begin(); iter != RLegJointsAddr.qposadr().end(); iter++) {
            fixed_leg_r_qpos.push_back(d->qpos[*iter]);
          }
          
          leg_joints_constrained = true;
          
          ROS_INFO("Left leg joints fixed at %zu positions", fixed_leg_l_qpos.size());
          ROS_INFO("Right leg joints fixed at %zu positions", fixed_leg_r_qpos.size());
        }
        else
        {
          ROS_WARN("Could not initialize leg joint constraints - joint addresses invalid");
        }
      }

      // allocate ctrlnoise
      // free(ctrlnoise);
      // ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      // mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  sensorsPub = g_nh_ptr->advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 10);
  pubGroundTruth = g_nh_ptr->advertise<nav_msgs::Odometry>("/ground_truth/state", 10);
  pubOdom = g_nh_ptr->advertise<nav_msgs::Odometry>("/odom", 10);
  pubTimeDiff = g_nh_ptr->advertise<std_msgs::Float64>("/monitor/time_cost/mujoco_loop_time", 10);
  pubLeftArmFT = g_nh_ptr->advertise<geometry_msgs::WrenchStamped>("/arm_force_torque/left", 10);
  pubRightArmFT = g_nh_ptr->advertise<geometry_msgs::WrenchStamped>("/arm_force_torque/right", 10);
  g_nh_ptr->getParam("task_body_names", task_body_names);
  for (const std::string &name : task_body_names) {
    if (mj_name2id(m, mjOBJ_BODY, name.c_str()) < 0) {
      ROS_WARN("[TaskBodyPose] body '%s' is not present in the loaded model", name.c_str());
      continue;
    }
    task_body_pose_publishers.emplace(
        name, g_nh_ptr->advertise<geometry_msgs::PoseStamped>("/mujoco/" + name + "/pose", 10));
  }
  initializeTask1LeverFingerLatch();
  task13_score_bodies.clear();
  for (const auto& name : {"cylinder_1", "cylinder_2", "cylinder_3",
       "task3_hollow_cylinder", "task3_hollow_cylinder_2", "task3_hollow_cylinder_3"}) {
    int body = mj_name2id(m, mjOBJ_BODY, name);
    if (body < 0) continue;
    bool bin = std::string(name).find("cylinder_") == 0;
    int source = mj_name2id(m, mjOBJ_GEOM, bin ? "source_bin_floor" : "task3_source_table_top");
    int target = mj_name2id(m, mjOBJ_GEOM, bin ? "target_bin_floor" : "task3_destination_table_top");
    task13_score_bodies.push_back({body, source, target, bin,
      g_nh_ptr->advertise<std_msgs::UInt8>(std::string("/mujoco/")+name+"/scoring_state", 1, true)});
  }
  source_bin_slide_id = mj_name2id(m, mjOBJ_JOINT, "source_bin_slide");
  source_bin_still_since = -1;
  if (source_bin_slide_id >= 0)
    source_bin_score_publisher = g_nh_ptr->advertise<std_msgs::UInt8>(
        "/mujoco/source_bin/scoring_state", 1, true);
  std::vector<std::string> bimanual_contact_follow_body_names;
  g_nh_ptr->getParam(
      "bimanual_contact_follow_body_names",
      bimanual_contact_follow_body_names);
  initializeBimanualContactFollowers(
      bimanual_contact_follow_body_names);
  std::vector<std::string> internal_contact_follow_body_names;
  g_nh_ptr->getParam(
      "internal_contact_follow_body_names",
      internal_contact_follow_body_names);
  std::vector<std::string> single_hand_body_names;
  g_nh_ptr->getParam("contact_follow_body_names", single_hand_body_names);
  internal_contact_follow_body_names.insert(internal_contact_follow_body_names.end(),
      single_hand_body_names.begin(), single_hand_body_names.end());
  initializeInternalContactFollowers(internal_contact_follow_body_names);
  task_camera_transport =
      std::make_unique<image_transport::ImageTransport>(*g_nh_ptr);
  bool camera_available = ConfigureDepthCameraForCurrentModel();
  bool task_cameras_available = ConfigureTaskRgbdCamerasForCurrentModel();
  if (camera_available) {
    depthImagePub = g_nh_ptr->advertise<sensor_msgs::Image>(mujoco_cpp::kDepthImageTopic, 10);
    depthImageArrayPub = g_nh_ptr->advertise<std_msgs::Float64MultiArray>(mujoco_cpp::kDepthImageArrayTopic, 10);
    depthHistoryPub = g_nh_ptr->advertise<std_msgs::Float64MultiArray>(mujoco_cpp::kDepthHistoryTopic, 10);
  }

  // // 创建服务
  ros::ServiceServer service = g_nh_ptr->advertiseService("sim_start", handleSimStart);
  ros::ServiceServer setObjectPositionService =
      g_nh_ptr->advertiseService("set_object_position", setObjectPositionCallback);
  ros::ServiceServer setJointPositionService =
      g_nh_ptr->advertiseService("set_joint_position", setJointPositionCallback);

  // // 创建订阅器
  ros::Subscriber clawCmdSub = g_nh_ptr->subscribe("/leju_claw_command", 10, clawCmdCallback);
#ifndef USE_DDS
  ros::Subscriber jointCmdSub = g_nh_ptr->subscribe("/joint_cmd", 10, jointCmdCallback);
#endif
  ros::Subscriber extWrenchSub = g_nh_ptr->subscribe("/external_wrench", 10, extWrenchCallback);
  ros::Subscriber task2ConveyorEnabledSub = g_nh_ptr->subscribe<std_msgs::Bool>(
      "/mujoco/task2_conveyor_enabled", 1, task2ConveyorEnabledCallback);

  if (camera_available) {
    depth_thread_running.store(true);
    depth_thread = std::thread([]() {
        ros::Rate depth_rate(depth_frequency);
        while (depth_thread_running.load() && ros::ok()) {
            // Lock mutex to safely copy MuJoCo data if needed
            std::unique_lock<std::mutex> lock(mujoco_data_mutex);

            if (g_depth_camera) {
                // auto t0 = std::chrono::high_resolution_clock::now();
                g_depth_camera->compute_distance();
                // auto t1 = std::chrono::high_resolution_clock::now();
                // double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                // std::cout << "RayCasterCamera runtime: " << ms << " ms" << std::endl;

                // Process and publish depth image
                sensor_msgs::Image depth_msg;
                depth_msg.header.stamp = ros::Time::now();
                depth_msg.header.frame_id = mujoco_cpp::kDepthCameraFrameId;
                depth_msg.height = DEPTH_CAMERA_HEIGHT;
                depth_msg.width = DEPTH_CAMERA_WIDTH;
                depth_msg.encoding = "32FC1";
                depth_msg.step = DEPTH_CAMERA_WIDTH * sizeof(float);
                depth_msg.is_bigendian = 0;
                depth_msg.data.resize(DEPTH_CAMERA_HEIGHT * DEPTH_CAMERA_WIDTH * sizeof(float));
                float* z_depth_data = reinterpret_cast<float*>(depth_msg.data.data());

                if (g_depth_camera->dist != nullptr) {
                    // const mjtNum range_inv = 1.0 / (DEPTH_CAMERA_MAX_RANGE - DEPTH_CAMERA_MIN_RANGE);
                    for (int v = 0; v < DEPTH_CAMERA_HEIGHT; ++v) {
                        for (int h = 0; h < DEPTH_CAMERA_WIDTH; ++h) {
                            int pixel_idx = v * DEPTH_CAMERA_WIDTH + h;
                            mjtNum dist = g_depth_camera->dist[pixel_idx];
                            mjtNum pixel_x =
                                (h + 0.5 - DEPTH_CAMERA_WIDTH / 2.0) * DEPTH_CAMERA_H_PIXEL_SIZE;
                            mjtNum pixel_y =
                                (DEPTH_CAMERA_HEIGHT / 2.0 - v - 0.5) * DEPTH_CAMERA_V_PIXEL_SIZE;
                            mjtNum z_depth =
                                RayDistanceToZDepth(dist, pixel_x, pixel_y, FOCAL_LENGTH);
                            // float norm = (dist - DEPTH_CAMERA_MIN_RANGE) * range_inv;
                            // norm = std::clamp(norm, 0.0f, 1.0f);
                            // depth_data[pixel_idx] = norm;
                            z_depth = std::clamp(z_depth, mjtNum(0), DEPTH_CAMERA_MAX_RANGE);
                            z_depth_data[pixel_idx] = z_depth / DEPTH_CAMERA_MAX_RANGE;
                        }
                    }
                }
  
                // Apply Gaussian blur
                cv::Mat depth_mat(DEPTH_CAMERA_HEIGHT, DEPTH_CAMERA_WIDTH, CV_32FC1, z_depth_data);
                cv::GaussianBlur(depth_mat, depth_mat, cv::Size(3, 3), 1, 1);

                // Update circular buffer with current frame
                std::unique_lock<std::mutex> buffer_lock(depth_buffer_mutex);
                depth_buffer[current_buffer_index].data.assign(z_depth_data, z_depth_data + DEPTH_CAMERA_HEIGHT * DEPTH_CAMERA_WIDTH);  // deep copy
                depth_buffer[current_buffer_index].timestamp = depth_msg.header.stamp;
                current_buffer_index = (current_buffer_index + 1) % DEPTH_BUFFER_SIZE;
                
                // Mark buffer as filled once we've cycled through all 43 frames
                if (current_buffer_index == 0 && !depth_buffer_filled) {
                  depth_buffer_filled = true;
                }
                buffer_lock.unlock();

                std_msgs::Float64MultiArray depth_array_msg;
                depth_array_msg.data.resize(DEPTH_CAMERA_HEIGHT * DEPTH_CAMERA_WIDTH);
                for (int i = 0; i < DEPTH_CAMERA_HEIGHT * DEPTH_CAMERA_WIDTH; ++i) {
                    depth_array_msg.data[i] = z_depth_data[i];
                }
                depthImagePub.publish(depth_msg);
                depthImageArrayPub.publish(depth_array_msg);

                // Publish depth history buffer
                // From 6*7+1=43 frames, take frame indices: 0, 6, 12, 18, 24, 30, 36, 42 (8 frames total)
                std::unique_lock<std::mutex> lock(depth_buffer_mutex);
                std::vector<int> selected_indices;
                // printf("buf filled:%d | ", depth_buffer_filled);
                for (int i = 0; i < 3 * 8; i += 3) {
                  int idx = (current_buffer_index + i) % DEPTH_BUFFER_SIZE;
                  // printf("idx %d ", idx);
                  selected_indices.push_back(idx); // 1st frame of each group
                }
                
                std::vector<float> first_frame_data;
                if (!depth_buffer[0].data.empty()) {
                  first_frame_data = depth_buffer[0].data;
                }
                
                ros::Time start_time = ros::Time::now();
                std_msgs::Float64MultiArray history_array_msg;
                for (int i = 0; i < selected_indices.size(); ++i) {
                  int idx = selected_indices[i];
                  // printf("selected idx %d | ", idx);
                  
                  // If buffer is not yet full and the idx is beyond the processed point, use first frame to pad
                  if (!depth_buffer_filled && idx >= current_buffer_index) {
                    for (float val : first_frame_data) {
                      history_array_msg.data.push_back(val);
                    }
                  } else if (!depth_buffer[idx].data.empty()) {
                    for (float val : depth_buffer[idx].data) {
                      history_array_msg.data.push_back(val);
                    }
                  } else if (!first_frame_data.empty()) {
                    // If this position is empty but buffer is full, use first frame as fallback
                    for (float val : first_frame_data) {
                      history_array_msg.data.push_back(val);
                    }
                  }
                }
                // printf("\n");
                lock.unlock();
                depthHistoryPub.publish(history_array_msg);
            }

            lock.unlock();
            depth_rate.sleep();
        }
        std::cout << "Depth thread exited." << std::endl;
    });
  }

  if (task_cameras_available) {
    task_camera_thread_running.store(true);
    task_camera_thread = std::thread([sim]() {
      // GL context 是本线程私有的，所以渲染器在这里创建、也只在这里用。
      task_offscreen_renderer = std::make_unique<mujoco_cpp::OffscreenCameraRenderer>();
      task_rgbd_camera_rgb.assign(
          static_cast<std::size_t>(kTaskCameraWidth) * kTaskCameraHeight * 3, 0);
      task_rgbd_camera_depth.assign(
          static_cast<std::size_t>(kTaskCameraWidth) * kTaskCameraHeight, 0.0f);
      int built_epoch = -1;
      std::string offscreen_failure;
      ros::Rate camera_rate(kTaskCameraFrequency);
      while (task_camera_thread_running.load() && ros::ok()) {
        std::unique_lock<std::recursive_mutex> camera_lock(task_camera_mutex);
        if (task_camera_data == nullptr || m == nullptr) {
          camera_lock.unlock();
          camera_rate.sleep();
          continue;
        }

        // 模型换过就重建 GL 侧的一切：mjrContext 的显示列表和纹理都绑在旧模型上。
        const int epoch = task_camera_model_epoch.load();
        if (epoch != built_epoch) {
          if (task_offscreen_renderer->Initialize(
                  m, kTaskCameraWidth, kTaskCameraHeight, &offscreen_failure)) {
            built_epoch = epoch;
            ROS_INFO("[TaskCamera] Offscreen renderer ready: %dx%d, %zu cameras",
                     kTaskCameraWidth, kTaskCameraHeight,
                     task_rgbd_cameras.size());
          } else {
            // 只报一次，避免 30 Hz 刷屏；每帧都会重试，X 恢复后自动接上。
            if (built_epoch != -2) {
              ROS_ERROR("[TaskCamera] Offscreen renderer unavailable: %s",
                        offscreen_failure.c_str());
              built_epoch = -2;
            }
            camera_lock.unlock();
            camera_rate.sleep();
            continue;
          }
        }

        // 物理线程此刻可能正在写 m/d，快照必须在锁内拷。
        {
          std::unique_lock<std::recursive_mutex> simulation_lock(sim->mtx);
          mj_copyData(task_camera_data, m, d);
        }

        // 渲染刻意放在 sim.mtx 之外：三路 640×480 约 6 ms，若持锁渲染会把
        // 1000 Hz 的物理步进拖停。m 本身由 task_camera_mutex 保护 —— 模型重载
        // 路径在 mj_deleteModel 之前也要拿这把锁，所以这里读 m 是安全的。
        const ros::Time stamp = ros::Time::now();
        std::vector<sensor_msgs::Image> color_messages;
        color_messages.reserve(task_rgbd_cameras.size());
        for (TaskRgbdCamera &camera : task_rgbd_cameras) {
          // 深度缓冲仍按原样传进去：Render 一次渲染同时填两个缓冲，去掉深度输出
          // 不改这条路径。要一并省掉渲染开销得改 Render 的签名，那是另一件事。
          if (!task_offscreen_renderer->Render(m, task_camera_data, camera.camera_id,
                                         task_rgbd_camera_rgb.data(),
                                         task_rgbd_camera_depth.data())) {
            continue;
          }
          color_messages.emplace_back(BuildTaskColorImage(
              camera, stamp, task_rgbd_camera_rgb.data()));
        }

        for (std::size_t index = 0; index < color_messages.size(); ++index) {
          task_rgbd_cameras[index].color_publisher.publish(color_messages[index]);
        }
        camera_lock.unlock();
        camera_rate.sleep();
      }
      // 必须在本线程析构：GL context 的释放要求当前线程持有它。
      task_offscreen_renderer.reset();
      ROS_INFO("[TaskCamera] RGB publisher thread exited");
    });
  }

#ifdef USE_DDS
      // 初始化DDS通信
    std::cout << "\033[33m[MuJoCo DDS] Initializing DDS communication...\033[0m" << std::endl;
    dds_client = std::make_unique<MujocoDdsClient<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>>();
    dds_client->setLowCmdCallback(ddsLowCmdCallback);
    dds_client->start();
    std::cout << "\033[33m[MuJoCo DDS] DDS communication started\033[0m" << std::endl;
#elif defined(USE_LEJU_DDS)
    // Initialize Leju DDS communication
    std::cout << "\033[33m[MuJoCo LEJU DDS] Initializing LEJU DDS communication...\033[0m" << std::endl;
    dds_client = std::make_unique<MujocoDdsClient<leju::msgs::JointCmd, leju::msgs::SensorsData>>();
    dds_client->setLowCmdCallback(lejuDdsLowCmdCallback);
    dds_client->start();
    std::cout << "\033[33m[MuJoCo LEJU DDS] LEJU DDS communication started\033[0m" << std::endl;
#endif
  ros::Subscriber chassicPoseSub = g_nh_ptr->subscribe("/chassic_pose", 10, chassicPoseCallback);
  ros::Subscriber cmdVelSub = g_nh_ptr->subscribe("/move_base/base_cmd_vel", 10, cmdVelCallback);
  ros::Subscriber chassicPoseForceSub = g_nh_ptr->subscribe("/chassic_pose_force", 10, chassicPoseForceCallback);

  // 初始化灵巧手ROS
  if(!RHandJointsAddr.ctrladr().invalid()) {
      std::cout << "[mujoco_node]: init dexhand node" << std::endl;
      g_dexhand_node = std::make_shared<DexHandMujocoRosNode>();

      // 优先从URDF自定义元数据中读取手类型
      mujoco_node::HandType hand_type = mujoco_node::HandType::QIANGNAO;
      int hand_type_id = mj_name2id(m, mjOBJ_NUMERIC, "hand_type");
      
      if (hand_type_id != -1) {
          const int data_adr = m->numeric_adr[hand_type_id];
          int hand_type_value = static_cast<int>(m->numeric_data[data_adr]);
          if (hand_type_value == 1) {
              hand_type = mujoco_node::HandType::LINKER_L6;
              std::cout << "[mujoco_node]: Detected LinkerL6 dexhand from URDF custom metadata" << std::endl;
          } else if (hand_type_value == 2) {
              hand_type = mujoco_node::HandType::LINKER_O6;
              std::cout << "[mujoco_node]: Detected LinkerO6 dexhand from URDF custom metadata" << std::endl;
          } else if (hand_type_value == 3) {
              hand_type = mujoco_node::HandType::HEIMAN;
              std::cout << "[mujoco_node]: Detected Heiman hand from MJCF custom metadata" << std::endl;
          } else {
              hand_type = mujoco_node::HandType::QIANGNAO;
              std::cout << "[mujoco_node]: Detected Qiangnao hand from URDF custom metadata" << std::endl;
          }
      } else {
          // 没有找到自定义元数据，默认使用Qiangnao手
          hand_type = mujoco_node::HandType::QIANGNAO;
          std::cout << "[mujoco_node]: No hand_type metadata in URDF, default to use Qiangnao hand" << std::endl;
      }

      if (!g_dexhand_node->init(*g_nh_ptr, m, RHandJointsAddr, LHandJointsAddr, hand_type)) {
          ROS_FATAL("[mujoco_node] Failed to initialize dexhand node");
          sim->exitrequest.store(1);
          g_nh_ptr->setParam("end_effector_joints_num", 0);
          return;
      }

      int hand_joints_num = g_dexhand_node->get_hand_joints_num();
      g_nh_ptr->setParam("end_effector_joints_num", hand_joints_num);
  }
  else {
    g_nh_ptr->setParam("end_effector_joints_num", 0);
  }

  std::cout << "[mujoco_node]: waiting for init qpos" << std::endl;
  while (ros::ok() && !sim->exitrequest.load())
  {
    if (g_nh_ptr->hasParam("robot_init_state_param"))
    {
      qpos_init.resize(m->nq);
      std::vector<double> qpos_init_temp;
      qpos_init_temp.resize(50);
      if (g_nh_ptr->getParam("robot_init_state_param", qpos_init_temp))
      {
        ROS_INFO("Get init qpos ");
        // // insert waist qpos
        // int waist_num = 0;
        // g_nh_ptr->getParam("waistRealDof", waist_num);
        // std::cout << "Mujoco waist_num: " << waist_num << std::endl;
        // if (waist_num > 0)
        // {
        //   for (int i = 0; i < waist_num; i++)
        //   {
        //     qpos_init_temp.insert(qpos_init_temp.begin() + 7, 0.0);
        //   }
        // }
        // waistNum = waist_num;
        if(robot_type == 2)
        {
          for (int i = 0; i < qpos_init_temp.size(); i++)
          {
            qpos_init[i] = qpos_init_temp[i];
            std::cout << qpos_init_temp[i] << ", ";
          }
          std::cout << std::endl;
        }
        else if (robot_type == 1)
        {
          // freejoint: qpos_init_temp[0..6] -> qpos[0..6]
          for (int i = 0; i < 7; i++)
          {
            qpos_init[i] = qpos_init_temp[i];
            std::cout << qpos_init[i] << ", ";
          }
          // Use name-based qposadr lookup instead of hardcoded +8 offset
          // qpos_init_temp[7..] = [leg, larm, rarm, head]
          int src_idx = 7;
          auto copyGroupQpos = [&](const JointGroupAddress& addr) {
            for (auto iter = addr.qposadr().begin(); iter != addr.qposadr().end() && src_idx < (int)qpos_init_temp.size(); ++iter, ++src_idx) {
              if (*iter < (int)qpos_init.size()) {
                qpos_init[*iter] = qpos_init_temp[src_idx];
                std::cout << qpos_init[*iter] << ", ";
              }
            }
          };
          copyGroupQpos(LegJointsAddr);
          copyGroupQpos(LArmJointsAddr);
          copyGroupQpos(RArmJointsAddr);
          copyGroupQpos(HeadJointsAddr);
        }
        
        // // 根据机器人类型调整初始高度
        // int robot_type = 2;
        // g_nh_ptr->getParam("robot_type", robot_type);
        // qpos_init[2] = (robot_type == 1) ? 0.195 : 0.74;
        
        break;
      }
      else
      {
        ROS_INFO("[mujoco_node]Failed to get init qpos, use default qpos");
        const std::vector<double> default_robot_qpos = {
            -0.00505, 0.00000, 0.84414, 0.99864, 0.00000, 0.05215, -0.00000,
            -0.01825, -0.00190, -0.52421, 0.73860, -0.31872, 0.01835,
            0.01825, 0.00190, -0.52421, 0.73860, -0.31872, -0.01835,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0};
        std::copy(default_robot_qpos.begin(), default_robot_qpos.end(),
                  qpos_init.begin());
        break;
      }
    }
    
    usleep(10000);
  }
  // Camera snapshots and ROS object updates already run at this point.
  // Use the same lock as PhysicsLoop; Load must stay outside it because
  // Load waits for the render thread to acquire that lock.
  {
    const std::unique_lock<std::recursive_mutex> simulation_lock(sim->mtx);
    InitRobotState(d);
    mj_forward(m, d);
  }
  sim->Load(m, d, filename);

  // 初始化时查找手臂末端执行器或link ID
  // 优先使用end_effector（更精确），fallback到link（兼容旧版本）
  left_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_l7_end_effector");
  if (left_arm_link_id_ == -1) {
    left_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_l7_link");
    if (left_arm_link_id_ == -1) {
      left_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_l4_link");
    }
  }
  
  right_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_r7_end_effector");
  if (right_arm_link_id_ == -1) {
    right_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_r7_link");
    if (right_arm_link_id_ == -1) {
      right_arm_link_id_ = mj_name2id(m, mjOBJ_BODY, "zarm_r4_link");
    }
  }

  // 躯干吊绳 + 直立力矩：直接作用于 torso 质心
  torso_body_id_ = mj_name2id(m, mjOBJ_BODY, "torso");
  if (torso_body_id_ < 0) {
    ROS_WARN("[TorsoRope] torso body not found, rope + upright disabled");
  }

  ROS_INFO("Arm force application IDs: left=%d, right=%d", left_arm_link_id_, right_arm_link_id_);

  // 躯干弹力绳订阅
  ros::Subscriber torsoRopeActiveSub = g_nh_ptr->subscribe<std_msgs::Bool>(
      "/mujoco/torso_rope/active", 10, [&](const std_msgs::Bool::ConstPtr &msg) {
        torso_rope_active_ = msg->data;
        ROS_INFO("[TorsoRope] active = %s", torso_rope_active_ ? "ON" : "OFF");
      });
  ros::Subscriber torsoRopeParamsSub = g_nh_ptr->subscribe<geometry_msgs::Vector3>(
      "/mujoco/torso_rope/params", 10, [&](const geometry_msgs::Vector3::ConstPtr &msg) {
        if (msg->x > 0.01) torso_rope_speed_ = msg->x;   // x: 绞盘拉速 m/s
        if (msg->y > 0.01) torso_rope_kv_ = msg->y;       // y: 速度追踪增益
        if (msg->z > 0.01) torso_rope_length_ = msg->z;   // z: 绳长 m
        ROS_INFO_THROTTLE(1.0, "[TorsoRope] speed=%.2f m/s, kv=%.0f, rope_len=%.3f m, anchor_z=%.3f m (upright_kp=%.0f, lin_damp=%.0f, ang_damp=%.0f)",
                 torso_rope_speed_, torso_rope_kv_, torso_rope_length_, torso_rope_anchor_z_,
                 torso_upright_kp_, torso_rope_lin_damp_, torso_rope_ang_damp_);
      });

  // 手臂外力订阅（存储外力值，在仿真循环中持续应用）
  ros::Subscriber lHandExtWrenchSub = g_nh_ptr->subscribe<geometry_msgs::Wrench>("/external_wrench/left_hand", 10, [&](const geometry_msgs::Wrench::ConstPtr &msg)
      {
        left_hand_wrench_ = *msg;
        // 判断是否有力（任意分量非零即为激活）
        left_hand_active_ = (std::abs(msg->force.x) > 1e-6 || std::abs(msg->force.y) > 1e-6 || std::abs(msg->force.z) > 1e-6 ||
                            std::abs(msg->torque.x) > 1e-6 || std::abs(msg->torque.y) > 1e-6 || std::abs(msg->torque.z) > 1e-6);
      }
    );  
  ros::Subscriber rHandExtWrenchSub = g_nh_ptr->subscribe<geometry_msgs::Wrench>("/external_wrench/right_hand", 10, [&](const geometry_msgs::Wrench::ConstPtr &msg)
      {
        right_hand_wrench_ = *msg;
        // 判断是否有力（任意分量非零即为激活）
        right_hand_active_ = (std::abs(msg->force.x) > 1e-6 || std::abs(msg->force.y) > 1e-6 || std::abs(msg->force.z) > 1e-6 ||
                             std::abs(msg->torque.x) > 1e-6 || std::abs(msg->torque.y) > 1e-6 || std::abs(msg->torque.z) > 1e-6);
      }
    );

  if (is_spin_thread)
  {
    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();
  }

  PhysicsLoop(*sim);

  if (g_dexhand_node)
    g_dexhand_node->stop();

  if (depth_thread.joinable()) {
      depth_thread_running.store(false);
      if (depth_thread.joinable()) {
          depth_thread.join();
      }
  }
  if (task_camera_thread.joinable()) {
      task_camera_thread_running.store(false);
      task_camera_thread.join();
  }
  task_rgbd_cameras.clear();
  if (task_camera_data != nullptr) {
      mj_deleteData(task_camera_data);
      task_camera_data = nullptr;
  }
  task_camera_transport.reset();

  // delete everything we allocated

  // free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

//**************************
// run event loop
void request_simulation_stop()
{
  std::lock_guard<std::mutex> lock(simulation_lifecycle_mutex);
  simulation_stop_requested = true;
  if (sim)
    sim->exitrequest.store(1);
}

int simulate_loop(ros::NodeHandle &nh, bool spin_thread = false)
{
  // ros::init(argc, argv, "mujoco_sim");
  // ros::NodeHandle nh;
  is_spin_thread = spin_thread;
  g_nh_ptr = &nh;
  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // 获取参数并设置频率
  if (!nh.hasParam("/wbc_frequency"))
  {
    ROS_INFO("wbc_frequency was deleted!\n");
  }
  else
  {
    nh.getParam("/wbc_frequency", frequency);
  }
  ROS_INFO("Mujoco Frequency: %f Hz", frequency);

  nh.param("/depth_frequency", depth_frequency, kDefaultDepthFrequency);
  if (depth_frequency <= 0.0)
  {
    ROS_WARN("Invalid depth_frequency: %f Hz, fallback to %f Hz",
             depth_frequency, kDefaultDepthFrequency);
    depth_frequency = kDefaultDepthFrequency;
  }
  ROS_INFO("Mujoco depth camera frequency: %f Hz", depth_frequency);
  
  // 获取相机是否启动的判断
  if (!nh.hasParam("/run_mujoco_camera"))
  {
    ROS_INFO("run_mujoco_camera was deleted!\n");
  }
  else
  {
    nh.getParam("/run_mujoco_camera", isRunCamera_);
  }
  ROS_INFO("run_mujoco_camera: %d", isRunCamera_);

  // 获取only_half_up_body参数
  bool only_half_up_body = false;
  if (nh.hasParam("/only_half_up_body"))
  {
    nh.getParam("/only_half_up_body", only_half_up_body);
    ROS_INFO("Only half up body mode: %s", only_half_up_body ? "true" : "false");
  }

  if (nh.hasParam("robot_type")) 
  {
    nh.getParam("robot_type", robot_type);
    std::cout << "[mujoco_node] robot_type param: " << robot_type << std::endl;
  }

  if(nh.hasParam("robot_version"))
  {
    int raw_version = 0;
    nh.getParam("robot_version", raw_version);
    robotVersion_ = RobotVersion::create(raw_version).version_number();
    std::cout << "[mujoco_node] robot_version normalized: " << robotVersion_ << " (from raw: " << raw_version << ")" << std::endl;
  }

  if(nh.hasParam("pure_sim"))
  {
    nh.getParam("pure_sim", pure_sim);
    std::cout << "[mujoco_node] pure_sim param: " << pure_sim << std::endl;
  }
  
  // 获取配置文件
  if(nh.hasParam("/kuavo_configuration")) {
    std::string kuavo_configuration;
    nh.getParam("/kuavo_configuration", kuavo_configuration);
    if (!kuavo_configuration.empty()) {
      try {
          nlohmann::json config_json;
          // 解析kuavo_configuration字符串为JSON对象
          std::istringstream config_stream(kuavo_configuration);
          config_stream >> config_json;
          if (config_json.contains("EndEffectorType") && config_json["EndEffectorType"].is_array()) {
            if (!config_json["EndEffectorType"].empty()) {
              std::string end_effector_type = config_json["EndEffectorType"][0];
              nh.setParam("end_effector_type", end_effector_type);
              ROS_INFO("\033[32mEnd effector type: %s\033[0m", end_effector_type.c_str());
            }
          }
          
          // 解析手臂末端关节名称
          if (config_json.contains("arm_end_joints") && config_json["arm_end_joints"].is_array()) {
            auto arm_end_joints = config_json["arm_end_joints"];
            if (arm_end_joints.size() >= 2) {
              left_arm_end_joint = arm_end_joints[0].get<std::string>();
              right_arm_end_joint = arm_end_joints[1].get<std::string>();
              
              ROS_INFO("\033[32mLeft arm end joint: %s\033[0m", left_arm_end_joint.c_str());
              ROS_INFO("\033[32mRight arm end joint: %s\033[0m", right_arm_end_joint.c_str());
            }
          }
          
          // 读取NUM_JOINT参数
          if (config_json.contains("NUM_JOINT") && config_json["NUM_JOINT"].is_number()) {
            numJoints = config_json["NUM_JOINT"].get<size_t>();
            ROS_INFO("\033[32mNUM_JOINT from config: %zu\033[0m", numJoints);
          } else {
            ROS_WARN("NUM_JOINT not found in config, using default value: %zu", numJoints);
          }

      } catch (const std::exception& e) {
        ROS_ERROR("Error parsing configuration file: %s", e.what());
      }
    }
  }
  else
  {
    ROS_WARN("kuavo_configuration not found, using default value");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);
  // simulate object encapsulates the UI
  {
    std::lock_guard<std::mutex> lock(simulation_lifecycle_mutex);
    if (simulation_stop_requested)
      return 0;
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);
  }
  // A nodelet shares the manager process with the controller. Preserve its
  // ROS handler so SIGINT stops all plugins, not just the sensor simulation.
  const auto previous_sigint_handler =
      spin_thread ? signal(SIGINT, signalHandler) : SIG_DFL;
  std::cout << "Physics thread started." << std::endl;

  std::string filename_str;
  if (nh.getParam("legged_robot_scene_param", filename_str))
  {
    ROS_INFO("[mujoco_node.cc]: Get legged_robot_scene_param: %s", filename_str.c_str());
  }
  else
  {
    std::cerr << "Failed to get legged_robot_scene_param" << std::endl;
    exit(1);
  }
  const char *filename = filename_str.c_str();
    
    
 
  // if (argc > 1)
  // {
  //   filename = argv[1];
  // }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename, only_half_up_body);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();
  if (spin_thread)
    signal(SIGINT, previous_sigint_handler);
  {
    // Destroy the UI on its owning thread, before GLFW's atexit handler.
    std::lock_guard<std::mutex> lock(simulation_lifecycle_mutex);
    sim.reset();
  }

  return 0;
}
