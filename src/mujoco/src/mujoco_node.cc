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
#include "ros/ros.h"
#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointData.h"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/FTsensorData.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <csignal>
#include <atomic>
#include <queue>
#include "kuavo_msgs/lejuClawCommand.h"
#include "sensor_msgs/JointState.h"

#include "mujoco_cpp/depth_camera_config.h"
#include "joint_address.hpp"
#include "dexhand_mujoco_node.h"
#include "dexhand/json.hpp"
#include "mujoco_cpp/ActuatorDynamics.hpp"

#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "mujoco_dds.h"
#endif

//  ******************* raycaster camera *********************

#include "RayCasterCamera.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <opencv2/opencv.hpp>

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
  // raycaster camera thread
  std::thread depth_thread;
  std::atomic<bool> depth_thread_running{true};
  std::mutex mujoco_data_mutex;  // Protects access to m and d
  double depth_frequency = 60.0;  // Hz
  bool isRunCamera_{false};

  // Depth image history buffer (6*6+7=43 frames)
  struct DepthImageFrame {
      std::vector<float> data;
      ros::Time timestamp;
  };
  static const int DEPTH_BUFFER_SIZE = 43;
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
  std::unique_ptr<mujoco_sim::ActuatorDynamicsCompensator> actuatorDynamicsCompensator;
  constexpr int kArmCompensationDof = 14;

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

  // control noise variables
  // mjtNum* ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  //---------------------------------- depth history publisher -----------------------------------
  
  void publish_depth_history()
  {
    std::unique_lock<std::mutex> lock(depth_buffer_mutex);

    // From 6*7+1=43 frames, take frame indices: 0, 6, 12, 18, 24, 30, 36, 42 (8 frames total)
    std::vector<int> selected_indices;
    for (int i = 0; i < 7; ++i) {
      selected_indices.push_back(i * 6); // 1st frame of each group
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
  JointGroupAddress LArmJointsAddr("r_arm_joints");
  JointGroupAddress RArmJointsAddr("r_arm_joints");
  JointGroupAddress HeadJointsAddr("head_joints");
  JointGroupAddress LHandJointsAddr("l_hand_joints");
  JointGroupAddress RHandJointsAddr("r_hand_joints");
  /*********************************************************************************************/
  // Mujoco Dexhand
  std::shared_ptr<mujoco_node::DexHandMujocoRosNode> g_dexhand_node = nullptr;
  /*********************************************************************************************/

  bool buildArmCompensationMeasuredDq(Eigen::VectorXd& measuredDq) {
    measuredDq = Eigen::VectorXd::Zero(kArmCompensationDof);
    if (!sim || !d || LArmJointsAddr.qdofadr().invalid() || RArmJointsAddr.qdofadr().invalid()) {
      return false;
    }
    if (LArmJointsAddr.qdofadr().size() != kArmCompensationDof / 2 ||
        RArmJointsAddr.qdofadr().size() != kArmCompensationDof / 2) {
      return false;
    }

    std::unique_lock<std::recursive_mutex> lock(sim->mtx);
    int idx = 0;
    for (auto iter = LArmJointsAddr.qdofadr().begin(); iter != LArmJointsAddr.qdofadr().end(); ++iter) {
      measuredDq[idx++] = d->qvel[*iter];
    }
    for (auto iter = RArmJointsAddr.qdofadr().begin(); iter != RArmJointsAddr.qdofadr().end(); ++iter) {
      measuredDq[idx++] = d->qvel[*iter];
    }
    return true;
  }

  void applyArmActuatorDynamicsCompensation(const kuavo_msgs::jointCmd::ConstPtr &msg, std::vector<double>& tau) {
    if (!actuatorDynamicsCompensator || tau.size() < static_cast<size_t>(numJoints) ||
        msg->joint_v.size() < static_cast<size_t>(numJoints)) {
      return;
    }

    const int headDof = static_cast<int>(HeadJointsAddr.qdofadr().size());
    const int armStartIndex = static_cast<int>(numJoints) - headDof - kArmCompensationDof;
    if (armStartIndex < 0 || armStartIndex + kArmCompensationDof > static_cast<int>(numJoints)) {
      return;
    }

    Eigen::VectorXd tauCmd = Eigen::VectorXd::Zero(kArmCompensationDof);
    Eigen::VectorXd dqCmd = Eigen::VectorXd::Zero(kArmCompensationDof);
    Eigen::VectorXd dqMeas = Eigen::VectorXd::Zero(kArmCompensationDof);
    const Eigen::VectorXd ddq = Eigen::VectorXd::Zero(kArmCompensationDof);

    for (int i = 0; i < kArmCompensationDof; ++i) {
      const int jointIndex = armStartIndex + i;
      tauCmd[i] = msg->tau[jointIndex];
      dqCmd[i] = msg->joint_v[jointIndex];
    }

    if (!buildArmCompensationMeasuredDq(dqMeas)) {
      dqMeas = dqCmd;
    }

    const Eigen::VectorXd compensatedTau = actuatorDynamicsCompensator->compute(tauCmd, ddq, dqCmd, dqMeas);
    if (compensatedTau.size() != kArmCompensationDof) {
      return;
    }
    for (int i = 0; i < kArmCompensationDof; ++i) {
      tau[armStartIndex + i] = compensatedTau[i];
    }
  }

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

      /* dexhand joint address */
      if(mj_name2id(mnew, mjOBJ_JOINT, "l_thumbCMC") != -1) {
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
    const double robot_x_dis = 0.232489;  // 机器人中心到轮子的距离
    const double robot_y_dis = 0.232489;  // 机器人中心到轮子的距离

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
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

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
          if (cmd_updated || is_chassic_cmd_changed || is_chassic_cmd_vel_changed || claw_cmd_updated || pure_sim)
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
              d->ctrl[i + 28] = claw_cmd[i];
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
                else if(robotVersion_ == 61 || robotVersion_ == 62 || robotVersion_ == 63)
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
            mj_step(m, d);
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
    // ****************************
    // mujocolcm.joinLCMThread();
    // ****************************
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
  applyArmActuatorDynamicsCompensation(msg, tau);
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
  d->xfrc_applied[6 * link_index + 3] = torque[0]; // 劳动 x
  d->xfrc_applied[6 * link_index + 4] = torque[1]; // 劳动 y
  d->xfrc_applied[6 * link_index + 5] = torque[2]; // 劳动 z
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

void PhysicsThread(mj::Simulate *sim, const char *filename, bool only_half_up_body = false)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
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
      qpos_init.resize(m->nq);
      std::fill(qpos_init.begin(), qpos_init.end(), 0);
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
      sim->Load(m, d, filename);
      mj_forward(m, d);

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
  bool camera_available = ConfigureDepthCameraForCurrentModel();
  if (camera_available) {
    depthImagePub = g_nh_ptr->advertise<sensor_msgs::Image>(mujoco_cpp::kDepthImageTopic, 10);
    depthImageArrayPub = g_nh_ptr->advertise<std_msgs::Float64MultiArray>(mujoco_cpp::kDepthImageArrayTopic, 10);
    depthHistoryPub = g_nh_ptr->advertise<std_msgs::Float64MultiArray>(mujoco_cpp::kDepthHistoryTopic, 10);
  }

  // // 创建服务
  ros::ServiceServer service = g_nh_ptr->advertiseService("sim_start", handleSimStart);

  // // 创建订阅器
  ros::Subscriber clawCmdSub = g_nh_ptr->subscribe("/leju_claw_command", 10, clawCmdCallback);
#ifndef USE_DDS
  ros::Subscriber jointCmdSub = g_nh_ptr->subscribe("/joint_cmd", 10, jointCmdCallback);
#endif
  ros::Subscriber extWrenchSub = g_nh_ptr->subscribe("/external_wrench", 10, extWrenchCallback);

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
                float* depth_data = reinterpret_cast<float*>(depth_msg.data.data());

                if (g_depth_camera->dist != nullptr) {
                    // const mjtNum range_inv = 1.0 / (DEPTH_CAMERA_MAX_RANGE - DEPTH_CAMERA_MIN_RANGE);
                    for (int v = 0; v < DEPTH_CAMERA_HEIGHT; ++v) {
                        for (int h = 0; h < DEPTH_CAMERA_WIDTH; ++h) {
                            int pixel_idx = v * DEPTH_CAMERA_WIDTH + h;
                            mjtNum dist = g_depth_camera->dist[pixel_idx];
                            // float norm = (dist - DEPTH_CAMERA_MIN_RANGE) * range_inv;
                            // norm = std::clamp(norm, 0.0f, 1.0f);
                            // depth_data[pixel_idx] = norm;
                            dist = std::clamp(dist, mjtNum(0), DEPTH_CAMERA_MAX_RANGE);
                            depth_data[pixel_idx] = dist / DEPTH_CAMERA_MAX_RANGE;
                        }
                    }
                }
  
                // Apply Gaussian blur
                cv::Mat depth_mat(DEPTH_CAMERA_HEIGHT, DEPTH_CAMERA_WIDTH, CV_32FC1, depth_data);
                cv::GaussianBlur(depth_mat, depth_mat, cv::Size(3, 3), 1, 1);

                // Update circular buffer with current frame
                std::unique_lock<std::mutex> buffer_lock(depth_buffer_mutex);
                depth_buffer[current_buffer_index].data.assign(depth_data, depth_data + DEPTH_CAMERA_HEIGHT * DEPTH_CAMERA_WIDTH);  // deep copy
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
                    depth_array_msg.data[i] = depth_data[i];
                }
                depthImagePub.publish(depth_msg);
                depthImageArrayPub.publish(depth_array_msg);

                // Publish depth history buffer
                // From 6*7+1=43 frames, take frame indices: 0, 6, 12, 18, 24, 30, 36, 42 (8 frames total)
                std::unique_lock<std::mutex> lock(depth_buffer_mutex);
                std::vector<int> selected_indices;
                // printf("buf filled:%d | ", depth_buffer_filled);
                for (int i = 0; i < 6 * 8; i += 6) {
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
      g_dexhand_node->init(*g_nh_ptr, m, RHandJointsAddr, LHandJointsAddr);

      int hand_joints_num = g_dexhand_node->get_hand_joints_num();
      g_nh_ptr->setParam("end_effector_joints_num", hand_joints_num);
  }
  else {
    g_nh_ptr->setParam("end_effector_joints_num", 0);
  }

  std::cout << "[mujoco_node]: waiting for init qpos" << std::endl;
  while (ros::ok())
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
          for (int i = 0; i < 7; i++)
          {
            qpos_init[i] = qpos_init_temp[i];
            std::cout << qpos_init[i] << ", ";
          }
          for (int i = 7 ; i < qpos_init_temp.size(); i++)
          {
            qpos_init[i + 8] = qpos_init_temp[i];
            std::cout << qpos_init[i + 8] << ", ";
          }
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
        qpos_init = {-0.00505, 0.00000, 0.84414, 0.99864, 0.00000, 0.05215, -0.00000,
                     -0.01825, -0.00190, -0.52421, 0.73860, -0.31872, 0.01835, 
                     0.01825, 0.00190, -0.52421, 0.73860, -0.31872, -0.01835, 
                     0, 0, 0, 0, 0, 0, 0, 
                     0, 0, 0, 0, 0, 0, 0};
        break;
      }
    }
    
    usleep(10000);
  }
  // 更新机器人初始状态,从rosparam获取
  InitRobotState(d);
  sim->Load(m, d, filename);
  mj_forward(m, d);

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
  
  ROS_INFO("Arm force application IDs: left=%d, right=%d", left_arm_link_id_, right_arm_link_id_);

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

  if (depth_thread.joinable()) {
      depth_thread_running.store(false);
      if (depth_thread.joinable()) {
          depth_thread.join();
      }
  }

  // delete everything we allocated

  // free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

//**************************
// run event loop
int simulate_loop(ros::NodeHandle &nh, bool spin_thread = false)
{
  // ros::init(argc, argv, "mujoco_sim");
  // ros::NodeHandle nh;
  is_spin_thread = spin_thread;
  g_nh_ptr = &nh;
  if (!actuatorDynamicsCompensator) {
    actuatorDynamicsCompensator = std::make_unique<mujoco_sim::ActuatorDynamicsCompensator>();
  }
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
    nh.getParam("robot_version", robotVersion_);
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
  sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);
  signal(SIGINT, signalHandler);
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

  return 0;
}
