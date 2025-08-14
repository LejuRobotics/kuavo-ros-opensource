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
#include "kuavo_msgs/Rq2f85ClawCmd.h"
#include "kuavo_msgs/Rq2f85ClawState.h"
#include "kuavo_msgs/SetObjectPosition.h"
#include <random>
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <csignal>
#include <atomic>
#include <queue>
#include <sensor_msgs/JointState.h>
#include "joint_address.hpp"
#include "dexhand_mujoco_node.h"
#include "dexhand/json.hpp"
#include "geometry_msgs/Vector3Stamped.h"

#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
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

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length
  double frequency = 1000.0;             // simulation frequency (Hz)
  ros::Publisher sensorsPub;
  ros::Publisher pubOdom;
  ros::Publisher pubTimeDiff;

  std::queue<std::vector<double>> controlCommands;
  std::vector<double> joint_tau_cmd;
  bool cmd_updated = false;

  geometry_msgs::Wrench external_wrench_;
  bool external_wrench_updated_ = false;

  std::mutex queueMutex;
  ros::NodeHandle *g_nh_ptr;
  size_t numJoints = 12;      /* LLeg+RLeg+LArm+RArm+Head (without the dexhand joints) */
  double is_spin_thread = true;
  ros::Time sim_time;
  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;
  std::vector<double> qpos_init;


  // 夹爪控制变量 - 合并版本
  struct GripperCommand {
    double left_cmd = 0.0;   // 左夹爪命令 (0-255)
    double right_cmd = 0.0;  // 右夹爪命令 (0-255)
    bool updated = false;
  };

  struct GripperState {
    double left_position = 0.0;   // 左夹爪当前位置
    double left_velocity = 0.0;   // 左夹爪当前速度
    double left_force = 0.0;      // 左夹爪当前力
    double right_position = 0.0;  // 右夹爪当前位置
    double right_velocity = 0.0;  // 右夹爪当前速度
    double right_force = 0.0;     // 右夹爪当前力
  };

  GripperCommand gripper_cmd;
  GripperState gripper_state;
  std::mutex gripper_mutex;
  
  // 夹爪发布器和订阅器 - 合并版本
  ros::Publisher gripperStatePub;

  
  // 夹爪执行器地址
  int left_gripper_actuator_id = -1;
  int right_gripper_actuator_id = -1;

  // 夹爪关节ID（用于读取状态）- 使用driver joint作为主要状态指示器
  int left_right_driver_joint_id = -1;  // 左夹爪右侧driver关节
  int left_left_driver_joint_id = -1;   // 左夹爪左侧driver关节
  int right_right_driver_joint_id = -1; // 右夹爪右侧driver关节
  int right_left_driver_joint_id = -1;  // 右夹爪左侧driver关节

  // tendon ID（用于读取力）
  int left_tendon_id = -1;  // split1 tendon
  int right_tendon_id = -1; // split2 tendon


  // 传送带控制变量
  double belt_speed_cmd = 0.0;      // 传送带速度命令 (-0.1 到 0.1 m/s)
  bool belt_speed_updated = false;
  std::mutex belt_mutex;
  
  // 传送带执行器地址
  int belt_actuator_id = -1;

  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<std::string> body_names_to_publish = {
  "box_grab", "shampoo1", "shampoo2", "shampoo3", "shampoo4", "item1", "item2"
  };

  std::unordered_map<std::string, ros::Publisher> body_pose_publishers;
  // ******
  low_cmd_t recvCmd;
  // ******

  // control noise variables
  // mjtNum* ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  /************************************* Joint Address******************************************/
  // This section defines the joint addresses for various body parts of the robot.
  using namespace mujoco_node;
  JointGroupAddress LLegJointsAddr("l_leg_joints");
  JointGroupAddress RLegJointsAddr("r_leg_joints");
  JointGroupAddress LArmJointsAddr("r_arm_joints");
  JointGroupAddress RArmJointsAddr("r_arm_joints");
  JointGroupAddress HeadJointsAddr("head_joints");
  JointGroupAddress LHandJointsAddr("l_hand_joints");
  JointGroupAddress RHandJointsAddr("r_hand_joints");
  // JointGroupAddress GRIPJointsAddr("grip_joints");
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
  void init_additional_actuators(mjModel* model) {
    // 获取夹爪执行器ID
    left_gripper_actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "left_fingers_actuator");
    right_gripper_actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "right_fingers_actuator");
    belt_actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "belt_speed");

    // 获取夹爪driver关节ID（用于读取位置和速度状态）
    left_right_driver_joint_id = mj_name2id(model, mjOBJ_JOINT, "left_right_driver_joint");
    left_left_driver_joint_id = mj_name2id(model, mjOBJ_JOINT, "left_left_driver_joint");
    right_right_driver_joint_id = mj_name2id(model, mjOBJ_JOINT, "right_right_driver_joint");
    right_left_driver_joint_id = mj_name2id(model, mjOBJ_JOINT, "right_left_driver_joint");
    
    // 获取tendon ID（用于读取力）
    left_tendon_id = mj_name2id(model, mjOBJ_TENDON, "split1");
    right_tendon_id = mj_name2id(model, mjOBJ_TENDON, "split2");

    if (left_gripper_actuator_id >= 0) {
      std::cout << "\033[32mLeft gripper actuator found, ID: " << left_gripper_actuator_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Left gripper actuator not found!\033[0m" << std::endl;
    }
    
    if (right_gripper_actuator_id >= 0) {
      std::cout << "\033[32mRight gripper actuator found, ID: " << right_gripper_actuator_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Right gripper actuator not found!\033[0m" << std::endl;
    }

    if (left_right_driver_joint_id >= 0 && left_left_driver_joint_id >= 0) {
      std::cout << "\033[32mLeft gripper driver joints found, IDs: " << left_right_driver_joint_id 
                << ", " << left_left_driver_joint_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Left gripper driver joints not found!\033[0m" << std::endl;
    }
    
    if (right_right_driver_joint_id >= 0 && right_left_driver_joint_id >= 0) {
      std::cout << "\033[32mRight gripper driver joints found, IDs: " << right_right_driver_joint_id 
                << ", " << right_left_driver_joint_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Right gripper driver joints not found!\033[0m" << std::endl;
    }
    
    if (left_tendon_id >= 0) {
      std::cout << "\033[32mLeft gripper tendon found, ID: " << left_tendon_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Left gripper tendon not found!\033[0m" << std::endl;
    }
    
    if (right_tendon_id >= 0) {
      std::cout << "\033[32mRight gripper tendon found, ID: " << right_tendon_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Right gripper tendon not found!\033[0m" << std::endl;
    }

      if (belt_actuator_id >= 0) {
      std::cout << "\033[32mBelt actuator found, ID: " << belt_actuator_id << "\033[0m" << std::endl;
    } else {
      std::cout << "\033[31mWarning: Belt actuator not found!\033[0m" << std::endl;
    }
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
      init_joint_address(mnew, LLegJointsAddr, "leg_l1_joint", "leg_l6_joint");
      init_joint_address(mnew, RLegJointsAddr, "leg_r1_joint", "leg_r6_joint");
      init_joint_address(mnew, LArmJointsAddr, "zarm_l1_joint", "zarm_l7_joint");

      init_joint_address(mnew, RArmJointsAddr, "zarm_r1_joint", "zarm_r7_joint");
      init_joint_address(mnew, HeadJointsAddr, "zhead_1_joint", "zhead_2_joint");

      // init_joint_address(mnew, GRIPJointsAddr, "right_driver_joint", "left_follower_joint");
      /* dexhand joint address */
      if(mj_name2id(mnew, mjOBJ_JOINT, "l_thumbCMC") != -1) {
        init_joint_address(mnew, LHandJointsAddr, "l_thumbCMC", "l_littlePIP");
        init_joint_address(mnew, RHandJointsAddr, "r_thumbCMC", "r_littlePIP");
      }
      init_additional_actuators(mnew);

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
    for (int i = 0; i < 49; i++)
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


  // 添加夹爪状态读取函数（在publish_ros_data函数之前）
  void read_gripper_states(const mjData *d) {
      // 读取左夹爪状态 - 使用右侧driver关节作为主要指示器
      if (left_right_driver_joint_id >= 0) {
          int qpos_addr = m->jnt_qposadr[left_right_driver_joint_id];
          int dof_addr = m->jnt_dofadr[left_right_driver_joint_id];
          
          if (qpos_addr >= 0 && qpos_addr < m->nq) {
              gripper_state.left_position = d->qpos[qpos_addr];
          }
          
          if (dof_addr >= 0 && dof_addr < m->nv) {
              gripper_state.left_velocity = d->qvel[dof_addr];
          }
      }
      
      // 读取左夹爪tendon力
      if (left_tendon_id >= 0 && left_tendon_id < m->ntendon) {
          gripper_state.left_force = d->actuator_force[left_gripper_actuator_id];
      }
      
      // 读取右夹爪状态 - 使用右侧driver关节作为主要指示器  
      if (right_right_driver_joint_id >= 0) {
          int qpos_addr = m->jnt_qposadr[right_right_driver_joint_id];
          int dof_addr = m->jnt_dofadr[right_right_driver_joint_id];
          
          if (qpos_addr >= 0 && qpos_addr < m->nq) {
              gripper_state.right_position = d->qpos[qpos_addr];
          }
          
          if (dof_addr >= 0 && dof_addr < m->nv) {
              gripper_state.right_velocity = d->qvel[dof_addr];
          }
      }
      
      // 读取右夹爪tendon力
      if (right_tendon_id >= 0 && right_tendon_id < m->ntendon) {
          gripper_state.right_force = d->actuator_force[right_gripper_actuator_id];
      }
  }

  // 添加夹爪状态发布函数（在publish_ros_data函数之前）
  void publish_gripper_states() {
    kuavo_msgs::Rq2f85ClawState gripper_state_msg;
    
    // 设置消息头
    gripper_state_msg.header.stamp = ros::Time::now();
    gripper_state_msg.header.frame_id = "gripper";
    
    // 填充左夹爪状态
    gripper_state_msg.left_position = gripper_state.left_position;
    gripper_state_msg.left_velocity = gripper_state.left_velocity;
    gripper_state_msg.left_force = gripper_state.left_force;
    
    // 填充右夹爪状态
    gripper_state_msg.right_position = gripper_state.right_position;
    gripper_state_msg.right_velocity = gripper_state.right_velocity;
    gripper_state_msg.right_force = gripper_state.right_force;
    
    // 发布状态消息
    gripperStatePub.publish(gripper_state_msg);
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
    updateJointData(LLegJointsAddr);
    updateJointData(RLegJointsAddr);
    updateJointData(LArmJointsAddr);
    updateJointData(RArmJointsAddr);
    updateJointData(HeadJointsAddr);
    // updateJointData(GRIPJointsAddr);
    // Dexhand: read state
    if(g_dexhand_node) {
      g_dexhand_node->readCallback(d);
    }
    
    kuavo_msgs::imuData imu_data;
    nav_msgs::Odometry bodyOdom;
    int pos_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyPos")];
    int ori_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyQuat")];
    int gyro_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyGyro")];
    int acc_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyAcc")];
    // std::cout << "pos_addr: " << pos_addr << std::endl;
    // std::cout << "ori_addr: " << ori_addr << std::endl;
    // std::cout << "gyro_addr: " << gyro_addr << std::endl;
    // std::cout << "acc_addr: " << acc_addr << std::endl;
    mjtNum *pos = d->sensordata + pos_addr;
    mjtNum *ori = d->sensordata + ori_addr;

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
    sensorsPub.publish(sensors_data);

    // bodyOdom = Odometry();
    bodyOdom.header.stamp = sim_time;
    bodyOdom.pose.pose.position.x = pos[0];
    bodyOdom.pose.pose.position.y = pos[1];
    bodyOdom.pose.pose.position.z = pos[2];
    bodyOdom.pose.pose.orientation.x = ori[1];
    bodyOdom.pose.pose.orientation.y = ori[2];
    bodyOdom.pose.pose.orientation.z = ori[3];
    bodyOdom.pose.pose.orientation.w = ori[0];
    bodyOdom.twist.twist.linear.x = d->qvel[0];
    bodyOdom.twist.twist.linear.y = d->qvel[1];
    bodyOdom.twist.twist.linear.z = d->qvel[2];

    bodyOdom.twist.twist.angular.x = angVel[0];
    bodyOdom.twist.twist.angular.y = angVel[1];
    bodyOdom.twist.twist.angular.z = angVel[2];
    pubOdom.publish(bodyOdom);

    for (const auto& name : body_names_to_publish) {
      int id = mj_name2id(m, mjOBJ_BODY, name.c_str());
      if (id < 0) {
        continue; // body 不存在就跳过
      }

      // 获取位置
      const mjtNum* pos = d->xpos + 3 * id;
      const mjtNum* quat = d->xquat + 4 * id;

      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "odom";
      msg.pose.position.x = pos[0];
      msg.pose.position.y = pos[1];
      msg.pose.position.z = pos[2];
      msg.pose.orientation.w = quat[0];
      msg.pose.orientation.x = quat[1];
      msg.pose.orientation.y = quat[2];
      msg.pose.orientation.z = quat[3];

      body_pose_publishers[name].publish(msg);
    }
    read_gripper_states(d);
    publish_gripper_states();
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
    queueMutex.unlock();
    uint64_t step_count = 0;
    sim_time = ros::Time::now();
    // run until asked to exit
    ros::Rate loop_rate(frequency);
    while (!sim.exitrequest.load())
    {
      // if (sim.droploadrequest.load()) {

      //   sim.LoadMessage(sim.dropfilename);
      //   mjModel* mnew = LoadModel(sim.dropfilename, sim);
      //   sim.droploadrequest.store(false);

      //   mjData* dnew = nullptr;
      //   if (mnew) dnew = mj_makeData(mnew);
      //   if (dnew) {
      //     sim.Load(mnew, dnew, sim.dropfilename);

      //     mj_deleteData(d);
      //     mj_deleteModel(m);

      //     m = mnew;
      //     d = dnew;

      //   // ********************************
      //     InitRobotState(d);
      //   // ********************************
      //     mj_forward(m, d);

      //     // allocate ctrlnoise
      //     free(ctrlnoise);
      //     ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
      //     mju_zero(ctrlnoise, m->nu);
      //   } else {
      //     sim.LoadMessageClear();
      //   }
      // }

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

          // allocate ctrlnoise
          // free(ctrlnoise);
          // ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
          // mju_zero(ctrlnoise, m->nu);
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
        joint_tau_cmd = std::vector<double>(numJoints, 0);
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
          if (sim.run)
          {
            bool stepped = false;

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
            queueMutex.lock();
            // cmd_updated = !controlCommands.empty();
            // if (cmd_updated)
            // {
            //   tau_cmd = controlCommands.front(); // 按顺序取值
            //   // tau_cmd = controlCommands.back(); // 取最新的值
            //   controlCommands.pop();
            // }
            updated = cmd_updated;
            cmd_updated = false;
            tau_cmd = joint_tau_cmd;
            queueMutex.unlock();
            if (updated)
            {
              std::lock_guard<std::mutex> gripper_lock(gripper_mutex);
              if (gripper_cmd.updated) {
                // 左夹爪控制
                if (left_gripper_actuator_id >= 0) {
                  d->ctrl[left_gripper_actuator_id] = gripper_cmd.left_cmd;
                }
                // 右夹爪控制
                if (right_gripper_actuator_id >= 0) {
                  d->ctrl[right_gripper_actuator_id] = gripper_cmd.right_cmd;
                }
                gripper_cmd.updated = false;
              }
              std::lock_guard<std::mutex> belt_lock(belt_mutex);
              if (belt_speed_updated && belt_actuator_id >= 0) {
                d->ctrl[belt_actuator_id] = belt_speed_cmd;
                belt_speed_updated = false;
              }
              // update actuators/controls
              auto updateControl = [&](const JointGroupAddress& jointAddr, int &i) {
                  for (auto iter = jointAddr.ctrladr().begin(); iter != jointAddr.ctrladr().end(); iter++) {
                      d->ctrl[*iter] = tau_cmd[i++];
                  }
              };
              int i = 0;
              updateControl(LLegJointsAddr, i);
              updateControl(RLegJointsAddr, i);
              updateControl(LArmJointsAddr, i);
              updateControl(RArmJointsAddr, i);
              updateControl(HeadJointsAddr, i);
              // updateControl(GRIPJointsAddr, i);
              // Dexhand: ctrl/command
              if(g_dexhand_node) {
                g_dexhand_node->writeCallback(d);
              }

              mj_step(m, d);
              step_count++;
              sim_time += ros::Duration(1 / frequency);
              sim.AddToHistory();
            }
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
            //   stepped = true;
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
          else // paused
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
          publish_ros_data(d, sim.run);
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
      std::cerr << "jointCmdCallback Error: joint_q, joint_v, tau, tau_ratio, control_modes, joint_kp, joint_kd size not match!" << std::endl;
      std::cerr << "desire size:" << numJoints << std::endl;
      std::cerr << "joint_q size:" << msg->joint_q.size() << std::endl;
      std::cerr << "joint_v size:" << msg->joint_v.size() << std::endl;
      std::cerr << "tau size:" << msg->tau.size() << std::endl;
      std::cerr << "tau_ratio size:" << msg->tau_ratio.size() << std::endl;
      std::cerr << "control_modes size:" << msg->control_modes.size() << std::endl;
      std::cerr << "tau_max size:" << msg->tau_max.size() << std::endl;
      std::cerr << "joint_kp size:" << msg->joint_kp.size() << std::endl;
      std::cerr << "joint_kd size:" << msg->joint_kd.size() << std::endl;
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

// 添加设置物体位置的服务回调函数
bool setObjectPositionCallback(kuavo_msgs::SetObjectPosition::Request &req,
                                kuavo_msgs::SetObjectPosition::Response &res)
{
  if (!m || !d) {
    res.success = false;
    res.message = "MuJoCo model or data not initialized";
    ROS_ERROR("MuJoCo model or data not initialized");
    return true;
  }

  // 查找物体ID
  int body_id = mj_name2id(m, mjOBJ_BODY, req.object_name.c_str());
  if (body_id < 0) {
    res.success = false;
    res.message = "Object '" + req.object_name + "' not found in the model";
    ROS_ERROR("Object '%s' not found in the model", req.object_name.c_str());
    return true;
  }

  // 获取物体在qpos中的地址
  int joint_id = mj_name2id(m, mjOBJ_JOINT, req.object_name.c_str());
  if (joint_id < 0) {
    res.success = false;
    res.message = "Joint for object '" + req.object_name + "' not found. Object might be fixed.";
    ROS_ERROR("Joint for object '%s' not found. Object might be fixed.", req.object_name.c_str());
    return true;
  }

  int qpos_addr = m->jnt_qposadr[joint_id];
  if (qpos_addr < 0) {
    res.success = false;
    res.message = "Invalid qpos address for object '" + req.object_name + "'";
    ROS_ERROR("Invalid qpos address for object '%s'", req.object_name.c_str());
    return true;
  }

  try {
    // 锁定仿真以安全修改
    const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
    
    double x, y, z;
    double qw, qx, qy, qz;
    
    if (req.randomize) {
      // 使用随机位置
      std::uniform_real_distribution<double> x_dist(req.x_min, req.x_max);
      std::uniform_real_distribution<double> y_dist(req.y_min, req.y_max);
      std::uniform_real_distribution<double> z_dist(req.z_min, req.z_max);
      
      x = x_dist(gen);
      y = y_dist(gen);
      z = z_dist(gen);
      
      // 如果没有指定姿态，保持原有姿态
      if (req.orientation.w == 0 && req.orientation.x == 0 && 
          req.orientation.y == 0 && req.orientation.z == 0) {
        qw = d->qpos[qpos_addr + 3];
        qx = d->qpos[qpos_addr + 4];
        qy = d->qpos[qpos_addr + 5];
        qz = d->qpos[qpos_addr + 6];
      } else {
        qw = req.orientation.w;
        qx = req.orientation.x;
        qy = req.orientation.y;
        qz = req.orientation.z;
      }
    } else {
      // 使用指定位置
      x = req.position.x;
      y = req.position.y;
      z = req.position.z;
      
      if (req.orientation.w == 0 && req.orientation.x == 0 && 
          req.orientation.y == 0 && req.orientation.z == 0) {
        // 保持原有姿态
        qw = d->qpos[qpos_addr + 3];
        qx = d->qpos[qpos_addr + 4];
        qy = d->qpos[qpos_addr + 5];
        qz = d->qpos[qpos_addr + 6];
      } else {
        qw = req.orientation.w;
        qx = req.orientation.x;
        qy = req.orientation.y;
        qz = req.orientation.z;
      }
    }

    // 设置位置 (x, y, z)
    d->qpos[qpos_addr + 0] = x;
    d->qpos[qpos_addr + 1] = y;
    d->qpos[qpos_addr + 2] = z;
    
    // 设置姿态 (quaternion: w, x, y, z)
    d->qpos[qpos_addr + 3] = qw;
    d->qpos[qpos_addr + 4] = qx;
    d->qpos[qpos_addr + 5] = qy;
    d->qpos[qpos_addr + 6] = qz;
    
    // 清零速度
    int dof_addr = m->jnt_dofadr[joint_id];
    if (dof_addr >= 0) {
      for (int i = 0; i < 6; i++) {  // 6DOF (3 linear + 3 angular)
        if (dof_addr + i < m->nv) {
          d->qvel[dof_addr + i] = 0.0;
        }
      }
    }
    
    // 更新仿真状态
    mj_forward(m, d);
    
    // 准备响应
    res.success = true;
    res.message = "Object '" + req.object_name + "' position updated successfully";
    res.final_position.x = x;
    res.final_position.y = y;
    res.final_position.z = z;
    
    ROS_INFO("Object '%s' moved to position (%.3f, %.3f, %.3f) with quaternion (%.3f, %.3f, %.3f, %.3f)", 
              req.object_name.c_str(), x, y, z, qw, qx, qy, qz);
              
  } catch (const std::exception& e) {
    res.success = false;
    res.message = "Error updating object position: " + std::string(e.what());
    ROS_ERROR("Error updating object position: %s", e.what());
  }
  
  return true;
}


void gripperCommandCallback(const kuavo_msgs::Rq2f85ClawCmd::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(gripper_mutex);
  
  // 解析并限制命令范围
  gripper_cmd.left_cmd = std::max(0.0, std::min(255.0, msg->left_cmd));
  gripper_cmd.right_cmd = std::max(0.0, std::min(255.0, msg->right_cmd));
  gripper_cmd.updated = true;
  
  ROS_INFO("Gripper commands received - Left: %.2f, Right: %.2f (timestamp: %f)", 
           gripper_cmd.left_cmd, gripper_cmd.right_cmd, msg->header.stamp.toSec());
}
void beltSpeedCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(belt_mutex);
  belt_speed_cmd = std::max(-0.1, std::min(0.1, msg->vector.x)); // 限制范围 -0.1 到 0.1 m/s
  belt_speed_updated = true;
  ROS_INFO("Belt speed command: %.3f m/s", belt_speed_cmd);
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

//-----------------------m--------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    m->opt.timestep = 1 / frequency;
    
    // Init numJoints
    numJoints = 0;
    std::cout << "LLeg joints size: " << LLegJointsAddr.qdofadr().size() << std::endl;
    std::cout << "RLeg joints size: " << RLegJointsAddr.qdofadr().size() << std::endl;
    std::cout << "LArm joints size: " << LArmJointsAddr.qdofadr().size() << std::endl;
    std::cout << "RArm joints size: " << RArmJointsAddr.qdofadr().size() << std::endl;
    std::cout << "Head joints size: " << HeadJointsAddr.qdofadr().size() << std::endl;
    // std::cout << "GRIP joints size: " << GRIPJointsAddr.qdofadr().size() << std::endl;
    numJoints += LLegJointsAddr.qdofadr().size();
    numJoints += RLegJointsAddr.qdofadr().size();
    numJoints += LArmJointsAddr.qdofadr().size();
    numJoints += RArmJointsAddr.qdofadr().size();
    numJoints += HeadJointsAddr.qdofadr().size();
    // numJoints += GRIPJointsAddr.qdofadr().size();
    std::cout << "\033[32mnumJoints: " << (m->nq - 7) << "\033[0m" << std::endl;
    std::cout << "\033[32mnumJoints(without dexhand): " << numJoints << "\033[0m" << std::endl;

    if (d)
    {
      // ********************************
      init_cmd(d);
      qpos_init.resize(m->nq);
      std::fill(qpos_init.begin(), qpos_init.end(), 0);
      qpos_init[2] = 0.99;// 初始化位置
      InitRobotState(d);
      // ********************************
      sim->Load(m, d, filename);
      mj_forward(m, d);

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
  pubOdom = g_nh_ptr->advertise<nav_msgs::Odometry>("/ground_truth/state", 10);
  pubTimeDiff = g_nh_ptr->advertise<std_msgs::Float64>("/monitor/time_cost/mujoco_loop_time", 10);

    // 添加夹爪状态发布器
  gripperStatePub = g_nh_ptr->advertise<kuavo_msgs::Rq2f85ClawState>("/gripper/state", 10);
  // // 创建服务
  ros::ServiceServer service = g_nh_ptr->advertiseService("sim_start", handleSimStart);
  ros::ServiceServer setObjectPositionService = g_nh_ptr->advertiseService("set_object_position", setObjectPositionCallback);
  // // 创建订阅器
  ros::Subscriber jointCmdSub = g_nh_ptr->subscribe("/joint_cmd", 10, jointCmdCallback);
  ros::Subscriber extWrenchSub = g_nh_ptr->subscribe("/external_wrench", 10, extWrenchCallback);

  ros::Subscriber gripperCmdSub = g_nh_ptr->subscribe<kuavo_msgs::Rq2f85ClawCmd>("/gripper/command", 10, gripperCommandCallback);
  ros::Subscriber beltSpeedSub = g_nh_ptr->subscribe<geometry_msgs::Vector3Stamped>("/belt/speed_command", 10, beltSpeedCallback);

  for (const auto& name : body_names_to_publish) {
  std::string topic_name = "/mujoco/" + name + "/pose";
  body_pose_publishers[name] = g_nh_ptr->advertise<geometry_msgs::PoseStamped>(topic_name, 10);
  }

  // 初始化灵巧手ROS
  if(!RHandJointsAddr.ctrladr().invalid()) {
      std::cout << "[mujoco_node]: init dexhand node" << std::endl;
      g_dexhand_node = std::make_shared<DexHandMujocoRosNode>();
      g_dexhand_node->init(*g_nh_ptr, m, RHandJointsAddr, LHandJointsAddr);

      int hand_joints_num = g_dexhand_node->get_hand_joints_num();
      g_nh_ptr->setParam("end_effector_joints_num", hand_joints_num);
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

        for (int i = 0; i < qpos_init_temp.size(); i++)
        {
          qpos_init[i] = qpos_init_temp[i];
          std::cout << qpos_init_temp[i] << ", ";
        }
        std::cout << std::endl;
        break;
      }
      else
      {
        ROS_INFO("[mujoco_node]Failed to get init qpos, use default qpos");
        qpos_init = {-0.00505, 0.00000, 0.84414, 0.99864, 0.00000, 0.05215, -0.00000,
                     -0.01825, -0.00190, -0.52421, 0.73860, -0.31872, 0.01835, 
                     0.01825, 0.00190, -0.52421, 0.73860, -0.31872, -0.01835, 
                     0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0, 
                     0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0};
        break;
      }
    }
    
    usleep(10000);
  }
  // 更新机器人初始状态,从rosparam获取
  InitRobotState(d);
  sim->Load(m, d, filename);
  mj_forward(m, d);

  ros::Subscriber lHandExtWrenchSub = g_nh_ptr->subscribe<geometry_msgs::Wrench>("/external_wrench/left_hand", 10, [&](const geometry_msgs::Wrench::ConstPtr &msg)
      {
        apply_wrench_to_link(m, d, "zarm_l7_link", &msg->force.x, &msg->torque.x);
      }
    );  
  ros::Subscriber rHandExtWrenchSub = g_nh_ptr->subscribe<geometry_msgs::Wrench>("/external_wrench/right_hand", 10, [&](const geometry_msgs::Wrench::ConstPtr &msg)
      {
        apply_wrench_to_link(m, d, "zarm_r7_link", &msg->force.x, &msg->torque.x);
      }
    );


  if (is_spin_thread)
  {
    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();
  }

  PhysicsLoop(*sim);

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
      } catch (const std::exception& e) {
        ROS_ERROR("Error parsing configuration file: %s", e.what());
      }
    }
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
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
