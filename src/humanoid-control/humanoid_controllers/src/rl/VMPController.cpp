// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/VMPController.h"
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace {
// Match Isaac Lab ObsTerm and sim2sim: clip raw values, then scale per term.
Eigen::VectorXd applyVmpObsTerm(const Eigen::VectorXd& raw, double scale, double clip_val)
{
  return raw.cwiseMax(-clip_val).cwiseMin(clip_val) * scale;
}

Eigen::Vector3d applyVmpObsTerm(const Eigen::Vector3d& raw, double scale, double clip_val)
{
  Eigen::Vector3d clipped = raw.cwiseMax(-clip_val).cwiseMin(clip_val);
  return clipped * scale;
}
}  // namespace

// VAE全局归一化系数 - 与Python vae_module.py对齐
// v52格式：79维 = h(1) + theta(6) + v(6) + q(27) + q_dot(27) + p(12)
namespace vae_normalization {

// 全局均值 (GLOBAL_MEAN from vae_module.py)
static const std::vector<float> GLOBAL_MEAN_V52 = {
    0.922154f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  // theta
    0.02669f, -0.000877f, 0.000166f,      // linear_vel
    0.00087f, 0.00156f, 0.019071f,        // angular_vel
    0.04089f, 0.072492f, -0.349686f, 0.523682f, -0.218686f, -0.01539f,  // q legs left
    -0.050247f, -0.118396f, -0.367069f, 0.545694f, -0.188467f, 0.026756f,  // q legs right
    -0.011927f,  // q waist
    -0.078394f, 0.318945f, -0.214299f, -0.914104f, -0.030327f, -0.093376f, -0.01191f,  // q arm left
    -0.059819f, -0.338589f, 0.196448f, -0.94464f, 0.018283f, 0.123407f, -0.00647f,  // q arm right
    0.000178f, -0.0003f, -0.001319f, 0.002652f, -0.001509f, -0.000588f,  // q_dot legs left
    -0.000038f, -0.000349f, -0.000949f, 0.002098f, -0.00105f, 0.000447f,   // q_dot legs right
    0.000058f,  // q_dot waist
    -0.000058f, 0.000871f, 0.000555f, -0.004277f, -0.000668f, -0.000148f, 0.000856f,  // q_dot arm left
    -0.000069f, -0.000669f, -0.000882f, -0.003939f, 0.000806f, -0.000175f, 0.001084f,  // q_dot arm right
    -0.027083f, 0.094369f, -0.893601f,   // p feet left
    -0.023245f, -0.093565f, -0.892527f,  // p feet right
    0.087382f, 0.214246f, 0.007755f,     // p hand left
    0.067048f, -0.228547f, 0.01237f      // p hand right
};

// 全局标准差 (GLOBAL_STD from vae_module.py)
static const std::vector<float> GLOBAL_STD_V52 = {
    0.120294f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,  // theta (不归一化)
    0.51704f, 0.371453f, 0.175803f,       // linear_vel
    0.288322f, 0.311712f, 0.796679f,      // angular_vel
    0.126931f, 0.19857f, 0.38367f, 0.426874f, 0.186654f, 0.098035f,  // q legs left
    0.128399f, 0.191471f, 0.374976f, 0.422122f, 0.204839f, 0.093688f,  // q legs right
    0.257567f,  // q waist
    0.490215f, 0.340265f, 0.482016f, 0.654673f, 0.446165f, 0.26498f, 0.193018f,  // q arm left
    0.524628f, 0.323423f, 0.504285f, 0.649145f, 0.395095f, 0.246685f, 0.177818f,  // q arm right
    0.484252f, 0.605269f, 1.059297f, 1.437825f, 0.738639f, 0.496328f,  // q_dot legs left
    0.503498f, 0.604872f, 1.043878f, 1.455302f, 0.771837f, 0.489199f,   // q_dot legs right
    0.784581f,  // q_dot waist
    1.035726f, 0.823066f, 1.010207f, 1.268215f, 0.767943f, 0.546597f, 0.345311f,  // q_dot arm left
    1.002036f, 0.773281f, 1.021613f, 1.211352f, 0.735731f, 0.528987f, 0.33775f,   // q_dot arm right
    0.154139f, 0.140855f, 0.133042f,   // p feet left
    0.155033f, 0.140945f, 0.134038f,   // p feet right
    0.222893f, 0.258294f, 0.194389f,   // p hand left
    0.231892f, 0.260478f, 0.192215f    // p hand right
};

}  // namespace vae_normalization

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace ocs2::humanoid;

  VMPController::VMPController(const std::string& name,
                               const std::string& config_file,
                               ros::NodeHandle& nh,
                               TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::VMP_CONTROLLER, config_file, nh, ros_logger)
  {
    ROS_INFO("[%s] VMPController constructed", name_.c_str());
  }

  VMPController::~VMPController()
  {
    stopOnlineSampling();
    ROS_INFO("[%s] VMPController destroyed", name_.c_str());
  }

  bool VMPController::initialize()
  {
    ROS_INFO("[%s] Initializing VMP controller...", name_.c_str());
    
    // 获取控制周期
    double wbc_frequency = 500.0;
    if (!nh_.getParam("/wbc_frequency", wbc_frequency))
    {
      ROS_WARN("[%s] /wbc_frequency not found, using default: %.1f Hz", name_.c_str(), wbc_frequency);
    }
    dt_ = 1.0 / wbc_frequency;
    
    // 尝试加载RL滤波参数（可选，VMP可能不需要IMU滤波器）
    try {
      loadRLFilterParams(config_file_);
    } catch (const std::exception& e) {
      ROS_WARN("[%s] Failed to load RL IMU filter params from %s: %s", 
               name_.c_str(), config_file_.c_str(), e.what());
      // VMP不需要IMU滤波器也可以工作，所以这不是致命错误
    }
    
    // 加载配置
    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] Failed to load config", name_.c_str());
      return false;
    }
    
    // 获取基础参数（与 AmpWalkController/FallStandController 一致）
    nh_.getParam("/is_real", is_real_);
    if (!nh_.getParam("/is_roban", is_roban_))
    {
      ROS_WARN("[%s] /is_roban not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_roban_));
    }
    
    // 初始化AnkleSolver
    std::string ankle_solver_type = "5gen";
    nh_.getParam("/ankle_solver_type", ankle_solver_type);
    ankleSolver_.getconfig(ankle_solver_type);
    
    ROS_INFO("[%s] Robot config: is_real=%d, is_roban=%d, jointNum=%d, jointArmNum=%d, waistNum=%d, headNum=%d",
             name_.c_str(), is_real_, is_roban_, jointNum_, jointArmNum_, waistNum_, headNum_);
    
    // 初始化 Ruiwo 电机参数切换服务客户端（仅在真实机器人上且使能 VMP 专用增益时使用）
    if (is_real_ && use_vmp_ruiwo_kpkd_)
    {
      srv_change_motor_param_ = nh_.serviceClient<kuavo_msgs::ExecuteArmAction>("/hardware/change_ruiwo_motor_param");
      ROS_INFO("[%s] Ruiwo motor param service client initialized (vmp_kpkd)", name_.c_str());
    }
    
    // 初始化VMP
    if (!initializeVMP())
    {
      ROS_ERROR("[%s] Failed to initialize VMP", name_.c_str());
      return false;
    }

    // 初始化手臂控制（可选功能，需在 initializeVMP 之后，vmp_jointKp_/Kd_ 已就绪）
    std::string urdf_path;
    if (nh_.getParam("/urdfFile", urdf_path))
    {
      ROS_INFO("[%s] Using URDF path from ROS param /urdfFile: %s", name_.c_str(), urdf_path.c_str());
    }
    else
    {
      int robot_version_int = 45;
      nh_.param("/robot_version", robot_version_int, 45);
      int major = robot_version_int / 10;
      int minor = robot_version_int % 10;
      std::string package_path = ros::package::getPath("kuavo_assets");
      std::string version_str = "biped_s" + std::to_string(major) + std::to_string(minor);
      urdf_path = package_path + "/models/" + version_str + "/urdf/" + version_str + ".urdf";
      ROS_INFO("[%s] Constructed URDF path from robot_version: %s", name_.c_str(), urdf_path.c_str());
    }
    initArmControl(urdf_path);
    
    vmp_initialized_ = true;
    initialized_ = true;
    
    ROS_INFO("[%s] VMP controller initialized successfully", name_.c_str());
    return true;
  }

  bool VMPController::loadConfig(const std::string& config_file)
  {
    ROS_INFO("[%s] Loading VMP config from: %s", name_.c_str(), config_file.c_str());
    
    try {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(config_file, pt);
      
      // 从配置文件路径提取基础模型路径
      // 配置文件路径格式: .../config/kuavo_vXX/vmp/vmp_param.info
      // baseModelPath_ 应该是配置文件所在的目录: .../config/kuavo_vXX/vmp
      // 这样不同的机器人版本可以使用不同的模型和参考轨迹
      boost::filesystem::path config_path(config_file);
      baseModelPath_ = config_path.parent_path().string();
      ROS_INFO("[%s] VMP base model path: %s", name_.c_str(), baseModelPath_.c_str());
      
      // 加载关节数量（配置文件可选，优先使用 ROS 参数）
      // 注意：jointNum_ 和 jointArmNum_ 已经在基类中从 ROS 参数初始化
      // 这里从配置文件读取作为备份/覆盖（如果配置文件中有的话）
      int config_jointNum = pt.get<int>("jointNum", jointNum_);
      int config_jointArmNum = pt.get<int>("jointArmNum", jointArmNum_);
      int config_waistNum = pt.get<int>("waistNum", waistNum_);  // 从配置文件读取腰部数量
      
      // 日志输出配置对比
      ROS_INFO("[%s] Joint config from ROS params: jointNum=%d, jointArmNum=%d, waistNum=%d",
               name_.c_str(), jointNum_, jointArmNum_, waistNum_);
      ROS_INFO("[%s] Joint config from config file: jointNum=%d, jointArmNum=%d, waistNum=%d",
               name_.c_str(), config_jointNum, config_jointArmNum, config_waistNum);
      
      // 加载VMP参数
      vmp_actionScale_ = pt.get<double>("actionScale", 0.25);
      vmp_frameStack_ = pt.get<int>("frameStack", 1);
      vmp_numSingleObs_ = pt.get<int>("numSingleObs", 83);
      vmp_clipObservations_ = pt.get<double>("clipObservations", 18.0);
      vmp_clipActions_ = pt.get<double>("clipActions", 18.0);
      
      numRefMotionObs_ = pt.get<int>("numRefMotionObs", 77);
      numEncoderObs_ = pt.get<int>("numEncoderObs", 64);
      
      // numActions 从配置文件读取，这是策略网络的输入/输出关节维度
      // v46: 26 (腿12 + 手臂14)
      // v52: 27 (腿12 + 腰1 + 手臂14)，顺序：腿(0-11) + 腰(12) + 手臂(13-26)
      int expected_actions = jointNum_ + jointArmNum_ + waistNum_;
      numActions_ = pt.get<int>("numActions", expected_actions);
      
      // 观测格式: [joint_pos_offset(N), joint_vel(N), last_actions(N), ang_vel(3), euler_w(3), anchor_euler_b(3)]
      // 其中 N = numActions_
      int expected_numSingleObs = 3 * numActions_ + 3 + 3 + 3;  // 3N + 9
      if (vmp_numSingleObs_ != expected_numSingleObs) {
        ROS_WARN("[%s] numSingleObs (%d) may not match expected (%d = 3*%d + 9). "
                 "Please verify VMP observation dimension configuration.",
                 name_.c_str(), vmp_numSingleObs_, expected_numSingleObs, numActions_);
      }
      
      ROS_INFO("[%s] VMP config: numActions=%d, numSingleObs=%d, numRefMotionObs=%d",
               name_.c_str(), numActions_, vmp_numSingleObs_, numRefMotionObs_);
      
      // 检查配置一致性
      if (numActions_ != expected_actions) {
        ROS_WARN("[%s] numActions (%d) from config != expected (%d = %d + %d + %d).",
                 name_.c_str(), numActions_, expected_actions, jointNum_, jointArmNum_, waistNum_);
      }
      
      inference_frequency_ = pt.get<double>("inferenceFrequency", 100.0);
      vmp_enable_theta_normalization_ = pt.get<bool>("enableThetaNormalization", true);
      
      // 加载Estimator配置
      vmp_enable_estimator_ = pt.get<bool>("enableEstimator", false);
      vmp_estimator_history_frames_ = pt.get<int>("estimatorHistoryFrames", 8);
      vmp_estimator_input_dim_ = pt.get<int>("estimatorInputDim", vmp_numSingleObs_);
      vmp_estimator_output_dim_ = pt.get<int>("estimatorOutputDim", 64);
      
      if (vmp_enable_estimator_) {
        ROS_INFO("[%s] Estimator enabled: history_frames=%d, input_dim=%d, output_dim=%d",
                 name_.c_str(), vmp_estimator_history_frames_, vmp_estimator_input_dim_, vmp_estimator_output_dim_);
      }
      
      // 加载观测缩放参数
      vmp_obsScale_dof_pos_ = pt.get<double>("obsScale_dof_pos", 1.0);
      vmp_obsScale_dof_vel_ = pt.get<double>("obsScale_dof_vel", 0.05);
      vmp_obsScale_base_lin_vel_ = pt.get<double>("obsScale_base_lin_vel", 2.0);
      vmp_obsScale_base_ang_vel_ = pt.get<double>("obsScale_base_ang_vel", 0.25);
      vmp_obsScale_quat_ = pt.get<double>("obsScale_quat", 1.0);
      
      ROS_INFO("[%s] Final joint config: jointNum=%d, jointArmNum=%d, waistNum=%d, numActions=%d", 
               name_.c_str(), jointNum_, jointArmNum_, waistNum_, numActions_);

      bool arm_command_replacement_enabled = false;
      loadData::loadPtreeValue(pt, arm_command_replacement_enabled, "use_external_arm_controller", false);
      use_external_arm_controller(arm_command_replacement_enabled);
      ROS_INFO("[%s] Arm command replacement enabled: %s",
               name_.c_str(), arm_command_replacement_enabled ? "true" : "false");

      if (arm_command_replacement_enabled && jointArmNum_ > 0)
      {
        loadData::loadPtreeValue(pt, arm_max_tracking_velocity_, "armVelocityLimit.maxTrackingVelocity", false);
        loadData::loadPtreeValue(pt, arm_tracking_error_threshold_, "armVelocityLimit.trackingErrorThreshold", false);
        loadData::loadPtreeValue(pt, arm_mode_interpolation_velocity_, "armVelocityLimit.modeInterpolationVelocity", false);
        loadData::loadPtreeValue(pt, arm_mode2_cutoff_freq_, "armVelocityLimit.mode2CutoffFreq", false);
        ROS_INFO("[%s] Arm control parameters: max_vel=%.3f, err_th=%.3f, mode_interp_vel=%.3f, mode2_cutoff=%.1f Hz",
                 name_.c_str(), arm_max_tracking_velocity_,
                 arm_tracking_error_threshold_, arm_mode_interpolation_velocity_, arm_mode2_cutoff_freq_);
      }
      
      // 加载模型路径
      std::string vmpModelDir, vmpPolicyModelFile, vmpEncoderModelFile, vmpEstimatorModelFile;
      loadData::loadCppDataType(config_file, "vmpModelDir", vmpModelDir);
      
      // 先加载播放模式，决定使用离线还是在线模型
      try {
        loadData::loadCppDataType(config_file, "vmpPlaybackMode", vmp_playback_mode_);
      } catch (...) {
        vmp_playback_mode_ = "offline_trajectory";
      }
      enable_online_vr_mode_ = (vmp_playback_mode_ == "online_teleoperation");
      
      // 根据播放模式选择模型文件
      if (enable_online_vr_mode_) {
        // 在线模式：优先使用在线专用模型，如果不存在则回退到离线模型
        std::string onlinePolicyModelFile, onlineEncoderModelFile, onlineEstimatorModelFile;
        try {
          loadData::loadCppDataType(config_file, "onlinePolicyModelFile", onlinePolicyModelFile);
          vmpPolicyModelFile = onlinePolicyModelFile;
          ROS_INFO("[%s] Using online policy model: %s", name_.c_str(), vmpPolicyModelFile.c_str());
        } catch (...) {
          loadData::loadCppDataType(config_file, "vmpPolicyModelFile", vmpPolicyModelFile);
          ROS_WARN("[%s] onlinePolicyModelFile not found, falling back to offline: %s", name_.c_str(), vmpPolicyModelFile.c_str());
        }
        try {
          loadData::loadCppDataType(config_file, "onlineEncoderModelFile", onlineEncoderModelFile);
          vmpEncoderModelFile = onlineEncoderModelFile;
          ROS_INFO("[%s] Using online encoder model: %s", name_.c_str(), vmpEncoderModelFile.c_str());
        } catch (...) {
          loadData::loadCppDataType(config_file, "vmpEncoderModelFile", vmpEncoderModelFile);
          ROS_WARN("[%s] onlineEncoderModelFile not found, falling back to offline: %s", name_.c_str(), vmpEncoderModelFile.c_str());
        }
        if (vmp_enable_estimator_) {
          try {
            loadData::loadCppDataType(config_file, "onlineEstimatorModelFile", onlineEstimatorModelFile);
            vmpEstimatorModelFile = onlineEstimatorModelFile;
            ROS_INFO("[%s] Using online estimator model: %s", name_.c_str(), vmpEstimatorModelFile.c_str());
          } catch (...) {
            try {
              loadData::loadCppDataType(config_file, "vmpEstimatorModelFile", vmpEstimatorModelFile);
              ROS_WARN("[%s] onlineEstimatorModelFile not found, falling back to offline: %s", name_.c_str(), vmpEstimatorModelFile.c_str());
            } catch (...) {
              ROS_WARN("[%s] No estimator model found, disabling estimator", name_.c_str());
              vmp_enable_estimator_ = false;
            }
          }
        }
      } else {
        // 离线模式：使用离线模型
        loadData::loadCppDataType(config_file, "vmpPolicyModelFile", vmpPolicyModelFile);
        loadData::loadCppDataType(config_file, "vmpEncoderModelFile", vmpEncoderModelFile);
        if (vmp_enable_estimator_) {
          try {
            loadData::loadCppDataType(config_file, "vmpEstimatorModelFile", vmpEstimatorModelFile);
          } catch (...) {
            ROS_WARN("[%s] vmpEstimatorModelFile not found in config, disabling estimator", name_.c_str());
            vmp_enable_estimator_ = false;
          }
        }
        ROS_INFO("[%s] Using offline models: policy=%s, encoder=%s", name_.c_str(), vmpPolicyModelFile.c_str(), vmpEncoderModelFile.c_str());
      }
      
      loadData::loadCppDataType(config_file, "vmpRefDataDir", vmpRefDataDir_);
      loadData::loadCppDataType(config_file, "vmpTaskDataFile", vmpTaskDataFile_);
      
      // 构建完整路径（直接使用baseModelPath_）
      vmpModelPath_ = baseModelPath_ + "/" + vmpModelDir + vmpPolicyModelFile;
      vmpEncoderPath_ = baseModelPath_ + "/" + vmpModelDir + vmpEncoderModelFile;
      if (vmp_enable_estimator_) {
        vmpEstimatorPath_ = baseModelPath_ + "/" + vmpModelDir + vmpEstimatorModelFile;
        ROS_INFO("[%s] Estimator model path: %s", name_.c_str(), vmpEstimatorPath_.c_str());
      }
      
      // 加载关节参数（按照 numActions_ 维度）
      int jointTotal = numActions_;
      vmp_defalutJointPos_.resize(jointTotal);
      vmp_JointControlMode_.resize(jointTotal);
      vmp_JointPDMode_.resize(jointTotal);
      vmp_jointKp_.resize(jointTotal);
      vmp_jointKd_.resize(jointTotal);
      vmp_torqueLimits_.resize(jointTotal);
      vmp_actionScaleTest_.resize(jointTotal);
      
      for (int i = 0; i < jointTotal; i++) {
        std::string key = "(" + std::to_string(i) + ",0)";
        vmp_defalutJointPos_(i) = pt.get<double>("defaultJointState." + key, 0.0);
        vmp_JointControlMode_(i) = pt.get<int>("jointControlMode." + key, 0);
        vmp_JointPDMode_(i) = pt.get<int>("jointPDMode." + key, 0);
        vmp_jointKp_(i) = pt.get<double>("jointKp." + key, 15.0);
        vmp_jointKd_(i) = pt.get<double>("jointKd." + key, 3.0);
        vmp_torqueLimits_(i) = pt.get<double>("torqueLimits." + key, 20.0);
        vmp_actionScaleTest_(i) = pt.get<double>("actionScaleTest." + key, 1.0);
      }
      
      // 设置初始状态
      initialStateRL_.resize(jointTotal);
      initialStateRL_ = vmp_defalutJointPos_;
      
      // 加载VAE配置
      vmp_config_.in_c = pt.get<int>("vaeModel.in_c", 77);
      vmp_config_.window_l = pt.get<int>("vaeModel.window_l", 30);
      vmp_config_.window_r = pt.get<int>("vaeModel.window_r", 15);
      vmp_config_.latent_d = pt.get<int>("vaeModel.latent_d", 64);
      vmp_config_.use_global_coef = pt.get<bool>("vaeModel.use_global_coef", true);
      
      // 加载滤波器参数
      vmp_jointCmdFilterCutoffFreq_.resize(jointTotal);
      vmp_jointCmdFilterState_.resize(jointTotal);
      for (int i = 0; i < jointTotal; i++) {
        std::string key = "(" + std::to_string(i) + ",0)";
        vmp_jointCmdFilterCutoffFreq_(i) = pt.get<double>("jointCmdFilterCutoffFreq." + key, 50.0);
        vmp_jointCmdFilterState_(i) = pt.get<double>("jointCmdFilterState." + key, 0.0);
      }
      
      vmpJointCmdFilter_.setParams(dt_, vmp_jointCmdFilterCutoffFreq_);
      
      // 加载静止帧配置
      vmp_pre_standing_frames_ = pt.get<int>("preStandingFrames", 50);
      vmp_post_standing_frames_ = pt.get<int>("postStandingFrames", 50);
      vmp_enable_loop_ = pt.get<bool>("enableLoop", true);
      standingHeight_ = pt.get<double>("standingHeight", 0.87);
      
      // 加载插值配置
      vmp_pre_interpolation_frames_ = pt.get<int>("preInterpolationFrames", 40);
      vmp_post_interpolation_frames_ = pt.get<int>("postInterpolationFrames", 40);
      
      // 加载refTask配置（用于静止帧构建）
      vmp_config_.h_start_id = pt.get<int>("refTask.h_start_id", 0);
      vmp_config_.h_end_id = pt.get<int>("refTask.h_end_id", 1);
      vmp_config_.theta_start_id = pt.get<int>("refTask.theta_start_id", 1);
      vmp_config_.theta_end_id = pt.get<int>("refTask.theta_end_id", 7);
      vmp_config_.v_start_id = pt.get<int>("refTask.v_start_id", 7);
      vmp_config_.v_end_id = pt.get<int>("refTask.v_end_id", 13);
      vmp_config_.q_start_id = pt.get<int>("refTask.q_start_id", 13);
      vmp_config_.q_end_id = pt.get<int>("refTask.q_end_id", 39);
      vmp_config_.q_dot_start_id = pt.get<int>("refTask.q_dot_start_id", 39);
      vmp_config_.q_dot_end_id = pt.get<int>("refTask.q_dot_end_id", 65);
      vmp_config_.p_start_id = pt.get<int>("refTask.p_start_id", 65);
      vmp_config_.p_end_id = pt.get<int>("refTask.p_end_id", 77);
      
      // 加载静止帧关节位置
      vmp_standing_joint_pos_.resize(jointTotal);
      for (int i = 0; i < jointTotal; i++) {
        std::string key = "(" + std::to_string(i) + ",0)";
        vmp_standing_joint_pos_[i] = pt.get<float>("standingJointPos." + key, 0.0f);
      }
      
      // 加载静止帧末端执行器位置 (12维)
      // 顺序规则: v46先手后腿 [L_hand, R_hand, L_foot, R_foot]
      //          v52先腿后手 [L_foot, R_foot, L_hand, R_hand]
      // 配置文件中已按对应机型的正确顺序排列
      vmp_standing_end_effector_pos_.resize(12);
      for (int i = 0; i < 12; i++) {
        std::string key = "(" + std::to_string(i) + ",0)";
        vmp_standing_end_effector_pos_[i] = pt.get<float>("standingEndEffectorPos." + key, 0.0f);
      }
      
      // 构建完整的静止帧数据（维度由 vmp_config_.in_c 指定：v46=77, v52=79）
      vmp_standing_frame_.resize(vmp_config_.in_c, 0.0f);
      
      // 高度
      vmp_standing_frame_[vmp_config_.h_start_id] = static_cast<float>(standingHeight_);
      
      // 旋转矩阵 - 单位矩阵的6D表示（前两列）
      // 列0: [1, 0, 0]
      vmp_standing_frame_[vmp_config_.theta_start_id]     = 1.0f;  // R[0,0]
      vmp_standing_frame_[vmp_config_.theta_start_id + 1] = 0.0f;  // R[1,0]
      vmp_standing_frame_[vmp_config_.theta_start_id + 2] = 0.0f;  // R[2,0]
      // 列1: [0, 1, 0]
      vmp_standing_frame_[vmp_config_.theta_start_id + 3] = 0.0f;  // R[0,1]
      vmp_standing_frame_[vmp_config_.theta_start_id + 4] = 1.0f;  // R[1,1]
      vmp_standing_frame_[vmp_config_.theta_start_id + 5] = 0.0f;  // R[2,1]
      
      // 关节位置
      for (int i = 0; i < vmp_config_.q_end_id - vmp_config_.q_start_id && i < jointTotal; ++i) {
        vmp_standing_frame_[vmp_config_.q_start_id + i] = vmp_standing_joint_pos_[i];
      }
      
      // 末端执行器位置 (从配置文件读取, 12维)
      // 顺序规则: v46先手后腿 [L_hand, R_hand, L_foot, R_foot]
      //          v52先腿后手 [L_foot, R_foot, L_hand, R_hand]
      int p_dim = vmp_config_.p_end_id - vmp_config_.p_start_id;
      for (int i = 0; i < p_dim && i < static_cast<int>(vmp_standing_end_effector_pos_.size()); ++i) {
        vmp_standing_frame_[vmp_config_.p_start_id + i] = vmp_standing_end_effector_pos_[i];
      }
      
      ROS_INFO("[%s] Standing frame initialized:", name_.c_str());
      ROS_INFO("[%s]   Height: %.3f", name_.c_str(), standingHeight_);
      ROS_INFO("[%s]   Pre/Post standing frames: %d/%d", name_.c_str(), 
               vmp_pre_standing_frames_, vmp_post_standing_frames_);
      ROS_INFO("[%s]   Pre/Post interpolation frames: %d/%d", name_.c_str(),
               vmp_pre_interpolation_frames_, vmp_post_interpolation_frames_);
      
      // 同步VMP站立姿态到基类成员，供MPC→VMP插值过渡使用
      // （当 use_interploate_from_mpc=true 时，humanoidController 会读取这些值来做平滑过渡）
      defaultBaseHeightControl_ = standingHeight_;
      for (int i = 0; i < jointTotal && i < defalutJointPosRL_.size(); ++i) {
        defalutJointPosRL_(i) = static_cast<double>(vmp_standing_joint_pos_[i]);
      }
      
      // 加载在线VR配置
      if (enable_online_vr_mode_) {
        try {
          loadData::loadCppDataType(config_file, "onlineVRDataSource", online_vr_data_source_);
          loadData::loadCppDataType(config_file, "onlineBufferSize", online_buffer_size_);
          loadData::loadCppDataType(config_file, "onlineUpdateRate", online_update_rate_);
          
          // 加载在线VR控制模式和bin文件路径
          try {
            loadData::loadCppDataType(config_file, "onlineVRControlMode", online_vr_control_mode_);
          } catch (...) {
            online_vr_control_mode_ = "upper_body";  // 默认上半身模式
          }
          
          try {
            loadData::loadCppDataType(config_file, "onlineVRBinFile", online_vr_bin_file_);
          } catch (...) {
            online_vr_bin_file_ = "";
          }

          try {
            loadData::loadCppDataType(config_file, "onlineRetargetStaleFallbackEnable",
                                      online_retarget_stale_fallback_enable_);
          } catch (...) {
            online_retarget_stale_fallback_enable_ = true;
          }
          try {
            loadData::loadCppDataType(config_file, "onlineRetargetStaleTimeoutMs",
                                      online_retarget_stale_timeout_ms_);
          } catch (...) {
            online_retarget_stale_timeout_ms_ = 150.0;
          }
          try {
            loadData::loadCppDataType(config_file, "onlineStaleToStandingInterpFrames",
                                      online_stale_to_standing_interp_frames_);
          } catch (...) {
            online_stale_to_standing_interp_frames_ = 50;
          }
          online_retarget_stale_timeout_ms_ = std::max(10.0, online_retarget_stale_timeout_ms_);
          online_stale_to_standing_interp_frames_ = std::max(1, online_stale_to_standing_interp_frames_);
          try {
            loadData::loadCppDataType(config_file, "onlineRetargetStaleRequireReentry",
                                      online_retarget_stale_require_reentry_);
          } catch (...) {
            online_retarget_stale_require_reentry_ = true;
          }
          
          ROS_INFO("[%s] Online VR config:", name_.c_str());
          ROS_INFO("[%s]   Data source: %s", name_.c_str(), online_vr_data_source_.c_str());
          ROS_INFO("[%s]   Control mode: %s", name_.c_str(), online_vr_control_mode_.c_str());
          ROS_INFO("[%s]   Buffer size: %d", name_.c_str(), online_buffer_size_);
          ROS_INFO("[%s]   Update rate: %.1f Hz", name_.c_str(), online_update_rate_);
          if (!online_vr_bin_file_.empty()) {
            ROS_INFO("[%s]   Bin file: %s", name_.c_str(), online_vr_bin_file_.c_str());
          }
          ROS_INFO("[%s]   Stale fallback: enable=%d, timeout=%.0f ms, standing_interp=%d frames, require_reentry=%d",
                   name_.c_str(),
                   static_cast<int>(online_retarget_stale_fallback_enable_),
                   online_retarget_stale_timeout_ms_,
                   online_stale_to_standing_interp_frames_,
                   static_cast<int>(online_retarget_stale_require_reentry_));
        } catch (...) {
          ROS_WARN("[%s] Online VR config not found, using defaults", name_.c_str());
        }
      }
      
      // 加载GMR数据平滑配置
      try {
        loadData::loadCppDataType(config_file, "enableGMRSmoothing", enable_gmr_smoothing_);
      } catch (...) {
        enable_gmr_smoothing_ = true;  // 默认启用
      }
      
      if (enable_gmr_smoothing_) {
        try {
          loadData::loadCppDataType(config_file, "gmrGaussianSigma", gmr_gaussian_sigma_);
          gmr_gaussian_sigma_ = std::max(0.1, gmr_gaussian_sigma_);  // 最小值0.1
        } catch (...) {
          gmr_gaussian_sigma_ = 1.0;
        }
        
        try {
          loadData::loadCppDataType(config_file, "gmrVelocityClip", gmr_velocity_clip_);
          gmr_velocity_clip_ = std::max(0.0, gmr_velocity_clip_);
        } catch (...) {
          gmr_velocity_clip_ = 20.0;
        }
        
        try {
          loadData::loadCppDataType(config_file, "gmrVelocityDeadzone", gmr_velocity_deadzone_);
          gmr_velocity_deadzone_ = std::max(0.0, gmr_velocity_deadzone_);
        } catch (...) {
          gmr_velocity_deadzone_ = 0.05;
        }
        
        ROS_INFO("[%s] GMR smoothing config: enabled=%d, sigma=%.1f, clip=%.1f, deadzone=%.3f", 
                 name_.c_str(), static_cast<int>(enable_gmr_smoothing_), 
                 gmr_gaussian_sigma_, gmr_velocity_clip_, gmr_velocity_deadzone_);
        
        // 加载GMR关节位置平滑配置（独立开关和sigma）
        try {
          loadData::loadCppDataType(config_file, "enableGMRQposSmoothing", enable_gmr_qpos_smoothing_);
        } catch (...) {
          enable_gmr_qpos_smoothing_ = false;  // 默认关闭
        }
        
        if (enable_gmr_qpos_smoothing_) {
          try {
            loadData::loadCppDataType(config_file, "gmrQposSigma", gmr_qpos_sigma_);
            gmr_qpos_sigma_ = std::max(0.1, gmr_qpos_sigma_);
          } catch (...) {
            gmr_qpos_sigma_ = 1.0;
          }
          ROS_INFO("[%s] GMR qpos smoothing: enabled, sigma=%.1f", name_.c_str(), gmr_qpos_sigma_);
        }
      }
      
      // 加载Qpos平滑配置（重复帧回溯插值）
      try {
        loadData::loadCppDataType(config_file, "enableQposSmoothing", enable_qpos_smoothing_);
      } catch (...) {
        enable_qpos_smoothing_ = true;
      }
      
      if (enable_qpos_smoothing_) {
        try {
          loadData::loadCppDataType(config_file, "qposSmoothBufferSize", qpos_smooth_buffer_size_);
          qpos_smooth_buffer_size_ = std::max(2, std::min(qpos_smooth_buffer_size_, 20));
        } catch (...) {
          qpos_smooth_buffer_size_ = 5;
        }
        
        try {
          loadData::loadCppDataType(config_file, "qposNewJointEps", qpos_new_joint_eps_);
          qpos_new_joint_eps_ = std::max(0.0, qpos_new_joint_eps_);
        } catch (...) {
          qpos_new_joint_eps_ = 0.001;
        }
        
        ROS_INFO("[%s] Qpos smoothing config: enabled=%d, buffer_size=%d, joint_eps=%.4f",
                 name_.c_str(), static_cast<int>(enable_qpos_smoothing_),
                 qpos_smooth_buffer_size_, qpos_new_joint_eps_);
      }
      
      ROS_INFO("[%s] VMP config loaded successfully", name_.c_str());
      ROS_INFO("[%s]   Playback mode: %s", name_.c_str(), vmp_playback_mode_.c_str());
      ROS_INFO("[%s]   Action scale: %.3f", name_.c_str(), vmp_actionScale_);
      ROS_INFO("[%s]   Num observations: %d", name_.c_str(), vmp_numSingleObs_);
      ROS_INFO("[%s]   Num ref motion obs: %d", name_.c_str(), numRefMotionObs_);
      ROS_INFO("[%s]   VAE latent dim: %d", name_.c_str(), vmp_config_.latent_d);
      ROS_INFO("[%s]   Estimator enabled: %s", name_.c_str(), vmp_enable_estimator_ ? "true" : "false");
      if (vmp_enable_estimator_) {
        ROS_INFO("[%s]   Estimator output dim: %d", name_.c_str(), vmp_estimator_output_dim_);
      }
      int expected_policy_input = vmp_numSingleObs_ + numRefMotionObs_ + vmp_config_.latent_d;
      if (vmp_enable_estimator_) {
        expected_policy_input += vmp_estimator_output_dim_;
      }
      ROS_INFO("[%s]   Expected policy input dim: %d", name_.c_str(), expected_policy_input);
      ROS_INFO("[%s]   Policy model: %s", name_.c_str(), vmpModelPath_.c_str());
      
      // 是否使用 VMP 专用 Ruiwo 增益（对应 vmp_param.info 中 use_vmp_ruiwo_kpkd，默认不使用）
      loadData::loadPtreeValue(pt, use_vmp_ruiwo_kpkd_, "use_vmp_ruiwo_kpkd", false);
      ROS_INFO("[%s]   Use VMP Ruiwo KP/KD: %s", name_.c_str(), use_vmp_ruiwo_kpkd_ ? "true" : "false");
      
      return true;
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Error loading config: %s", name_.c_str(), e.what());
      return false;
    }
  }

  void VMPController::reset()
  {
    vmp_actions_.setZero();
    vmp_ref_motion_buffer_.clear();
    vmp_ref_motion_raw_buffer_.clear();
    current_task_idx_ = 0;
    trajectoryFrameCounter_ = 0;
    trajectoryPlaybackCompleted_ = false;
    // Ensure the shared actions_ vector used by the base class is also initialized
    try {
      const int desired_size = std::max(numActions_, jointNum_ + jointArmNum_);
      setCurrentAction(Eigen::VectorXd::Zero(desired_size));
    } catch (...) {
      // ignore
    }
    
    // 重置 Estimator 历史缓冲区（与 humanoidController_vmp.cpp 保持一致）
    if (vmp_enable_estimator_) {
      std::lock_guard<std::mutex> lock(vmp_estimator_mtx_);
      vmp_estimator_obs_buffer_.clear();
      Eigen::VectorXd zero_obs = Eigen::VectorXd::Zero(vmp_estimator_input_dim_);
      for (int i = 0; i < vmp_estimator_history_frames_; ++i) {
        vmp_estimator_obs_buffer_.push_back(zero_obs);
      }
      vmp_estimator_output_.setZero();
      ROS_INFO("[%s] Estimator buffer reset with %d zero frames", name_.c_str(), vmp_estimator_history_frames_);
    }
    
    // 普通 reset 保持原行为；控制器切换入 VMP 时则继续冻结，
    // 直到 GMR 确认 RT+X 偏航重定向已应用到新帧。
    bool waiting_for_retarget_resume = false;
    bool using_retarget_entry_frame = false;
    std::vector<float> reset_prefill_frame = vmp_standing_frame_;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      waiting_for_retarget_resume = retarget_resume_waiting_;
      if (retarget_entry_frame_pending_ && !retarget_entry_frame_.empty())
      {
        reset_prefill_frame = retarget_entry_frame_;
        using_retarget_entry_frame = true;
      }
    }
    if (!waiting_for_retarget_resume)
    {
      resumeRetargetedStreaming();
    }
    
    // 重置Qpos平滑缓冲区，并用静止帧预填充，消除启动空窗期
    // 注意：需要加锁，避免与采样线程（onlineSamplingLoop）的 race condition
    {
      std::lock_guard<std::mutex> lock(qpos_pending_mutex_);
      qpos_pending_buffer_.clear();
      qpos_last_ingested_.clear();
      qpos_stat_new_count_ = 0;
      qpos_stat_repeat_count_ = 0;
      qpos_stat_interp_count_ = 0;
      {
        std::lock_guard<std::mutex> lock2(latest_frame_mutex_);
        last_consumed_frame_seq_ = latest_frame_seq_;  // 同步序列号，避免reset后误判
      }
      if (enable_qpos_smoothing_ && !reset_prefill_frame.empty()) {
        for (int i = 0; i < qpos_smooth_buffer_size_; ++i) {
          PendingQposSample s;
          s.frame    = reset_prefill_frame;
          s.anchor   = (i == 0);  // 首帧作为anchor，其余作为held
          s.smoothed = false;
          qpos_pending_buffer_.push_back(std::move(s));
        }
        qpos_last_ingested_ = reset_prefill_frame;
        ROS_INFO("[%s] Qpos smooth buffer pre-filled with %d %s frames",
                 name_.c_str(), qpos_smooth_buffer_size_,
                 using_retarget_entry_frame ? "VMP-entry" : "standing");
      }
    }
    if (using_retarget_entry_frame)
    {
      prefillRetargetedReferenceBuffers(reset_prefill_frame);
    }
    vmp_entry_imu_quat_valid_ = false;

    if (enable_online_vr_mode_ && online_ref_buffer_.is_initialized)
    {
      initializeOnlineReferenceBuffer();
    }

    resetOnlineStaleFallbackState();

    if (arm_controller_)
      arm_controller_->reset();
    
    ROS_INFO("[%s] VMP controller reset", name_.c_str());
  }

  void VMPController::setExternalCommandBufferCallback(std::function<bool()> callback)
  {
    external_command_buffer_callback_ = std::move(callback);
    if (arm_controller_)
    {
      arm_controller_->setExternalCommandBufferCallback(external_command_buffer_callback_);
    }
  }

  void VMPController::initArmControl(const std::string& urdf_path)
  {
    if (!arm_command_replacement_enabled_ || jointArmNum_ <= 0)
    {
      ROS_INFO("[%s] Arm command replacement disabled or no arm joints", name_.c_str());
      return;
    }
    try
    {
      arm_controller_ = std::make_unique<ArmController>(
          nh_, jointNum_, waistNum_, jointArmNum_, ros_logger_);
      arm_controller_->setExternalCommandBufferCallback(external_command_buffer_callback_);

      Eigen::VectorXd arm_kp, arm_kd;
      if (is_roban_)
      {
        arm_kp = vmp_jointKp_.segment(waistNum_ + jointNum_, jointArmNum_);
        arm_kd = vmp_jointKd_.segment(waistNum_ + jointNum_, jointArmNum_);
      }
      else
      {
        arm_kp = vmp_jointKp_.segment(jointNum_ + waistNum_, jointArmNum_);
        arm_kd = vmp_jointKd_.segment(jointNum_ + waistNum_, jointArmNum_);
      }

      if (!arm_controller_->initialize(urdf_path, arm_kp, arm_kd))
      {
        ROS_ERROR("[%s] Failed to initialize arm controller", name_.c_str());
        arm_command_replacement_enabled_ = false;
        arm_controller_.reset();
        return;
      }

      Eigen::VectorXd default_arm_pos = Eigen::VectorXd::Zero(jointArmNum_);
      if (vmp_defalutJointPos_.size() >= jointNum_ + waistNum_ + jointArmNum_)
      {
        if (is_roban_)
          default_arm_pos = vmp_defalutJointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
        else
          default_arm_pos = vmp_defalutJointPos_.segment(jointNum_ + waistNum_, jointArmNum_);
      }
      else
      {
        ROS_WARN("[%s] Cannot get default arm position, using zeros", name_.c_str());
      }

      arm_controller_->loadSettings(arm_max_tracking_velocity_, arm_tracking_error_threshold_,
                                    arm_mode_interpolation_velocity_, default_arm_pos,
                                    arm_mode2_cutoff_freq_);
      ROS_INFO("[%s] Arm controller initialized (urdf=%s)", name_.c_str(), urdf_path.c_str());
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to initialize arm controller: %s", name_.c_str(), e.what());
      arm_command_replacement_enabled_ = false;
      arm_controller_.reset();
    }
  }

  bool VMPController::updateArmCommand(const ros::Time& time,
                                       const SensorData& sensor_data,
                                       kuavo_msgs::jointCmd& joint_cmd)
  {
    if (!arm_command_replacement_enabled_ || jointArmNum_ == 0 || !arm_controller_)
      return false;

    // Pico uses pico_streaming_paused_; Xsens sets the arm-only request from
    // the shared /pico/joy RT+Y/RT+X callback below.
    arm_controller_->setExternalControlPaused(
        pico_streaming_paused_.load() || arm_stream_pause_requested_.load());

    double dt = dt_;
    if (dt <= 0.0 || dt > 0.1) dt = 0.002;

    Eigen::VectorXd full_joint_pos(jointNum_ + waistNum_ + jointArmNum_);
    Eigen::VectorXd full_joint_vel(jointNum_ + waistNum_ + jointArmNum_);

    if (is_roban_)
    {
      Eigen::VectorXd waist_pos = sensor_data.jointPos_.segment(0, waistNum_);
      Eigen::VectorXd leg_pos = sensor_data.jointPos_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_pos = sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
      Eigen::VectorXd waist_vel = sensor_data.jointVel_.segment(0, waistNum_);
      Eigen::VectorXd leg_vel = sensor_data.jointVel_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_vel = sensor_data.jointVel_.segment(waistNum_ + jointNum_, jointArmNum_);
      full_joint_pos << leg_pos, waist_pos, arm_pos;
      full_joint_vel << leg_vel, waist_vel, arm_vel;
    }
    else
    {
      full_joint_pos = sensor_data.jointPos_.head(jointNum_ + waistNum_ + jointArmNum_);
      full_joint_vel = sensor_data.jointVel_.head(jointNum_ + waistNum_ + jointArmNum_);
    }

    const int cmd_stance = 0;  // VMP 无 gait_receiver；固定行走语义
    arm_controller_->update(time, dt, full_joint_pos, full_joint_vel, cmd_stance, joint_cmd);

    if (arm_controller_->getMode() == 1)
      return false;
    return true;
  }

  int VMPController::getArmControlModeOnControllerActivate() const
  {
    if (arm_command_replacement_enabled_ && enable_online_vr_mode_)
      return 2;
    return 1;
  }

  void VMPController::activateExternalArmControlIfNeeded()
  {
    if (!arm_command_replacement_enabled_ || !enable_online_vr_mode_ || !arm_controller_)
      return;
    if (arm_controller_->getMode() == 2)
      return;
    arm_controller_->changeMode(2);
    ROS_INFO("[%s] Auto-switched arm to external mode (2) for online teleoperation",
             name_.c_str());
  }

void VMPController::resume()
  {
    // A warm target entry does not call reset(). Install the newly captured
    // entry frame into both policy paths before RUNNING for the same first-tick
    // behavior as the AMP->VMP cold-reset path.
    std::vector<float> pending_entry_frame;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      if (retarget_entry_frame_pending_ && !retarget_entry_frame_.empty())
      {
        pending_entry_frame = retarget_entry_frame_;
      }
    }
    if (enable_online_vr_mode_ && !pending_entry_frame.empty())
    {
      prefillRetargetedReferenceBuffers(pending_entry_frame);
      initializeOnlineReferenceBuffer();
    }

    // Invalidate before RUNNING so the first VMP inference cannot use the
    // previous activation's robot/anchor alignment.
    vmp_entry_imu_quat_valid_ = false;
    RLControllerBase::resume();
    if (enable_online_vr_mode_) {
      if (use_interpolate_from_mpc_) {
        ROS_INFO("[%s] MPC->VMP interpolation active, starting sampling early (inference not running yet)", name_.c_str());
      }
      startOnlineSampling();
    }
    
    // 真实机器人恢复 VMP 控制时切换到 VMP 专用 Ruiwo 电机参数（异步调用避免阻塞控制线程）
    if (is_real_ && use_vmp_ruiwo_kpkd_)
    {
      changeRuiwoMotorParamAsync("vmp_kpkd");
    }
    
    ROS_INFO("[%s] VMP controller resumed", name_.c_str());
  }

  void VMPController::resumeRetargetedStreaming()
  {
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      retarget_resume_waiting_ = false;
      retarget_frozen_frame_valid_ = false;
      retarget_auto_freeze_valid_ = false;
      retarget_resume_min_stamp_ = ros::Time(0);
      retarget_resume_ready_stamp_ = ros::Time(0);
      retarget_resume_min_frame_seq_ = latest_retargeted_frame_seq_;
      pico_streaming_paused_.store(false);
    }
    resetOnlineStaleFallbackState(false);
  }

  void VMPController::notifyOnlineFrameReceived()
  {
    std::lock_guard<std::mutex> lock(online_stale_mutex_);
    latest_online_frame_recv_wall_time_ = ros::Time::now();
    latest_online_frame_recv_time_valid_ = true;
  }

  void VMPController::resetOnlineStaleFallbackState(bool clear_recv_time)
  {
    std::lock_guard<std::mutex> lock(online_stale_mutex_);
    online_stale_standing_active_ = false;
    online_stale_interp_progress_ = 0;
    online_stale_interp_start_frame_.clear();
    if (clear_recv_time) {
      latest_online_frame_recv_time_valid_ = false;
      online_retarget_stale_latched_ = false;
    }
  }

  bool VMPController::applyOnlineRetargetStaleFallback(std::vector<float>& processed_frame,
                                                       bool callback_updated)
  {
    if (!online_retarget_stale_fallback_enable_ || vmp_standing_frame_.empty()) {
      return false;
    }

    // RT+Y 主动暂停走 pico_frozen_frame_ 专用路径，不在此处改写参考。
    if (pico_streaming_paused_.load()) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      if (retarget_resume_waiting_) {
        return false;
      }
    }

    std::lock_guard<std::mutex> lock(online_stale_mutex_);
    bool enforce_standing = false;

    if (online_retarget_stale_latched_ && online_retarget_stale_require_reentry_) {
      enforce_standing = true;
      if (callback_updated) {
        ROS_WARN_THROTTLE(2.0,
                          "[%s] Retarget recovered while stale latch active; "
                          "live teleop disabled. Switch to MPC/AMP, then re-enter VMP to resume.",
                          name_.c_str());
      }
    } else if (callback_updated) {
      if (online_stale_standing_active_) {
        ROS_INFO("[%s] Retarget stream recovered, resuming live reference", name_.c_str());
      }
      online_stale_standing_active_ = false;
      online_stale_interp_progress_ = 0;
      online_stale_interp_start_frame_.clear();
      return false;
    } else {
      bool is_stale = false;
      if (latest_online_frame_recv_time_valid_) {
        const double age_ms =
            (ros::Time::now() - latest_online_frame_recv_wall_time_).toSec() * 1000.0;
        is_stale = age_ms > online_retarget_stale_timeout_ms_;
      }
      if (!is_stale) {
        return false;
      }
      enforce_standing = true;
      if (!online_stale_standing_active_) {
        online_stale_standing_active_ = true;
        online_stale_interp_progress_ = 0;
        online_stale_interp_start_frame_ = processed_frame;
        if (online_stale_interp_start_frame_.size() < vmp_standing_frame_.size()) {
          online_stale_interp_start_frame_.resize(vmp_standing_frame_.size(), 0.0f);
        }
        if (online_retarget_stale_require_reentry_) {
          online_retarget_stale_latched_ = true;
          ROS_ERROR("[%s] Retarget stream stale (>%.0f ms); blending to standing. "
                    "Teleop locked: switch to MPC/AMP, then re-enter VMP to resume.",
                    name_.c_str(), online_retarget_stale_timeout_ms_);
        } else {
          ROS_WARN("[%s] Retarget stream stale (>%.0f ms), blending reference to standing frame over %d samples",
                   name_.c_str(), online_retarget_stale_timeout_ms_,
                   online_stale_to_standing_interp_frames_);
        }
      }
    }

    if (!enforce_standing) {
      return false;
    }

    const int interp_frames = std::max(1, online_stale_to_standing_interp_frames_);
    if (online_stale_interp_progress_ < interp_frames) {
      const float alpha = static_cast<float>(online_stale_interp_progress_ + 1) /
                          static_cast<float>(interp_frames);
      processed_frame = interpolateFrame(online_stale_interp_start_frame_, vmp_standing_frame_, alpha);
      online_stale_interp_progress_++;
      if (online_stale_interp_progress_ >= interp_frames &&
          online_retarget_stale_latched_ && online_retarget_stale_require_reentry_) {
        ROS_WARN("[%s] Standing hold active after retarget stale. "
                 "Switch to MPC/AMP, then re-enter VMP to resume teleoperation.",
                 name_.c_str());
      }
    } else {
      processed_frame = vmp_standing_frame_;
    }
    return true;
  }

  void VMPController::prepareRetargetedStreamingResume(
      const Eigen::Quaterniond& robot_imu_quat,
      const ros::Time& robot_sensor_stamp)
  {
    const bool supports_yaw_resume_ack =
        isOnlineVRDeviceMode() &&
        (online_vr_data_source_ == "pico" || online_vr_data_source_ == "xsense");

    bool use_frozen_entry_frame = false;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      use_frozen_entry_frame = supports_yaw_resume_ack &&
                               pico_streaming_paused_.load() &&
                               retarget_auto_freeze_valid_;
    }

    // Keep source data canonical. Repeated entries anchor to the frame frozen
    // on VMP exit because the Python RT+X correction returns new GMR frames to
    // that heading. A first AMP->VMP entry anchors to the latest live frame.
    std::vector<float> entry_frame;
    bool entry_frame_is_live = false;
    if (use_frozen_entry_frame)
    {
      std::lock_guard<std::mutex> lock(pico_frozen_frame_mutex_);
      if (!pico_frozen_frame_.empty())
      {
        entry_frame = pico_frozen_frame_;
        entry_frame_is_live = true;
      }
    }
    if (!entry_frame_is_live)
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      if (has_received_online_data_ && !latest_online_raw_frame_.empty())
      {
        entry_frame = latest_online_raw_frame_;
        entry_frame_is_live = true;
      }
    }
    if (entry_frame.empty() && !vmp_standing_frame_.empty())
    {
      entry_frame = vmp_standing_frame_;
    }

    double reference_yaw = 0.0;
    const bool reference_yaw_valid =
        entry_frame_is_live && extractRetargetedFrameYaw(entry_frame, reference_yaw);
    Eigen::Quaterniond normalized_robot_quat = robot_imu_quat;
    const double robot_quat_norm = normalized_robot_quat.norm();
    const bool robot_yaw_valid = !robot_sensor_stamp.isZero() &&
                                 normalized_robot_quat.coeffs().allFinite() &&
                                 std::isfinite(robot_quat_norm) &&
                                 robot_quat_norm > 1e-8;
    double robot_yaw = 0.0;
    if (robot_yaw_valid)
    {
      normalized_robot_quat.normalize();
      const double w = normalized_robot_quat.w();
      const double x = normalized_robot_quat.x();
      const double y = normalized_robot_quat.y();
      const double z = normalized_robot_quat.z();
      robot_yaw = std::atan2(2.0 * (w * z + x * y),
                             1.0 - 2.0 * (y * y + z * z));
    }

    // A first-ever VMP entry has no automatic RT+Y freeze reference to
    // preserve, so it resumes immediately after installing this alignment.
    bool wait_for_corrected_frame = false;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      retarget_entry_frame_ = entry_frame;
      retarget_entry_frame_pending_ = supports_yaw_resume_ack && !entry_frame.empty();

      if (supports_yaw_resume_ack && robot_yaw_valid && reference_yaw_valid)
      {
        retarget_heading_offset_ = std::atan2(
            std::sin(robot_yaw - reference_yaw),
            std::cos(robot_yaw - reference_yaw));
        retarget_heading_offset_valid_ = true;
      }
      else if (supports_yaw_resume_ack)
      {
        // Never reuse a previous activation's robot heading when the new
        // source snapshot is unavailable.
        retarget_heading_offset_ = 0.0;
        retarget_heading_offset_valid_ = false;
      }
      else if (!supports_yaw_resume_ack)
      {
        retarget_heading_offset_ = 0.0;
        retarget_heading_offset_valid_ = false;
        retarget_entry_frame_pending_ = false;
        retarget_entry_frame_.clear();
      }

      wait_for_corrected_frame = use_frozen_entry_frame;
      if (wait_for_corrected_frame)
      {
        retarget_resume_waiting_ = true;
        retarget_resume_min_stamp_ = latest_retargeted_frame_stamp_;
        retarget_resume_ready_stamp_ = ros::Time(0);
        retarget_resume_min_frame_seq_ = latest_retargeted_frame_seq_;
      }
    }

    if (supports_yaw_resume_ack && robot_yaw_valid && reference_yaw_valid)
    {
      ROS_INFO("[%s] VMP entry reference aligned: robot_yaw=%.4f, reference_yaw=%.4f, offset=%.4f rad, sensor_age=%.3f s",
               name_.c_str(), robot_yaw, reference_yaw,
               std::atan2(std::sin(robot_yaw - reference_yaw),
                          std::cos(robot_yaw - reference_yaw)),
               std::max(0.0, (ros::Time::now() - robot_sensor_stamp).toSec()));
    }
    else if (supports_yaw_resume_ack)
    {
      ROS_WARN("[%s] VMP entry heading alignment unavailable (robot_imu=%s, live_reference=%s); alignment disabled for this activation",
               name_.c_str(), robot_yaw_valid ? "valid" : "invalid",
               reference_yaw_valid ? "valid" : "invalid");
    }

    if (!wait_for_corrected_frame)
    {
      resumeRetargetedStreaming();
      return;
    }

    ROS_INFO("[%s] Waiting for %s GMR yaw-resume frame before unfreezing retargeted streaming",
             name_.c_str(), online_vr_data_source_.c_str());
  }

  bool VMPController::extractRetargetedFrameYaw(const std::vector<float>& frame,
                                                 double& yaw) const
  {
    const int theta_start = vmp_config_.theta_start_id;
    if (theta_start < 0 || theta_start + 6 > static_cast<int>(frame.size()))
    {
      return false;
    }

    const double c0_x = static_cast<double>(frame[theta_start]);
    const double c0_y = static_cast<double>(frame[theta_start + 1]);
    if (!std::isfinite(c0_x) || !std::isfinite(c0_y) ||
        std::hypot(c0_x, c0_y) < 1e-8)
    {
      return false;
    }

    yaw = std::atan2(c0_y, c0_x);
    return true;
  }

  void VMPController::applyRetargetedHeadingAlignment(std::vector<float>& frame) const
  {
    if (static_cast<int>(frame.size()) < vmp_config_.in_c)
    {
      return;
    }
    applyRetargetedHeadingAlignment(frame.data(), 1);
  }

  void VMPController::applyRetargetedHeadingAlignment(float* frames,
                                                       int frame_count) const
  {
    if (frames == nullptr || frame_count <= 0 || vmp_config_.in_c <= 0)
    {
      return;
    }

    double yaw_offset = 0.0;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      if (!retarget_heading_offset_valid_)
      {
        return;
      }
      yaw_offset = retarget_heading_offset_;
    }

    const float c = static_cast<float>(std::cos(yaw_offset));
    const float s = static_cast<float>(std::sin(yaw_offset));
    const int frame_dim = vmp_config_.in_c;
    const int theta_start = vmp_config_.theta_start_id;
    const int v_start = vmp_config_.v_start_id;
    const int p_start = vmp_config_.p_start_id;

    const auto rotate_xy = [c, s](float* frame, int index)
    {
      const float x = frame[index];
      const float y = frame[index + 1];
      frame[index] = c * x - s * y;
      frame[index + 1] = s * x + c * y;
    };

    for (int frame_index = 0; frame_index < frame_count; ++frame_index)
    {
      float* frame = frames + frame_index * frame_dim;

      // rot6 is column-major [R[:,0], R[:,1]], so left-multiply each
      // stored column by the same world-frame yaw rotation.
      if (theta_start >= 0 && theta_start + 6 <= frame_dim)
      {
        rotate_xy(frame, theta_start);
        rotate_xy(frame, theta_start + 3);
      }

      // Base linear and angular velocities are both world-frame vectors.
      if (v_start >= 0 && v_start + 6 <= frame_dim)
      {
        rotate_xy(frame, v_start);
        rotate_xy(frame, v_start + 3);
      }

      // End-effector positions are four base-relative world-frame vectors;
      // rotate all of them without adding a translation.
      if (p_start >= 0 && p_start + 12 <= frame_dim)
      {
        for (int end_effector = 0; end_effector < 4; ++end_effector)
        {
          rotate_xy(frame, p_start + end_effector * 3);
        }
      }
    }
  }

  void VMPController::prefillRetargetedReferenceBuffers(
      const std::vector<float>& canonical_frame)
  {
    if (static_cast<int>(canonical_frame.size()) != vmp_config_.in_c)
    {
      return;
    }

    std::vector<float> aligned_frame = canonical_frame;
    applyRetargetedHeadingAlignment(aligned_frame);

    Eigen::VectorXd raw_frame(vmp_config_.in_c);
    for (int i = 0; i < vmp_config_.in_c; ++i)
    {
      raw_frame[i] = static_cast<double>(aligned_frame[i]);
    }
    Eigen::VectorXd normalized_frame = raw_frame;
    if (vmp_enable_theta_normalization_)
    {
      normalizeRefMotion(normalized_frame);
    }

    vmp_ref_motion_raw_buffer_.clear();
    vmp_ref_motion_buffer_.clear();
    for (int i = 0; i < vmp_config_.window_l; ++i)
    {
      vmp_ref_motion_raw_buffer_.push_back(raw_frame);
      vmp_ref_motion_buffer_.push_back(normalized_frame);
    }
  }

  void VMPController::recordRetargetedFrameStamp(const ros::Time& stamp)
  {
    bool resume_streaming = false;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      latest_retargeted_frame_stamp_ = stamp;
      ++latest_retargeted_frame_seq_;
      if (retarget_resume_waiting_ &&
          !retarget_resume_ready_stamp_.isZero() &&
          latest_retargeted_frame_seq_ > retarget_resume_min_frame_seq_ &&
          latest_retargeted_frame_stamp_ >= retarget_resume_ready_stamp_)
      {
        retarget_resume_waiting_ = false;
        retarget_frozen_frame_valid_ = false;
        retarget_auto_freeze_valid_ = false;
        pico_streaming_paused_.store(false);
        resume_streaming = true;
      }
    }

    if (resume_streaming)
    {
      ROS_INFO("[%s] Corrected GMR frame received; retargeted streaming resumed", name_.c_str());
    }
  }

  void VMPController::retargetYawResumeReadyCallback(const std_msgs::Header::ConstPtr& msg)
  {
    bool resume_streaming = false;
    {
      std::lock_guard<std::mutex> lock(retarget_resume_mutex_);
      if (!retarget_resume_waiting_ || msg->stamp < retarget_resume_min_stamp_)
      {
        return;
      }

      // Keep the first valid acknowledgement for this activation. Repeated
      // readiness messages are retries, not a moving timestamp threshold.
      if (retarget_resume_ready_stamp_.isZero())
      {
        retarget_resume_ready_stamp_ = msg->stamp;
      }
      if (latest_retargeted_frame_seq_ > retarget_resume_min_frame_seq_ &&
          latest_retargeted_frame_stamp_ >= retarget_resume_ready_stamp_)
      {
        retarget_resume_waiting_ = false;
        retarget_frozen_frame_valid_ = false;
        retarget_auto_freeze_valid_ = false;
        pico_streaming_paused_.store(false);
        resume_streaming = true;
      }
    }

    if (resume_streaming)
    {
      ROS_INFO("[%s] GMR yaw-resume acknowledged; retargeted streaming resumed", name_.c_str());
    }
  }

  void VMPController::onInterpolationComplete()
  {
    // MPC→VMP 插值完成后，重定位 buffer 读指针
    // 采样线程在插值期间已经稳定运行了 1~2 秒，write_index 已推进了很多帧
    // 将 read_index 重定位到 write_index - (future_frames+2) 处：
    //   - future_frames+1 是最小安全距离（读完整未来帧的必要条件）
    //   - +1 是 1 帧 margin，应对 PICO 偶发单帧丢包时不触发 WARN
    // 注意：插值期间推理线程不运行（humanoidController 走 MPC 流程），所以 read_index 不会被推进
    if (enable_online_vr_mode_) {
      std::lock_guard<std::mutex> lock(online_buffer_mutex_);
      int future_frames = vmp_config_.window_r; // e.g. 15
      int write_idx = online_ref_buffer_.current_write_index;
      // distance = future_frames+2（最小安全距离+1帧余量，与 initializeOnlineReferenceBuffer 一致）
      int offset = future_frames + 2;
      int new_read_index = (write_idx - offset + online_buffer_size_) % online_buffer_size_;
      online_ref_buffer_.current_read_index = new_read_index;
      ROS_INFO("[%s] Interpolation complete, repositioned read_index=%d (write=%d, future_frames=%d, offset=%d, latency=%.0fms)",
               name_.c_str(), new_read_index, write_idx, future_frames, offset,
               static_cast<float>(offset) * 1000.0 / online_update_rate_);
    }
    activateExternalArmControlIfNeeded();
  }

  void VMPController::pause()
  {
    RLControllerBase::pause();

    // Leaving VMP is equivalent to RT+Y. Keep this state if VMP is temporarily
    // resumed as the source of an RL-to-RL interpolation.
    if (isOnlineVRDeviceMode())
    {
      std::lock_guard<std::mutex> resume_lock(retarget_resume_mutex_);
      retarget_resume_waiting_ = false;
      retarget_resume_ready_stamp_ = ros::Time(0);

      if (!pico_streaming_paused_.load())
      {
        std::lock_guard<std::mutex> frozen_lock(pico_frozen_frame_mutex_);
        std::lock_guard<std::mutex> latest_lock(latest_frame_mutex_);
        retarget_frozen_frame_valid_ = has_received_online_data_ && !latest_online_raw_frame_.empty();
        if (retarget_frozen_frame_valid_)
        {
          pico_frozen_frame_ = latest_online_raw_frame_;
        }
        else if (!vmp_standing_frame_.empty())
        {
          pico_frozen_frame_ = vmp_standing_frame_;
        }
        pico_streaming_paused_.store(true);
        resetOnlineStaleFallbackState(false);
      }

      // Only a controller lifecycle exit creates an automatic-switch freeze.
      // Manual/CLI pause alone must not arm an automatic resume handshake.
      retarget_auto_freeze_valid_ = retarget_frozen_frame_valid_;
    }
    
    // 真实机器人暂停时切回正常 Ruiwo 电机参数（异步调用避免阻塞控制线程）
    if (is_real_ && use_vmp_ruiwo_kpkd_)
    {
      changeRuiwoMotorParamAsync("normal_kpkd");
    }
    
    // 如果是在线VR模式，停止采样线程
    if (enable_online_vr_mode_) {
      stopOnlineSampling();
    }
    
    ROS_INFO("[%s] VMP controller paused", name_.c_str());
  }

  void VMPController::changeRuiwoMotorParamAsync(const std::string& param_name)
  {
    // 仿真环境下直接返回
    if (!is_real_)
    {
      return;
    }

    // 在独立线程中调用服务，避免在控制循环中发生阻塞
    std::thread([this, param_name]()
    {
      try
      {
        const ros::Duration timeout(2.0);
        if (!srv_change_motor_param_.waitForExistence(timeout))
        {
          ROS_WARN_THROTTLE(1.0, "[%s] Motor param service not available (timeout: 2s)", name_.c_str());
          return;
        }

        kuavo_msgs::ExecuteArmAction srv;
        srv.request.action_name = param_name;

        if (srv_change_motor_param_.call(srv))
        {
          if (srv.response.success)
          {
            ROS_INFO("[VMPController] Successfully changed Ruiwo motor param to: %s", param_name.c_str());
          }
          else
          {
            ROS_WARN("[VMPController] Failed to change Ruiwo motor param: %s", srv.response.message.c_str());
          }
        }
        else
        {
          ROS_WARN("[VMPController] Failed to call Ruiwo motor param service");
        }
      }
      catch (const ros::Exception& e)
      {
        ROS_ERROR("[VMPController] ROS exception in changeRuiwoMotorParamAsync: %s", e.what());
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[VMPController] Exception in changeRuiwoMotorParamAsync: %s", e.what());
      }
    }).detach();
  }

  bool VMPController::requestToExit() const
  {
    // VMP模式下，如果轨迹播放完成且非循环模式，则可以退出
    if (!multiTrajectoryMode_ && !vmp_enable_loop_ && trajectoryPlaybackCompleted_) {
      return true;
    }
    return false;
  }
  bool VMPController::isAllowToExit() const
  {
    if (vmp_enable_loop_ || multiTrajectoryMode_) {
      return true;
    }
    return trajectoryPlaybackCompleted_;
  }

  bool VMPController::updateImpl(const ros::Time& time,
                                  const SensorData& sensor_data,
                                  const Eigen::VectorXd& measuredRbdState,
                                  kuavo_msgs::jointCmd& joint_cmd)
  {
    if (!vmp_initialized_) {
      ROS_ERROR_THROTTLE(1.0, "[%s] VMP not initialized", name_.c_str());
      return false;
    }
    
    // 计算执行器输出
    Eigen::VectorXd actuation = updateVMPcmd(measuredRbdState);
    
    // 转换为关节命令
    actionToJointCmd(actuation, measuredRbdState, joint_cmd);
    joint_cmd.header.stamp = time;
    if (ros_logger_) {
      std::vector<double> tau_values(joint_cmd.tau.begin(), joint_cmd.tau.end());
      ros_logger_->publishVector("/vmp_controller/joint_cmd_tau", tau_values);
    }
    return true;
  }

  bool VMPController::inference(const Eigen::VectorXd& observation,
                                 Eigen::VectorXd& action)
  {
    // VMP推理在单独的线程中通过vmpInference调用
    // 这里只是接口实现
    action = vmp_actions_;
    return true;
  }

  void VMPController::updateObservation(const Eigen::VectorXd& state_est,
                                        const SensorData& sensor_data)
  {
    // 这个函数在推理线程中被调用
    // 实际的VMP推理逻辑在vmpInference中
    vmpInference(state_est, sensor_data);
  }

  void VMPController::actionToJointCmd(const Eigen::VectorXd& actuation,
                                       const Eigen::VectorXd& measuredRbdState,
                                       kuavo_msgs::jointCmd& joint_cmd)
  {
    // 清空关节命令
    joint_cmd.joint_q.clear();
    joint_cmd.joint_v.clear();
    joint_cmd.tau.clear();
    joint_cmd.tau_ratio.clear();
    joint_cmd.tau_max.clear();
    joint_cmd.joint_kp.clear();
    joint_cmd.joint_kd.clear();
    joint_cmd.control_modes.clear();
    
    // 计算躯干关节总数（与 AmpWalkController/FallStandController 一致）
    // 包括：腿部 + 手臂 + 腰部（如果有）
    int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
    
    // 获取当前动作并扩展到 total_body_joints 维度
    Eigen::VectorXd raw_action = getCurrentAction();
    Eigen::VectorXd local_action(total_body_joints);
    local_action.setZero();
    int action_copy_size = std::min((int)raw_action.size(), total_body_joints);
    local_action.head(action_copy_size) = raw_action.head(action_copy_size);
    
    // 扩展 actuation 到 total_body_joints 维度
    Eigen::VectorXd local_actuation(total_body_joints);
    local_actuation.setZero();
    int actuation_copy_size = std::min((int)actuation.size(), total_body_joints);
    local_actuation.head(actuation_copy_size) = actuation.head(actuation_copy_size);
    
    // 检查参数向量维度（必须与 total_body_joints 匹配）
    if (vmp_jointKp_.size() < total_body_joints ||
        vmp_jointKd_.size() < total_body_joints ||
        vmp_torqueLimits_.size() < total_body_joints ||
        vmp_defalutJointPos_.size() < total_body_joints ||
        vmp_actionScaleTest_.size() < total_body_joints ||
        vmp_JointControlMode_.size() < total_body_joints) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Parameter vectors not properly initialized in actionToJointCmd", name_.c_str());
      ROS_ERROR_THROTTLE(1.0, "[%s] Sizes: jointKp=%zu, jointKd=%zu, torqueLimits=%zu, defPos=%zu, actionScaleTest=%zu, JointControlMode=%zu, total_body_joints=%d", 
                         name_.c_str(), vmp_jointKp_.size(), vmp_jointKd_.size(), vmp_torqueLimits_.size(), vmp_defalutJointPos_.size(), vmp_actionScaleTest_.size(), vmp_JointControlMode_.size(), total_body_joints);
      // 填充零值（包括头部）
      int total_joints_with_head = total_body_joints + headNum_;
      for (int i = 0; i < total_joints_with_head; i++) {
        joint_cmd.tau.push_back(0.0);
        joint_cmd.tau_ratio.push_back(1.0);
        joint_cmd.tau_max.push_back(100.0);
        joint_cmd.joint_kp.push_back(0.0);
        joint_cmd.joint_kd.push_back(0.0);
        joint_cmd.control_modes.push_back(0);
        joint_cmd.joint_q.push_back(0.0);
        joint_cmd.joint_v.push_back(0.0);
      }
      return;
    }
    
    // 获取传感器数据（用于 CSP 模式需要当前位置）
    SensorData sensor_data = getRobotSensorData();
    Eigen::VectorXd current_jointPos = sensor_data.jointPos_.head(total_body_joints);
    
    // 填充躯干关节命令（与 AmpWalkController/FallStandController 一致）
    if (!is_real_) {
      // 仿真环境
      for (int i = 0; i < total_body_joints; ++i) {
        joint_cmd.joint_q.push_back(0.0);
        joint_cmd.joint_v.push_back(0.0);
        joint_cmd.joint_kp.push_back(vmp_jointKp_[i]);
        joint_cmd.joint_kd.push_back(vmp_jointKd_[i]);
        joint_cmd.tau.push_back(local_actuation[i]);
        joint_cmd.tau_ratio.push_back(1.0);
        joint_cmd.tau_max.push_back(vmp_torqueLimits_[i]);
        joint_cmd.control_modes.push_back(static_cast<int>(vmp_JointControlMode_[i]));
      }
    } else {
      // 真实机器人
      for (int i = 0; i < total_body_joints; ++i) {
        int control_mode = static_cast<int>(vmp_JointControlMode_[i]);
        int pd_mode = (i < vmp_JointPDMode_.size()) ? static_cast<int>(vmp_JointPDMode_[i]) : 0;
        
        if (control_mode == 0) {
          if (pd_mode == 0) {
            // CST 纯力矩模式
            joint_cmd.joint_q.push_back(0.0);
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(0.0);
            joint_cmd.joint_kd.push_back(0.0);
            joint_cmd.tau.push_back(local_actuation[i]);
            joint_cmd.tau_ratio.push_back(1.0);
            joint_cmd.tau_max.push_back(vmp_torqueLimits_[i]);
            joint_cmd.control_modes.push_back(control_mode);
          } else {
            // CST PD 模式：位置通过 actuation 传入
            joint_cmd.joint_q.push_back(local_actuation[i]);
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(vmp_jointKp_[i]);
            joint_cmd.joint_kd.push_back(vmp_jointKd_[i]);
            joint_cmd.tau.push_back(0.0);
            joint_cmd.tau_ratio.push_back(1.0);
            joint_cmd.tau_max.push_back(vmp_torqueLimits_[i]);
            joint_cmd.control_modes.push_back(control_mode);
          }
        } else if (control_mode == 2) {
          // CSP 模式
          joint_cmd.joint_q.push_back(current_jointPos[i]);
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(vmp_jointKp_[i]);
          joint_cmd.joint_kd.push_back(vmp_jointKd_[i]);
          joint_cmd.tau.push_back(local_actuation[i]);
          joint_cmd.tau_ratio.push_back(1.0);
          joint_cmd.tau_max.push_back(vmp_torqueLimits_[i]);
          joint_cmd.control_modes.push_back(control_mode);
        } else {
          // 其他模式：零输出
          joint_cmd.joint_q.push_back(0.0);
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(0.0);
          joint_cmd.joint_kd.push_back(0.0);
          joint_cmd.tau.push_back(0.0);
          joint_cmd.tau_ratio.push_back(1.0);
          joint_cmd.tau_max.push_back(vmp_torqueLimits_[i]);
          joint_cmd.control_modes.push_back(control_mode);
        }
      }
    }
    
    // 设置头部关节（保持零位，与 AmpWalkController/FallStandController 一致）
    for (int i = 0; i < headNum_; ++i) {
      joint_cmd.joint_q.push_back(0.0);
      joint_cmd.joint_v.push_back(0.0);
      joint_cmd.tau.push_back(0.0);
      joint_cmd.tau_ratio.push_back(1.0);
      joint_cmd.tau_max.push_back(10.0);
      joint_cmd.joint_kp.push_back(0.0);
      joint_cmd.joint_kd.push_back(0.0);
      joint_cmd.control_modes.push_back(0);
    }
    
    // 腰部关节处理说明：
    // VMP模型的输入/输出顺序与硬件顺序一致：[腿(0-11), 腰(12), 手臂(13-26)]
    // 因此不需要像 AmpWalkController/FallStandController 那样进行腰部重排
    // 注意：腰部方向是否需要取反取决于VMP模型训练时的坐标系定义
    // 如果VMP模型训练时与硬件方向一致，则不需要取反
    // 目前VMP模型训练时已考虑硬件方向，不需要取反
  }

  void VMPController::preprocessSensorData(SensorData& sensor_data)
  {
    // 调用基类的IMU滤波
    RLControllerBase::preprocessSensorData(sensor_data);
    
    // 腰部关节处理说明：
    // VMP模型的输入顺序与硬件顺序一致：[腿(0-11), 腰(12), 手臂(13-26)]
    // 因此不需要像 AmpWalkController/FallStandController 那样进行腰部重排
    // 注意：腰部方向是否需要取反取决于VMP模型训练时的坐标系定义
    // 如果VMP模型训练时与硬件方向一致，则不需要取反
    // 目前VMP模型训练时已考虑硬件方向，不需要取反
  }

  bool VMPController::shouldRunInference() const
  {
    // VMP总是需要运行推理
    return vmp_initialized_ && RLControllerBase::shouldRunInference();
  }

  // ========== VMP核心推理函数实现 ==========
  
  void VMPController::vmpInference(const Eigen::VectorXd& state_est, 
                                   const SensorData& sensor_data)
  {
    if (!vmp_initialized_) {
      return;
    }
    
    try {
      // 观测和动作维度都使用 numActions_（从配置文件读取）
      // v46: 26 (腿12 + 手臂14)
      // v52: 27 (腿12 + 腰1 + 手臂14)，顺序：腿(0-11) + 腰(12) + 手臂(13-26)
      int joint_dim = numActions_;
      
      // 检查传感器数据维度
      if (sensor_data.jointPos_.size() < joint_dim || 
          sensor_data.jointVel_.size() < joint_dim) {
        ROS_ERROR_THROTTLE(1.0, "[%s] Sensor data dimension mismatch: jointPos=%ld, jointVel=%ld, need=%d",
                           name_.c_str(), sensor_data.jointPos_.size(), sensor_data.jointVel_.size(), joint_dim);
        return;
      }
      
      // 检查状态估计维度
      int min_state_size = 12 + 2 * joint_dim;
      if (state_est.size() < min_state_size) {
        ROS_ERROR_THROTTLE(1.0, "[%s] State estimate too small: %ld, need at least %d", 
                           name_.c_str(), state_est.size(), min_state_size);
        return;
      }
      
      Eigen::VectorXd robot_observation(vmp_numSingleObs_);
      robot_observation.setZero();

      // state_est: [yaw(0), pitch(1), roll(2), ...]
      const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));  // roll, pitch, yaw
      // 从传感器数据获取角速度
      const Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;

      // 关节信息从传感器数据获取
      Eigen::VectorXd current_jointPos = sensor_data.jointPos_.head(joint_dim);
      Eigen::VectorXd current_jointVel = sensor_data.jointVel_.head(joint_dim);

      // 观测索引
      // 观测格式: [joint_pos_offset(N), joint_vel(N), last_actions(N), ang_vel(3), euler_w(3), anchor_euler_b(3)]
      int dof_pos_idx = 0;
      int dof_vel_idx = dof_pos_idx + joint_dim;
      int actions_idx = dof_vel_idx + joint_dim;
      int ang_vel_idx = actions_idx + joint_dim;
      int anchor_euler_idx = ang_vel_idx + 6;  // ang_vel(3) + robot_anchor_euler_w(3)

      // 关节位置偏移（先 clip 再 scale，与训练 ObsTerm 一致）
      if (vmp_defalutJointPos_.size() >= joint_dim) {
        Eigen::VectorXd joint_pos_offset = current_jointPos - vmp_defalutJointPos_.head(joint_dim);
        robot_observation.segment(dof_pos_idx, joint_dim) =
            applyVmpObsTerm(joint_pos_offset, vmp_obsScale_dof_pos_, vmp_clipObservations_);
      }

      // 关节速度（先 clip 再 scale）
      robot_observation.segment(dof_vel_idx, joint_dim) =
          applyVmpObsTerm(current_jointVel, vmp_obsScale_dof_vel_, vmp_clipObservations_);

      // 上一次动作（scale + offset + clip，与训练 last_processed_action 一致，不再做 ObsTerm clip/scale）
      if (vmp_actions_.size() >= joint_dim &&
          vmp_actionScaleTest_.size() >= joint_dim &&
          vmp_defalutJointPos_.size() >= joint_dim) {
        // robot_observation.segment(actions_idx, joint_dim) = vmp_actions_.head(joint_dim);
        robot_observation.segment(actions_idx, joint_dim) =
            (vmp_actions_.head(joint_dim).cwiseProduct(vmp_actionScaleTest_.head(joint_dim)) * vmp_actionScale_
             + vmp_defalutJointPos_.head(joint_dim))
                .cwiseMax(-vmp_clipActions_)
                .cwiseMin(vmp_clipActions_);
      }

      // 角速度、姿态（先 clip 再 scale）
      robot_observation.segment(ang_vel_idx, 3) =
          applyVmpObsTerm(bodyAngVel, vmp_obsScale_base_ang_vel_, vmp_clipObservations_);

      robot_observation.segment(ang_vel_idx + 3, 3) =
          applyVmpObsTerm(baseEuler, vmp_obsScale_quat_, vmp_clipObservations_);
      // motion_anchor_euler_b
      robot_observation.segment(anchor_euler_idx, 3) =
          applyVmpObsTerm(computeAnchorEulerB(sensor_data), vmp_obsScale_quat_, vmp_clipObservations_);
      // 验证观测维度
      int expected_dim = ang_vel_idx + 3 + 3 + 3;  // 3*joint_dim + 9
      if (expected_dim != vmp_numSingleObs_) {
        ROS_WARN_THROTTLE(5.0, "[%s] Observation dimension mismatch! Built %d, expected %d",
                          name_.c_str(), expected_dim, vmp_numSingleObs_);
      }

      // 更新 Estimator 观测缓存
      if (vmp_enable_estimator_) {
        updateEstimatorObservation(robot_observation, sensor_data);
      }
      
      // 计算VMP动作
      Eigen::VectorXd vmp_computed_actions = computeVMPAction(robot_observation);
      
      // 更新动作
      if (static_cast<int>(vmp_computed_actions.size()) == numActions_) {
        std::lock_guard<std::mutex> lock(action_mtx_);
        
        // 确保 vmp_actions_ 和 actions_ 大小正确
        if (vmp_actions_.size() != numActions_) {
          vmp_actions_.resize(numActions_);
          vmp_actions_.setZero();
        }
        if (actions_.size() != numActions_) {
          actions_.resize(numActions_);
          actions_.setZero();
        }
        
        // 直接复制动作
        vmp_actions_ = vmp_computed_actions;
        actions_ = vmp_computed_actions;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[%s] VMP computed actions size mismatch: expected %d, got %ld",
                           name_.c_str(), numActions_, vmp_computed_actions.size());
      }
      
      // 更新参考运动
      updateVMPReferenceMotion();
      
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error in VMP inference: %s", name_.c_str(), e.what());
    }
  }

  Eigen::VectorXd VMPController::computeVMPAction(const Eigen::VectorXd& observation)
  {
    try {
      // 准备编码器输入
      ov::Shape encoder_input_shape = {1, static_cast<size_t>(vmp_config_.in_c), 
                                       static_cast<size_t>(vmp_config_.window_l)};
      ov::Tensor encoder_input_tensor(ov::element::f32, encoder_input_shape);
      vmp_encoder_request_.set_input_tensor(0, encoder_input_tensor);
      
      float* input_data = encoder_input_tensor.data<float>();

      // 从缓冲区或轨迹数据读取窗口数据
      // VAE 窗口定义：future_frames 为 VAE 查看未来帧数，past_frames 为历史帧数
      // 窗口偏移范围为 [-past_frames, +future_frames]，当前帧位于索引 past_frames
      int future_frames = vmp_config_.window_r;                   // e.g. 15（未来帧）
      int past_frames = vmp_config_.window_l - future_frames - 1; // e.g. 16（历史帧，不含当前帧）
      // window布局：[0, past-1]=历史, [past]=当前帧, [past+1, window_l-1]=未来
      // 共 past + 1 + future = window_l 帧
      std::vector<float> sample_batch(vmp_config_.window_l * vmp_config_.in_c);
      bool data_valid = false;  // 标记数据是否有效

      if (enable_online_vr_mode_ && online_ref_buffer_.is_initialized) {
        // 在线模式：读取缓冲区数据
        // 【关键优化】将锁的持有时间最小化：
        //   锁内只做索引操作（推进 read_index、做距离检查），微秒级完成后立即释放；
        //   锁外再按计算好的索引读取帧数据。
        //   由于环形缓冲区距离充足（write 始终领先 read 至少 future_frames+1），
        //   write 线程写入的槽位永远不与 read 线程读取的槽位重叠，故无需持锁读帧数据。
        //   这样消除了写线程（采样线程）因等待锁而丢失 100Hz 节拍的问题。
        int current_frame_index;
        int max_future_offset;
        bool pointer_tight = false;
        int write_index = 0;
        int pointer_distance = 0;
        int min_safe_distance = future_frames + 1;
        {
          std::lock_guard<std::mutex> lock(online_buffer_mutex_);

          current_frame_index = online_ref_buffer_.current_read_index;
          write_index = online_ref_buffer_.current_write_index;

          // 计算读写指针距离（环形缓冲区）
          pointer_distance = (write_index - current_frame_index + online_buffer_size_) % online_buffer_size_;

          // 安全检查：确保写指针比读指针超前至少 future_frames+1 帧
          pointer_tight = (pointer_distance < min_safe_distance);

          // 计算可安全读取的最远未来帧偏移量
          max_future_offset = future_frames;
          if (pointer_tight) {
            max_future_offset = std::min(future_frames, std::max(0, pointer_distance - 1));
          }

          // 推进读指针：采样已就绪且间距充足时才推进。
          // - 采样启动前（~30ms）：read 不动，等待 write 先行建立正确间距。
          // - pointer_tight：冻结 read，等待 write 重新拉开 >= future_frames+1，
          //   避免 clamp 未来帧的同时继续消费 buffer 导致 distance 归零。
          if (online_sampling_has_written_.load(std::memory_order_acquire) && !pointer_tight) {
            online_ref_buffer_.current_read_index = (current_frame_index + 1) % online_buffer_size_;
          }
        }
        if (pointer_tight) {
          ROS_WARN_THROTTLE(1.0, "[%s] Online buffer pointer distance tight! "
                            "read=%d, write=%d, distance=%d, min_safe=%d, max_future_allowed=%d. "
                            "Clamping future frames and freezing read pointer.",
                            name_.c_str(), current_frame_index, write_index,
                            pointer_distance, min_safe_distance, max_future_offset);
        }
        // 锁已释放——读取帧数据（无锁，ring buffer distance 保证无竞争槽位）
        for (int t = 0; t < vmp_config_.window_l; ++t) {
          int offset = t - past_frames;
          int clamped_offset = std::min(offset, max_future_offset);
          int actual_index = (current_frame_index + clamped_offset + online_buffer_size_) % online_buffer_size_;

          const std::vector<float>& frame_data = online_ref_buffer_.data_buffer[actual_index];
          int sample_base_idx = t * vmp_config_.in_c;
          for (int f = 0; f < vmp_config_.in_c; ++f) {
            sample_batch[sample_base_idx + f] = (f < static_cast<int>(frame_data.size())) ? frame_data[f] : 0.0f;
          }
        }
        data_valid = true;
      } else {
        // 离线模式：从轨迹数据读取
        // 注意：帧计数器的递增和重置在 updateVMPReferenceMotion 中处理
        // 这里只需要读取数据并应用循环/边界处理
        int total_frames = vmp_task_data_.size() / vmp_config_.in_c;
        int current_frame = current_task_idx_;
        
        // 安全检查：如果没有轨迹数据，使用静止帧填充
        if (total_frames <= 0) {
          ROS_WARN_THROTTLE(1.0, "[%s] No trajectory data available, using standing frame", name_.c_str());
          for (int t = 0; t < vmp_config_.window_l; ++t) {
            int sample_base_idx = t * vmp_config_.in_c;
            for (int f = 0; f < vmp_config_.in_c; ++f) {
              sample_batch[sample_base_idx + f] = 
                  (f < static_cast<int>(vmp_standing_frame_.size())) ? vmp_standing_frame_[f] : 0.0f;
            }
          }
          data_valid = true;
        } else {
          // 应用循环播放和边界处理逻辑（与 updateVMPReferenceMotion 保持一致）
          if (vmp_enable_loop_) {
            current_frame = current_frame % total_frames;
          } else {
            if (current_frame >= total_frames) {
              // 轨迹播放完成，停止在最后一帧
              current_frame = total_frames - 1;
            }
          }
          
          for (int t = 0; t < vmp_config_.window_l; ++t) {
            int frame_idx = current_frame + (t - past_frames);
            frame_idx = std::clamp(frame_idx, 0, total_frames - 1);
          
            int sample_base_idx = t * vmp_config_.in_c;
            int task_base_idx = frame_idx * vmp_config_.in_c;
          
            for (int f = 0; f < vmp_config_.in_c; ++f) {
              sample_batch[sample_base_idx + f] = vmp_task_data_[task_base_idx + f];
            }
          }
          data_valid = true;  // 离线模式数据有效
        }
      }
      
      // 如果数据无效（仅在线模式可能发生），使用静止帧填充
      if (!data_valid) {
        // 使用静止帧填充窗口数据，VAE编码器仍能产生合理的latent code
        for (int t = 0; t < vmp_config_.window_l; ++t) {
          int sample_base_idx = t * vmp_config_.in_c;
          for (int f = 0; f < vmp_config_.in_c; ++f) {
            sample_batch[sample_base_idx + f] = 
                (f < static_cast<int>(vmp_standing_frame_.size())) ? vmp_standing_frame_[f] : 0.0f;
          }
        }
        ROS_WARN_THROTTLE(1.0, "[%s] Using standing frame as fallback for VAE input", name_.c_str());
      }

      // Re-express online reference copies in the robot's AMP-entry heading.
      // Canonical latest/frozen/ring frames are deliberately left unchanged.
      if (enable_online_vr_mode_) {
        applyRetargetedHeadingAlignment(sample_batch.data(), vmp_config_.window_l);
      }
      
      // 应用时间归一化
      applyTemporalNormalization(sample_batch.data(), sample_batch.size());
      
      // 转置数据
      for (int f = 0; f < vmp_config_.in_c; ++f) {
        for (int t = 0; t < vmp_config_.window_l; ++t) {
          int src_idx = t * vmp_config_.in_c + f;
          int dst_idx = f * vmp_config_.window_l + t;
          input_data[dst_idx] = sample_batch[src_idx];
        }
      }
      
      // 编码器推理
      vmp_encoder_request_.infer();
      auto encoder_output = vmp_encoder_request_.get_output_tensor(0);
      const float* encoder_data = encoder_output.data<float>();
      
      // ===== 日志记录: VAE 输入输出 =====
      if (ros_logger_) {
        // VAE 输入: 当前帧数据 (取窗口中心帧)
  int center_frame_idx = past_frames * vmp_config_.in_c;
        std::vector<double> vae_input_center_frame(vmp_config_.in_c);
        for (int i = 0; i < vmp_config_.in_c; ++i) {
          vae_input_center_frame[i] = static_cast<double>(sample_batch[center_frame_idx + i]);
        }
        ros_logger_->publishVector("/vmp_controller/vae_input_center_frame", vae_input_center_frame);
        
        // VAE 输出: latent code
        std::vector<double> vae_output(vmp_config_.latent_d);
        for (int i = 0; i < vmp_config_.latent_d && i < static_cast<int>(encoder_output.get_size()); ++i) {
          vae_output[i] = static_cast<double>(encoder_data[i]);
        }
        ros_logger_->publishVector("/vmp_controller/vae_output_latent", vae_output);
      }
      
      // 运行Estimator推理（如果启用）
      Eigen::VectorXd estimator_output;
      if (vmp_enable_estimator_) {
        estimator_output = runEstimatorInference();
      }
      
      // 准备策略输入
      // 格式: observation + refMotion + vaeLatent + [estimatorOutput]
      int policy_input_size = observation.size() + numRefMotionObs_ + vmp_config_.latent_d;
      if (vmp_enable_estimator_) {
        policy_input_size += vmp_estimator_output_dim_;
      }
      Eigen::VectorXd policy_input(policy_input_size);
      
      int idx = 0;
      policy_input.segment(idx, observation.size()) = observation;
      idx += observation.size();
      
      // 参考运动
      if (vmp_ref_motion_buffer_.size() > 0) {
        policy_input.segment(idx, numRefMotionObs_) = vmp_ref_motion_buffer_.back().head(numRefMotionObs_);
      } else {
        policy_input.segment(idx, numRefMotionObs_).setZero();
      }
      idx += numRefMotionObs_;
      
      // VAE编码
      for (int i = 0; i < vmp_config_.latent_d && i < encoder_output.get_size(); ++i) {
        policy_input[idx + i] = static_cast<double>(encoder_data[i]);
      }
      idx += vmp_config_.latent_d;
      
      // Estimator输出（如果启用）
      if (vmp_enable_estimator_ && estimator_output.size() > 0) {
        int est_copy_size = std::min(vmp_estimator_output_dim_, (int)estimator_output.size());
        policy_input.segment(idx, est_copy_size) = estimator_output.head(est_copy_size);
        
        // ===== 日志记录: Estimator 输出 =====
        if (ros_logger_) {
          std::vector<double> est_out(estimator_output.data(), estimator_output.data() + estimator_output.size());
          ros_logger_->publishVector("/vmp_controller/estimator_output", est_out);
        }
      }
      
      // ===== 日志记录: Policy 输入各部分 =====
      if (ros_logger_) {
        // 机器人观测部分
        std::vector<double> obs_part(observation.data(), observation.data() + observation.size());
        ros_logger_->publishVector("/vmp_controller/policy_input_observation", obs_part);
        
        // 参考运动部分
        if (vmp_ref_motion_buffer_.size() > 0) {
          Eigen::VectorXd ref_motion = vmp_ref_motion_buffer_.back().head(numRefMotionObs_);
          std::vector<double> ref_part(ref_motion.data(), ref_motion.data() + ref_motion.size());
          ros_logger_->publishVector("/vmp_controller/policy_input_ref_motion", ref_part);
        }
        
        // 完整 Policy 输入
        std::vector<double> policy_in(policy_input.data(), policy_input.data() + policy_input.size());
        ros_logger_->publishVector("/vmp_controller/policy_input_full", policy_in);
      }
      
      // 策略推理
      ov::Shape policy_input_shape = {1, static_cast<size_t>(policy_input.size())};
      ov::Tensor policy_input_tensor(ov::element::f32, policy_input_shape);
      vmp_policy_request_.set_input_tensor(0, policy_input_tensor);
      
      float* policy_input_data = policy_input_tensor.data<float>();
      for (int i = 0; i < policy_input.size(); ++i) {
        policy_input_data[i] = static_cast<float>(policy_input[i]);
      }
      
      vmp_policy_request_.infer();
      auto policy_output = vmp_policy_request_.get_output_tensor(0);
      const float* policy_data = policy_output.data<float>();
      
      // 提取动作（使用配置的动作维度）
      Eigen::VectorXd actions(numActions_);
      int copy_size = std::min(numActions_, (int)policy_output.get_size());
      for (int i = 0; i < copy_size; ++i) {
        actions[i] = static_cast<double>(policy_data[i]);
      }
      
      // 裁剪动作
      actions = actions.cwiseMax(-vmp_clipActions_).cwiseMin(vmp_clipActions_);
      
      // ===== 日志记录: Policy 输出 =====
      if (ros_logger_) {
        std::vector<double> policy_out(actions.data(), actions.data() + actions.size());
        ros_logger_->publishVector("/vmp_controller/policy_output_actions", policy_out);
      }
      
      return actions;
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Error in computeVMPAction: %s", name_.c_str(), e.what());
      return Eigen::VectorXd::Zero(numActions_);
    }
  }

  void VMPController::updateVMPReferenceMotion()
  {
    // ========== 在线模式：从环形缓冲区读取当前帧 ==========
    if (enable_online_vr_mode_) {
      // 【注意】读取指针的移动已经在VAE编码器输入准备时处理（computeVMPAction中）
      // 这里只负责将当前帧添加到vmp_ref_motion_buffer_用于policy输入
      std::vector<float> frame_data;
      {
      std::lock_guard<std::mutex> lock(online_buffer_mutex_);
      
      if (!online_ref_buffer_.is_initialized) {
        ROS_WARN_THROTTLE(1.0, "[%s] Online buffer not initialized", name_.c_str());
        return;
      }
      
        // Copy under the ring-buffer lock, then release it before taking the
        // independent entry-alignment state lock.
      int read_index = online_ref_buffer_.current_read_index;
      if (read_index < 0 || read_index >= static_cast<int>(online_ref_buffer_.data_buffer.size())) {
        ROS_WARN_THROTTLE(1.0, "[%s] Invalid read index %d", name_.c_str(), read_index);
        return;
        }
        frame_data = online_ref_buffer_.data_buffer[read_index];
      }
      
      applyRetargetedHeadingAlignment(frame_data);
      
      if (static_cast<int>(frame_data.size()) == vmp_config_.in_c) {
        // 转换为Eigen::VectorXd
        Eigen::VectorXd current_ref_frame_raw(vmp_config_.in_c);
        for (int i = 0; i < vmp_config_.in_c; ++i) {
          current_ref_frame_raw[i] = static_cast<double>(frame_data[i]);
        }
        Eigen::VectorXd current_ref_frame = current_ref_frame_raw;
        if (vmp_enable_theta_normalization_) {
          normalizeRefMotion(current_ref_frame);
        }
        
        // 维护窗口大小
        if (static_cast<int>(vmp_ref_motion_buffer_.size()) >= vmp_config_.window_l) {
          vmp_ref_motion_buffer_.erase(vmp_ref_motion_buffer_.begin());
        }
        if (static_cast<int>(vmp_ref_motion_raw_buffer_.size()) >= vmp_config_.window_l) {
          vmp_ref_motion_raw_buffer_.erase(vmp_ref_motion_raw_buffer_.begin());
        }
        vmp_ref_motion_raw_buffer_.push_back(current_ref_frame_raw);
        vmp_ref_motion_buffer_.push_back(current_ref_frame);
      }
      
      return;
    }
    
    // ========== 离线模式：从预加载轨迹数据读取 ==========
    int total_frames = vmp_task_data_.size() / vmp_config_.in_c;
    if (total_frames == 0) return;
    
    // 检查是否需要重置帧计数器
    if (resetSingleTrajectoryFrame_) {
      current_task_idx_ = 0;
      resetSingleTrajectoryFrame_ = false;
      trajectoryPlaybackCompleted_ = false;
    }
    
    int current_frame = current_task_idx_;
    
    if (vmp_enable_loop_) {
      current_frame = current_task_idx_ % total_frames;
    } else {
      if (current_frame >= total_frames) {
        trajectoryPlaybackCompleted_ = true;
        current_frame = total_frames - 1;
      }
    }
    
    Eigen::VectorXd current_ref_frame_raw(vmp_config_.in_c);
    int start_idx = current_frame * vmp_config_.in_c;
    
    for (int i = 0; i < vmp_config_.in_c; ++i) {
      int data_idx = start_idx + i;
      if (data_idx < static_cast<int>(vmp_task_data_.size())) {
        current_ref_frame_raw[i] = static_cast<double>(vmp_task_data_[data_idx]);
      } else {
        current_ref_frame_raw[i] = 0.0;
      }
    }
    Eigen::VectorXd current_ref_frame = current_ref_frame_raw;
    if (vmp_enable_theta_normalization_) {
      normalizeRefMotion(current_ref_frame);
    }
    
    // 更新参考运动缓冲区
    if (static_cast<int>(vmp_ref_motion_buffer_.size()) >= vmp_config_.window_l) {
      vmp_ref_motion_buffer_.erase(vmp_ref_motion_buffer_.begin());
    }
    if (static_cast<int>(vmp_ref_motion_raw_buffer_.size()) >= vmp_config_.window_l) {
      vmp_ref_motion_raw_buffer_.erase(vmp_ref_motion_raw_buffer_.begin());
    }
    vmp_ref_motion_raw_buffer_.push_back(current_ref_frame_raw);
    vmp_ref_motion_buffer_.push_back(current_ref_frame);
    
    // 只有在轨迹未完成时才递增帧计数器
    if (!trajectoryPlaybackCompleted_) {
      current_task_idx_++;
    }
  }

  Eigen::VectorXd VMPController::updateVMPcmd(const Eigen::VectorXd& state)
  {
    // 计算躯干关节总数（与 AmpWalkController/FallStandController 一致）
    int total_joints = jointNum_ + jointArmNum_ + waistNum_;
    Eigen::VectorXd actuation(total_joints);
    Eigen::VectorXd cmd(total_joints);
    Eigen::VectorXd cmd_filter(total_joints);
    Eigen::VectorXd cmd_out(total_joints);
    Eigen::VectorXd torque(total_joints);
    actuation.setZero();
    cmd.setZero();
    cmd_filter.setZero();
    cmd_out.setZero();
    torque.setZero();
    
    // 使用 getRobotSensorData() 获取传感器数据（与AmpWalkController/FallStandController一致）
    SensorData sensor_data = getRobotSensorData();
    
    // 检查传感器数据维度
    if (sensor_data.jointPos_.size() < total_joints || 
        sensor_data.jointVel_.size() < total_joints) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Sensor data vector too small: jointPos=%ld, jointVel=%ld, need at least %d", 
                         name_.c_str(), sensor_data.jointPos_.size(), sensor_data.jointVel_.size(), total_joints);
      return cmd_out;
    }
    
    Eigen::VectorXd jointPos = sensor_data.jointPos_.head(total_joints);
    Eigen::VectorXd jointVel = sensor_data.jointVel_.head(total_joints);
    
    // AnkleSolver 电机位置/速度转换（与 AmpWalkController/FallStandController/humanoidController 一致）
    Eigen::VectorXd motorPos = jointPos;
    Eigen::VectorXd motorVel = jointVel;
    
    // 获取动作并扩展到 total_joints 维度
    Eigen::VectorXd raw_action = getCurrentAction();
    Eigen::VectorXd local_action(total_joints);
    local_action.setZero();
    
    // 复制动作，处理维度不匹配情况
    // VMP 策略可能只输出 26 维（腿+手臂），需要扩展到 27 维（加腰部）
    int copy_size = std::min((int)raw_action.size(), total_joints);
    local_action.head(copy_size) = raw_action.head(copy_size);
    
    // 检查参数向量维度（必须与 total_joints 匹配）
    if (vmp_jointKp_.size() < total_joints ||
        vmp_jointKd_.size() < total_joints ||
        vmp_torqueLimits_.size() < total_joints ||
        vmp_defalutJointPos_.size() < total_joints ||
        vmp_actionScaleTest_.size() < total_joints ||
        vmp_JointControlMode_.size() < total_joints) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Parameter vectors not properly initialized in updateVMPcmd", name_.c_str());
      ROS_ERROR_THROTTLE(1.0, "[%s]   vmp_jointKp_: %ld, vmp_jointKd_: %ld, total_joints: %d",
                         name_.c_str(), vmp_jointKp_.size(), vmp_jointKd_.size(), total_joints);
      ROS_ERROR_THROTTLE(1.0, "[%s]   vmp_torqueLimits_: %ld, vmp_defalutJointPos_: %ld, vmp_actionScaleTest_: %ld",
                         name_.c_str(), vmp_torqueLimits_.size(), vmp_defalutJointPos_.size(), vmp_actionScaleTest_.size());
      return cmd_out;
    }
    
    // AnkleSolver 处理
    // 注意：VMP模型的顺序与硬件一致：[腿(0-11), 腰(12), 手臂(13-26)]
    // 因此腿部关节始终从索引 0 开始，不需要像 AmpWalkController 那样区分
    // （AMP模型训练顺序是 [腰, 腿, 手臂]，需要重排后腿部从 waistNum_ 开始）
    Eigen::VectorXd jointTor(total_joints);
    jointTor.setZero();
    
    // VMP: 腿部始终从索引 0 开始（无论 v46 还是 v52）
    motorPos.head(jointNum_) = 
        ankleSolver_.joint_to_motor_position(jointPos.head(jointNum_));
    motorVel.head(jointNum_) = 
        ankleSolver_.joint_to_motor_velocity(jointPos.head(jointNum_),
                                             motorPos.head(jointNum_),
                                             jointVel.head(jointNum_));
    jointTor = -(vmp_jointKd_.cwiseProduct(motorVel));
    jointTor.head(jointNum_) = 
        ankleSolver_.motor_to_joint_torque(jointPos.head(jointNum_),
                                           motorPos.head(jointNum_),
                                           jointTor.head(jointNum_));
    
    // 计算关节力矩（与 humanoidController::updateVMPcmd 一致）
    for (int i = 0; i < total_joints; i++) {
      jointTor(i) = jointTor(i) + vmp_jointKp_(i) * (local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] 
                    - jointPos[i] + vmp_defalutJointPos_[i]);
    }
    
    // 根据实物/仿真模式计算输出命令（与 humanoidController::updateVMPcmd 一致）
    if (is_real_) {
      for (int i = 0; i < total_joints; i++) {
        int control_mode = static_cast<int>(vmp_JointControlMode_(i));
        int pd_mode = (i < vmp_JointPDMode_.size()) ? static_cast<int>(vmp_JointPDMode_(i)) : 0;
        
        if (control_mode == 0) {
          if (pd_mode == 0) {
            // CST 纯力矩模式
            cmd[i] = vmp_jointKp_[i] * (local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] 
                     - jointPos[i] + vmp_defalutJointPos_[i]) - vmp_jointKd_[i] * jointVel[i];
            cmd[i] = std::clamp(cmd[i], -vmp_torqueLimits_[i], vmp_torqueLimits_[i]);
            torque[i] = cmd[i];
          } else {
            // CST PD模式：输出位置，力矩另计
            cmd[i] = local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] + vmp_defalutJointPos_[i];
            torque[i] = vmp_jointKp_[i] * (local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] 
                        - jointPos[i] + vmp_defalutJointPos_[i]) - vmp_jointKd_[i] * jointVel[i];
          }
        } else if (control_mode == 2) {
          // CSP 模式
          cmd[i] = vmp_jointKp_[i] * (local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] 
                   - jointPos[i] + vmp_defalutJointPos_[i]);
          torque[i] = cmd[i] - vmp_jointKd_[i] * jointVel[i];
        }
      }
    } else {
      // 仿真模式
      for (int i = 0; i < total_joints; i++) {
        int control_mode = static_cast<int>(vmp_JointControlMode_(i));
        
        if (control_mode == 0) {
          // CST模式
          cmd[i] = vmp_jointKp_[i] * (local_action[i] * vmp_actionScale_ * vmp_actionScaleTest_[i] 
                   - jointPos[i] + vmp_defalutJointPos_[i]) - vmp_jointKd_[i] * jointVel[i];
        } else if (control_mode == 2) {
          // CSP模式：使用ankleSolver计算的力矩
          cmd[i] = jointTor[i];
        }
        cmd[i] = std::clamp(cmd[i], -vmp_torqueLimits_[i], vmp_torqueLimits_[i]);
        torque[i] = cmd[i];
      }
    }
    
    // 应用滤波
    cmd_filter = vmpJointCmdFilter_.update(cmd);
    cmd_out = cmd_filter.cwiseProduct(vmp_jointCmdFilterState_) + 
              cmd.cwiseProduct(Eigen::VectorXd::Ones(total_joints) - vmp_jointCmdFilterState_);
    
    // Debug topics, matching AmpWalkController style:
    if (ros_logger_) {
      ros_logger_->publishVector("/vmp_controller/cmd", cmd);
      ros_logger_->publishVector("/vmp_controller/torque", torque);
      ros_logger_->publishVector("/vmp_controller/actuation", cmd_out);
    }
    
    return cmd_out;
  }

  // ========== VMP模型管理 ==========
  
  bool VMPController::initializeVMP()
  {
    ROS_INFO("[%s] Initializing VMP...", name_.c_str());
    
    try {
      // 初始化变量
      vmp_actions_.resize(numActions_);
      vmp_actions_.setZero();
      // Ensure base-class actions_ is initialized so getCurrentAction() returns a valid vector
      try {
        const int desired_size = std::max(numActions_, jointNum_ + jointArmNum_);
        setCurrentAction(Eigen::VectorXd::Zero(desired_size));
      } catch (const std::exception& e) {
        ROS_WARN("[%s] Failed to set initial actions_ size: %s", name_.c_str(), e.what());
      }
      vmp_ref_motion_.resize(vmp_config_.in_c);
      vmp_ref_motion_.setZero();
      vmp_latent_code_.resize(vmp_config_.latent_d);
      vmp_latent_code_.setZero();
      
      // 设置VMP模型
      setupVMPModels();
      
      // 加载参考数据
      loadVMPRefData();
      
      // 初始化参考运动缓冲区
      if (!vmp_task_data_.empty() && vmp_task_data_.size() >= vmp_config_.in_c) {
        Eigen::VectorXd first_frame(vmp_config_.in_c);
        for (int i = 0; i < vmp_config_.in_c; ++i) {
          first_frame[i] = vmp_task_data_[i];
        }
        Eigen::VectorXd first_frame_norm = first_frame;
        if (vmp_enable_theta_normalization_) {
          normalizeRefMotion(first_frame_norm);
        }
        for (int i = 0; i < vmp_config_.window_l; i++) {
          vmp_ref_motion_raw_buffer_.push_back(first_frame);
          vmp_ref_motion_buffer_.push_back(first_frame_norm);
        }
        ROS_INFO("[%s] Reference buffer pre-filled with first frame data", name_.c_str());
      } else {
        // 没有任务数据时，使用零帧填充
        for (int i = 0; i < vmp_config_.window_l; i++) {
          Eigen::VectorXd zero_motion = Eigen::VectorXd::Zero(vmp_config_.in_c);
          vmp_ref_motion_raw_buffer_.push_back(zero_motion);
          vmp_ref_motion_buffer_.push_back(zero_motion);
        }
        ROS_WARN("[%s] Reference buffer pre-filled with zeros (no task data available)", name_.c_str());
      }
      
      // 初始化GMR高斯平滑参数
      if (enable_gmr_smoothing_) {
        // 根据sigma计算高斯核大小和权重
        // 使用scipy的规则：kernel_size = int(truncate * sigma + 0.5) * 2 + 1，默认truncate=4.0
        gmr_gaussian_kernel_size_ = static_cast<int>(4.0 * gmr_gaussian_sigma_ + 0.5) * 2 + 1;
        gmr_gaussian_kernel_size_ = std::max(3, gmr_gaussian_kernel_size_);  // 最小核大小为3
        
        // 预计算高斯核权重
        gmr_gaussian_kernel_.resize(gmr_gaussian_kernel_size_);
        double sum = 0.0;
        int half_size = gmr_gaussian_kernel_size_ / 2;
        for (int i = 0; i < gmr_gaussian_kernel_size_; ++i) {
          double x = i - half_size;
          gmr_gaussian_kernel_[i] = std::exp(-0.5 * (x * x) / (gmr_gaussian_sigma_ * gmr_gaussian_sigma_));
          sum += gmr_gaussian_kernel_[i];
        }
        // 归一化
        for (int i = 0; i < gmr_gaussian_kernel_size_; ++i) {
          gmr_gaussian_kernel_[i] /= sum;
        }
        
        // 初始化历史缓存
        has_gmr_history_ = false;
        gmr_history_buffer_.clear();
        
        ROS_INFO("[%s] GMR smoothing initialized: sigma=%.1f, kernel_size=%d, deadzone=%.3f rad/s, clip=%.1f rad/s",
                 name_.c_str(), gmr_gaussian_sigma_, gmr_gaussian_kernel_size_, 
                 gmr_velocity_deadzone_, gmr_velocity_clip_);
        
        // 初始化GMR关节位置平滑的高斯核（独立sigma → 独立核大小）
        if (enable_gmr_qpos_smoothing_) {
          gmr_qpos_kernel_size_ = static_cast<int>(4.0 * gmr_qpos_sigma_ + 0.5) * 2 + 1;
          gmr_qpos_kernel_size_ = std::max(3, gmr_qpos_kernel_size_);
          
          gmr_qpos_kernel_.resize(gmr_qpos_kernel_size_);
          double qsum = 0.0;
          int qhalf = gmr_qpos_kernel_size_ / 2;
          for (int i = 0; i < gmr_qpos_kernel_size_; ++i) {
            double x = i - qhalf;
            gmr_qpos_kernel_[i] = std::exp(-0.5 * (x * x) / (gmr_qpos_sigma_ * gmr_qpos_sigma_));
            qsum += gmr_qpos_kernel_[i];
          }
          for (int i = 0; i < gmr_qpos_kernel_size_; ++i) {
            gmr_qpos_kernel_[i] /= qsum;
          }
          
          // history buffer 大小取两者中更大的核
          if (gmr_qpos_kernel_size_ > gmr_gaussian_kernel_size_) {
            ROS_INFO("[%s] GMR history buffer enlarged to %d for qpos kernel (vel kernel=%d)",
                     name_.c_str(), gmr_qpos_kernel_size_, gmr_gaussian_kernel_size_);
          }
          
          ROS_INFO("[%s] GMR qpos smoothing initialized: sigma=%.1f, kernel_size=%d",
                   name_.c_str(), gmr_qpos_sigma_, gmr_qpos_kernel_size_);
        }
      }
      
      // 初始化在线缓冲区
      if (enable_online_vr_mode_) {
        // 初始化VMP输入数据发布器（用于调试）
        vmp_input_data_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/vmp/input_data", 10);
        ROS_INFO("[%s] VMP input data publisher initialized", name_.c_str());
        
        initializeOnlineReferenceBuffer();
        
        // 初始化PICO数据订阅器（如果使用PICO设备模式）
        if (online_vr_data_source_ == "pico") {
          pico_retargeted_pose_sub_ = nh_.subscribe<kuavo_msgs::picoPoseRetarget>(
            "/pico/retargeted_pose", 10, 
            &VMPController::picoRetargetedPoseCallback, this);
          retarget_yaw_resume_ready_sub_ = nh_.subscribe<std_msgs::Header>(
            "/pico/vmp_yaw_resume_ready", 1,
            &VMPController::retargetYawResumeReadyCallback, this);
          ROS_INFO("[%s] PICO subscriber initialized: /pico/retargeted_pose", name_.c_str());
        }
        // 初始化Mocap数据订阅器（如果使用Mocap设备模式）
        else if (online_vr_data_source_ == "mocap") {
          mocap_retargeted_pose_sub_ = nh_.subscribe<kuavo_msgs::MocapPoseRetarget>(
            "/gmr/vmp_input", 10,
            &VMPController::mocapRetargetedPoseCallback, this);
          ROS_INFO("[%s] Mocap subscriber initialized: /gmr/vmp_input", name_.c_str());
        }
        // 初始化Xsense数据订阅器（如果使用Xsense设备模式）
        else if (online_vr_data_source_ == "xsense") {
          xsense_retargeted_pose_sub_ = nh_.subscribe<kuavo_msgs::xsensePoseRetarget>(
            "/xsense/retargeted_pose", 10,
            &VMPController::xsenseRetargetedPoseCallback, this);
          std::string pico_joy_topic = "/pico/joy";
          nh_.param<std::string>("/vmp/pico_joy_topic", pico_joy_topic, pico_joy_topic);
          pico_stream_control_joy_sub_ = nh_.subscribe<kuavo_msgs::JoySticks>(
            pico_joy_topic, 10, &VMPController::picoJoyStreamControlCallback, this);
          retarget_yaw_resume_ready_sub_ = nh_.subscribe<std_msgs::Header>(
            "/xsense/vmp_yaw_resume_ready", 1,
            &VMPController::retargetYawResumeReadyCallback, this);
          ROS_INFO("[%s] Xsense subscriber initialized: /xsense/retargeted_pose", name_.c_str());
        }
        // 如果使用bin文件数据源，加载bin文件
        else if (online_vr_data_source_ == "bin_file" && !online_vr_bin_file_.empty()) {
          std::string bin_file_path = baseModelPath_ + "/" + vmpRefDataDir_ + online_vr_bin_file_;
          loadBinFileForOnlinePlayback(bin_file_path);
        }
        
        // ========== 初始化PICO推流中断/恢复功能 ==========
        // 注册PICO推流控制服务（供 pico-body-tracking-server 的手柄按键回调调用）
        // 按键触发由 Python 端 JoySticksHandler 处理：RT+Y=暂停, RT+X=恢复
        if (online_vr_data_source_ == "pico" || online_vr_data_source_ == "mocap" || online_vr_data_source_ == "xsense") {
          pico_stream_control_srv_ = nh_.advertiseService(
            "/vmp/pico_stream_control",
            &VMPController::picoStreamControlServiceCallback, this);
          ROS_INFO("[%s] PICO stream control service initialized at /vmp/pico_stream_control", name_.c_str());
        }
      }
      
      ROS_INFO("[%s] VMP initialized successfully", name_.c_str());
      return true;
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] VMP initialization failed: %s", name_.c_str(), e.what());
      return false;
    }
  }

  void VMPController::setupVMPModels()
  {
    ROS_INFO("[%s] Loading VMP models...", name_.c_str());
    
    // VMP算法需要policy和encoder两个模型
    std::string policy_model_path = vmpModelPath_;
    std::string encoder_model_path = vmpEncoderPath_;
    
    ROS_INFO("[%s] Policy model: %s", name_.c_str(), policy_model_path.c_str());
    ROS_INFO("[%s] Encoder model: %s", name_.c_str(), encoder_model_path.c_str());
    
    try {
      // 加载策略模型（必需）
      ROS_INFO("[%s] Loading VMP policy model...", name_.c_str());
      vmp_policy_model_ = core_.compile_model(policy_model_path, "CPU");
      ROS_INFO("[%s] Policy model compiled successfully", name_.c_str());
      
      // 加载VAE编码器模型（VMP算法必需）
      ROS_INFO("[%s] Loading VAE encoder model...", name_.c_str());
      vmp_encoder_model_ = core_.compile_model(encoder_model_path, "CPU");
      ROS_INFO("[%s] Encoder model compiled successfully", name_.c_str());
      
      // 加载Estimator模型（可选，根据enableEstimator配置）
      if (vmp_enable_estimator_) {
        ROS_INFO("[%s] Loading Estimator model from: %s", name_.c_str(), vmpEstimatorPath_.c_str());
        vmp_estimator_model_ = core_.compile_model(vmpEstimatorPath_, "CPU");
        ROS_INFO("[%s] Estimator model compiled successfully", name_.c_str());
        
        // 创建Estimator推理请求
        vmp_estimator_request_ = vmp_estimator_model_.create_infer_request();
        ROS_INFO("[%s] Estimator inference request created", name_.c_str());
        
        // 初始化Estimator输出缓存
        vmp_estimator_output_.resize(vmp_estimator_output_dim_);
        vmp_estimator_output_.setZero();
        
        // 初始化Estimator历史观测缓存（预填充零向量）
        vmp_estimator_obs_buffer_.clear();
        Eigen::VectorXd zero_obs = Eigen::VectorXd::Zero(vmp_estimator_input_dim_);
        for (int i = 0; i < vmp_estimator_history_frames_; ++i) {
          vmp_estimator_obs_buffer_.push_back(zero_obs);
        }
        ROS_INFO("[%s] Estimator observation buffer initialized with %d zero frames", 
                 name_.c_str(), vmp_estimator_history_frames_);
        
        // 打印Estimator模型信息
        auto estimator_input = vmp_estimator_model_.input();
        auto estimator_output = vmp_estimator_model_.output();
        ROS_INFO("[%s] Estimator input shape: %s", name_.c_str(), 
                 estimator_input.get_partial_shape().to_string().c_str());
        ROS_INFO("[%s] Estimator output shape: %s", name_.c_str(), 
                 estimator_output.get_partial_shape().to_string().c_str());
      }
      
      // 创建推理请求
      vmp_policy_request_ = vmp_policy_model_.create_infer_request();
      ROS_INFO("[%s] Policy inference request created", name_.c_str());
      
      vmp_encoder_request_ = vmp_encoder_model_.create_infer_request();
      ROS_INFO("[%s] Encoder inference request created", name_.c_str());
      
      // 打印模型信息
      auto policy_input = vmp_policy_model_.input();
      auto policy_output = vmp_policy_model_.output();
      
      ROS_INFO("[%s] Policy input shape: %s", name_.c_str(), 
               policy_input.get_partial_shape().to_string().c_str());
      ROS_INFO("[%s] Policy output shape: %s", name_.c_str(), 
               policy_output.get_partial_shape().to_string().c_str());
      
      // 预热模型
      warmupVMPModels();
      
      ROS_INFO("[%s] VMP models loaded successfully", name_.c_str());
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Failed to load VMP models: %s", name_.c_str(), e.what());
      throw;
    }
  }

  void VMPController::loadVMPRefData()
  {
    ROS_INFO("[%s] Loading VMP reference data...", name_.c_str());
    
    try {
      loadSingleTrajectoryData();
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Failed to load reference data: %s", name_.c_str(), e.what());
      throw;
    }
  }

  void VMPController::loadSingleTrajectoryData()
  {
    // 构建数据文件路径（直接使用baseModelPath_）
    std::string task_data_file = baseModelPath_ + "/" + vmpRefDataDir_ + vmpTaskDataFile_;
    
    ROS_INFO("[%s] Loading from: %s", name_.c_str(), task_data_file.c_str());
    
    // 打开文件
    std::ifstream data_file(task_data_file, std::ios::binary);
    if (!data_file.is_open()) {
      throw std::runtime_error("Cannot open task data file: " + task_data_file);
    }
    
    // 计算帧数
    data_file.seekg(0, std::ios::end);
    size_t file_size = data_file.tellg();
    data_file.seekg(0, std::ios::beg);
    
    int total_frames = file_size / (vmp_config_.in_c * sizeof(float));
    int expected_data_count = total_frames * vmp_config_.in_c;
    
    ROS_INFO("[%s] Total frames: %d", name_.c_str(), total_frames);
    
    // 读取数据
    vmp_task_data_.resize(expected_data_count);
    data_file.read(reinterpret_cast<char*>(vmp_task_data_.data()), file_size);
    data_file.close();
    
    if (vmp_task_data_.empty()) {
      throw std::runtime_error("Failed to load task data");
    }
    
    ROS_INFO("[%s] Loaded %zu data points", name_.c_str(), vmp_task_data_.size());
    
    // 如果配置了静止帧，添加到轨迹前后
    if (vmp_pre_standing_frames_ > 0 || vmp_post_standing_frames_ > 0) {
      appendStandingFramesToTaskData();
    }
  }

  // ========== 在线VR相关函数（简化版） ==========
  
  void VMPController::initializeOnlineReferenceBuffer()
  {
    std::vector<float> prefill_frame = vmp_standing_frame_;
    bool using_retarget_entry_frame = false;
    {
      std::lock_guard<std::mutex> resume_lock(retarget_resume_mutex_);
      if (retarget_entry_frame_pending_ && !retarget_entry_frame_.empty())
      {
        prefill_frame = retarget_entry_frame_;
        using_retarget_entry_frame = true;
        retarget_entry_frame_pending_ = false;
      }
    }
    std::lock_guard<std::mutex> lock(online_buffer_mutex_);
    
    try {
      ROS_INFO("[%s] Initializing online reference buffer...", name_.c_str());
      
      // 设置缓存区大小
      online_ref_buffer_.buffer_size = online_buffer_size_;
      
      // 重置缓存区
      online_ref_buffer_.reset();
      
      // 预分配固定大小的缓存区空间
      online_ref_buffer_.data_buffer.reserve(online_buffer_size_);
      
      // 使用静止帧填充整个缓存区
      if (prefill_frame.empty() || static_cast<int>(prefill_frame.size()) != vmp_config_.in_c) {
        ROS_WARN("[%s] Online buffer prefill frame is invalid, using zero frame", name_.c_str());
        std::vector<float> zero_frame(vmp_config_.in_c, 0.0f);
        zero_frame[0] = static_cast<float>(standingHeight_);  // 至少设置高度
        for (int i = 0; i < online_buffer_size_; ++i) {
          online_ref_buffer_.data_buffer.push_back(zero_frame);
        }
      } else {
        // 普通启动用静止帧；控制器切入时用同一 canonical entry 帧填充。
        for (int i = 0; i < online_buffer_size_; ++i) {
          online_ref_buffer_.data_buffer.push_back(prefill_frame);
        }
      }
      
      // 【核心设计】双指针策略：
      // 1. 写入指针从安全偏移处开始，以100Hz循环写入VR数据
      // 2. 读取指针从index=0开始，以100Hz固定滑动
      // 3. 两者相差 future_frames+1+margin，保证VAE窗口能读到完整未来帧
      //
      // VAE读取窗口：read_index + [-past_frames, +future_frames]（共 window_l 帧）
      // 最远未来帧 = read_index + future_frames
      // 安全条件：distance >= future_frames+1（写指针已超前到 read+future_frames 位置的下一帧）
      //
      // 明确采用 future/past 的概念：future_frames 为 VAE 查看未来帧数
      int future_frames = vmp_config_.window_r;                   // e.g. 15
      int past_frames = vmp_config_.window_l - future_frames - 1; // e.g. 16（不含当前帧）
      (void)past_frames;
      // 要安全读取 future_frames 个未来帧，写指针需要超前至少 future_frames+1
      int min_safe_distance = future_frames + 1;
      // margin = 1：仅保留 1 帧最小安全余量，onlineSamplingLoop 以 100Hz 匀速推进写指针，不会停滞
      const int initial_margin = 1;
      int initial_write_offset = min_safe_distance + initial_margin; // = future_frames+2
      // 确保偏移不超过缓冲区一半（防止极端配置下溢出）
      initial_write_offset = std::min(initial_write_offset, online_buffer_size_ / 2);
      
      online_ref_buffer_.current_write_index = initial_write_offset;  // 写入从最小安全偏移处开始
      online_ref_buffer_.current_read_index = 0;                      // 读取从头开始
      
      // 重要：设置已接收的帧数为缓冲区大小（因为已经用静止帧预填充）
      online_ref_buffer_.total_frames_received = online_buffer_size_;
      
      online_ref_buffer_.is_initialized = true;
      online_ref_buffer_.last_update_time = ros::Time::now();
      
      ROS_INFO("[%s] ========== Online Reference Buffer Initialized ==========", name_.c_str());
      ROS_INFO("[%s] Buffer size: %d frames", name_.c_str(), online_buffer_size_);
  ROS_INFO("[%s] Write pointer: starts at index %d (future_frames=%d), moves at %.1f Hz", 
       name_.c_str(), online_ref_buffer_.current_write_index, future_frames, online_update_rate_);
      ROS_INFO("[%s] Read pointer: starts at index %d, slides at inference rate", 
               name_.c_str(), online_ref_buffer_.current_read_index);
      ROS_INFO("[%s] Pointer offset: %d frames = %.2f second(s) latency", 
               name_.c_str(), 
               initial_write_offset,
               static_cast<float>(initial_write_offset) / online_update_rate_);
      ROS_INFO("[%s] Read window: [read_index ~ read_index+%d), includes past+current+future frames", 
               name_.c_str(), vmp_config_.window_l);
      ROS_INFO("[%s] Frame dimension: %d per frame", name_.c_str(), vmp_config_.in_c);
      ROS_INFO("[%s] Prefill source: %s", name_.c_str(),
               using_retarget_entry_frame ? "VMP-entry frame" : "standing frame");
      ROS_INFO("[%s] =========================================================", name_.c_str());
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Error initializing online buffer: %s", name_.c_str(), e.what());
      online_ref_buffer_.is_initialized = false;
    }
  }

  void VMPController::startOnlineSampling()
  {
    if (online_sampling_running_) {
      return;
    }
    
    online_sampling_running_ = true;
    online_sampling_has_written_.store(false, std::memory_order_release);  // 确保 read 指针等待首次写入
    online_sampling_thread_ = std::thread(&VMPController::onlineSamplingLoop, this);
    
    ROS_INFO("[%s] Online sampling started", name_.c_str());
  }

  void VMPController::stopOnlineSampling()
  {
    if (!online_sampling_running_) {
      return;
    }
    
    online_sampling_running_ = false;
    online_sampling_has_written_.store(false, std::memory_order_release);  // 重置就绪标志
    if (online_sampling_thread_.joinable()) {
      online_sampling_thread_.join();
    }
    
    ROS_INFO("[%s] Online sampling stopped", name_.c_str());
  }

  void VMPController::onlineSamplingLoop()
  {
    ROS_INFO("[%s] ===== Online Sampling Thread Started =====", name_.c_str());
    ROS_INFO("[%s] online_sampling_running_ = %d", name_.c_str(), static_cast<int>(online_sampling_running_.load()));
    ROS_INFO("[%s] online_update_rate_ = %.1f Hz", name_.c_str(), online_update_rate_);
    ROS_INFO("[%s] Data source: %s", name_.c_str(), online_vr_data_source_.c_str());
    
    ros::Rate rate(online_update_rate_);  // 固定采样率
    int sample_count = 0;
    
    // 频率监控变量
    ros::Time last_freq_check_time = ros::Time::now();
    int freq_check_count = 0;
    
    while (ros::ok() && online_sampling_running_) {
      std::vector<float> processed_frame;
      bool has_data = false;
      
      // ========== PICO推流暂停状态处理 ==========
      // 当PICO推流被中断时，持续使用冻结帧（维持当前姿态）
      if (pico_streaming_paused_.load()) {
        std::lock_guard<std::mutex> lock(pico_frozen_frame_mutex_);
        if (!pico_frozen_frame_.empty()) {
          processed_frame = pico_frozen_frame_;
          has_data = true;
        } else if (!vmp_standing_frame_.empty()) {
          // 安全回退：如果没有冻结帧，使用静止帧
          processed_frame = vmp_standing_frame_;
          has_data = true;
        }
        
        if (has_data) {
          // 暂停期间不需要GMR平滑（帧不变化），直接写入缓冲区
          updateOnlineReferenceBuffer(processed_frame);
          sample_count++;
          // 通知推理线程：采样线程已就绪（此路径也需要设置标志）
          if (!online_sampling_has_written_.load(std::memory_order_relaxed)) {
            online_sampling_has_written_.store(true, std::memory_order_release);
            ROS_INFO("[%s] Online sampling loop ready (paused path): first write done, read pointer unlock.", name_.c_str());
          }
        }
        
        // 频率监控
        freq_check_count++;
        if (freq_check_count >= 100) {
          double time_elapsed = (ros::Time::now() - last_freq_check_time).toSec();
          last_freq_check_time = ros::Time::now();
          freq_check_count = 0;
        }
        
        rate.sleep();
        continue;  // 跳过正常数据获取流程
      }
      
      // 根据数据源类型获取数据
      bool callback_updated = false;  // 本tick是否有新的callback写入
      if (online_vr_data_source_ == "pico" || online_vr_data_source_ == "mocap" || online_vr_data_source_ == "xsense") {
        // PICO设备、Mocap 或 Xsense 实时数据模式
        {
          std::lock_guard<std::mutex> lock(latest_frame_mutex_);
          if (has_received_online_data_ && !latest_online_raw_frame_.empty()) {
            processed_frame = latest_online_raw_frame_;
            has_data = true;
            // 检查序列号：判断callback是否在上一个tick之后写入了新数据
            if (latest_frame_seq_ != last_consumed_frame_seq_) {
              callback_updated = true;
              last_consumed_frame_seq_ = latest_frame_seq_;
            }
          }
        }
        
        // 如果没有VR/Mocap数据，使用静止帧
        if (!has_data && !vmp_standing_frame_.empty()) {
          processed_frame = vmp_standing_frame_;
          has_data = true;
        }
        
      } else if (online_vr_data_source_ == "bin_file") {
        // bin文件回放模式 - 直接调用更新函数
        updateOnlineBufferFromBinFile();
        sample_count++;
        if (!online_sampling_has_written_.load(std::memory_order_relaxed)) {
          online_sampling_has_written_.store(true, std::memory_order_release);
          ROS_INFO("[%s] Online sampling loop ready (bin_file path): first write done, read pointer unlock.", name_.c_str());
        }
        rate.sleep();
        continue;
        
      } else {
        ROS_ERROR_THROTTLE(5.0, "[%s] Unknown data source: %s", 
                           name_.c_str(), online_vr_data_source_.c_str());
        rate.sleep();
        continue;
      }
      
      // 首次获取到数据时打印日志
      if (has_data && sample_count == 0) {
        ROS_INFO("[%s] First data ready, starting to sample! (source: %s)", 
                 name_.c_str(), online_vr_data_source_.c_str());
      }

      // 非主动暂停时，retarget 断流超时后过渡到站立参考写入 buffer。
      if (has_data) {
        applyOnlineRetargetStaleFallback(processed_frame, callback_updated);
      }
      
      if (has_data) {
        if (enable_qpos_smoothing_) {
          // === Qpos平滑模式：检测重复帧 → 缓冲 → 回溯插值 → 弹出 ===

          // 1. 判断是否为新帧（关节位置变化是否超过阈值）
          bool is_new = isNewQpos(processed_frame);

          // 诊断日志：区分三种情况
          // A) callback_updated=true  && is_new=true  → 正常新数据
          // B) callback_updated=true  && is_new=false  → callback来了但q变化<eps（原因2）
          // C) callback_updated=false && is_new=false  → callback没来，读到旧数据（原因3）
          // D) callback_updated=false && is_new=true   → 不应出现
          static int diag_cb_new = 0, diag_cb_held = 0, diag_nocb_held = 0, diag_nocb_new = 0;
          if (callback_updated && is_new)        diag_cb_new++;
          else if (callback_updated && !is_new)  diag_cb_held++;
          else if (!callback_updated && !is_new) diag_nocb_held++;
          else                                   diag_nocb_new++;

          int diag_total = diag_cb_new + diag_cb_held + diag_nocb_held + diag_nocb_new;

          std::vector<float> output_frame;
          {
            std::lock_guard<std::mutex> lock(qpos_pending_mutex_);

            // 2. 构造 PendingQposSample 并追加到缓冲区
            PendingQposSample sample;
            sample.frame    = processed_frame;
            sample.anchor   = is_new;
            sample.smoothed = false;
            qpos_pending_buffer_.push_back(std::move(sample));

            if (is_new) {
              // 记录为最近一次实际更新的帧（用于后续比对）
              qpos_last_ingested_ = processed_frame;
              qpos_stat_new_count_++;

              // 3. 有新锚点到达 → 回溯插值之前的held帧
              retroInterpolatePendingTail();
            } else {
              qpos_stat_repeat_count_++;
            }

            // 4. 尝试弹出最旧的帧（缓冲区超过配置大小时）
            output_frame = popOldestPendingIfReady();
          }

          if (!output_frame.empty()) {
            // 弹出成功 → 经过GMR平滑 → 写入引用缓冲区
            output_frame = applyGMRSmoothing(output_frame);
            updateOnlineReferenceBuffer(output_frame);
            sample_count++;
          }
          // else: 缓冲区仍在填充，本轮不输出

        } else {
          // === 原始模式：直接GMR平滑 → 写入缓冲区 ===
          processed_frame = applyGMRSmoothing(processed_frame);
          updateOnlineReferenceBuffer(processed_frame);
          sample_count++;
        }
      }
      
      // 通知推理线程：采样线程已就绪（首次写入已完成）
      // 推理线程在此标志置位前会保持 read 指针不动，从而保证初始 gap = initial_write_offset
      if (sample_count > 0 && !online_sampling_has_written_.load(std::memory_order_relaxed)) {
        online_sampling_has_written_.store(true, std::memory_order_release);
        ROS_INFO("[%s] Online sampling loop ready: first write done (sample#1), read pointer unlock.", name_.c_str());
      }

      // 频率监控：每100次采样检查一次频率
      freq_check_count++;
      if (freq_check_count >= 100) {
        double time_elapsed = (ros::Time::now() - last_freq_check_time).toSec();
        double avg_freq = 100.0 / time_elapsed;
        
        if (avg_freq < online_update_rate_ * 0.8) {
          ROS_WARN("[%s] Sampling frequency low: %.1f Hz (expected %.1f Hz)", 
                   name_.c_str(), avg_freq, online_update_rate_);
        }
        
        last_freq_check_time = ros::Time::now();
        freq_check_count = 0;
      }
      
      rate.sleep();
    }
    
    ROS_INFO("[%s] Online sampling thread stopped, total samples: %d", name_.c_str(), sample_count);
  }

  void VMPController::picoRetargetedPoseCallback(const kuavo_msgs::picoPoseRetarget::ConstPtr& msg)
  {
    // ========== PICO VR 重定向姿态回调 ==========
    // 将 picoPoseRetarget 消息转换为 VMP 输入数据
    // 格式由 vmp_config_ 中的索引定义（从配置文件读取）
    
    // 判断控制模式：上半身 vs 全身
    bool is_upper_body_mode = (online_vr_control_mode_ == "upper_body");
    
    // 关节维度信息
    int joint_dim = numActions_;  // 从配置文件读取（v46=26, v52=27）
    int q_start = vmp_config_.q_start_id;
    int q_dot_start = vmp_config_.q_dot_start_id;
    int p_start = vmp_config_.p_start_id;
    
    // 初始化数据数组
    std::vector<float> vmp_data(vmp_config_.in_c, 0.0f);
    
    // 如果是上半身模式，从静止帧初始化（保持下半身不变）
    if (is_upper_body_mode && !vmp_standing_frame_.empty()) {
      if (vmp_standing_frame_.size() >= static_cast<size_t>(vmp_config_.in_c)) {
        for (int i = 0; i < vmp_config_.in_c; ++i) {
          vmp_data[i] = vmp_standing_frame_[i];
        }
      }
    }
    
    // ===== 1~3. 全身模式数据（机身高度、旋转矩阵、速度、腿部关节） =====
    // 注意：上半身模式下，h/theta/v/腿部关节全部保持静止帧，避免机身变化导致腿部猛烈补偿
    if (!is_upper_body_mode) {
      // 机身高度 [h_start_id]
      vmp_data[vmp_config_.h_start_id] = static_cast<float>(msg->base_link_pose.position.z);
      
      // 旋转矩阵前两列 [theta_start_id : theta_end_id]
      // 从四元数转换为旋转矩阵，存储前两列（6D表示）
      // VMP使用列主序格式：[col0(3), col1(3)] = [R[:,0], R[:,1]]
      double qw = msg->base_link_pose.orientation.w;
      double qx = msg->base_link_pose.orientation.x;
      double qy = msg->base_link_pose.orientation.y;
      double qz = msg->base_link_pose.orientation.z;
      
      int theta_idx = vmp_config_.theta_start_id;
      // 列主序存储：第一列完整3元素 + 第二列完整3元素
      // col0: [R[0,0], R[1,0], R[2,0]]
      vmp_data[theta_idx]     = static_cast<float>(1.0 - 2.0*(qy*qy + qz*qz));  // R[0,0]
      vmp_data[theta_idx + 1] = static_cast<float>(2.0*(qx*qy + qw*qz));        // R[1,0]
      vmp_data[theta_idx + 2] = static_cast<float>(2.0*(qx*qz - qw*qy));        // R[2,0]
      // col1: [R[0,1], R[1,1], R[2,1]]
      vmp_data[theta_idx + 3] = static_cast<float>(2.0*(qx*qy - qw*qz));        // R[0,1]
      vmp_data[theta_idx + 4] = static_cast<float>(1.0 - 2.0*(qx*qx + qz*qz));  // R[1,1]
      vmp_data[theta_idx + 5] = static_cast<float>(2.0*(qy*qz + qw*qx));        // R[2,1]

      // 机身速度 [v_start_id : v_end_id]
      if (msg->base_velocity.size() >= 6) {
        for (int i = 0; i < 6; i++) {
          vmp_data[vmp_config_.v_start_id + i] = static_cast<float>(msg->base_velocity[i]);
        }
      }
      
      // 关节位置 [q_start_id : q_end_id]
      // PICO消息格式已扩展为27关节（含腰部）：腿(0-11) + 腰(12) + 手臂(13-26)
      // 与Mocap消息格式一致，可直接复制
      int joint_count = std::min(static_cast<int>(msg->joint_position.size()), joint_dim);
      for (int i = 0; i < joint_count; i++) {
        vmp_data[q_start + i] = static_cast<float>(msg->joint_position[i]);
      }
      
      // 关节速度 [q_dot_start_id : q_dot_end_id]
      joint_count = std::min(static_cast<int>(msg->joint_velocity.size()), joint_dim);
      for (int i = 0; i < joint_count; i++) {
        vmp_data[q_dot_start + i] = static_cast<float>(msg->joint_velocity[i]);
      }
      
      // 末端位置 [p_start_id : p_end_id]，转换为相对于base_link的位置
      // PICO输出顺序：[left_foot, right_foot, left_hand, right_hand]
      // v46 VMP需要顺序：[left_hand, right_hand, left_foot, right_foot] (先手后腿)
      // v52 VMP需要顺序：[left_foot, right_foot, left_hand, right_hand] (先腿后手)
      if (msg->end_effector_poses.size() >= 4) {
        double base_x = msg->base_link_pose.position.x;
        double base_y = msg->base_link_pose.position.y;
        double base_z = msg->base_link_pose.position.z;
        
        if (waistNum_ == 0) {
          // v46: 先手后腿顺序
          // 左手 [p_start : p_start+3] <- PICO索引2
          vmp_data[p_start]     = static_cast<float>(msg->end_effector_poses[2].x - base_x);
          vmp_data[p_start + 1] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
          vmp_data[p_start + 2] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
          
          // 右手 [p_start+3 : p_start+6] <- PICO索引3
          vmp_data[p_start + 3] = static_cast<float>(msg->end_effector_poses[3].x - base_x);
          vmp_data[p_start + 4] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
          vmp_data[p_start + 5] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
          
          // 左脚 [p_start+6 : p_start+9] <- PICO索引0
          vmp_data[p_start + 6] = static_cast<float>(msg->end_effector_poses[0].x - base_x);
          vmp_data[p_start + 7] = static_cast<float>(msg->end_effector_poses[0].y - base_y);
          vmp_data[p_start + 8] = static_cast<float>(msg->end_effector_poses[0].z - base_z);
          
          // 右脚 [p_start+9 : p_start+12] <- PICO索引1
          vmp_data[p_start + 9]  = static_cast<float>(msg->end_effector_poses[1].x - base_x);
          vmp_data[p_start + 10] = static_cast<float>(msg->end_effector_poses[1].y - base_y);
          vmp_data[p_start + 11] = static_cast<float>(msg->end_effector_poses[1].z - base_z);
        } else {
          // v52: 先腿后手顺序
          // 左脚 [p_start : p_start+3] <- PICO索引0
          vmp_data[p_start]     = static_cast<float>(msg->end_effector_poses[0].x - base_x);
          vmp_data[p_start + 1] = static_cast<float>(msg->end_effector_poses[0].y - base_y);
          vmp_data[p_start + 2] = static_cast<float>(msg->end_effector_poses[0].z - base_z);
          
          // 右脚 [p_start+3 : p_start+6] <- PICO索引1
          vmp_data[p_start + 3] = static_cast<float>(msg->end_effector_poses[1].x - base_x);
          vmp_data[p_start + 4] = static_cast<float>(msg->end_effector_poses[1].y - base_y);
          vmp_data[p_start + 5] = static_cast<float>(msg->end_effector_poses[1].z - base_z);
          
          // 左手 [p_start+6 : p_start+9] <- PICO索引2
          vmp_data[p_start + 6] = static_cast<float>(msg->end_effector_poses[2].x - base_x);
          vmp_data[p_start + 7] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
          vmp_data[p_start + 8] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
          
          // 右手 [p_start+9 : p_start+12] <- PICO索引3
          vmp_data[p_start + 9]  = static_cast<float>(msg->end_effector_poses[3].x - base_x);
          vmp_data[p_start + 10] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
          vmp_data[p_start + 11] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
        }
      }
    }
    // ===== 4. 上半身模式数据 =====
    else {
      // 手臂关节索引计算
      // PICO消息格式已扩展为27关节（含腰部）：腿(0-11) + 腰(12) + 手臂(13-26)
      // 与Mocap消息格式一致，msg中arm永远从index 13开始
      int arm_start_in_msg = jointNum_ + 1;           // 消息中手臂起始位置固定为13（腿12 + 腰1）
      int arm_start_in_vmp = waistNum_ + jointNum_;   // VMP中手臂索引（v46=12, v52=13）
      int arm_count = jointArmNum_;
      
      // v52: 更新腰部关节（上半身模式应包含腰部）
      if (waistNum_ > 0 && msg->joint_position.size() > static_cast<size_t>(jointNum_)) {
        vmp_data[q_start + jointNum_] = static_cast<float>(msg->joint_position[jointNum_]);  // 腰部位置
        if (msg->joint_velocity.size() > static_cast<size_t>(jointNum_)) {
          vmp_data[q_dot_start + jointNum_] = static_cast<float>(msg->joint_velocity[jointNum_]);  // 腰部速度
        }
      }
      
      // 手臂关节位置
      if (static_cast<int>(msg->joint_position.size()) >= arm_start_in_msg + arm_count) {
        for (int i = 0; i < arm_count; i++) {
          vmp_data[q_start + arm_start_in_vmp + i] = static_cast<float>(msg->joint_position[arm_start_in_msg + i]);
        }
      }
      
      // 手臂关节速度
      if (static_cast<int>(msg->joint_velocity.size()) >= arm_start_in_msg + arm_count) {
        for (int i = 0; i < arm_count; i++) {
          vmp_data[q_dot_start + arm_start_in_vmp + i] = static_cast<float>(msg->joint_velocity[arm_start_in_msg + i]);
        }
      }
      
      // 手部末端位置（只替换左手和右手，脚部保持静止帧）
      // v46: 先手后腿顺序，手部在 p_start 开始
      // v52: 先腿后手顺序，手部在 p_start+6 开始
      if (msg->end_effector_poses.size() >= 4) {
        // 上半身模式：末端位置需要转换为相对于base的坐标
        // PICO消息中的末端位置是世界坐标系，需要减去base位置
        double base_x = msg->base_link_pose.position.x;
        double base_y = msg->base_link_pose.position.y;
        double base_z = msg->base_link_pose.position.z;
        
        int hand_offset = (waistNum_ == 0) ? 0 : 6;  // v46: 0, v52: 6
        
        // 左手 <- PICO索引2，转换为相对于base的位置
        vmp_data[p_start + hand_offset]     = static_cast<float>(msg->end_effector_poses[2].x - base_x);
        vmp_data[p_start + hand_offset + 1] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
        vmp_data[p_start + hand_offset + 2] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
        
        // 右手 <- PICO索引3，转换为相对于base的位置
        vmp_data[p_start + hand_offset + 3] = static_cast<float>(msg->end_effector_poses[3].x - base_x);
        vmp_data[p_start + hand_offset + 4] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
        vmp_data[p_start + hand_offset + 5] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
        
        // 脚部保持静止帧不变（已在初始化时设置）
      }
      
      ROS_INFO_THROTTLE(5.0, "[%s] Upper body mode: updated arm joints [%d-%d]", 
                        name_.c_str(), arm_start_in_vmp, arm_start_in_vmp + arm_count - 1);
    }
    
    // ===== 5. 存储原始数据供VMP使用 =====
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_online_raw_frame_ = vmp_data;
      has_received_online_data_ = true;
      latest_frame_seq_++;  // 每次callback写入递增序列号
    }
    notifyOnlineFrameReceived();
    recordRetargetedFrameStamp(msg->header.stamp);
    
    // ===== 6. 首次接收日志 =====
    static bool first_pico_data = false;
    static int pico_callback_count = 0;
    pico_callback_count++;
    
    if (!first_pico_data) {
      ROS_INFO("[%s] First VR retargeted pose received (mode: %s)", 
               name_.c_str(), is_upper_body_mode ? "upper_body" : "full_body");
      ROS_INFO("[%s] Base height: %.3f m, Rotation R[0,0]: %.3f",
               name_.c_str(), vmp_data[vmp_config_.h_start_id], vmp_data[vmp_config_.theta_start_id]);
      first_pico_data = true;
    }
    
    // 频率监控日志
    ROS_DEBUG_THROTTLE(1.0, "[%s] PICO callback rate: %d Hz (approx)", name_.c_str(), pico_callback_count);
  }

  void VMPController::mocapRetargetedPoseCallback(const kuavo_msgs::MocapPoseRetarget::ConstPtr& msg)
  {
    // ========== Mocap 重定向姿态回调 ==========
    // 使用与PICO相同的消息格式，处理逻辑一致
    // 格式由 vmp_config_ 中的索引定义（从配置文件读取）
    
    // 判断控制模式：上半身 vs 全身
    bool is_upper_body_mode = (online_vr_control_mode_ == "upper_body");
    
    // 关节维度信息
    int joint_dim = numActions_;  // 从配置文件读取（v46=26, v52=27）
    int q_start = vmp_config_.q_start_id;
    int q_dot_start = vmp_config_.q_dot_start_id;
    int p_start = vmp_config_.p_start_id;
    
    // 初始化数据数组
    std::vector<float> vmp_data(vmp_config_.in_c, 0.0f);
    
    // 如果是上半身模式，从静止帧初始化（保持下半身不变）
    if (is_upper_body_mode && !vmp_standing_frame_.empty()) {
      if (vmp_standing_frame_.size() >= static_cast<size_t>(vmp_config_.in_c)) {
        for (int i = 0; i < vmp_config_.in_c; ++i) {
          vmp_data[i] = vmp_standing_frame_[i];
        }
      }
    }
    
    // ===== 1~3. 全身模式数据（机身高度、旋转矩阵、速度、腿部关节） =====
    // 注意：上半身模式下，h/theta/v/腿部关节全部保持静止帧，避免机身变化导致腿部猛烈补偿
    if (!is_upper_body_mode) {
      // 机身高度 [h_start_id]
      vmp_data[vmp_config_.h_start_id] = static_cast<float>(msg->base_link_pose.position.z);
      
      // 旋转矩阵前两列 [theta_start_id : theta_end_id]
      // 从四元数转换为旋转矩阵，存储前两列（6D表示）
      // VMP使用列主序格式：[col0(3), col1(3)] = [R[:,0], R[:,1]]
      double qw = msg->base_link_pose.orientation.w;
      double qx = msg->base_link_pose.orientation.x;
      double qy = msg->base_link_pose.orientation.y;
      double qz = msg->base_link_pose.orientation.z;
      
      int theta_idx = vmp_config_.theta_start_id;
      // 列主序存储：第一列完整3元素 + 第二列完整3元素
      // col0: [R[0,0], R[1,0], R[2,0]]
      vmp_data[theta_idx]     = static_cast<float>(1.0 - 2.0*(qy*qy + qz*qz));  // R[0,0]
      vmp_data[theta_idx + 1] = static_cast<float>(2.0*(qx*qy + qw*qz));        // R[1,0]
      vmp_data[theta_idx + 2] = static_cast<float>(2.0*(qx*qz - qw*qy));        // R[2,0]
      // col1: [R[0,1], R[1,1], R[2,1]]
      vmp_data[theta_idx + 3] = static_cast<float>(2.0*(qx*qy - qw*qz));        // R[0,1]
      vmp_data[theta_idx + 4] = static_cast<float>(1.0 - 2.0*(qx*qx + qz*qz));  // R[1,1]
      vmp_data[theta_idx + 5] = static_cast<float>(2.0*(qy*qz + qw*qx));        // R[2,1]

      // 机身速度 [v_start_id : v_end_id]
      if (msg->base_velocity.size() >= 6) {
        for (int i = 0; i < 6; i++) {
          vmp_data[vmp_config_.v_start_id + i] = static_cast<float>(msg->base_velocity[i]);
        }
      }
      
      // 关节位置 [q_start_id : q_end_id]
      int joint_count = std::min(static_cast<int>(msg->joint_position.size()), joint_dim);
      for (int i = 0; i < joint_count; i++) {
        vmp_data[q_start + i] = static_cast<float>(msg->joint_position[i]);
      }
      
      // 关节速度 [q_dot_start_id : q_dot_end_id]
      joint_count = std::min(static_cast<int>(msg->joint_velocity.size()), joint_dim);
      for (int i = 0; i < joint_count; i++) {
        vmp_data[q_dot_start + i] = static_cast<float>(msg->joint_velocity[i]);
      }
      
      // 末端位置 [p_start_id : p_end_id]，转换为相对于base_link的位置
      // Mocap输出顺序：[left_foot, right_foot, left_hand, right_hand]
      // v46 VMP需要顺序：[left_hand, right_hand, left_foot, right_foot] (先手后腿)
      // v52 VMP需要顺序：[left_foot, right_foot, left_hand, right_hand] (先腿后手)
      if (msg->end_effector_poses.size() >= 4) {
        double base_x = msg->base_link_pose.position.x;
        double base_y = msg->base_link_pose.position.y;
        double base_z = msg->base_link_pose.position.z;
        
        if (waistNum_ == 0) {
          // v46: 先手后腿顺序
          // 左手 [p_start : p_start+3] <- Mocap索引2
          vmp_data[p_start]     = static_cast<float>(msg->end_effector_poses[2].x - base_x);
          vmp_data[p_start + 1] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
          vmp_data[p_start + 2] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
          
          // 右手 [p_start+3 : p_start+6] <- Mocap索引3
          vmp_data[p_start + 3] = static_cast<float>(msg->end_effector_poses[3].x - base_x);
          vmp_data[p_start + 4] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
          vmp_data[p_start + 5] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
          
          // 左脚 [p_start+6 : p_start+9] <- Mocap索引0
          vmp_data[p_start + 6] = static_cast<float>(msg->end_effector_poses[0].x - base_x);
          vmp_data[p_start + 7] = static_cast<float>(msg->end_effector_poses[0].y - base_y);
          vmp_data[p_start + 8] = static_cast<float>(msg->end_effector_poses[0].z - base_z);
          
          // 右脚 [p_start+9 : p_start+12] <- Mocap索引1
          vmp_data[p_start + 9]  = static_cast<float>(msg->end_effector_poses[1].x - base_x);
          vmp_data[p_start + 10] = static_cast<float>(msg->end_effector_poses[1].y - base_y);
          vmp_data[p_start + 11] = static_cast<float>(msg->end_effector_poses[1].z - base_z);
        } else {
          // v52: 先腿后手顺序
          // 左脚 [p_start : p_start+3] <- Mocap索引0
          vmp_data[p_start]     = static_cast<float>(msg->end_effector_poses[0].x - base_x);
          vmp_data[p_start + 1] = static_cast<float>(msg->end_effector_poses[0].y - base_y);
          vmp_data[p_start + 2] = static_cast<float>(msg->end_effector_poses[0].z - base_z);
          
          // 右脚 [p_start+3 : p_start+6] <- Mocap索引1
          vmp_data[p_start + 3] = static_cast<float>(msg->end_effector_poses[1].x - base_x);
          vmp_data[p_start + 4] = static_cast<float>(msg->end_effector_poses[1].y - base_y);
          vmp_data[p_start + 5] = static_cast<float>(msg->end_effector_poses[1].z - base_z);
          
          // 左手 [p_start+6 : p_start+9] <- Mocap索引2
          vmp_data[p_start + 6] = static_cast<float>(msg->end_effector_poses[2].x - base_x);
          vmp_data[p_start + 7] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
          vmp_data[p_start + 8] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
          
          // 右手 [p_start+9 : p_start+12] <- Mocap索引3
          vmp_data[p_start + 9]  = static_cast<float>(msg->end_effector_poses[3].x - base_x);
          vmp_data[p_start + 10] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
          vmp_data[p_start + 11] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
        }
      }
    }
    // ===== 4. 上半身模式数据 =====
    else {
      // 手臂关节索引计算（Mocap消息格式固定为27关节：腿(0-11) + 腰(12) + 手臂(13-26)）
      // VMP参考运动格式与机器人版本一致：
      //   v46: 26关节，腿(0-11) + 手臂(12-25)，无腰部
      //   v52: 27关节，腿(0-11) + 腰(12) + 手臂(13-26)，有腰部
      int arm_start_in_msg = jointNum_ + 1;          // Mocap消息中手臂起始位置固定为13（腿12 + 腰1）
      int arm_start_in_vmp = waistNum_ + jointNum_;  // VMP中手臂索引（v46=12, v52=13）
      int arm_count = jointArmNum_;
      
      // v52: 更新腰部关节（上半身模式应包含腰部）
      if (waistNum_ > 0 && msg->joint_position.size() > static_cast<size_t>(jointNum_)) {
        vmp_data[q_start + jointNum_] = static_cast<float>(msg->joint_position[jointNum_]);  // 腰部位置
        if (msg->joint_velocity.size() > static_cast<size_t>(jointNum_)) {
          vmp_data[q_dot_start + jointNum_] = static_cast<float>(msg->joint_velocity[jointNum_]);  // 腰部速度
        }
      }
      
      // 手臂关节位置
      if (static_cast<int>(msg->joint_position.size()) >= arm_start_in_msg + arm_count) {
        for (int i = 0; i < arm_count; i++) {
          vmp_data[q_start + arm_start_in_vmp + i] = static_cast<float>(msg->joint_position[arm_start_in_msg + i]);
        }
      }
      
      // 手臂关节速度
      if (static_cast<int>(msg->joint_velocity.size()) >= arm_start_in_msg + arm_count) {
        for (int i = 0; i < arm_count; i++) {
          vmp_data[q_dot_start + arm_start_in_vmp + i] = static_cast<float>(msg->joint_velocity[arm_start_in_msg + i]);
        }
      }
      
      // 手部末端位置（只替换左手和右手，脚部保持静止帧）
      if (msg->end_effector_poses.size() >= 4) {
        // 上半身模式：末端位置需要转换为相对于base的坐标
        // Mocap消息中的末端位置是世界坐标系，需要减去base位置
        double base_x = msg->base_link_pose.position.x;
        double base_y = msg->base_link_pose.position.y;
        double base_z = msg->base_link_pose.position.z;
        
        int hand_offset = (waistNum_ == 0) ? 0 : 6;  // v46: 0, v52: 6
        
        // 左手 <- Mocap索引2，转换为相对于base的位置
        vmp_data[p_start + hand_offset]     = static_cast<float>(msg->end_effector_poses[2].x - base_x);
        vmp_data[p_start + hand_offset + 1] = static_cast<float>(msg->end_effector_poses[2].y - base_y);
        vmp_data[p_start + hand_offset + 2] = static_cast<float>(msg->end_effector_poses[2].z - base_z);
        
        // 右手 <- Mocap索引3，转换为相对于base的位置
        vmp_data[p_start + hand_offset + 3] = static_cast<float>(msg->end_effector_poses[3].x - base_x);
        vmp_data[p_start + hand_offset + 4] = static_cast<float>(msg->end_effector_poses[3].y - base_y);
        vmp_data[p_start + hand_offset + 5] = static_cast<float>(msg->end_effector_poses[3].z - base_z);
      }
      
      ROS_INFO_THROTTLE(5.0, "[%s] Mocap upper body mode: updated arm joints [%d-%d]", 
                        name_.c_str(), arm_start_in_vmp, arm_start_in_vmp + arm_count - 1);
    }
    
    // ===== 5. 存储原始数据供VMP使用 =====
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_online_raw_frame_ = vmp_data;
      has_received_online_data_ = true;
      latest_frame_seq_++;  // 每次callback写入递增序列号
    }
    notifyOnlineFrameReceived();
    recordRetargetedFrameStamp(msg->header.stamp);
    
    // ===== 6. 首次接收日志 =====
    static bool first_mocap_data = false;
    static int mocap_callback_count = 0;
    mocap_callback_count++;
    
    if (!first_mocap_data) {
      ROS_INFO("[%s] First Mocap retargeted pose received (mode: %s)", 
               name_.c_str(), is_upper_body_mode ? "upper_body" : "full_body");
      ROS_INFO("[%s] Base height: %.3f m, Rotation R[0,0]: %.3f",
               name_.c_str(), vmp_data[vmp_config_.h_start_id], vmp_data[vmp_config_.theta_start_id]);
      first_mocap_data = true;
    }
    
    // 频率监控日志
    ROS_DEBUG_THROTTLE(1.0, "[%s] Mocap callback rate: %d Hz (approx)", name_.c_str(), mocap_callback_count);
  }

  void VMPController::xsenseRetargetedPoseCallback(const kuavo_msgs::xsensePoseRetarget::ConstPtr& msg)
  {
    kuavo_msgs::MocapPoseRetarget converted_msg;
    converted_msg.header = msg->header;
    converted_msg.base_link_pose = msg->base_link_pose;
    converted_msg.base_velocity = msg->base_velocity;
    converted_msg.joint_position = msg->joint_position;
    converted_msg.joint_velocity = msg->joint_velocity;
    converted_msg.end_effector_poses = msg->end_effector_poses;

    const kuavo_msgs::MocapPoseRetarget::ConstPtr converted_ptr(
        new kuavo_msgs::MocapPoseRetarget(converted_msg));
    mocapRetargetedPoseCallback(converted_ptr);
  }

  std::vector<float> VMPController::fuseMultiTopicData()
  {
    // 多话题数据融合（简化版）
    return std::vector<float>(vmp_config_.in_c, 0.0f);
  }

  std::vector<float> VMPController::preprocessOnlineMotionData(const std::vector<float>& raw_data)
  {
    return raw_data;
  }

  bool VMPController::isOnlineVRDeviceMode() const
  {
    // 支持 pico、mocap 和 xsense 在线设备模式
    return enable_online_vr_mode_ && (online_vr_data_source_ == "pico" || online_vr_data_source_ == "mocap" || online_vr_data_source_ == "xsense");
  }

  // ========== PICO推流中断/恢复控制 ==========
  // 按键检测由 pico-body-tracking-server (Python端) JoySticksHandler 处理：
  //   RT + Y = 暂停推流 → 调用 /vmp/pico_stream_control (data=true)
  //   RT + X = 恢复推流 → 调用 /vmp/pico_stream_control (data=false)
  // 也可通过命令行直接调用：rosservice call /vmp/pico_stream_control "data: true/false"
  
  bool VMPController::picoStreamControlServiceCallback(std_srvs::SetBool::Request& req,
                                                       std_srvs::SetBool::Response& res)
  {
    if (req.data) {
      // 暂停推流
      std::lock_guard<std::mutex> resume_lock(retarget_resume_mutex_);
      retarget_resume_waiting_ = false;
      retarget_resume_ready_stamp_ = ros::Time(0);
      retarget_auto_freeze_valid_ = false;
      if (!pico_streaming_paused_.load()) {
        {
          std::lock_guard<std::mutex> lock_frozen(pico_frozen_frame_mutex_);
          std::lock_guard<std::mutex> lock_latest(latest_frame_mutex_);
          retarget_frozen_frame_valid_ = has_received_online_data_ && !latest_online_raw_frame_.empty();
          if (retarget_frozen_frame_valid_) {
            pico_frozen_frame_ = latest_online_raw_frame_;
          } else if (!vmp_standing_frame_.empty()) {
            pico_frozen_frame_ = vmp_standing_frame_;
          }
        }
        pico_streaming_paused_.store(true);
        resetOnlineStaleFallbackState(false);
        ROS_INFO("[%s] PICO推流已通过服务暂停", name_.c_str());
      }
      res.success = true;
      res.message = "PICO streaming paused, robot holds current pose";
    } else {
      // 恢复推流
      if (pico_streaming_paused_.load()) {
        resumeRetargetedStreaming();
        ROS_INFO("[%s] PICO推流已通过服务恢复", name_.c_str());
      }
      res.success = true;
      res.message = "PICO streaming resumed";
    }
    return true;
  }

  void VMPController::picoJoyStreamControlCallback(const kuavo_msgs::JoySticks::ConstPtr& msg)
  {
    if (!msg)
      return;

    const bool rt_pressed = msg->right_trigger >= 0.5f;
    const bool pause_active = rt_pressed && msg->left_second_button_pressed;  // RT+Y
    const bool resume_active = rt_pressed && msg->left_first_button_pressed; // RT+X
    const bool pause_was_active = joy_pause_combo_active_.exchange(pause_active);
    const bool resume_was_active = joy_resume_combo_active_.exchange(resume_active);

    if (resume_active && !resume_was_active)
      arm_stream_pause_requested_.store(false);
    else if (pause_active && !pause_was_active)
      arm_stream_pause_requested_.store(true);
  }

  // ========== 数学工具函数 ==========
  
  void VMPController::normalizeRefMotion(Eigen::VectorXd& ref_motion)
  {
    try {
      if (vmp_config_.theta_start_id < 0 || vmp_config_.theta_end_id >= static_cast<int>(ref_motion.size()) || 
          vmp_config_.theta_start_id >= vmp_config_.theta_end_id) {
        return; // 索引无效时跳过归一化
      }
      
      int theta_length = vmp_config_.theta_end_id - vmp_config_.theta_start_id;
      if (theta_length != 6) {
        return; // 只处理6D旋转表示
      }
      
      // 从 theta 范围提取6D旋转向量
      // 6D格式: [col0(3), col1(3)] = [R[:,0], R[:,1]]
      double rot6[6];
      for (int i = 0; i < 6; ++i) {
        rot6[i] = ref_motion[vmp_config_.theta_start_id + i];
      }
      
      // col0 = normalize(rot6[0:3])
      Eigen::Vector3d c0(rot6[0], rot6[1], rot6[2]);
      c0.normalize();
      // col1 = normalize(rot6[3:6] - proj(rot6[3:6] onto c0))
      Eigen::Vector3d c1_raw(rot6[3], rot6[4], rot6[5]);
      Eigen::Vector3d c1 = c1_raw - c1_raw.dot(c0) * c0;
      c1.normalize();
      // col2 = cross(c0, c1)
      Eigen::Vector3d c2 = c0.cross(c1);
      Eigen::Matrix3d R;
      R.col(0) = c0;
      R.col(1) = c1;
      R.col(2) = c2;
      // yaw = atan2(R[1,0], R[0,0]) = atan2(c0[1], c0[0])
      double yaw = std::atan2(R(1, 0), R(0, 0));
      double cos_yaw = std::cos(-yaw);
      double sin_yaw = std::sin(-yaw);
      Eigen::Matrix3d Rz;
      Rz << cos_yaw, -sin_yaw, 0.0,
            sin_yaw,  cos_yaw, 0.0,
            0.0,      0.0,     1.0;
      
      // === Step 3: 应用去yaw旋转 ===
      Eigen::Matrix3d R_no_yaw = Rz * R;
      for (int i = 0; i < 3; ++i) {
        ref_motion[vmp_config_.theta_start_id + i] = R_no_yaw(i, 0);      // col0
        ref_motion[vmp_config_.theta_start_id + 3 + i] = R_no_yaw(i, 1);  // col1
      }
      
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error in normalizeRefMotion: %s", name_.c_str(), e.what());
    }
  }

  Eigen::Vector4d VMPController::mat_to_quat(const Eigen::Matrix3d& matrix)
  {
    Eigen::Vector4d quat;
    double trace = matrix.trace();
    
    if (trace > 0) {
      double s = sqrt(trace + 1.0) * 2;
      quat[0] = 0.25 * s;
      quat[1] = (matrix(2, 1) - matrix(1, 2)) / s;
      quat[2] = (matrix(0, 2) - matrix(2, 0)) / s;
      quat[3] = (matrix(1, 0) - matrix(0, 1)) / s;
    } else if ((matrix(0, 0) > matrix(1, 1)) && (matrix(0, 0) > matrix(2, 2))) {
      double s = sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2)) * 2;
      quat[0] = (matrix(2, 1) - matrix(1, 2)) / s;
      quat[1] = 0.25 * s;
      quat[2] = (matrix(0, 1) + matrix(1, 0)) / s;
      quat[3] = (matrix(0, 2) + matrix(2, 0)) / s;
    } else if (matrix(1, 1) > matrix(2, 2)) {
      double s = sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2)) * 2;
      quat[0] = (matrix(0, 2) - matrix(2, 0)) / s;
      quat[1] = (matrix(0, 1) + matrix(1, 0)) / s;
      quat[2] = 0.25 * s;
      quat[3] = (matrix(1, 2) + matrix(2, 1)) / s;
    } else {
      double s = sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1)) * 2;
      quat[0] = (matrix(1, 0) - matrix(0, 1)) / s;
      quat[1] = (matrix(0, 2) + matrix(2, 0)) / s;
      quat[2] = (matrix(1, 2) + matrix(2, 1)) / s;
      quat[3] = 0.25 * s;
    }
    
    return quat;
  }

  Eigen::Vector3d VMPController::get_euler_xyz(const Eigen::Vector4d& quat)
  {
    double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    
    // Roll (x轴旋转)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    double sinp = 2 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(M_PI / 2, sinp);
    else
      pitch = asin(sinp);
    
    // Yaw (z轴旋转)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  Eigen::Matrix3d VMPController::euler_to_rotation_matrix(const Eigen::Vector3d& euler)
  {
    double roll = euler[0], pitch = euler[1], yaw = euler[2];
    
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);  
    double cy = cos(yaw), sy = sin(yaw);
    
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0, 0) = cy * cp;
    rotation_matrix(0, 1) = cy * sp * sr - sy * cr;
    rotation_matrix(0, 2) = cy * sp * cr + sy * sr;
    rotation_matrix(1, 0) = sy * cp;
    rotation_matrix(1, 1) = sy * sp * sr + cy * cr;
    rotation_matrix(1, 2) = sy * sp * cr - cy * sr;
    rotation_matrix(2, 0) = -sp;
    rotation_matrix(2, 1) = cp * sr;
    rotation_matrix(2, 2) = cp * cr;
    
    return rotation_matrix;
  }
  Eigen::Vector3d VMPController::computeAnchorEulerB(const SensorData& sensor_data)
  {
    Eigen::Vector3d anchor_euler = Eigen::Vector3d::Zero();
    if (vmp_ref_motion_raw_buffer_.empty()) {
      return anchor_euler;
    }
    // Match training semantics: motion_anchor_euler_b is computed from raw anchor quat.
    const Eigen::VectorXd& ref = vmp_ref_motion_raw_buffer_.back();
    const int ts = vmp_config_.theta_start_id;
    const int te = vmp_config_.theta_end_id;
    if (ts < 0 || te > static_cast<int>(ref.size()) || (te - ts) != 6) {
      return anchor_euler;
    }
    Eigen::Vector3d c0_raw(ref[ts + 0], ref[ts + 1], ref[ts + 2]);
    Eigen::Vector3d c1_raw(ref[ts + 3], ref[ts + 4], ref[ts + 5]);
    if (c0_raw.norm() < 1e-8) {
      return anchor_euler;  // 参考帧 rot6d 尚未填充
    }
    Eigen::Vector3d c0 = c0_raw.normalized();
    Eigen::Vector3d c1 = c1_raw - c1_raw.dot(c0) * c0;
    if (c1.norm() < 1e-8) {
      return anchor_euler;
    }
    c1.normalize();
    Eigen::Vector3d c2 = c0.cross(c1);
    Eigen::Matrix3d R_anchor;
    R_anchor.col(0) = c0;
    R_anchor.col(1) = c1;
    R_anchor.col(2) = c2;
    Eigen::Quaterniond q_anchor(R_anchor);
    q_anchor.normalize();
    Eigen::Quaterniond q_robot(sensor_data.quat_.w(), sensor_data.quat_.x(),
                               sensor_data.quat_.y(), sensor_data.quat_.z());
    if (q_robot.norm() < 1e-8) {
      return anchor_euler;
    }
    q_robot.normalize();
    if (!vmp_entry_imu_quat_valid_) {
      // Align the current robot yaw with the current motion-anchor yaw. This
      // makes switch-in yaw error zero even if AMP rotated the robot and/or
      // the incoming GMR heading is nonzero. Subsequent anchor error then
      // represents motion-yaw delta minus robot-yaw delta since this entry.
      const double anchor_yaw = std::atan2(R_anchor(1, 0), R_anchor(0, 0));
      const Eigen::Quaterniond q_anchor_heading(
          Eigen::AngleAxisd(anchor_yaw, Eigen::Vector3d::UnitZ()));
      vmp_entry_imu_quat_ = q_robot * q_anchor_heading.conjugate();
      vmp_entry_imu_quat_.normalize();
      vmp_entry_imu_quat_valid_ = true;
      ROS_INFO("[%s] Captured VMP entry alignment (anchor_yaw=%.4f rad): w=%.4f x=%.4f y=%.4f z=%.4f",
               name_.c_str(), anchor_yaw,
               vmp_entry_imu_quat_.w(), vmp_entry_imu_quat_.x(),
               vmp_entry_imu_quat_.y(), vmp_entry_imu_quat_.z());
    }
    q_robot = vmp_entry_imu_quat_.conjugate() * q_robot;
    q_robot.normalize();
    Eigen::Quaterniond q_rel = q_robot.conjugate() * q_anchor;
    q_rel.normalize();
    // 提取 (roll, pitch, yaw)，与训练代码 _quat_to_euler_xyz 一致
    const double w = q_rel.w(), x = q_rel.x(), y = q_rel.y(), z = q_rel.z();
    const double sin_roll = 2.0 * (w * x + y * z);
    const double cos_roll = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sin_roll, cos_roll);
    const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double sin_yaw = 2.0 * (w * z + x * y);
    const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(sin_yaw, cos_yaw);
    anchor_euler[0] = roll;
    anchor_euler[1] = pitch;
    anchor_euler[2] = yaw;
    return anchor_euler;
  }

  void VMPController::applyTemporalNormalization(float* input_data, size_t data_size)
  {
    int window_length = vmp_config_.window_l;
    int feature_dim = vmp_config_.in_c;
    int future_frames = vmp_config_.window_r;           // e.g. 15
    int past_frames = window_length - future_frames - 1;  // e.g. 16，当前帧位于索引 past_frames
    
    if (data_size != static_cast<size_t>(window_length * feature_dim)) {
      ROS_WARN("[%s] Data size mismatch in temporal normalization: %zu vs expected %d",
               name_.c_str(), data_size, window_length * feature_dim);
      return;
    }
    
    // 提取中心帧方向向量 col0（用于估计 yaw）
    float cfr_data[3];
    for (int i = 0; i < 3; i++) {
      int tensor_idx = past_frames * feature_dim + (vmp_config_.theta_start_id + i);
      cfr_data[i] = input_data[tensor_idx];
    }
    
    float c0[3] = {cfr_data[0], cfr_data[1], cfr_data[2]};
    float c0_norm = std::sqrt(c0[0]*c0[0] + c0[1]*c0[1] + c0[2]*c0[2]);
    if (c0_norm > 1e-6f) {
      c0[0] /= c0_norm; c0[1] /= c0_norm; c0[2] /= c0_norm;
    }

    // 计算yaw角和逆旋转矩阵 (inv_z_rotate)
    float yaw = std::atan2(c0[1], c0[0]);
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);

    float inv_rot[3][3] = {
      { cos_yaw,  sin_yaw, 0.0f},
      {-sin_yaw,  cos_yaw, 0.0f},
      { 0.0f,     0.0f,    1.0f}
    };
    
    // === 与Python vae_module.py对齐：对theta也应用yaw去旋转 ===
    // Python流程:
    // 1. rot6 -> reshape(3,2) -> complete_orthogonal -> R_full(3,3)
    // 2. R_deyaw = inv_rot @ R_full
    // 3. new_rot6 = R_deyaw[:,:2] (取前两列)
    int theta_start = vmp_config_.theta_start_id;
    for (int t = 0; t < window_length; t++) {
      // 读取当前帧的6D旋转表示
      float rot6[6];
      for (int i = 0; i < 6; i++) {
        rot6[i] = input_data[t * feature_dim + theta_start + i];
      }
      
      // === Step 1: complete_orthogonal - 从6D构建3x3正交矩阵 ===
      // col0 = normalize(rot6[0:3])
      float col0[3] = {rot6[0], rot6[1], rot6[2]};
      float col0_norm = std::sqrt(col0[0]*col0[0] + col0[1]*col0[1] + col0[2]*col0[2]);
      if (col0_norm > 1e-6f) {
        col0[0] /= col0_norm; col0[1] /= col0_norm; col0[2] /= col0_norm;
      }
      
      // col1 = normalize(rot6[3:6] - proj(rot6[3:6] onto col0))
      float col1_raw[3] = {rot6[3], rot6[4], rot6[5]};
      float dot_col0_col1 = col0[0]*col1_raw[0] + col0[1]*col1_raw[1] + col0[2]*col1_raw[2];
      float col1[3] = {
        col1_raw[0] - dot_col0_col1 * col0[0],
        col1_raw[1] - dot_col0_col1 * col0[1],
        col1_raw[2] - dot_col0_col1 * col0[2]
      };
      float col1_norm = std::sqrt(col1[0]*col1[0] + col1[1]*col1[1] + col1[2]*col1[2]);
      if (col1_norm > 1e-6f) {
        col1[0] /= col1_norm; col1[1] /= col1_norm; col1[2] /= col1_norm;
      }
      
      // R_full 前两列 = [col0, col1]；deyaw 只需这两列
      // === Step 2: R_deyaw = inv_rot @ R_full ===
      // 对每一列应用inv_rot
      float new_c0[3], new_c1[3];
      for (int i = 0; i < 3; i++) {
        new_c0[i] = inv_rot[i][0] * col0[0] + inv_rot[i][1] * col0[1] + inv_rot[i][2] * col0[2];
        new_c1[i] = inv_rot[i][0] * col1[0] + inv_rot[i][1] * col1[1] + inv_rot[i][2] * col1[2];
      }
      
      // === Step 3: 取前两列作为新的6D ===
      input_data[t * feature_dim + theta_start + 0] = new_c0[0];
      input_data[t * feature_dim + theta_start + 1] = new_c0[1];
      input_data[t * feature_dim + theta_start + 2] = new_c0[2];
      input_data[t * feature_dim + theta_start + 3] = new_c1[0];
      input_data[t * feature_dim + theta_start + 4] = new_c1[1];
      input_data[t * feature_dim + theta_start + 5] = new_c1[2];
    }
    
    // === 应用旋转到速度向量 [v_start_id : v_end_id] ===
    // 速度是6D: [linear_vel(3), angular_vel(3)]
    int v_start = vmp_config_.v_start_id;
    for (int t = 0; t < window_length; t++) {
      float vec1[3], vec2[3];
      for (int i = 0; i < 3; i++) {
        vec1[i] = input_data[t * feature_dim + (v_start + i)];      // linear_vel
        vec2[i] = input_data[t * feature_dim + (v_start + 3 + i)];  // angular_vel
      }

      float rot_vec1[3], rot_vec2[3];
      for (int i = 0; i < 3; i++) {
        rot_vec1[i] = inv_rot[i][0] * vec1[0] + inv_rot[i][1] * vec1[1] + inv_rot[i][2] * vec1[2];
        rot_vec2[i] = inv_rot[i][0] * vec2[0] + inv_rot[i][1] * vec2[1] + inv_rot[i][2] * vec2[2];
      }

      for (int i = 0; i < 3; i++) {
        input_data[t * feature_dim + (v_start + i)] = rot_vec1[i];
        input_data[t * feature_dim + (v_start + 3 + i)] = rot_vec2[i];
      }
    }

    // === 应用旋转到末端位置向量 [p_start_id : p_end_id]，共4个末端 * 3D ===
    int p_start = vmp_config_.p_start_id;
    for (int t = 0; t < window_length; t++) {
      float vecs[4][3];
      for (int v = 0; v < 4; v++) {
        for (int i = 0; i < 3; i++) {
          vecs[v][i] = input_data[t * feature_dim + (p_start + v*3 + i)];
        }
      }

      float rot_vecs[4][3];
      for (int v = 0; v < 4; v++) {
        for (int i = 0; i < 3; i++) {
          rot_vecs[v][i] = inv_rot[i][0] * vecs[v][0] + 
                           inv_rot[i][1] * vecs[v][1] + 
                           inv_rot[i][2] * vecs[v][2];
        }
      }
      
      for (int v = 0; v < 4; v++) {
        for (int i = 0; i < 3; i++) {
          input_data[t * feature_dim + (p_start + v*3 + i)] = rot_vecs[v][i];
        }
      }
    }
    
    // === 归一化 ===
    // 与Python vae_module.py对齐：支持use_global_coef开关
    // 注意：v46（77维）不支持全局归一化，仅v52（79维）支持
    std::vector<float> pad_mu(feature_dim);
    std::vector<float> pad_std(feature_dim);
    
    bool use_global = vmp_config_.use_global_coef;
    
    // v46版本不支持全局归一化，强制使用局部归一化
    if (use_global && feature_dim == 77) {
      ROS_INFO_THROTTLE(10.0, "[%s] v46 format (77 dims) does not support global normalization, using local normalization",
                       name_.c_str());
      use_global = false;
    }
    
    if (use_global) {
      // 使用全局均值和标准差（仅v52支持）
      const std::vector<float>& global_mean = vae_normalization::GLOBAL_MEAN_V52;
      const std::vector<float>& global_std = vae_normalization::GLOBAL_STD_V52;
      
      if (static_cast<size_t>(feature_dim) != global_mean.size()) {
        ROS_WARN_THROTTLE(5.0, "[%s] Feature dim %d != global coef size %zu, falling back to local normalization",
                         name_.c_str(), feature_dim, global_mean.size());
        use_global = false;
      } else {
        for (int f = 0; f < feature_dim; f++) {
          pad_mu[f] = global_mean[f];
          pad_std[f] = global_std[f];
        }
      }
    }
    
    if (!use_global) {
      // 使用局部（per-window）均值和标准差
      // 创建归一化掩码（跳过方向特征 [theta_start_id : theta_end_id]）
      std::vector<bool> norm_mask(feature_dim, true);
      for (int i = vmp_config_.theta_start_id; i < vmp_config_.theta_end_id; i++) {
        norm_mask[i] = false;
      }

      // 计算特征均值
      std::vector<float> feature_means(feature_dim, 0.0f);
      for (int f = 0; f < feature_dim; f++) {
        if (norm_mask[f]) {
          double sum = 0.0;
          for (int t = 0; t < window_length; t++) {
            sum += static_cast<double>(input_data[t * feature_dim + f]);
          }
          feature_means[f] = static_cast<float>(sum / window_length);
        }
      }
      
      // 计算特征标准差
      std::vector<float> feature_stds(feature_dim, 0.0f);
      for (int f = 0; f < feature_dim; f++) {
        if (norm_mask[f]) {
          double sum_sq_diff = 0.0;
          for (int t = 0; t < window_length; t++) {
            double diff = static_cast<double>(input_data[t * feature_dim + f]) - static_cast<double>(feature_means[f]);
            sum_sq_diff += diff * diff;
          }
          feature_stds[f] = static_cast<float>(std::sqrt(sum_sq_diff / window_length));
        }
      }

      // 准备归一化参数
      for (int f = 0; f < feature_dim; f++) {
        // 方向特征（theta）不归一化
        if (f >= vmp_config_.theta_start_id && f < vmp_config_.theta_end_id) {
          pad_mu[f] = 0.0f;
          pad_std[f] = 1.0f;
        } else {
          pad_mu[f] = feature_means[f];
          pad_std[f] = feature_stds[f];
        }
      }
    }

    // 应用归一化
    for (int t = 0; t < window_length; t++) {
      for (int f = 0; f < feature_dim; f++) {
        float normalized_value = (input_data[t * feature_dim + f] - pad_mu[f]) / 
                                (pad_std[f] + 1e-8f);
        input_data[t * feature_dim + f] = normalized_value;
      }
    }
  }

  std::vector<float> VMPController::applyGaussianFilter(const std::vector<float>& velocities)
  {
    return velocities;
  }

  std::vector<float> VMPController::applyGMRSmoothing(const std::vector<float>& raw_frame)
  {
    // 线程安全：保护共享状态
    std::lock_guard<std::mutex> lock(gmr_smoothing_mutex_);
    
    // GMR数据平滑：使用高斯滤波平滑base速度和关节速度，防止噪声和速度尖峰
    // 输入：vmp_config_.in_c 维 VMP格式数据 [h + theta + v + q + q̇ + p]
    // 索引范围由配置文件定义，支持不同机型
    
    if (!enable_gmr_smoothing_) {
      return raw_frame;  // 未启用平滑，直接返回
    }
    
    // 从配置获取索引范围
    int v_start = vmp_config_.v_start_id;
    int v_end = vmp_config_.v_end_id;  // exclusive
    int q_dot_start = vmp_config_.q_dot_start_id;
    int q_dot_end = vmp_config_.q_dot_end_id;  // exclusive
    
    // 先对当前帧进行速度裁剪和异常值处理
    std::vector<float> clipped_data = raw_frame;
    float clip_value = static_cast<float>(gmr_velocity_clip_);
    
    // 1. 处理base速度 [v_start_id : v_end_id]
    for (int i = v_start; i < v_end && i < static_cast<int>(clipped_data.size()); i++) {
      float current_vel = clipped_data[i];
      if (!std::isfinite(current_vel)) {
        current_vel = 0.0f;
      }
      clipped_data[i] = std::clamp(current_vel, -clip_value, clip_value);
    }
    
    // 2. 处理关节速度 [q_dot_start_id : q_dot_end_id]
    for (int i = q_dot_start; i < q_dot_end && i < static_cast<int>(clipped_data.size()); i++) {
      float current_vel = clipped_data[i];
      if (!std::isfinite(current_vel)) {
        current_vel = 0.0f;
      }
      clipped_data[i] = std::clamp(current_vel, -clip_value, clip_value);
    }
    
    // 将当前帧加入历史缓冲区
    gmr_history_buffer_.push_back(clipped_data);
    
    // 限制缓冲区大小（取速度核和qpos核中较大者）
    int max_kernel = gmr_gaussian_kernel_size_;
    if (enable_gmr_qpos_smoothing_ && gmr_qpos_kernel_size_ > max_kernel) {
      max_kernel = gmr_qpos_kernel_size_;
    }
    while (static_cast<int>(gmr_history_buffer_.size()) > max_kernel) {
      gmr_history_buffer_.pop_front();
    }
    
    // 当前缓冲区大小
    int buffer_size = static_cast<int>(gmr_history_buffer_.size());
    
    // 关节位置索引范围（用于qpos平滑）
    int q_start = vmp_config_.q_start_id;
    int q_end = vmp_config_.q_end_id;  // exclusive
    
    // 高斯滤波输出
    std::vector<float> smoothed_data(raw_frame.size(), 0.0f);
    
    // 对每个维度进行高斯滤波
    // 速度维度：v[v_start_id:v_end_id] + q̇[q_dot_start_id:q_dot_end_id] → 使用速度核
    // 关节位置维度：q[q_start_id:q_end_id] → 使用qpos核（如果启用）
    // 不平滑：h, theta, p
    for (size_t i = 0; i < raw_frame.size(); i++) {
      int idx = static_cast<int>(i);
      bool is_velocity = (idx >= v_start && idx < v_end) ||
                         (idx >= q_dot_start && idx < q_dot_end);
      bool is_qpos = enable_gmr_qpos_smoothing_ &&
                     (idx >= q_start && idx < q_end);
      
      if (is_velocity && !gmr_gaussian_kernel_.empty()) {
        // 速度维度：使用速度高斯核（因果滤波：核右端对齐最新帧，仅使用历史数据）
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        
        for (int k = 0; k < gmr_gaussian_kernel_size_; k++) {
          int buffer_idx = buffer_size - gmr_gaussian_kernel_size_ + k;
          if (buffer_idx < 0) buffer_idx = 0;
          
          weighted_sum += gmr_gaussian_kernel_[k] * gmr_history_buffer_[buffer_idx][i];
          weight_sum += gmr_gaussian_kernel_[k];
        }
        smoothed_data[i] = static_cast<float>(weighted_sum / weight_sum);
        
      } else if (is_qpos && !gmr_qpos_kernel_.empty()) {
        // 关节位置维度：使用qpos专用高斯核（因果滤波：核右端对齐最新帧，仅使用历史数据）
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        
        for (int k = 0; k < gmr_qpos_kernel_size_; k++) {
          int buffer_idx = buffer_size - gmr_qpos_kernel_size_ + k;
          if (buffer_idx < 0) buffer_idx = 0;
          
          weighted_sum += gmr_qpos_kernel_[k] * gmr_history_buffer_[buffer_idx][i];
          weight_sum += gmr_qpos_kernel_[k];
        }
        smoothed_data[i] = static_cast<float>(weighted_sum / weight_sum);
        
      } else {
        // 不需要平滑的维度，使用最新帧的值
        smoothed_data[i] = clipped_data[i];
      }
    }
    
    // 3. 速度死区处理：将接近0的速度置为0，防止静止时抖动
    if (gmr_velocity_deadzone_ > 0.0) {
      float deadzone = static_cast<float>(gmr_velocity_deadzone_);
      
      // 处理base速度死区 [v_start_id : v_end_id]
      for (int i = v_start; i < v_end && i < static_cast<int>(smoothed_data.size()); i++) {
        if (std::abs(smoothed_data[i]) < deadzone) {
          smoothed_data[i] = 0.0f;
        }
      }
      
      // 处理关节速度死区 [q_dot_start_id : q_dot_end_id]
      for (int i = q_dot_start; i < q_dot_end && i < static_cast<int>(smoothed_data.size()); i++) {
        if (std::abs(smoothed_data[i]) < deadzone) {
          smoothed_data[i] = 0.0f;
        }
      }
    }
    
    // 标记已有历史数据
    if (!has_gmr_history_) {
      has_gmr_history_ = true;
    }
    
    return smoothed_data;
  }

  // ===== Qpos平滑：重复帧检测与回溯插值 =====

  bool VMPController::isNewQpos(const std::vector<float>& frame)
  {
    int q_start = vmp_config_.q_start_id;
    int q_end   = vmp_config_.q_end_id;

    // 首帧始终视为"新帧"
    if (qpos_last_ingested_.empty()) {
      return true;
    }

    // 安全检查
    if (static_cast<int>(frame.size()) < q_end ||
        static_cast<int>(qpos_last_ingested_.size()) < q_end) {
      return true;
    }

    // 计算关节位置维度的最大绝对差值
    float max_diff = 0.0f;
    for (int i = q_start; i < q_end; ++i) {
      float diff = std::abs(frame[i] - qpos_last_ingested_[i]);
      if (diff > max_diff) {
        max_diff = diff;
      }
    }

    return max_diff > static_cast<float>(qpos_new_joint_eps_);
  }

  std::vector<float> VMPController::interpolateFrame(
      const std::vector<float>& f0,
      const std::vector<float>& f1,
      float alpha)
  {
    size_t n = std::min(f0.size(), f1.size());
    std::vector<float> result(n);

    // 获取 theta（6D rotation）的索引范围
    int theta_start = vmp_config_.theta_start_id;
    int theta_end   = vmp_config_.theta_end_id;
    bool use_slerp  = (theta_end - theta_start == 6) &&
                      (static_cast<int>(n) >= theta_end);

    // ---- theta 维度：6D → 旋转矩阵 → 四元数 → slerp → 旋转矩阵 → 6D ----
    if (use_slerp) {
      // 辅助 lambda: 6D rotation → Eigen::Quaterniond
      // 6D = [col0(3), col1(3)]，即旋转矩阵的前两列
      auto rot6d_to_quat = [](const float* r6) -> Eigen::Quaterniond {
        // col0 = normalize(r6[0:3])
        Eigen::Vector3d c0(r6[0], r6[1], r6[2]);
        double c0_norm = c0.norm();
        if (c0_norm < 1e-8) c0 = Eigen::Vector3d::UnitX();
        else c0 /= c0_norm;

        // col1 = normalize(r6[3:6] - proj onto c0)  (Gram-Schmidt)
        Eigen::Vector3d c1_raw(r6[3], r6[4], r6[5]);
        Eigen::Vector3d c1 = c1_raw - c1_raw.dot(c0) * c0;
        double c1_norm = c1.norm();
        if (c1_norm < 1e-8) c1 = Eigen::Vector3d::UnitY();
        else c1 /= c1_norm;

        // col2 = cross(c0, c1)
        Eigen::Vector3d c2 = c0.cross(c1);

        Eigen::Matrix3d R;
        R.col(0) = c0;
        R.col(1) = c1;
        R.col(2) = c2;

        return Eigen::Quaterniond(R).normalized();
      };

      // 辅助 lambda: Eigen::Quaterniond → 6D rotation (float[6])
      auto quat_to_rot6d = [](const Eigen::Quaterniond& q, float* r6) {
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        // 存储前两列
        for (int i = 0; i < 3; ++i) {
          r6[i]     = static_cast<float>(R(i, 0));  // col0
          r6[i + 3] = static_cast<float>(R(i, 1));  // col1
        }
      };

      Eigen::Quaterniond q0 = rot6d_to_quat(&f0[theta_start]);
      Eigen::Quaterniond q1 = rot6d_to_quat(&f1[theta_start]);
      Eigen::Quaterniond q_interp = q0.slerp(static_cast<double>(alpha), q1);

      // 写入 theta 维度
      float rot6_out[6];
      quat_to_rot6d(q_interp, rot6_out);
      for (int i = 0; i < 6; ++i) {
        result[theta_start + i] = rot6_out[i];
      }
    }

    // ---- 其余维度：线性插值 ----
    for (size_t i = 0; i < n; ++i) {
      // 跳过已经用 slerp 处理的 theta 维度
      if (use_slerp && static_cast<int>(i) >= theta_start && static_cast<int>(i) < theta_end) {
        continue;
      }
      result[i] = (1.0f - alpha) * f0[i] + alpha * f1[i];
    }

    return result;
  }

  void VMPController::retroInterpolatePendingTail()
  {
    // 回溯插值：当新的锚点到达时，找到上一个锚点，
    // 将两个锚点之间的所有"held"帧进行线性插值
    if (qpos_pending_buffer_.size() < 2) {
      return;
    }

    // 最新帧必须是锚点（刚刚追加的新帧）
    int last_idx = static_cast<int>(qpos_pending_buffer_.size()) - 1;
    if (!qpos_pending_buffer_[last_idx].anchor) {
      return;
    }

    // 从倒数第二个往前找上一个锚点
    int prev_anchor_idx = -1;
    for (int i = last_idx - 1; i >= 0; --i) {
      if (qpos_pending_buffer_[i].anchor) {
        prev_anchor_idx = i;
        break;
      }
    }

    if (prev_anchor_idx < 0) {
      return;  // 没有前一个锚点，无法插值
    }

    // 两个锚点之间的帧数（包含两端）
    int span = last_idx - prev_anchor_idx;
    if (span <= 1) {
      return;  // 没有需要插值的中间帧
    }

    const auto& f0 = qpos_pending_buffer_[prev_anchor_idx].frame;
    const auto& f1 = qpos_pending_buffer_[last_idx].frame;

    // 对中间的held帧进行线性插值（q、h、theta、v、p等所有维度）
    for (int i = prev_anchor_idx + 1; i < last_idx; ++i) {
      float alpha = static_cast<float>(i - prev_anchor_idx) / static_cast<float>(span);
      qpos_pending_buffer_[i].frame = interpolateFrame(f0, f1, alpha);
      qpos_pending_buffer_[i].smoothed = true;
      qpos_stat_interp_count_++;
    }

    // q_dot 维度：用插值后的 q 通过有限差分重新计算，而非独立插值
    // q_dot[i] = (q[i+1] - q[i-1]) / (2 * dt)  （中心差分）
    // 边界帧（anchor本身）也重新计算，保证 q/q_dot 自洽
    int q_start = vmp_config_.q_start_id;
    int q_end   = vmp_config_.q_end_id;
    int qd_start = vmp_config_.q_dot_start_id;
    float dt = 1.0f / static_cast<float>(inference_frequency_);  // 每帧间隔 = 1/100Hz = 0.01s

    for (int i = prev_anchor_idx; i <= last_idx; ++i) {
      auto& frame = qpos_pending_buffer_[i].frame;
      for (int j = 0; j < (q_end - q_start); ++j) {
        float q_prev, q_next;
        if (i > 0) {
          q_prev = qpos_pending_buffer_[i - 1].frame[q_start + j];
        } else {
          q_prev = frame[q_start + j];  // 边界：用自身
        }
        if (i < static_cast<int>(qpos_pending_buffer_.size()) - 1) {
          q_next = qpos_pending_buffer_[i + 1].frame[q_start + j];
        } else {
          q_next = frame[q_start + j];  // 边界：用自身
        }

        if (i > 0 && i < static_cast<int>(qpos_pending_buffer_.size()) - 1) {
          // 中心差分: (q[i+1] - q[i-1]) / (2*dt)
          frame[qd_start + j] = (q_next - q_prev) / (2.0f * dt);
        } else {
          // 前向/后向差分
          frame[qd_start + j] = (q_next - q_prev) / dt;
        }
      }
    }
  }

  std::vector<float> VMPController::popOldestPendingIfReady()
  {
    // 当缓冲区超过配置大小时，弹出最旧的帧
    if (static_cast<int>(qpos_pending_buffer_.size()) > qpos_smooth_buffer_size_) {
      std::vector<float> frame = std::move(qpos_pending_buffer_.front().frame);
      qpos_pending_buffer_.pop_front();
      return frame;
    }
    return {};  // 缓冲区未满，返回空
  }

  Eigen::Quaterniond VMPController::slerpQuaternion(const Eigen::Quaterniond& q1,
                                                     const Eigen::Quaterniond& q2, double t)
  {
    return q1.slerp(t, q2);
  }

  std::vector<std::vector<float>> VMPController::interpolateFrameSequence(
      const std::vector<float>& frame1,
      const std::vector<float>& frame2,
      double alpha)
  {
    std::vector<std::vector<float>> result;
    result.push_back(frame1);
    result.push_back(frame2);
    return result;
  }

  std::vector<float> VMPController::interpolateWithTimestamp(
      const std::vector<float>& frame_prev,
      const std::vector<float>& frame_next,
      ros::Time t_prev, ros::Time t_next, ros::Time t_target)
  {
    return frame_prev;
  }

  // ========== 辅助函数 ==========
  
  bool VMPController::fileExists(const std::string& path)
  {
    std::ifstream file(path);
    return file.good();
  }

  std::string VMPController::determineVMPDataPath(const std::string& filename)
  {
    return baseModelPath_ + "/" + vmpRefDataDir_ + filename;
  }

  void VMPController::printMultiTrajectoryStatus()
  {
    ROS_INFO("[%s] Multi-trajectory status: index=%d/%zu", 
             name_.c_str(), currentTrajectoryIndex_, trajectorySequence_.size());
  }

  void VMPController::printOnlineBufferStatus()
  {
    ROS_INFO("[%s] Online buffer: %d/%d frames",
             name_.c_str(), online_ref_buffer_.getBufferFrameCount(), online_buffer_size_);
  }

  // ========== 未实现的函数（占位符） ==========
  
  void VMPController::warmupVMPModels() 
  {
    ROS_INFO("[%s] Starting VMP models warm-up...", name_.c_str());
    
    try {
      const int warmup_iterations = 3;
      
      // Encoder warm-up
      ROS_INFO("[%s] Warming up encoder (%d iterations)...", name_.c_str(), warmup_iterations);
      ov::Shape encoder_input_shape = {1, static_cast<size_t>(vmp_config_.in_c), 
                                       static_cast<size_t>(vmp_config_.window_l)};
      ov::Tensor encoder_dummy_tensor(ov::element::f32, encoder_input_shape);
      float* encoder_dummy_data = encoder_dummy_tensor.data<float>();
      
      for (size_t i = 0; i < encoder_dummy_tensor.get_size(); ++i) {
        encoder_dummy_data[i] = 0.0f;
      }
      
      auto encoder_start = std::chrono::high_resolution_clock::now();
      for (int i = 0; i < warmup_iterations; ++i) {
        vmp_encoder_request_.set_input_tensor(0, encoder_dummy_tensor);
        vmp_encoder_request_.infer();
      }
      auto encoder_end = std::chrono::high_resolution_clock::now();
      double encoder_time = std::chrono::duration<double, std::milli>(encoder_end - encoder_start).count();
      ROS_INFO("[%s] Encoder warm-up completed: %.2f ms total, %.2f ms avg", 
               name_.c_str(), encoder_time, encoder_time / warmup_iterations);
      
      // Policy warm-up
      ROS_INFO("[%s] Warming up policy (%d iterations)...", name_.c_str(), warmup_iterations);
      int policy_input_size = vmp_numSingleObs_ + numRefMotionObs_ + vmp_config_.latent_d;
      if (vmp_enable_estimator_) {
        policy_input_size += vmp_estimator_output_dim_;
      }
      ov::Shape policy_input_shape = {1, static_cast<size_t>(policy_input_size)};
      ov::Tensor policy_dummy_tensor(ov::element::f32, policy_input_shape);
      float* policy_dummy_data = policy_dummy_tensor.data<float>();
      
      for (size_t i = 0; i < policy_dummy_tensor.get_size(); ++i) {
        policy_dummy_data[i] = 0.0f;
      }
      
      auto policy_start = std::chrono::high_resolution_clock::now();
      for (int i = 0; i < warmup_iterations; ++i) {
        vmp_policy_request_.set_input_tensor(0, policy_dummy_tensor);
        vmp_policy_request_.infer();
      }
      auto policy_end = std::chrono::high_resolution_clock::now();
      double policy_time = std::chrono::duration<double, std::milli>(policy_end - policy_start).count();
      ROS_INFO("[%s] Policy warm-up completed: %.2f ms total, %.2f ms avg", 
               name_.c_str(), policy_time, policy_time / warmup_iterations);
      
      // Estimator warm-up (如果启用)
      if (vmp_enable_estimator_) {
        ROS_INFO("[%s] Warming up estimator (%d iterations)...", name_.c_str(), warmup_iterations);
        // 使用正确的3D形状 [batch=1, time=history_frames, features=input_dim]
        ov::Shape estimator_input_shape = {1, 
                                           static_cast<size_t>(vmp_estimator_history_frames_), 
                                           static_cast<size_t>(vmp_estimator_input_dim_)};
        ov::Tensor estimator_dummy_tensor(ov::element::f32, estimator_input_shape);
        float* estimator_dummy_data = estimator_dummy_tensor.data<float>();
        
        for (size_t i = 0; i < estimator_dummy_tensor.get_size(); ++i) {
          estimator_dummy_data[i] = 0.0f;
        }
        
        auto estimator_start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < warmup_iterations; ++i) {
          vmp_estimator_request_.set_input_tensor(0, estimator_dummy_tensor);
          vmp_estimator_request_.infer();
        }
        auto estimator_end = std::chrono::high_resolution_clock::now();
        double estimator_time = std::chrono::duration<double, std::milli>(estimator_end - estimator_start).count();
        ROS_INFO("[%s] Estimator warm-up completed: %.2f ms total, %.2f ms avg", 
                 name_.c_str(), estimator_time, estimator_time / warmup_iterations);
      }
      
      ROS_INFO("[%s] VMP models warm-up completed successfully", name_.c_str());
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Error during warm-up: %s", name_.c_str(), e.what());
    }
  }
  void VMPController::loadCurrentTrajectoryData() {}
  void VMPController::switchToNextTrajectory() {}
  void VMPController::appendStandingFramesToTaskData() 
  {
    if (vmp_task_data_.empty()) {
      ROS_WARN("[%s] Cannot append standing frames to empty task data", name_.c_str());
      return;
    }
    
    int original_total_frames = vmp_task_data_.size() / vmp_config_.in_c;
    int new_total_frames = original_total_frames + vmp_pre_standing_frames_ + vmp_post_standing_frames_;
    
    ROS_INFO("[%s] Appending standing frames with interpolation:", name_.c_str());
    ROS_INFO("[%s]   %d pre + %d original + %d post = %d total frames",
             name_.c_str(), vmp_pre_standing_frames_, original_total_frames, 
             vmp_post_standing_frames_, new_total_frames);
    ROS_INFO("[%s]   Interpolation: %d pre, %d post", 
             name_.c_str(), vmp_pre_interpolation_frames_, vmp_post_interpolation_frames_);
    
    // 获取运动轨迹的起始帧（用于前置插值）
    std::vector<float> start_real(vmp_config_.in_c);
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      start_real[j] = vmp_task_data_[j];
    }
    
    std::vector<float> new_task_data(new_total_frames * vmp_config_.in_c);
    
    // ===== 1. 添加前置静止帧（带插值） =====
    for (int i = 0; i < vmp_pre_standing_frames_; ++i) {
      int start_idx = i * vmp_config_.in_c;
      
      // 首先填充完整的静止帧
      for (int j = 0; j < vmp_config_.in_c; ++j) {
        new_task_data[start_idx + j] = vmp_standing_frame_[j];
      }
      
      // 在接近运动轨迹起点时进行插值（仅对关节位置）
      // 插值区间: [pre_standing - pre_interp, pre_standing - 1]
      // 当 i = pre_standing - pre_interp 时, alpha = 0 (纯静止帧)
      // 当 i = pre_standing - 1 时, alpha = 1 (接近运动轨迹起点)
      if (i >= vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_) {
        // alpha: 0 → 1，从静止帧过渡到运动轨迹起点
        // 修正公式: 使用 (pre_interp - 1) 作为分母，确保 alpha 范围为 [0, 1]
        int interp_idx = i - (vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_);
        float alpha = (vmp_pre_interpolation_frames_ > 1) 
                      ? float(interp_idx) / float(vmp_pre_interpolation_frames_ - 1)
                      : 1.0f;
        alpha = std::min(alpha, 1.0f);  // 确保不超过1
        
        // 对关节位置进行线性插值
        for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
          new_task_data[start_idx + j] = (1.0f - alpha) * vmp_standing_frame_[j] + alpha * start_real[j];
        }
      }
    }
    
    // ===== 2. 复制原始轨迹数据 =====
    int original_start_idx = vmp_pre_standing_frames_ * vmp_config_.in_c;
    for (size_t i = 0; i < vmp_task_data_.size(); ++i) {
      new_task_data[original_start_idx + i] = vmp_task_data_[i];
    }
    
    // ===== 3. 添加后置静止帧（带插值） =====
    // 获取运动轨迹的结束帧
    std::vector<float> end_real(vmp_config_.in_c);
    int last_frame_idx = (original_total_frames - 1) * vmp_config_.in_c;
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      end_real[j] = vmp_task_data_[last_frame_idx + j];
    }
    
    int post_start_idx = (vmp_pre_standing_frames_ + original_total_frames) * vmp_config_.in_c;
    for (int i = 0; i < vmp_post_standing_frames_; ++i) {
      int start_idx = post_start_idx + i * vmp_config_.in_c;
      
      // 首先填充完整的静止帧
      for (int j = 0; j < vmp_config_.in_c; ++j) {
        new_task_data[start_idx + j] = vmp_standing_frame_[j];
      }
      
      // 在刚离开运动轨迹时进行插值（仅对关节位置）
      // 插值区间: [0, post_interp - 1]
      // 当 i = 0 时, alpha = 1 (接近运动轨迹终点)
      // 当 i = post_interp - 1 时, alpha = 0 (纯静止帧)
      if (i < vmp_post_interpolation_frames_ && i < vmp_post_standing_frames_) {
        // alpha: 1 → 0，从运动轨迹终点过渡到静止帧
        // 修正公式: 使用 (post_interp - 1) 作为分母，确保 alpha 范围为 [1, 0]
        float alpha = (vmp_post_interpolation_frames_ > 1)
                      ? float(vmp_post_interpolation_frames_ - 1 - i) / float(vmp_post_interpolation_frames_ - 1)
                      : 0.0f;
        alpha = std::max(alpha, 0.0f);  // 确保不小于0
        
        // 对关节位置进行线性插值
        for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
          new_task_data[start_idx + j] = alpha * end_real[j] + (1.0f - alpha) * vmp_standing_frame_[j];
        }
      }
    }
    
    // ===== 4. 对插值区间的 q_dot 用有限差分重新计算（保证 q/q_dot 自洽） =====
    {
      int q_s   = vmp_config_.q_start_id;
      int q_e   = vmp_config_.q_end_id;
      int qd_s  = vmp_config_.q_dot_start_id;
      int dim   = q_e - q_s;
      float dt  = 1.0f / static_cast<float>(inference_frequency_);

      // 前置插值区间: [pre_standing - pre_interp, pre_standing - 1]
      int pre_interp_begin = vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_;
      int pre_interp_end   = vmp_pre_standing_frames_;  // 不含（下一帧是原始轨迹第0帧）
      for (int i = pre_interp_begin; i < pre_interp_end; ++i) {
        int cur_base = i * vmp_config_.in_c;
        for (int j = 0; j < dim; ++j) {
          float q_prev, q_next;
          if (i > 0) {
            q_prev = new_task_data[(i - 1) * vmp_config_.in_c + q_s + j];
          } else {
            q_prev = new_task_data[cur_base + q_s + j];
          }
          if (i < new_total_frames - 1) {
            q_next = new_task_data[(i + 1) * vmp_config_.in_c + q_s + j];
          } else {
            q_next = new_task_data[cur_base + q_s + j];
          }
          if (i > 0 && i < new_total_frames - 1) {
            new_task_data[cur_base + qd_s + j] = (q_next - q_prev) / (2.0f * dt);
          } else {
            new_task_data[cur_base + qd_s + j] = (q_next - q_prev) / dt;
          }
        }
      }

      // 后置插值区间: [post_start_frame, post_start_frame + post_interp - 1]
      int post_start_frame = vmp_pre_standing_frames_ + original_total_frames;
      int post_interp_end_frame = post_start_frame + std::min(vmp_post_interpolation_frames_, vmp_post_standing_frames_);
      for (int i = post_start_frame; i < post_interp_end_frame; ++i) {
        int cur_base = i * vmp_config_.in_c;
        for (int j = 0; j < dim; ++j) {
          float q_prev, q_next;
          if (i > 0) {
            q_prev = new_task_data[(i - 1) * vmp_config_.in_c + q_s + j];
          } else {
            q_prev = new_task_data[cur_base + q_s + j];
          }
          if (i < new_total_frames - 1) {
            q_next = new_task_data[(i + 1) * vmp_config_.in_c + q_s + j];
          } else {
            q_next = new_task_data[cur_base + q_s + j];
          }
          if (i > 0 && i < new_total_frames - 1) {
            new_task_data[cur_base + qd_s + j] = (q_next - q_prev) / (2.0f * dt);
          } else {
            new_task_data[cur_base + qd_s + j] = (q_next - q_prev) / dt;
          }
        }
      }
    }

    vmp_task_data_ = std::move(new_task_data);
    
    ROS_INFO("[%s] Standing frames appended successfully. New total frames: %d", 
             name_.c_str(), (int)(vmp_task_data_.size() / vmp_config_.in_c));
  }
  bool VMPController::preloadAllTrajectories() { return true; }
  bool VMPController::preloadSingleTrajectory(size_t index, const std::string& dataFile) { return true; }
  void VMPController::processPreloadedTrajectory(size_t index) {}
  bool VMPController::switchToPreloadedTrajectory(size_t index) { return true; }
  void VMPController::clearPreloadedTrajectories() {}
  size_t VMPController::getPreloadedMemoryUsage() const { return 0; }
  void VMPController::updateOnlineReferenceBuffer(const std::vector<float>& new_frame)
  {
    std::lock_guard<std::mutex> lock(online_buffer_mutex_);
    
    if (!online_ref_buffer_.is_initialized) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error: Online buffer not initialized", name_.c_str());
      return;
    }
    
    if (static_cast<int>(new_frame.size()) != vmp_config_.in_c) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error: Frame size mismatch. Expected %d, got %zu", 
                         name_.c_str(), vmp_config_.in_c, new_frame.size());
      return;
    }
    
    try {
      // FIFO策略：使用环形缓冲区，覆盖最旧的数据
      online_ref_buffer_.data_buffer[online_ref_buffer_.current_write_index] = new_frame;
      
      // 更新写入索引（循环）
      online_ref_buffer_.current_write_index = 
        (online_ref_buffer_.current_write_index + 1) % online_buffer_size_;
      
      // 增加总帧数计数
      online_ref_buffer_.total_frames_received++;
      
      // 更新时间戳
      online_ref_buffer_.last_update_time = ros::Time::now();
      
      // 发布VMP输入数据用于调试和录制
      if (vmp_input_data_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray vmp_input_msg;
        vmp_input_msg.data = new_frame;
        vmp_input_data_pub_.publish(vmp_input_msg);
      }
      
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error updating buffer: %s", name_.c_str(), e.what());
    }
  }
  std::vector<float> VMPController::getFrameFromOnlineBuffer(int relative_index)
  {
    std::lock_guard<std::mutex> lock(online_buffer_mutex_);
    
    if (!online_ref_buffer_.is_initialized) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Error: Online buffer not initialized", name_.c_str());
      return std::vector<float>(vmp_config_.in_c, 0.0f);
    }
    
    // 计算实际索引
    // relative_index = 0 表示从当前读取位置开始，正数往后（未来帧）
    int actual_index = (online_ref_buffer_.current_read_index + relative_index + online_buffer_size_) % online_buffer_size_;
    
    // 安全性检查
    if (actual_index < 0 || actual_index >= static_cast<int>(online_ref_buffer_.data_buffer.size())) {
      ROS_WARN_THROTTLE(1.0, "[%s] Invalid buffer index %d for relative index %d", 
                        name_.c_str(), actual_index, relative_index);
      return std::vector<float>(vmp_config_.in_c, 0.0f);
    }
    
    return online_ref_buffer_.data_buffer[actual_index];
  }
  void VMPController::loadBinFileForOnlinePlayback(const std::string& data_file)
  {
    try {
      ROS_INFO("[%s] Loading bin file for online playback simulation...", name_.c_str());
      ROS_INFO("[%s] Data file: %s", name_.c_str(), data_file.c_str());
      
      // 打开数据文件
      std::ifstream data_file_stream(data_file, std::ios::binary);
      if (!data_file_stream.is_open()) {
        throw std::runtime_error("Cannot open data file: " + data_file);
      }
      
      // 获取文件大小并计算总帧数
      data_file_stream.seekg(0, std::ios::end);
      size_t file_size = data_file_stream.tellg();
      data_file_stream.seekg(0, std::ios::beg);
      
      ROS_INFO("[%s] Data file size: %zu bytes", name_.c_str(), file_size);
      
      // 从文件大小计算总帧数：文件大小 / (每帧维度 * float大小)
      online_bin_playback_total_frames_ = static_cast<int>(file_size / (vmp_config_.in_c * sizeof(float)));
      
      ROS_INFO("[%s] Calculated total frames: %d (based on file size and dimension %d)", 
               name_.c_str(), online_bin_playback_total_frames_, vmp_config_.in_c);
      
      // 读取数据
      int expected_data_count = online_bin_playback_total_frames_ * vmp_config_.in_c;
      online_bin_playback_data_.resize(expected_data_count);
      
      data_file_stream.read(reinterpret_cast<char*>(online_bin_playback_data_.data()), 
                           expected_data_count * sizeof(float));
      
      if (data_file_stream.fail() && !data_file_stream.eof()) {
        throw std::runtime_error("Error reading data file");
      }
      data_file_stream.close();
      
      // 初始化播放状态
      online_bin_playback_frame_index_ = 0;
      online_bin_playback_start_time_ = ros::Time::now();
      
      ROS_INFO("[%s] Bin file loaded successfully", name_.c_str());
      ROS_INFO("[%s] Ready to simulate online playback at %.1f Hz", name_.c_str(), online_update_rate_);
      
      // 调试：打印前几帧的数据，检查是否正确
      ROS_INFO("[%s] === First 3 frames (first 10 dims) ===", name_.c_str());
      for (int frame = 0; frame < std::min(3, online_bin_playback_total_frames_); ++frame) {
        std::ostringstream oss;
        oss << "Frame " << frame << ": [";
        for (int dim = 0; dim < std::min(10, vmp_config_.in_c); ++dim) {
          int idx = frame * vmp_config_.in_c + dim;
          if (idx < static_cast<int>(online_bin_playback_data_.size())) {
            oss << std::fixed << std::setprecision(3) << online_bin_playback_data_[idx];
            if (dim < 9) oss << ", ";
          }
        }
        oss << "]";
        ROS_INFO("[%s] %s", name_.c_str(), oss.str().c_str());
      }
      
    } catch (const std::exception& e) {
      ROS_ERROR("[%s] Error loading bin file: %s", name_.c_str(), e.what());
      online_bin_playback_data_.clear();
    }
  }
  void VMPController::updateOnlineBufferFromBinFile()
  {
    if (online_vr_data_source_ != "bin_file" || online_bin_playback_data_.empty()) {
      static int warning_count = 0;
      if (warning_count++ % 100 == 0) {
        ROS_WARN_THROTTLE(1.0, "[%s] Cannot update buffer. data_source=%s, data_empty=%d", 
                          name_.c_str(), online_vr_data_source_.c_str(), 
                          static_cast<int>(online_bin_playback_data_.empty()));
      }
      return;
    }
    
    // 检查是否已播放完毕
    if (online_bin_playback_frame_index_ >= online_bin_playback_total_frames_) {
      ROS_INFO("[%s] Playback completed. Total frames: %d. Restarting playback...", 
               name_.c_str(), online_bin_playback_frame_index_);
      
      // 重置播放（循环播放）
      online_bin_playback_frame_index_ = 0;
      online_bin_playback_start_time_ = ros::Time::now();
    }
    
    // 提取当前帧数据
    std::vector<float> current_frame(vmp_config_.in_c);
    int frame_start_idx = online_bin_playback_frame_index_ * vmp_config_.in_c;
    
    for (int i = 0; i < vmp_config_.in_c; ++i) {
      if (frame_start_idx + i < static_cast<int>(online_bin_playback_data_.size())) {
        current_frame[i] = online_bin_playback_data_[frame_start_idx + i];
      } else {
        current_frame[i] = 0.0f;
      }
    }
    
    // 应用控制模式过滤（upper_body 模式）
    bool is_upper_body_mode = (online_vr_control_mode_ == "upper_body");
    
    if (is_upper_body_mode && static_cast<int>(vmp_standing_frame_.size()) == vmp_config_.in_c) {
      // 上半身模式：用静止帧替换腿部和机身数据
      // 索引从 vmp_config_ 中获取，支持不同机型
      
      // 机身高度 [h_start_id : h_end_id]
      for (int i = vmp_config_.h_start_id; i < vmp_config_.h_end_id && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      // 旋转矩阵前2行 [theta_start_id : theta_end_id]
      for (int i = vmp_config_.theta_start_id; i < vmp_config_.theta_end_id && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      // 机身速度 [v_start_id : v_end_id]
      for (int i = vmp_config_.v_start_id; i < vmp_config_.v_end_id && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      // 腿部关节位置 [q_start_id : q_start_id + jointNum_]
      // v46: q[0-11], v52: q[0-12] (包含腰部在最前面时腿部从索引1开始)
      int leg_q_start = vmp_config_.q_start_id;
      int leg_q_end = vmp_config_.q_start_id + waistNum_ + jointNum_;  // 腰部 + 腿部
      for (int i = leg_q_start; i < leg_q_end && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      // 腿部关节速度 [q_dot_start_id : q_dot_start_id + jointNum_]
      int leg_qdot_start = vmp_config_.q_dot_start_id;
      int leg_qdot_end = vmp_config_.q_dot_start_id + waistNum_ + jointNum_;
      for (int i = leg_qdot_start; i < leg_qdot_end && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      // 脚部末端位置（保持静止帧）
      // 末端位置格式 (12维):
      //   v46: [L_hand(3), R_hand(3), L_foot(3), R_foot(3)] - 先手后腿，脚在索引6-11
      //   v52: [L_foot(3), R_foot(3), L_hand(3), R_hand(3)] - 先腿后手，脚在索引0-5
      int foot_p_start, foot_p_end;
      if (waistNum_ == 0) {
        // v46: 脚部在末端位置的后6维
        foot_p_start = vmp_config_.p_start_id + 6;
        foot_p_end = vmp_config_.p_end_id;
      } else {
        // v52: 脚部在末端位置的前6维
        foot_p_start = vmp_config_.p_start_id;
        foot_p_end = vmp_config_.p_start_id + 6;
      }
      for (int i = foot_p_start; i < foot_p_end && i < vmp_config_.in_c; ++i) {
        current_frame[i] = vmp_standing_frame_[i];
      }
      
      if (online_bin_playback_frame_index_ % 500 == 0) {
        ROS_INFO_THROTTLE(5.0, "[%s] Applied upper_body mode filtering (frame %d)", 
                          name_.c_str(), online_bin_playback_frame_index_);
      }
    }
    
    // 应用GMR速度平滑（如果启用）
    current_frame = applyGMRSmoothing(current_frame);
    
    // 更新缓存区
    updateOnlineReferenceBuffer(current_frame);
    
    online_bin_playback_frame_index_++;
  }
  void VMPController::updateEstimatorObservation(const Eigen::VectorXd& robot_observation, 
                                                 const SensorData& sensor_data) {
    // 与Python sim2sim保持一致：Estimator直接复用Policy的观测向量
    // Python: estimator_input.append(obs)  # 直接复用同一个obs
    // 观测结构：
    // [0:N]     dof_pos (关节位置偏移，已缩放)
    // [N:2N]    dof_vel (关节速度，已缩放)
    // [2N:3N]   actions (上一次动作)
    // [3N:3N+3] ang_vel (角速度，已缩放)
    // [3N+3:3N+6] robot_anchor_euler_w (先 clip 再 scale)
    
    if (!vmp_enable_estimator_) return;
    const int N = numActions_;
    const int est_frame_dim = 3 * N + 6;
    Eigen::VectorXd estimator_obs = robot_observation.head(est_frame_dim);
    // 验证维度
    if (estimator_obs.size() != vmp_estimator_input_dim_) {
      ROS_WARN_THROTTLE(1.0, "[%s] Estimator input dim mismatch: got %ld, expected %d",
                        name_.c_str(), estimator_obs.size(), vmp_estimator_input_dim_);
      // 如果维度不匹配，调整大小
      if (estimator_obs.size() < vmp_estimator_input_dim_) {
        Eigen::VectorXd padded(vmp_estimator_input_dim_);
        padded.setZero();
        padded.head(estimator_obs.size()) = estimator_obs;
        estimator_obs = padded;
      } else {
        estimator_obs = estimator_obs.head(vmp_estimator_input_dim_);
      }
    }
    
    // ===== 日志记录: Estimator 输入 =====
    if (ros_logger_) {
      std::vector<double> est_input(estimator_obs.data(), estimator_obs.data() + estimator_obs.size());
      ros_logger_->publishVector("/vmp_controller/estimator_input", est_input);
    }
    
    // 更新历史缓冲区 (FIFO逻辑: 先删除最老的,再追加新的)
    {
      std::lock_guard<std::mutex> lock(vmp_estimator_mtx_);
      
      // 如果缓冲区已满,先删除最老的帧
      if (static_cast<int>(vmp_estimator_obs_buffer_.size()) >= vmp_estimator_history_frames_) {
        vmp_estimator_obs_buffer_.pop_front();
      }
      
      // 追加新观测
      vmp_estimator_obs_buffer_.push_back(estimator_obs);
    }
  }
  
  Eigen::VectorXd VMPController::runEstimatorInference() { 
    if (!vmp_enable_estimator_) {
      return Eigen::VectorXd::Zero(vmp_estimator_output_dim_); 
    }
    
    try {
      std::lock_guard<std::mutex> lock(vmp_estimator_mtx_);
      
      // 严格检查缓冲区大小 (必须精确等于history_frames)
      if (static_cast<int>(vmp_estimator_obs_buffer_.size()) != vmp_estimator_history_frames_) {
        ROS_WARN_THROTTLE(1.0, "[%s] Estimator buffer size mismatch! Expected %d, got %zu",
                          name_.c_str(), vmp_estimator_history_frames_, vmp_estimator_obs_buffer_.size());
        return vmp_estimator_output_;
      }
      
      // 准备输入张量 [batch=1, time=history_frames, features=input_dim]
      // 内存布局: Row-major (C-style), 与PyTorch默认一致
      ov::Shape input_shape = {1, 
                               static_cast<size_t>(vmp_estimator_history_frames_), 
                               static_cast<size_t>(vmp_estimator_input_dim_)};
      ov::Tensor input_tensor(ov::element::f32, input_shape);
      float* input_data = input_tensor.data<float>();
      
      // 填充输入数据 (row-major顺序: 先遍历时间维度,再遍历特征维度)
      size_t data_idx = 0;
      for (const auto& obs : vmp_estimator_obs_buffer_) {
        // 验证观测维度
        if (static_cast<int>(obs.size()) != vmp_estimator_input_dim_) {
          ROS_WARN_THROTTLE(1.0, "[%s] Observation dimension mismatch! Expected %d, got %zu",
                            name_.c_str(), vmp_estimator_input_dim_, obs.size());
          return vmp_estimator_output_;
        }
        
        // 拷贝一帧的所有特征
        for (int i = 0; i < vmp_estimator_input_dim_; ++i) {
          float value = static_cast<float>(obs(i));
          
          // 数据合法性检查
          if (std::isnan(value) || std::isinf(value)) {
            value = 0.0f;  // 替换为零
          }
          
          input_data[data_idx++] = value;
        }
      }
      
      // 执行推理
      vmp_estimator_request_.set_input_tensor(0, input_tensor);
      vmp_estimator_request_.infer();
      
      // 获取输出张量
      auto output_tensor = vmp_estimator_request_.get_output_tensor(0);
      const float* output_data = output_tensor.data<float>();
      
      // 验证输出维度
      size_t output_size = output_tensor.get_size();
      if (static_cast<int>(output_size) != vmp_estimator_output_dim_) {
        ROS_WARN_THROTTLE(1.0, "[%s] Estimator output dimension mismatch! Expected %d, got %zu",
                          name_.c_str(), vmp_estimator_output_dim_, output_size);
        return vmp_estimator_output_;
      }
      
      // 复制输出数据并验证
      for (int i = 0; i < vmp_estimator_output_dim_; ++i) {
        float value = output_data[i];
        
        if (std::isnan(value) || std::isinf(value)) {
          value = 0.0f;  // 替换为零
        }
        
        vmp_estimator_output_(i) = static_cast<double>(value);
      }
      
      return vmp_estimator_output_;
      
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "[%s] Estimator inference error: %s", name_.c_str(), e.what());
      return Eigen::VectorXd::Zero(vmp_estimator_output_dim_);
    } 
  }

} // namespace humanoid_controller
