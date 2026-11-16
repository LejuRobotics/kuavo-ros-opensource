/**
 * @brief SG100(黑漫)手 ROS 桥接实现
 *
 * 手势库加载、/sg100/ service、/sg100_hand_command 100Hz 发布线程。
 * 由真机 SG100HandROSNode 与仿真 DexHandMujocoRosNode 启动,不在 VR IK 进程里。
 * 服务控制独立执行手势；VR 控制的扳机来自 /sg100/trigger，断流时停止发布。
 */
#include "motion_capture_ik/SG100HandBridge.h"

#include <ros/package.h>

#include <kuavo_msgs/SG100Curve.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace HighlyDynamic {

namespace {

// srv 的 SG100Curve → 折线关键点列表
std::vector<CurvePoint> toCurve(const kuavo_msgs::SG100Curve& c) {
  std::vector<CurvePoint> out;
  out.reserve(c.points.size());
  for (const auto& p : c.points) {
    out.push_back(CurvePoint{p.trigger, p.value});
  }
  return out;
}

// 折线关键点列表 → srv 的 SG100Curve
kuavo_msgs::SG100Curve fromCurve(const std::vector<CurvePoint>& curve) {
  kuavo_msgs::SG100Curve out;
  for (const CurvePoint& p : curve) {
    kuavo_msgs::SG100Keypoint kp;
    kp.trigger = p.trigger;
    kp.value = p.value;
    out.points.push_back(kp);
  }
  return out;
}

}  // namespace

SG100HandBridge::SG100HandBridge(ros::NodeHandle& nh) : nh_(nh), gesture_nh_(nh) {
  gesture_nh_.setCallbackQueue(&callback_queue_);
}

SG100HandBridge::~SG100HandBridge() { stop(); }

void SG100HandBridge::setCommandHandler(CommandHandler handler) {
  std::lock_guard<std::mutex> lock(handler_mutex_);
  command_handler_ = std::move(handler);
}

void SG100HandBridge::setConfiguredHands(bool left, bool right) {
  left_enabled_ = left;
  right_enabled_ = right;
}

void SG100HandBridge::setInputResumeHandler(std::function<void()> handler) {
  std::lock_guard<std::mutex> lock(handler_mutex_);
  input_resume_handler_ = std::move(handler);
}

void SG100HandBridge::requestFilterReseed() {
  std::function<void()> handler;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handler = input_resume_handler_;
  }
  if (handler) handler();
}

void SG100HandBridge::onTrigger(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2 || !std::isfinite(msg->data[0]) ||
      !std::isfinite(msg->data[1])) {
    // The VR client sends an empty frame once on raw-input loss. Invalidate
    // immediately rather than waiting another 0.5 s on the bridge heartbeat.
    last_trigger_received_.store(0.0);
    return;
  }
  const bool resuming = !vrInputReady();
  left_trigger_.store(msg->data[0]);
  right_trigger_.store(msg->data[1]);
  if (resuming) requestFilterReseed();
  last_trigger_received_.store(
      std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}
bool SG100HandBridge::vrInputReady() const {
  const double received = last_trigger_received_.load();
  const double now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  return received > 0.0 &&
         now >= received &&
         now - received < 0.5;
}

void SG100HandBridge::stop() {
  stop_.store(true);
  if (spinner_) {
    spinner_->stop();
    spinner_.reset();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool SG100HandBridge::onSetEnabled(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  res.success = gate_.enable(req.data, ready_.load() && vrInputReady() && (left_enabled_ || right_enabled_));
  if (res.success && req.data) requestFilterReseed();
  res.message = res.success ? (req.data ? "SG100 control enabled" : "SG100 control disabled; last target held")
                            : "SG100 control requires configured hands and fresh input in VR mode";
  return true;
}

bool SG100HandBridge::onUseVrControl(std_srvs::SetBool::Request& req,
                                      std_srvs::SetBool::Response& res) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  std::lock_guard<std::mutex> gesture_lock(mutex_);
  if (req.data) {
    if (!ready_ || !vrInputReady()) {
      res.success = false;
      res.message = "VR control requires fresh valid trigger input";
      return true;
    }
    selectVrLocked();
    requestFilterReseed();
  } else {
    gate_.selectService();
  }
  res.success = true;
  res.message = req.data ? "VR trigger control selected" : "Service control selected";
  return true;
}

void SG100HandBridge::start() {
  { std::lock_guard<std::mutex> lock(gate_mutex_); gate_ = SG100BridgeGate{}; }
  last_trigger_received_.store(0.0);
  nh_.param("/sg100/service_progress", service_progress_, 0.0);
  if (!std::isfinite(service_progress_) || service_progress_ < 0.0 || service_progress_ > 1.0) {
    ROS_FATAL("[SG100HandBridge] service_progress must be in [0, 1]");
    return;
  }
  // 第一件事:确认关节限位可用 —— 限位不可用就不启动。
  // 没有正确的限位,clamp() 无法保护硬件(曾用 [0, 1.57] 兜底,而 thumb_j2 真实限位是
  // [-2.62, 0],方向相反)。
  // 优先从整机 URDF 读取手指限位；URDF 中没有 SG100 手指关节时，
  // 按左右手分别回退到 hardware_node 的 joint_angle_map.json。
  int robot_version = 0;
  bool left_requested = true;
  bool right_requested = true;
  nh_.param("/sg100/left_enabled", left_requested, left_requested);
  nh_.param("/sg100/right_enabled", right_requested, right_requested);
  left_enabled_ = left_enabled_ && left_requested;
  right_enabled_ = right_enabled_ && right_requested;
  std::string robot_urdf;
  if (nh_.getParam("/robot_version", robot_version)) {
    robot_urdf =
        ros::package::getPath("kuavo_assets") +
        "/models/biped_s" + std::to_string(robot_version) +
        "/urdf/biped_s" + std::to_string(robot_version) + ".urdf";
  } else {
    ROS_WARN("[SG100HandBridge] /robot_version unavailable; "
             "using joint_angle_map.json for SG100 limits");
  }

  const std::string sg100_config_dir =
      ros::package::getPath("hardware_node") + "/config/sg100";
  const auto loadLimits = [&](bool left) {
    SG100JointLimits& limits = left ? left_limits_ : right_limits_;
    const char* side_name = left ? "left" : "right";
    const std::string prefix = left ? "l_" : "r_";
    if (!robot_urdf.empty() && limits.load(robot_urdf, prefix)) {
      ROS_INFO("[SG100HandBridge] %s SG100 limits loaded from URDF: %s",
               side_name, robot_urdf.c_str());
      return true;
    }

    const std::string json_path =
        sg100_config_dir + "/" + side_name + "/joint_angle_map.json";
    if (limits.loadJson(json_path)) {
      ROS_WARN("[SG100HandBridge] %s SG100 limits loaded from JSON fallback: %s",
               side_name, json_path.c_str());
      return true;
    }
    return false;
  };

  const bool ok_left_limits = !left_enabled_ || loadLimits(true);
  const bool ok_right_limits = !right_enabled_ || loadLimits(false);
  if (!ok_left_limits || !ok_right_limits) {
    ROS_FATAL("[SG100HandBridge] SG100 joint limits unavailable "
              "(left=%d right=%d); refusing to start",
              ok_left_limits ? 1 : 0, ok_right_limits ? 1 : 0);
    return;
  }

  // 限位就绪后再加载手势库(避免为一次注定失败的启动做无用功)
  const std::string gesture_dir =
      ros::package::getPath("motion_capture_ik") + "/config";
  const std::string left_path = gesture_dir + "/sg100_left_gesture.yaml";
  const std::string right_path = gesture_dir + "/sg100_right_gesture.yaml";

  const bool ok_left = left_lib_.load(left_path);
  const bool ok_right = right_lib_.load(right_path);
  left_yaml_path_ = left_path;
  right_yaml_path_ = right_path;


  // service / trigger 用独立队列,不跟 nodelet_manager 抢公共 spinner。
  srv_enable_ = gesture_nh_.advertiseService(
      "/sg100/set_enabled", &SG100HandBridge::onSetEnabled, this);
  srv_use_vr_ = gesture_nh_.advertiseService(
      "/sg100/use_vr_control", &SG100HandBridge::onUseVrControl, this);
  srv_get_ = gesture_nh_.advertiseService(
      "/sg100/get_gestures", &SG100HandBridge::onGetGestures, this);
  srv_add_ = gesture_nh_.advertiseService(
      "/sg100/add_gestures", &SG100HandBridge::onAddGestures, this);
  srv_delete_ = gesture_nh_.advertiseService(
      "/sg100/delete_gesture", &SG100HandBridge::onDeleteGesture, this);
  srv_update_ = gesture_nh_.advertiseService(
      "/sg100/update_gesture", &SG100HandBridge::onUpdateGesture, this);
  srv_switch_ = gesture_nh_.advertiseService(
      "/sg100/switch_gesture", &SG100HandBridge::onSwitchGesture, this);
  srv_step_ = gesture_nh_.advertiseService(
      "/sg100/step_gesture", &SG100HandBridge::onStepGesture, this);

  srv_vr_step_ = gesture_nh_.advertiseService(
      "/sg100/vr_step_gesture", &SG100HandBridge::onVrStepGesture, this);

  cmd_pub_ =
      nh_.advertise<kuavo_msgs::SG100HandCommand>("/sg100_hand_command", 10);
  trigger_sub_ = gesture_nh_.subscribe(
      kSg100TriggerTopic, 10, &SG100HandBridge::onTrigger, this);

  spinner_ = std::make_unique<ros::AsyncSpinner>(1, &callback_queue_);
  spinner_->start();
  ROS_INFO("[SG100HandBridge] dedicated AsyncSpinner started for /sg100/* and %s",
           kSg100TriggerTopic);

  ready_ = ok_left && ok_right;
  if (ready_) {
    ROS_INFO("[SG100HandBridge] SG100 gesture libraries loaded "
             "(left=%zu, right=%zu gestures); publishing /sg100_hand_command",
             left_lib_.modeCount(), right_lib_.modeCount());
  } else {
    ROS_WARN("[SG100HandBridge] SG100 gesture libraries FAILED to "
             "load (left=%d, right=%d); /sg100_hand_command disabled",
             ok_left ? 1 : 0, ok_right ? 1 : 0);
  }
  if (ready_) {
    stop_.store(false);
    thread_ = std::thread(&SG100HandBridge::publishLoop, this);
    ROS_INFO("[SG100HandBridge] publish thread started (100 Hz /sg100_hand_command)");
  }
}

void SG100HandBridge::publishLoop() {
  // SG100 command contract: 100 Hz, independent of other finger publishers.
  ros::WallRate rate(100.0);

  while (!stop_.load() && ros::ok()) {
    if (!ready_ || (!left_enabled_ && !right_enabled_)) {
      rate.sleep();
      continue;
    }

    std::unique_lock<std::mutex> gate_lock(gate_mutex_);
    const bool allowed = gate_.allow(vrInputReady());
    const bool service_control = gate_.serviceControl();
    if (!allowed) {
      gate_lock.unlock();
      rate.sleep();
      continue;
    }

    const float left_trigger =
        std::max(0.0f, std::min(1.0f, left_trigger_.load()));
    const float right_trigger =
        std::max(0.0f, std::min(1.0f, right_trigger_.load()));

    const int key_mode = key_mode_.load();
    bool command_left = left_enabled_, command_right = right_enabled_;

    HighlyDynamic::SG100HandPose left_pose;
    HighlyDynamic::SG100HandPose right_pose;
    bool ok_left = false;
    bool ok_right = false;
    {
      // 只在读手势库时持锁。sleep / publish / 直送驱动绝不能握着,
      // 否则 /sg100/step_gesture 会等整拍 sleep(仿真卡顿时可达十几秒)。
      std::lock_guard<std::mutex> gesture_lock(mutex_);
      if (service_control) {
        command_left &= service_left_active_;
        command_right &= service_right_active_;
      }

      ok_left = !command_left || left_lib_.buildTarget(
          service_control ? service_left_mode_ : key_mode,
          service_control ? service_progress_ : left_trigger, left_pose);
      ok_right = !command_right || right_lib_.buildTarget(
          service_control ? service_right_mode_ : key_mode,
          service_control ? service_progress_ : right_trigger, right_pose);
      if (ok_left && ok_right &&
          (!command_left || left_pose.positions.size() ==
              static_cast<std::size_t>(HighlyDynamic::SG100_HAND_CMD_DOF)) &&
          (!command_right || right_pose.positions.size() ==
              static_cast<std::size_t>(HighlyDynamic::SG100_HAND_CMD_DOF))) {
        if (command_left) left_limits_.clamp(left_pose.positions);
        if (command_right) right_limits_.clamp(right_pose.positions);
      } else {
        ok_left = false;
        ok_right = false;
      }
    }
    if (!ok_left || !ok_right || (!command_left && !command_right)) {
      gate_lock.unlock();
      rate.sleep();
      continue;
    }

    kuavo_msgs::SG100HandCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.left_hand_positions = left_pose.positions;
    cmd.right_hand_positions = right_pose.positions;
    cmd.left_enable_mask = command_left ? 0x07FF : 0;
    cmd.right_enable_mask = command_right ? 0x07FF : 0;
    cmd.source_id = kVrSourceId;
    cmd.skip_filter = false;
    cmd.control_mode = kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;

    // 力控/阻抗 per-joint 字段仅在 hybrid 手势时填充；否则留空走全局位置控制。
    if (left_pose.has_force_control || right_pose.has_force_control) {
      const std::size_t dof =
          static_cast<std::size_t>(HighlyDynamic::SG100_HAND_CMD_DOF);
      cmd.left_hand_control_mode = left_pose.modes;
      cmd.right_hand_control_mode = right_pose.modes;
      cmd.left_hand_kp = left_pose.kp;
      cmd.right_hand_kp = right_pose.kp;
      cmd.left_hand_kd = left_pose.kd;
      cmd.right_hand_kd = right_pose.kd;
      cmd.left_hand_torque_ff = left_pose.torque_ff;
      cmd.right_hand_torque_ff = right_pose.torque_ff;
      cmd.left_hand_output_limit = left_pose.output_limit;
      cmd.right_hand_output_limit = right_pose.output_limit;
      // 数组维度不符时清空，交给全局 control_mode 兜底。
      if (cmd.left_hand_control_mode.size() != dof) {
        cmd.left_hand_control_mode.clear();
      }
      if (cmd.right_hand_control_mode.size() != dof) {
        cmd.right_hand_control_mode.clear();
      }
    }

    // 同进程直送驱动/仿真,避免 nodelet 公共队列把订阅回调拖到十几秒。
    // 话题仍发布,给外部监控用。
    CommandHandler handler;
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      handler = command_handler_;
    }
    {
      if (gate_.allow(vrInputReady())) {
        if (handler) {
          handler(cmd);
        }
        cmd_pub_.publish(cmd);
      }
    }
    gate_lock.unlock();
    rate.sleep();
  }
}

bool SG100HandBridge::applyHandSide(int hand_side, bool& do_left,
                                                bool& do_right) {
  switch (hand_side) {
    case 0:
      do_left = true;
      do_right = false;
      return true;
    case 1:
      do_left = false;
      do_right = true;
      return true;
    case 2:
      do_left = true;
      do_right = true;
      return true;
    default:
      do_left = false;
      do_right = false;
      return false;
  }
}

void SG100HandBridge::clampKeyModeLocked(std::size_t mode_count) {
  const auto clamp = [](int mode, std::size_t count) {
    return count == 0 ? 0 : std::max(0, std::min(mode, static_cast<int>(count) - 1));
  };
  key_mode_.store(clamp(key_mode_.load(), mode_count));
  service_left_mode_ = clamp(service_left_mode_, left_lib_.modeCount());
  service_right_mode_ = clamp(service_right_mode_, right_lib_.modeCount());
}

bool SG100HandBridge::onGetGestures(
    kuavo_msgs::SG100GetGestures::Request& req,
    kuavo_msgs::SG100GetGestures::Response& res) {
  std::lock_guard<std::mutex> gate_lock(gate_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  const SG100GestureLibrary* lib = nullptr;
  switch (req.hand_side) {
    case kuavo_msgs::SG100GetGestures::Request::LEFT_HAND:
      lib = &left_lib_;
      break;
    case kuavo_msgs::SG100GetGestures::Request::RIGHT_HAND:
      lib = &right_lib_;
      break;
    default:
      res.success = false;
      res.message = "invalid hand_side";
      return true;
  }
  res.total_count = static_cast<int32_t>(lib->modeCount());

  if (req.id == kuavo_msgs::SG100GetGestures::Request::QUERY_ALL) {
    // 仅返回总数
    res.success = true;
    res.message = "ok";
    return true;
  }

  int query_id = req.id;
  if (query_id == kuavo_msgs::SG100GetGestures::Request::QUERY_CURRENT) {
    query_id = gate_.serviceControl()
        ? (req.hand_side == kuavo_msgs::SG100GetGestures::Request::LEFT_HAND
               ? service_left_mode_ : service_right_mode_)
        : key_mode_.load();  // 当前激活手势
  }
  if (query_id < 0 || query_id >= static_cast<int32_t>(lib->modeCount())) {
    res.success = false;
    res.message = "id out of range";
    return true;
  }

  HighlyDynamic::CurveGesture g;
  lib->getGesture(query_id, g);
  res.name = g.name;
  const int n = static_cast<int>(g.joints.size());
  res.position.resize(n);
  res.torque_ff.resize(n);
  res.kp.resize(n);
  res.kd.resize(n);
  res.output_limit.resize(n);
  res.mode.resize(n);
  for (int i = 0; i < n; ++i) {
    res.position[i] = fromCurve(g.joints[i].position);
    res.torque_ff[i] = fromCurve(g.joints[i].torque_ff);
    res.kp[i] = fromCurve(g.joints[i].kp);
    res.kd[i] = fromCurve(g.joints[i].kd);
    res.output_limit[i] = fromCurve(g.joints[i].output_limit);
    res.mode[i] = fromCurve(g.joints[i].mode);
  }
  res.success = true;
  res.message = "ok";
  return true;
}

bool SG100HandBridge::onAddGestures(
    kuavo_msgs::SG100AddGestures::Request& req,
    kuavo_msgs::SG100AddGestures::Response& res) {
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  const int n = static_cast<int>(req.position.size());
  if (n == 0) {
    res.success = false;
    res.message = "position 折线为空";
    return true;
  }
  HighlyDynamic::CurveGesture g;
  g.name = req.name;
  g.joints.resize(n);
  for (int i = 0; i < n; ++i) {
    g.joints[i].position = toCurve(req.position[i]);
    g.joints[i].torque_ff =
        (i < static_cast<int>(req.torque_ff.size()))
            ? toCurve(req.torque_ff[i])
            : std::vector<CurvePoint>{};
    g.joints[i].kp = (i < static_cast<int>(req.kp.size()))
                         ? toCurve(req.kp[i])
                         : std::vector<CurvePoint>{};
    g.joints[i].kd = (i < static_cast<int>(req.kd.size()))
                         ? toCurve(req.kd[i])
                         : std::vector<CurvePoint>{};
    g.joints[i].output_limit =
        (i < static_cast<int>(req.output_limit.size()))
            ? toCurve(req.output_limit[i])
            : std::vector<CurvePoint>{};
    g.joints[i].mode = (i < static_cast<int>(req.mode.size()))
                           ? toCurve(req.mode[i])
                           : std::vector<CurvePoint>{};
  }
  for (const auto& jc : g.joints) {
    if (!HighlyDynamic::validateCurve(jc.position)) {
      res.success = false;
      res.message = "position 折线未全覆盖 0-100";
      return true;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (do_left && !left_lib_.addGesture(g.name, g.joints)) {
      res.success = false;
      res.message = "add left failed (name dup or bad keyframes): " + g.name;
      return true;
    }
    if (do_right && !right_lib_.addGesture(g.name, g.joints)) {
      res.success = false;
      res.message = "add right failed (name dup or bad keyframes): " + g.name;
      return true;
    }
    if (do_left && !left_lib_.save(left_yaml_path_)) {
      res.success = false;
      res.message = "save left yaml failed";
      return true;
    }
    if (do_right && !right_lib_.save(right_yaml_path_)) {
      res.success = false;
      res.message = "save right yaml failed";
      return true;
    }
    res.total_count = static_cast<int32_t>(left_lib_.modeCount());
    clampKeyModeLocked(left_lib_.modeCount());
  }
  res.success = true;
  res.message = "ok";
  return true;
}

bool SG100HandBridge::onDeleteGesture(
    kuavo_msgs::SG100DeleteGesture::Request& req,
    kuavo_msgs::SG100DeleteGesture::Response& res) {
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (do_left && !left_lib_.deleteGesture(req.id)) {
      res.success = false;
      res.message = "delete left failed (id out of range)";
      return true;
    }
    if (do_right && !right_lib_.deleteGesture(req.id)) {
      res.success = false;
      res.message = "delete right failed (id out of range)";
      return true;
    }
    if (do_left && !left_lib_.save(left_yaml_path_)) {
      res.success = false;
      res.message = "save left yaml failed";
      return true;
    }
    if (do_right && !right_lib_.save(right_yaml_path_)) {
      res.success = false;
      res.message = "save right yaml failed";
      return true;
    }
    res.total_count = static_cast<int32_t>(left_lib_.modeCount());
    clampKeyModeLocked(left_lib_.modeCount());
  }
  res.success = true;
  res.message = "ok";
  return true;
}

bool SG100HandBridge::onUpdateGesture(
    kuavo_msgs::SG100UpdateGesture::Request& req,
    kuavo_msgs::SG100UpdateGesture::Response& res) {
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  const int n = static_cast<int>(req.position.size());
  if (n == 0) {
    res.success = false;
    res.message = "position 折线为空";
    return true;
  }
  HighlyDynamic::CurveGesture g;
  g.name = req.name;
  g.joints.resize(n);
  for (int i = 0; i < n; ++i) {
    g.joints[i].position = toCurve(req.position[i]);
    g.joints[i].torque_ff =
        (i < static_cast<int>(req.torque_ff.size()))
            ? toCurve(req.torque_ff[i])
            : std::vector<CurvePoint>{};
    g.joints[i].kp = (i < static_cast<int>(req.kp.size()))
                         ? toCurve(req.kp[i])
                         : std::vector<CurvePoint>{};
    g.joints[i].kd = (i < static_cast<int>(req.kd.size()))
                         ? toCurve(req.kd[i])
                         : std::vector<CurvePoint>{};
    g.joints[i].output_limit =
        (i < static_cast<int>(req.output_limit.size()))
            ? toCurve(req.output_limit[i])
            : std::vector<CurvePoint>{};
    g.joints[i].mode = (i < static_cast<int>(req.mode.size()))
                           ? toCurve(req.mode[i])
                           : std::vector<CurvePoint>{};
  }
  for (const auto& jc : g.joints) {
    if (!HighlyDynamic::validateCurve(jc.position)) {
      res.success = false;
      res.message = "position 折线未全覆盖 0-100";
      return true;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (do_left && !left_lib_.updateGesture(req.id, g.name, g.joints)) {
      res.success = false;
      res.message = "update left failed (id out of range or bad keyframes)";
      return true;
    }
    if (do_right && !right_lib_.updateGesture(req.id, g.name, g.joints)) {
      res.success = false;
      res.message = "update right failed (id out of range or bad keyframes)";
      return true;
    }
    if (do_left && !left_lib_.save(left_yaml_path_)) {
      res.success = false;
      res.message = "save left yaml failed";
      return true;
    }
    if (do_right && !right_lib_.save(right_yaml_path_)) {
      res.success = false;
      res.message = "save right yaml failed";
      return true;
    }
  }
  res.success = true;
  res.message = "ok";
  return true;
}

void SG100HandBridge::selectVrLocked() {
  service_left_active_ = false;
  service_right_active_ = false;
  // A later service step starts from the current VR gesture, not an old target.
  service_left_mode_ = key_mode_.load();
  service_right_mode_ = key_mode_.load();
  gate_.selectVr();
}

void SG100HandBridge::logGestureLocked(bool left, int id, bool vr_control) const {
  CurveGesture gesture;
  const auto& library = left ? left_lib_ : right_lib_;
  if (library.getGesture(id, gesture)) {
    ROS_INFO("[SG100HandBridge] %s gesture switched: hand=%s id=%d name=%s",
             vr_control ? "VR" : "service", left ? "left" : "right",
             id, gesture.name.c_str());
  }
}

bool SG100HandBridge::onSwitchGesture(
    kuavo_msgs::SG100SwitchGesture::Request& req,
    kuavo_msgs::SG100SwitchGesture::Response& res) {
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  {
    std::lock_guard<std::mutex> gate_lock(gate_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || (do_left && !left_enabled_) || (do_right && !right_enabled_)) {
      res.success = false;
      res.message = "Requested hand is unavailable";
      return true;
    }
    const std::size_t mode_count = do_left ? left_lib_.modeCount() : right_lib_.modeCount();
    if (req.id < 0 || req.id >= static_cast<int32_t>(mode_count) ||
        (do_right && req.id >= static_cast<int32_t>(right_lib_.modeCount()))) {
      res.success = false;
      res.message = "id out of range";
      return true;
    }
    if (do_left) { service_left_mode_ = req.id; service_left_active_ = true; }
    if (do_right) { service_right_mode_ = req.id; service_right_active_ = true; }
    key_mode_.store(req.id);
    gate_.selectService();
    requestFilterReseed();
    if (do_left) logGestureLocked(true, service_left_mode_, false);
    if (do_right) logGestureLocked(false, service_right_mode_, false);
  }
  res.success = true;
  res.message = "ok";
  return true;
}

bool SG100HandBridge::onStepGesture(
    kuavo_msgs::SG100StepGesture::Request& req,
    kuavo_msgs::SG100StepGesture::Response& res) {
  return stepGesture(req, res, false);
}

bool SG100HandBridge::onVrStepGesture(
    kuavo_msgs::SG100StepGesture::Request& req,
    kuavo_msgs::SG100StepGesture::Response& res) {
  return stepGesture(req, res, true);
}

bool SG100HandBridge::stepGesture(
    kuavo_msgs::SG100StepGesture::Request& req,
    kuavo_msgs::SG100StepGesture::Response& res, bool vr_control) {
  if (vr_control && req.hand_side != 2) {
    res.success = false;
    res.message = "VR gesture switch requires both hands";
    return true;
  }
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  {
    std::lock_guard<std::mutex> gate_lock(gate_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || (do_left && !left_enabled_) || (do_right && !right_enabled_)) {
      res.success = false;
      res.message = "Requested hand is unavailable";
      return true;
    }
    if (vr_control && !vrInputReady()) {
      res.success = false;
      res.message = "VR gesture switch requires fresh trigger input";
      return true;
    }
    const std::size_t mode_count = do_left ? left_lib_.modeCount() : right_lib_.modeCount();
    if (mode_count == 0) {
      res.success = false;
      res.message = "no gestures";
      return true;
    }
    const int dir = (req.direction >= 0) ? 1 : -1;
    // 负数安全取模：C++ 里 (-1) % n == -1，需 +n 再取模
    const auto step = [dir](int mode, std::size_t count) {
      const int size = static_cast<int>(count);
      return ((mode + dir) % size + size) % size;
    };
    if (do_right && right_lib_.modeCount() == 0) {
      res.success = false;
      res.message = "no right gestures";
      return true;
    }
    if (vr_control) {
      const auto count = std::min(left_lib_.modeCount(), right_lib_.modeCount());
      if (count == 0) {
        res.success = false;
        res.message = "no VR gestures";
        return true;
      }
      key_mode_.store(step(key_mode_.load(), count));
      res.current_id = key_mode_.load();
      selectVrLocked();
    } else {
      if (do_left) { service_left_mode_ = step(service_left_mode_, left_lib_.modeCount()); service_left_active_ = true; }
      if (do_right) { service_right_mode_ = step(service_right_mode_, right_lib_.modeCount()); service_right_active_ = true; }
      res.current_id = do_left ? service_left_mode_ : service_right_mode_;
      key_mode_.store(res.current_id);
      gate_.selectService();
    }
    requestFilterReseed();
    if (do_left) logGestureLocked(true, vr_control ? key_mode_.load() : service_left_mode_, vr_control);
    if (do_right) logGestureLocked(false, vr_control ? key_mode_.load() : service_right_mode_, vr_control);
  }
  res.success = true;
  res.message = "ok";
  return true;
}

}  // namespace HighlyDynamic
