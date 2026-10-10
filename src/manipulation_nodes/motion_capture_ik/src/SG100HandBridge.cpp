/**
 * @brief SG100(黑漫)手 ROS 桥接实现
 *
 * 手势库加载、/sg100/* service、/sg100_hand_command 30Hz 发布线程。
 * 由真机 SG100HandROSNode 与仿真 DexHandMujocoRosNode 启动,不在 VR IK 进程里。
 * 扳机来自 /sg100/trigger;没有该话题时按 0 解算。
 */
#include "motion_capture_ik/SG100HandBridge.h"

#include <ros/package.h>

#include <kuavo_msgs/SG100Curve.h>

#include <algorithm>
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

void SG100HandBridge::onTrigger(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) {
    return;
  }
  left_trigger_.store(msg->data[0]);
  right_trigger_.store(msg->data[1]);
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

void SG100HandBridge::start() {
  // 第一件事:确认关节限位可用 —— 限位不可用就不启动。
  // 没有正确的限位,clamp() 无法保护硬件(曾用 [0, 1.57] 兜底,而 thumb_j2 真实限位是
  // [-2.62, 0],方向相反)。
  // 手部片段 heiman_sg100/urdf/*.urdf 无 <robot> 根元素,urdf 库无法解析;
  // 完整 biped_s{version}.urdf 含 revolute 手关节 + <limit>,左右手共用。
  int robot_version = 0;
  if (!nh_.getParam("/robot_version", robot_version)) {
    ROS_ERROR("[SG100HandBridge] failed to read /robot_version, "
              "aborting SG100 start");
    return;
  }
  const std::string robot_urdf =
      ros::package::getPath("kuavo_assets") +
      "/models/biped_s" + std::to_string(robot_version) +
      "/urdf/biped_s" + std::to_string(robot_version) + ".urdf";
  const bool ok_left_limits = left_limits_.load(robot_urdf, "l_");
  const bool ok_right_limits = right_limits_.load(robot_urdf, "r_");
  if (!ok_left_limits || !ok_right_limits) {
    ROS_FATAL("[SG100HandBridge] SG100 joint limits unavailable "
              "(left=%d right=%d) from %s; refusing to start",
              ok_left_limits ? 1 : 0, ok_right_limits ? 1 : 0,
              robot_urdf.c_str());
    return;
  }
  ROS_INFO("[SG100HandBridge] SG100 joint limits loaded from %s",
           robot_urdf.c_str());

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
    ROS_INFO("[SG100HandBridge] publish thread started (30 Hz /sg100_hand_command)");
  }
}

void SG100HandBridge::publishLoop() {
  // 用 WallRate:手势发布不跟 /use_sim_time 绑死。仿真时钟一顿,
  // ros::Rate::sleep 会把 mutex 和下一帧命令一起拖到十几秒。
  ros::WallRate rate(30.0);

  while (!stop_.load() && ros::ok()) {
    if (!ready_) {
      rate.sleep();
      continue;
    }

    const float left_trigger =
        std::max(0.0f, std::min(1.0f, left_trigger_.load()));
    const float right_trigger =
        std::max(0.0f, std::min(1.0f, right_trigger_.load()));

    const int key_mode = key_mode_.load();

    HighlyDynamic::SG100HandPose left_pose;
    HighlyDynamic::SG100HandPose right_pose;
    bool ok_left = false;
    bool ok_right = false;
    {
      // 只在读手势库时持锁。sleep / publish / 直送驱动绝不能握着,
      // 否则 /sg100/step_gesture 会等整拍 sleep(仿真卡顿时可达十几秒)。
      std::lock_guard<std::mutex> gesture_lock(mutex_);
      ok_left = left_lib_.buildTarget(key_mode, left_trigger, left_pose);
      ok_right = right_lib_.buildTarget(key_mode, right_trigger, right_pose);
      if (ok_left && ok_right &&
          left_pose.positions.size() ==
              static_cast<std::size_t>(HighlyDynamic::SG100_HAND_CMD_DOF) &&
          right_pose.positions.size() ==
              static_cast<std::size_t>(HighlyDynamic::SG100_HAND_CMD_DOF)) {
        left_limits_.clamp(left_pose.positions);
        right_limits_.clamp(right_pose.positions);
      } else {
        ok_left = false;
        ok_right = false;
      }
    }
    if (!ok_left || !ok_right) {
      rate.sleep();
      continue;
    }

    kuavo_msgs::SG100HandCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.left_hand_positions = left_pose.positions;
    cmd.right_hand_positions = right_pose.positions;
    cmd.left_enable_mask = 0x07FF;
    cmd.right_enable_mask = 0x07FF;
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
    if (handler) {
      handler(cmd);
    }
    cmd_pub_.publish(cmd);
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
  if (mode_count == 0) {
    key_mode_.store(0);
    return;
  }
  const int current = key_mode_.load();
  const int max_id = static_cast<int>(mode_count) - 1;
  if (current < 0) {
    key_mode_.store(0);
  } else if (current > max_id) {
    key_mode_.store(max_id);
  }
}

bool SG100HandBridge::onGetGestures(
    kuavo_msgs::SG100GetGestures::Request& req,
    kuavo_msgs::SG100GetGestures::Response& res) {
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
    query_id = key_mode_.load();  // 当前激活手势
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
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t mode_count = left_lib_.modeCount();
    if (req.id < 0 || req.id >= static_cast<int32_t>(mode_count)) {
      res.success = false;
      res.message = "id out of range";
      return true;
    }
    key_mode_.store(req.id);
  }
  res.success = true;
  res.message = "ok";
  return true;
}

bool SG100HandBridge::onStepGesture(
    kuavo_msgs::SG100StepGesture::Request& req,
    kuavo_msgs::SG100StepGesture::Response& res) {
  bool do_left = false, do_right = false;
  if (!applyHandSide(req.hand_side, do_left, do_right)) {
    res.success = false;
    res.message = "invalid hand_side";
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t mode_count = left_lib_.modeCount();
    if (mode_count == 0) {
      res.success = false;
      res.message = "no gestures";
      return true;
    }
    const int dir = (req.direction >= 0) ? 1 : -1;
    const int n = static_cast<int>(mode_count);
    // 负数安全取模：C++ 里 (-1) % n == -1，需 +n 再取模
    const int next = ((key_mode_.load() + dir) % n + n) % n;
    key_mode_.store(next);
    res.current_id = next;
  }
  res.success = true;
  res.message = "ok";
  return true;
}

}  // namespace HighlyDynamic
