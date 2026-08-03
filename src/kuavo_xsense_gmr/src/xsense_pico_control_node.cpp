#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotBodyMatrices.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/robotHeadMotionData.h>

namespace
{

using Matrix4 = std::array<double, 16>;

double clamp(const double value, const double low, const double high)
{
  return std::max(low, std::min(high, value));
}

double radToDeg(const double radians)
{
  return radians * 180.0 / M_PI;
}

uint8_t clampToUint8(const double value, const double low, const double high)
{
  const double clamped = clamp(value, low, high);
  return static_cast<uint8_t>(std::round(clamped));
}

bool isDexhandEndEffector(const std::string& end_effector_type)
{
  return end_effector_type == "qiangnao" ||
         end_effector_type == "linker_o6" ||
         end_effector_type == "linker_l6";
}

bool xmlRpcValueToDouble(XmlRpc::XmlRpcValue& value, double& out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(value);
    return true;
  }
  return false;
}

std::vector<double> readLimitParam(ros::NodeHandle& private_nh,
                                   const std::string& name,
                                   const double default_min,
                                   const double default_max)
{
  std::vector<double> limits{default_min, default_max};
  XmlRpc::XmlRpcValue value;
  if (!private_nh.getParam(name, value))
    return limits;

  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 2)
  {
    ROS_WARN_STREAM("Parameter ~" << name << " should be a two-item list, using default ["
                                  << default_min << ", " << default_max << "]");
    return limits;
  }

  double low = limits[0];
  double high = limits[1];
  if (!xmlRpcValueToDouble(value[0], low) || !xmlRpcValueToDouble(value[1], high))
  {
    ROS_WARN_STREAM("Parameter ~" << name << " list items should be numbers, using default ["
                                                  << default_min << ", " << default_max << "]");
    return limits;
  }
  limits[0] = low;
  limits[1] = high;

  if (limits[0] > limits[1])
    std::swap(limits[0], limits[1]);
  return limits;
}

Matrix4 identityMatrix()
{
  return Matrix4{1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiplyMatrix4(const Matrix4& lhs, const Matrix4& rhs)
{
  Matrix4 out{};
  for (int row = 0; row < 4; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      double value = 0.0;
      for (int k = 0; k < 4; ++k)
        value += lhs[row * 4 + k] * rhs[k * 4 + col];
      out[row * 4 + col] = value;
    }
  }
  return out;
}

Matrix4 inverseRigidTransform(const Matrix4& mat)
{
  Matrix4 out = identityMatrix();

  // /robot_body_matrices 使用行主序 4x4 齐次矩阵；刚体逆变换为 R^T 和 -R^T * t。
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
      out[row * 4 + col] = mat[col * 4 + row];
  }

  const double tx = mat[3];
  const double ty = mat[7];
  const double tz = mat[11];
  out[3] = -(out[0] * tx + out[1] * ty + out[2] * tz);
  out[7] = -(out[4] * tx + out[5] * ty + out[6] * tz);
  out[11] = -(out[8] * tx + out[9] * ty + out[10] * tz);
  return out;
}

bool getMatrixByName(const kuavo_msgs::robotBodyMatrices& msg,
                     const std::string& name,
                     Matrix4& matrix)
{
  for (std::size_t idx = 0; idx < msg.body_parts.size(); ++idx)
  {
    if (msg.body_parts[idx] != name)
      continue;

    const std::size_t start = idx * 16;
    if (start + 16 > msg.matrices_data.size())
      return false;

    std::copy(msg.matrices_data.begin() + static_cast<std::ptrdiff_t>(start),
              msg.matrices_data.begin() + static_cast<std::ptrdiff_t>(start + 16),
              matrix.begin());
    return true;
  }
  return false;
}

bool extractYawPitch(const Matrix4& rel, double& yaw_rad, double& pitch_rad)
{
  if (!std::isfinite(rel[0]) || !std::isfinite(rel[1]) || !std::isfinite(rel[2]) ||
      !std::isfinite(rel[4]) || !std::isfinite(rel[5]) || !std::isfinite(rel[6]) ||
      !std::isfinite(rel[8]) || !std::isfinite(rel[9]) || !std::isfinite(rel[10]))
  {
    return false;
  }

  tf::Matrix3x3 rotation(rel[0], rel[1], rel[2],
                         rel[4], rel[5], rel[6],
                         rel[8], rel[9], rel[10]);
  double roll = 0.0;
  rotation.getRPY(roll, pitch_rad, yaw_rad);
  return std::isfinite(yaw_rad) && std::isfinite(pitch_rad);
}

class PicoEndEffectorController
{
public:
  PicoEndEffectorController(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("joy_topic", joy_topic_, "/pico/joy");
    private_nh.param<std::string>("claw_topic", claw_topic_, "/leju_claw_command");
    private_nh.param<std::string>("dexhand_topic", dexhand_topic_, "/control_robot_hand_position");
    private_nh.param<double>("claw_trigger_threshold", trigger_threshold_, 0.5);
    private_nh.param<double>("claw_open_position", open_position_, 10.0);
    private_nh.param<double>("claw_close_position", close_position_, 90.0);
    private_nh.param<double>("claw_velocity", velocity_, 90.0);
    private_nh.param<double>("claw_effort", effort_, 1.0);
    private_nh.param<double>("claw_publish_rate_hz", publish_rate_hz_, 10.0);
    private_nh.param<double>("dexhand_command_min", dexhand_command_min_, 0.0);
    private_nh.param<double>("dexhand_command_max", dexhand_command_max_, 100.0);
    private_nh.param<double>("dexhand_grip_deadzone", dexhand_grip_deadzone_, 0.02);
    private_nh.param<double>("dexhand_smoothing_alpha", dexhand_smoothing_alpha_, 0.35);

    if (!ros::param::get("/end_effector_type", end_effector_type_))
    {
      end_effector_type_ = "lejuclaw";
      ROS_WARN("Parameter /end_effector_type is not set, assume lejuclaw for Pico end-effector control");
    }

    trigger_threshold_ = clamp(trigger_threshold_, 0.0, 1.0);
    publish_rate_hz_ = std::max(0.1, publish_rate_hz_);
    dexhand_grip_deadzone_ = clamp(dexhand_grip_deadzone_, 0.0, 1.0);
    dexhand_smoothing_alpha_ = clamp(dexhand_smoothing_alpha_, 0.0, 1.0);
    if (dexhand_command_min_ > dexhand_command_max_)
      std::swap(dexhand_command_min_, dexhand_command_max_);

    if (end_effector_type_ == "lejuclaw")
    {
      claw_pub_ = nh.advertise<kuavo_msgs::lejuClawCommand>(claw_topic_, 10);
      ROS_INFO_STREAM("Pico claw control enabled: " << joy_topic_ << " -> " << claw_topic_
                                                    << ", threshold=" << trigger_threshold_
                                                    << ", open=" << open_position_
                                                    << ", close=" << close_position_);
    }
    else if (isDexhandEndEffector(end_effector_type_))
    {
      hand_pub_ = nh.advertise<kuavo_msgs::robotHandPosition>(dexhand_topic_, 10);
      ROS_INFO_STREAM("Pico dexhand control enabled: " << joy_topic_ << " -> " << dexhand_topic_
                                                       << ", eef=" << end_effector_type_
                                                       << ", command_range=[" << dexhand_command_min_
                                                       << ", " << dexhand_command_max_
                                                       << "], grip_deadzone=" << dexhand_grip_deadzone_);
    }
    else
    {
      ROS_WARN_STREAM("Pico end-effector control idle because /end_effector_type='"
                      << end_effector_type_ << "' is neither lejuclaw nor a supported dexhand");
    }

    joy_sub_ = nh.subscribe(joy_topic_, 20, &PicoEndEffectorController::joyCallback, this);
    publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                                    &PicoEndEffectorController::publishTimerCallback,
                                    this);
  }

private:
  void joyCallback(const kuavo_msgs::JoySticks::ConstPtr& joy)
  {
    if (!joy)
      return;

    // 需求语义：扣下扳机夹爪闭合，松开扳机夹爪张开。
    left_closed_ = joy->left_trigger >= trigger_threshold_;
    right_closed_ = joy->right_trigger >= trigger_threshold_;
    left_grip_ = clamp(joy->left_grip, 0.0, 1.0);
    right_grip_ = clamp(joy->right_grip, 0.0, 1.0);
    have_joy_ = true;
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    if (!have_joy_)
      return;

    if (end_effector_type_ == "lejuclaw")
      publishClawCommand();
    else if (isDexhandEndEffector(end_effector_type_))
      publishDexhandCommand();
  }

  void publishClawCommand()
  {
    kuavo_msgs::lejuClawCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.data.name = {"left_claw", "right_claw"};
    msg.data.position = {left_closed_ ? close_position_ : open_position_,
                         right_closed_ ? close_position_ : open_position_};
    msg.data.velocity = {velocity_, velocity_};
    msg.data.effort = {effort_, effort_};
    claw_pub_.publish(msg);
  }

  void publishDexhandCommand()
  {
    const uint8_t left_cmd = gripToDexhandCommand(left_grip_, filtered_left_hand_, have_filtered_left_hand_);
    const uint8_t right_cmd = gripToDexhandCommand(right_grip_, filtered_right_hand_, have_filtered_right_hand_);

    kuavo_msgs::robotHandPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "pico_dexhand_control";
    // 当前阶段只做整手抓握：每只手 6 个手指给同一个开合值，满足仿真和实机旧接口。
    msg.left_hand_position.assign(6, left_cmd);
    msg.right_hand_position.assign(6, right_cmd);
    hand_pub_.publish(msg);
  }

  uint8_t gripToDexhandCommand(const double grip, double& filtered_value, bool& have_filtered_value)
  {
    double normalized = grip < dexhand_grip_deadzone_ ? 0.0 : grip;
    const double raw = dexhand_command_min_ + normalized * (dexhand_command_max_ - dexhand_command_min_);

    if (!have_filtered_value)
    {
      filtered_value = raw;
      have_filtered_value = true;
    }
    else
      filtered_value = (1.0 - dexhand_smoothing_alpha_) * filtered_value + dexhand_smoothing_alpha_ * raw;

    return clampToUint8(filtered_value, dexhand_command_min_, dexhand_command_max_);
  }

  ros::Subscriber joy_sub_;
  ros::Publisher claw_pub_;
  ros::Publisher hand_pub_;
  ros::Timer publish_timer_;

  std::string joy_topic_;
  std::string claw_topic_;
  std::string dexhand_topic_;
  std::string end_effector_type_;
  double trigger_threshold_ = 0.5;
  double open_position_ = 10.0;
  double close_position_ = 90.0;
  double velocity_ = 90.0;
  double effort_ = 1.0;
  double publish_rate_hz_ = 10.0;
  double dexhand_command_min_ = 0.0;
  double dexhand_command_max_ = 100.0;
  double dexhand_grip_deadzone_ = 0.02;
  double dexhand_smoothing_alpha_ = 0.35;
  bool have_joy_ = false;
  bool left_closed_ = false;
  bool right_closed_ = false;
  bool have_filtered_left_hand_ = false;
  bool have_filtered_right_hand_ = false;
  double left_grip_ = 0.0;
  double right_grip_ = 0.0;
  double filtered_left_hand_ = 0.0;
  double filtered_right_hand_ = 0.0;
};

class PicoHeadVrFollowController
{
public:
  PicoHeadVrFollowController(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : yaw_limit_(readLimitParam(private_nh, "head_yaw_limit", -30.0, 30.0)),
      pitch_limit_(readLimitParam(private_nh, "head_pitch_limit", -25.0, 25.0))
  {
    private_nh.param<std::string>("body_matrices_topic", body_matrices_topic_, "/robot_body_matrices");
    private_nh.param<std::string>("head_output_topic", head_output_topic_, "/robot_head_motion_data");
    private_nh.param<double>("head_publish_hz", publish_hz_, 50.0);
    private_nh.param<double>("head_stale_timeout_s", stale_timeout_s_, 0.5);
    private_nh.param<double>("head_smoothing_factor", smoothing_factor_, 0.2);
    private_nh.param<double>("head_yaw_sign", yaw_sign_, 1.0);
    private_nh.param<double>("head_pitch_sign", pitch_sign_, 1.0);

    publish_hz_ = std::max(1.0, publish_hz_);
    stale_timeout_s_ = std::max(0.0, stale_timeout_s_);
    smoothing_factor_ = clamp(smoothing_factor_, 0.0, 1.0);
    yaw_sign_ = yaw_sign_ >= 0.0 ? 1.0 : -1.0;
    pitch_sign_ = pitch_sign_ >= 0.0 ? 1.0 : -1.0;

    head_pub_ = nh.advertise<kuavo_msgs::robotHeadMotionData>(head_output_topic_, 10);
    body_matrices_sub_ = nh.subscribe(body_matrices_topic_,
                                      20,
                                      &PicoHeadVrFollowController::bodyMatricesCallback,
                                      this);
    publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_hz_),
                                    &PicoHeadVrFollowController::publishTimerCallback,
                                    this);

    ROS_INFO_STREAM("Pico vr_follow head control enabled: " << body_matrices_topic_
                                                            << " -> " << head_output_topic_
                                                            << ", publish_hz=" << publish_hz_
                                                            << ", yaw_limit=[" << yaw_limit_[0]
                                                            << ", " << yaw_limit_[1]
                                                            << "], pitch_limit=[" << pitch_limit_[0]
                                                            << ", " << pitch_limit_[1] << "]");
  }

private:
  void bodyMatricesCallback(const kuavo_msgs::robotBodyMatrices::ConstPtr& msg)
  {
    if (!msg)
      return;

    Matrix4 pelvis;
    Matrix4 head;
    if (!getMatrixByName(*msg, "Pelvis", pelvis))
    {
      ROS_WARN_THROTTLE(2.0, "Pico head control skips frame: missing or invalid Pelvis matrix");
      return;
    }
    if (!getMatrixByName(*msg, "HEAD", head))
    {
      ROS_WARN_THROTTLE(2.0, "Pico head control skips frame: missing or invalid HEAD matrix");
      return;
    }

    // vr_follow 只关心头显相对骨盆的姿态，避免人体整体转身导致机器人头部误跟随世界朝向。
    const Matrix4 rel = multiplyMatrix4(inverseRigidTransform(pelvis), head);

    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    if (!extractYawPitch(rel, yaw_rad, pitch_rad))
    {
      ROS_WARN_THROTTLE(2.0, "Pico head control skips frame: invalid relative HEAD matrix");
      return;
    }

    double target_yaw = yaw_sign_ * radToDeg(yaw_rad);
    double target_pitch = pitch_sign_ * radToDeg(pitch_rad);
    target_yaw = clamp(target_yaw, yaw_limit_[0], yaw_limit_[1]);
    target_pitch = clamp(target_pitch, pitch_limit_[0], pitch_limit_[1]);

    if (!have_valid_pose_)
    {
      current_yaw_ = target_yaw;
      current_pitch_ = target_pitch;
      have_valid_pose_ = true;
    }
    else
    {
      current_yaw_ = (1.0 - smoothing_factor_) * current_yaw_ + smoothing_factor_ * target_yaw;
      current_pitch_ = (1.0 - smoothing_factor_) * current_pitch_ + smoothing_factor_ * target_pitch;
    }

    last_valid_time_ = ros::Time::now();
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    if (!have_valid_pose_)
      return;

    const ros::Time now = ros::Time::now();
    if (stale_timeout_s_ > 0.0 && (now - last_valid_time_).toSec() > stale_timeout_s_)
    {
      ROS_WARN_THROTTLE(2.0, "Pico head control has no fresh /robot_body_matrices, stop publishing head command");
      return;
    }

    kuavo_msgs::robotHeadMotionData msg;
    // robotHeadMotionData 约定单位是度：joint_data[0]=yaw，joint_data[1]=pitch。
    msg.joint_data = {current_yaw_, current_pitch_};
    head_pub_.publish(msg);
  }

  ros::Subscriber body_matrices_sub_;
  ros::Publisher head_pub_;
  ros::Timer publish_timer_;

  std::string body_matrices_topic_;
  std::string head_output_topic_;
  double publish_hz_ = 50.0;
  double stale_timeout_s_ = 0.5;
  double smoothing_factor_ = 0.2;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;
  std::vector<double> yaw_limit_{-30.0, 30.0};
  std::vector<double> pitch_limit_{-25.0, 25.0};
  bool have_valid_pose_ = false;
  double current_yaw_ = 0.0;
  double current_pitch_ = 0.0;
  ros::Time last_valid_time_;
};

class XsensePicoControlNode
{
public:
  XsensePicoControlNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<bool>("enable_head_control", enable_head_control_, false);
    private_nh.param<bool>("enable_claw_control", enable_claw_control_, false);

    if (enable_claw_control_)
      end_effector_controller_.reset(new PicoEndEffectorController(nh, private_nh));
    if (enable_head_control_)
      head_controller_.reset(new PicoHeadVrFollowController(nh, private_nh));

    ROS_INFO_STREAM("Pico control node started: head="
                    << (enable_head_control_ ? "enabled" : "disabled")
                    << ", claw=" << (enable_claw_control_ ? "enabled" : "disabled"));
  }

private:
  bool enable_head_control_ = false;
  bool enable_claw_control_ = false;
  std::unique_ptr<PicoEndEffectorController> end_effector_controller_;
  std::unique_ptr<PicoHeadVrFollowController> head_controller_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xsense_pico_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  XsensePicoControlNode node(nh, private_nh);
  (void)node;  // keep node alive for callbacks while spinning
  ros::spin();
  return 0;
}
