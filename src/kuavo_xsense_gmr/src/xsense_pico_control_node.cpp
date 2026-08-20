#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <kuavo_common/common/json.hpp>
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/SetHeadControlMode.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotBodyMatrices.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/robotHeadMotionData.h>

#include <kuavo_xsense_gmr/head_active_hand_selector.hpp>

namespace
{

using Matrix4 = std::array<double, 16>;

double clamp(const double value, const double low, const double high)
{
  return std::max(low, std::min(high, value));
}

double clampFinite(const double value, const double low, const double high)
{
  if (!std::isfinite(value))
    return 0.0;
  return clamp(value, low, high);
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

using SteadyClock = std::chrono::steady_clock;

double elapsedSeconds(const SteadyClock::time_point start, const SteadyClock::time_point end)
{
  return std::chrono::duration<double>(end - start).count();
}

class MomentaryLongPressButton
{
public:
  void configure(const double long_press_seconds)
  {
    long_press_seconds_ = std::max(0.0, long_press_seconds);
    state_ = State::IDLE;
    press_start_ = SteadyClock::time_point{};
  }

  void update(const bool pressed, const SteadyClock::time_point now)
  {
    switch (state_)
    {
      case State::IDLE:
        if (pressed)
        {
          press_start_ = now;
          state_ = long_press_seconds_ <= 0.0 ? State::ACTIVE : State::PRESSING;
        }
        break;

      case State::PRESSING:
        if (!pressed)
          state_ = State::IDLE;
        else if (elapsedSeconds(press_start_, now) >= long_press_seconds_)
          state_ = State::ACTIVE;
        break;

      case State::ACTIVE:
        if (!pressed)
          state_ = State::IDLE;
        break;

      case State::WAIT_RELEASE:
        if (!pressed)
          state_ = State::IDLE;
        break;
    }
  }

  void handleInputTimeout()
  {
    state_ = State::WAIT_RELEASE;
  }

  bool active() const
  {
    return state_ == State::ACTIVE;
  }

private:
  enum class State
  {
    IDLE,
    PRESSING,
    ACTIVE,
    WAIT_RELEASE
  };

  State state_ = State::IDLE;
  double long_press_seconds_ = 0.5;
  SteadyClock::time_point press_start_{};
};

class ToggleLongPressButton
{
public:
  void configure(const double long_press_seconds)
  {
    long_press_seconds_ = std::max(0.0, long_press_seconds);
    // Require one observed release before accepting a gesture.  This prevents
    // a button already held during node startup from creating a synthetic
    // long press.
    state_ = State::WAIT_RELEASE;
    latched_ = false;
    press_start_ = SteadyClock::time_point{};
  }

  void update(const bool pressed, const SteadyClock::time_point now)
  {
    switch (state_)
    {
      case State::IDLE:
        if (pressed)
        {
          press_start_ = now;
          if (long_press_seconds_ <= 0.0)
          {
            // 无需长按，立�? toggle
            latched_ = !latched_;
            state_ = State::HOLDING;
          }
          else
          {
            state_ = State::PRESSING;
          }
        }
        break;

      case State::PRESSING:
        if (!pressed)
        {
          // The release sample may be the first sample at/after the threshold.
          // Count it as a long press when it reaches this node's independent
          // 0.8 s threshold.  The converter intentionally stops classifying A
          // as a short press at 0.5 s, leaving a no-action guard interval.
          if (elapsedSeconds(press_start_, now) >= long_press_seconds_)
            latched_ = !latched_;
          state_ = State::IDLE;
        }
        else if (elapsedSeconds(press_start_, now) >= long_press_seconds_)
        {
          // 长按达标，toggle 锁存状�?
          latched_ = !latched_;
          state_ = State::HOLDING;
        }
        break;

      case State::HOLDING:
        // 等松开后才能再次触发，防止按住期间重复 toggle
        if (!pressed)
          state_ = State::IDLE;
        break;

      case State::WAIT_RELEASE:
        // After startup or an input discontinuity, a physical release is
        // required before a new long press can begin.  The latched output is
        // intentionally preserved.
        if (!pressed)
          state_ = State::IDLE;
        break;
    }
  }

  void handleInputTimeout()
  {
    state_ = State::WAIT_RELEASE;
  }

  bool active() const
  {
    return latched_;
  }

private:
  enum class State
  {
    IDLE,         // 空闲，等待按�?
    PRESSING,     // 按下中，未达长按阈�?
    HOLDING,      // �? toggle，等待松开后才能再次触�?
    WAIT_RELEASE  // 启动或输入中断后，等待一次明确松开
  };

  State state_ = State::WAIT_RELEASE;
  bool latched_ = false;
  double long_press_seconds_ = 0.5;
  SteadyClock::time_point press_start_{};
};

bool isDexhandEndEffector(const std::string& end_effector_type)
{
  return end_effector_type == "qiangnao" ||
         end_effector_type == "linker_o6" ||
         end_effector_type == "linker_l6";
}

enum class EndEffectorControlMode
{
  CLAW,
  DEXHAND,
  NONE,
  INVALID
};

struct EndEffectorSelection
{
  EndEffectorControlMode mode = EndEffectorControlMode::INVALID;
  std::string normalized_type;
  std::string source;
  std::string error;
};

std::string normalizeEndEffectorType(const std::string& value)
{
  std::size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])))
    ++begin;

  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])))
    --end;

  std::string normalized = value.substr(begin, end - begin);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return normalized;
}

bool isBlank(const std::string& value)
{
  return std::all_of(value.begin(), value.end(), [](const unsigned char ch) {
    return std::isspace(ch) != 0;
  });
}

const char* endEffectorModeName(const EndEffectorControlMode mode)
{
  switch (mode)
  {
    case EndEffectorControlMode::CLAW:
      return "claw";
    case EndEffectorControlMode::DEXHAND:
      return "dexhand";
    case EndEffectorControlMode::NONE:
      return "none";
    case EndEffectorControlMode::INVALID:
      return "invalid";
  }
  return "invalid";
}

EndEffectorSelection selectNormalizedEndEffectorType(const std::string& normalized_type,
                                                     const std::string& source)
{
  EndEffectorSelection selection;
  selection.normalized_type = normalized_type;
  selection.source = source;

  if (normalized_type == "lejuclaw")
    selection.mode = EndEffectorControlMode::CLAW;
  else if (isDexhandEndEffector(normalized_type))
    selection.mode = EndEffectorControlMode::DEXHAND;
  else if (normalized_type == "none")
    selection.mode = EndEffectorControlMode::NONE;
  else
    selection.error = "unsupported end-effector type '" + normalized_type + "'";

  return selection;
}

EndEffectorSelection resolveEndEffectorFromKuavoConfiguration()
{
  const std::string source = "kuavo_configuration";
  EndEffectorSelection invalid_selection;
  invalid_selection.source = source;

  std::string configuration_text;
  if (!ros::param::get("/kuavo_configuration", configuration_text))
  {
    invalid_selection.error =
        "end_effector_type=auto but /kuavo_configuration is unavailable or is not a string";
    return invalid_selection;
  }
  if (configuration_text.empty() || isBlank(configuration_text))
  {
    invalid_selection.error = "end_effector_type=auto but /kuavo_configuration is empty";
    return invalid_selection;
  }

  try
  {
    const nlohmann::json configuration = nlohmann::json::parse(configuration_text);
    if (!configuration.is_object())
    {
      invalid_selection.error = "/kuavo_configuration JSON root must be an object";
      return invalid_selection;
    }

    const auto type_it = configuration.find("EndEffectorType");
    if (type_it == configuration.end())
    {
      invalid_selection.error = "/kuavo_configuration is missing EndEffectorType";
      return invalid_selection;
    }
    if (!type_it->is_array())
    {
      invalid_selection.error = "/kuavo_configuration.EndEffectorType must be an array";
      return invalid_selection;
    }
    if (type_it->size() != 2)
    {
      invalid_selection.error = "/kuavo_configuration.EndEffectorType must contain exactly two items";
      return invalid_selection;
    }
    if (!(*type_it)[0].is_string() || !(*type_it)[1].is_string())
    {
      invalid_selection.error = "/kuavo_configuration.EndEffectorType items must both be strings";
      return invalid_selection;
    }

    const std::string left_type =
        normalizeEndEffectorType((*type_it)[0].get<std::string>());
    const std::string right_type =
        normalizeEndEffectorType((*type_it)[1].get<std::string>());
    if (left_type != right_type)
    {
      invalid_selection.error = "EndEffectorType mismatch: left='" + left_type +
                                "', right='" + right_type +
                                "'; mixed end-effector control is not supported";
      return invalid_selection;
    }

    EndEffectorSelection selection = selectNormalizedEndEffectorType(left_type, source);
    if (selection.mode == EndEffectorControlMode::INVALID)
    {
      selection.error = "unsupported /kuavo_configuration.EndEffectorType '" + left_type + "'";
    }
    return selection;
  }
  catch (const nlohmann::json::exception& exception)
  {
    invalid_selection.error =
        std::string("failed to parse /kuavo_configuration JSON: ") + exception.what();
    return invalid_selection;
  }
  catch (const std::exception& exception)
  {
    invalid_selection.error =
        std::string("failed to resolve EndEffectorType from /kuavo_configuration: ") + exception.what();
    return invalid_selection;
  }
}

EndEffectorSelection resolveEndEffectorSelection(ros::NodeHandle& private_nh)
{
  std::string requested_type = "auto";
  XmlRpc::XmlRpcValue requested_value;
  if (private_nh.getParam("end_effector_type", requested_value))
  {
    if (requested_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      EndEffectorSelection invalid_selection;
      invalid_selection.source = "launch_explicit";
      invalid_selection.error =
          "invalid private parameter ~end_effector_type: expected a string";
      return invalid_selection;
    }
    requested_type = static_cast<std::string>(requested_value);
  }
  requested_type = normalizeEndEffectorType(requested_type);

  if (requested_type == "auto")
    return resolveEndEffectorFromKuavoConfiguration();

  EndEffectorSelection selection =
      selectNormalizedEndEffectorType(requested_type, "launch_explicit");
  if (selection.mode == EndEffectorControlMode::INVALID)
  {
    selection.error = "invalid explicit Pico end-effector type '" + requested_type +
                      "'; expected auto, qiangnao, lejuclaw, none, linker_o6 or linker_l6";
  }
  return selection;
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

  // /robot_body_matrices 使用行主�? 4x4 齐次矩阵；刚体逆变换为 R^T �? -R^T * t�?
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
  PicoEndEffectorController(ros::NodeHandle& nh,
                            ros::NodeHandle& private_nh,
                            const std::string& end_effector_type)
    : end_effector_type_(end_effector_type)
  {
    private_nh.param<std::string>("joy_topic", joy_topic_, "/pico/joy");
    private_nh.param<std::string>("claw_topic", claw_topic_, "/leju_claw_command");
    private_nh.param<std::string>("dexhand_topic", dexhand_topic_, "/control_robot_hand_position");
    private_nh.param<double>("claw_trigger_threshold", claw_trigger_threshold_, 0.5);
    private_nh.param<double>("claw_open_position", open_position_, 10.0);
    private_nh.param<double>("claw_close_position", close_position_, 90.0);
    private_nh.param<double>("claw_velocity", velocity_, 90.0);
    private_nh.param<double>("claw_effort", effort_, 1.0);
    private_nh.param<double>("claw_publish_rate_hz", publish_rate_hz_, 10.0);
    private_nh.param<double>("dexhand_command_min", dexhand_command_min_, 0.0);
    private_nh.param<double>("dexhand_command_max", dexhand_command_max_, 100.0);
    if (!private_nh.getParam("dexhand_trigger_deadzone", dexhand_trigger_deadzone_))
    {
      private_nh.param<double>("dexhand_grip_deadzone", dexhand_trigger_deadzone_, 0.02);
      if (private_nh.hasParam("dexhand_grip_deadzone"))
      {
        ROS_WARN("~dexhand_grip_deadzone is deprecated; use ~dexhand_trigger_deadzone");
      }
    }
    private_nh.param<double>("dexhand_smoothing_alpha", dexhand_smoothing_alpha_, 0.35);
    private_nh.param<double>("gait_a_short_press_max_s", gait_a_short_press_max_s_, 0.5);
    private_nh.param<double>("dexhand_thumb_aux_long_press_s", dexhand_thumb_aux_long_press_s_, 0.8);
    private_nh.param<double>("dexhand_thumb_aux_inactive_command",
                             dexhand_thumb_aux_inactive_command_,
                             0.0);
    private_nh.param<double>("dexhand_thumb_aux_active_command",
                             dexhand_thumb_aux_active_command_,
                             100.0);
    private_nh.param<double>("dexhand_thumb_aux_joy_timeout_s",
                             dexhand_thumb_aux_joy_timeout_s_,
                             2.0);
    private_nh.param<double>("dexhand_thumb_aux_max_sample_gap_s",
                             dexhand_thumb_aux_max_sample_gap_s_,
                             0.5);

    if (!std::isfinite(claw_trigger_threshold_))
      throw std::invalid_argument("claw_trigger_threshold must be finite");
    if (!std::isfinite(dexhand_trigger_deadzone_))
      throw std::invalid_argument("dexhand_trigger_deadzone must be finite");
    if (!std::isfinite(dexhand_smoothing_alpha_))
      throw std::invalid_argument("dexhand_smoothing_alpha must be finite");
    if (!std::isfinite(gait_a_short_press_max_s_) || gait_a_short_press_max_s_ <= 0.0)
      throw std::invalid_argument("gait_a_short_press_max_s must be finite and > 0");
    if (!std::isfinite(dexhand_thumb_aux_long_press_s_) ||
        dexhand_thumb_aux_long_press_s_ <= 0.0)
    {
      throw std::invalid_argument(
          "dexhand_thumb_aux_long_press_s must be finite and > 0");
    }
    if (gait_a_short_press_max_s_ >= dexhand_thumb_aux_long_press_s_)
    {
      throw std::invalid_argument(
          "gait_a_short_press_max_s must be < dexhand_thumb_aux_long_press_s");
    }
    if (!std::isfinite(dexhand_thumb_aux_joy_timeout_s_) ||
        dexhand_thumb_aux_joy_timeout_s_ < 0.0)
    {
      throw std::invalid_argument(
          "dexhand_thumb_aux_joy_timeout_s must be finite and >= 0");
    }
    if (!std::isfinite(dexhand_thumb_aux_max_sample_gap_s_) ||
        dexhand_thumb_aux_max_sample_gap_s_ <= 0.0)
    {
      throw std::invalid_argument(
          "dexhand_thumb_aux_max_sample_gap_s must be finite and > 0");
    }

    claw_trigger_threshold_ = clamp(claw_trigger_threshold_, 0.0, 1.0);
    publish_rate_hz_ = std::max(0.1, publish_rate_hz_);
    dexhand_trigger_deadzone_ = clamp(dexhand_trigger_deadzone_, 0.0, 1.0);
    dexhand_smoothing_alpha_ = clamp(dexhand_smoothing_alpha_, 0.0, 1.0);
    if (dexhand_command_min_ > dexhand_command_max_)
      std::swap(dexhand_command_min_, dexhand_command_max_);

    dexhand_thumb_aux_inactive_cmd_ =
        clampToUint8(dexhand_thumb_aux_inactive_command_, dexhand_command_min_, dexhand_command_max_);
    dexhand_thumb_aux_active_cmd_ =
        clampToUint8(dexhand_thumb_aux_active_command_, dexhand_command_min_, dexhand_command_max_);
    left_thumb_aux_button_.configure(dexhand_thumb_aux_long_press_s_);
    right_thumb_aux_button_.configure(dexhand_thumb_aux_long_press_s_);

    if (end_effector_type_ == "lejuclaw")
    {
      claw_pub_ = nh.advertise<kuavo_msgs::lejuClawCommand>(claw_topic_, 10);
      ROS_INFO_STREAM("Pico claw control enabled: " << joy_topic_ << " -> " << claw_topic_
                                                    << ", threshold=" << claw_trigger_threshold_
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
                                                       << "], main_flex_source=trigger"
                                                       << ", main_flex_indices=[0,2,3,4,5]"
                                                       << ", trigger_deadzone=" << dexhand_trigger_deadzone_
                                                       << ", thumb_aux_source=left_X/right_A"
                                                       << ", gait_A_short_press_max="
                                                       << gait_a_short_press_max_s_ << "s"
                                                       << ", thumb_aux_long_press="
                                                       << dexhand_thumb_aux_long_press_s_
                                                       << "s, thumb_aux_commands=["
                                                       << static_cast<int>(dexhand_thumb_aux_inactive_cmd_)
                                                       << ", "
                                                       << static_cast<int>(dexhand_thumb_aux_active_cmd_)
                                                       << "], thumb_aux_joy_timeout="
                                                       << dexhand_thumb_aux_joy_timeout_s_
                                                       << "s, thumb_aux_max_sample_gap="
                                                       << dexhand_thumb_aux_max_sample_gap_s_ << "s");
    }
    else
    {
      ROS_ERROR_STREAM("Pico end-effector controller received unsupported validated type '"
                       << end_effector_type_ << "'");
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

    // The same upper trigger has mode-specific semantics: binary open/close
    // for claws, and continuous main-flex input for dexterous hands.
    left_trigger_ = clampFinite(joy->left_trigger, 0.0, 1.0);
    right_trigger_ = clampFinite(joy->right_trigger, 0.0, 1.0);
    left_closed_ = left_trigger_ >= claw_trigger_threshold_;
    right_closed_ = right_trigger_ >= claw_trigger_threshold_;

    if (isDexhandEndEffector(end_effector_type_))
    {
      const bool left_was_active = left_thumb_aux_button_.active();
      const bool right_was_active = right_thumb_aux_button_.active();
      updateThumbAuxButtons(joy->left_first_button_pressed,
                            joy->right_first_button_pressed,
                            SteadyClock::now());
      logThumbAuxTransitions(left_was_active, right_was_active);
    }

    have_joy_ = true;
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    if (!have_joy_)
      return;

    if (end_effector_type_ == "lejuclaw")
      publishClawCommand();
    else if (isDexhandEndEffector(end_effector_type_))
    {
      const bool left_was_active = left_thumb_aux_button_.active();
      const bool right_was_active = right_thumb_aux_button_.active();
      if (updateThumbAuxInputTimeout(SteadyClock::now()) &&
          (left_was_active || right_was_active))
      {
        ROS_WARN("Pico Joy input timed out; dexhand thumb auxiliary latched state preserved");
      }
      publishDexhandCommand();
    }
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
    // trigger(上扳�?)统一驱动索引0�?2~5；索�?1由同�? X/A 长按锁存状态独立控制�?
    // grip(侧扳�?)仅保留组合键用途，不参与灵巧手主抓握�?
    const uint8_t left_flex_cmd =
        triggerToDexhandCommand(left_trigger_, filtered_left_flex_, have_filtered_left_flex_);
    const uint8_t right_flex_cmd =
        triggerToDexhandCommand(right_trigger_, filtered_right_flex_, have_filtered_right_flex_);
    const uint8_t left_thumb_aux_cmd =
        left_thumb_aux_button_.active() ? dexhand_thumb_aux_active_cmd_ : dexhand_thumb_aux_inactive_cmd_;
    const uint8_t right_thumb_aux_cmd =
        right_thumb_aux_button_.active() ? dexhand_thumb_aux_active_cmd_ : dexhand_thumb_aux_inactive_cmd_;

    kuavo_msgs::robotHandPosition msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "pico_dexhand_control";
    // 索引约定�?0=拇指屈曲�?1=拇指辅助/横摆/第二关节�?2~5=食指至小指�?
    msg.left_hand_position = {
        left_flex_cmd,
        left_thumb_aux_cmd,
        left_flex_cmd,
        left_flex_cmd,
        left_flex_cmd,
        left_flex_cmd,
    };
    msg.right_hand_position = {
        right_flex_cmd,
        right_thumb_aux_cmd,
        right_flex_cmd,
        right_flex_cmd,
        right_flex_cmd,
        right_flex_cmd,
    };
    hand_pub_.publish(msg);
  }

  void updateThumbAuxButtons(const bool left_x_pressed,
                             const bool right_a_pressed,
                             const SteadyClock::time_point now)
  {
    if (have_thumb_aux_joy_time_ && !thumb_aux_input_timed_out_)
    {
      const double sample_gap_s = elapsedSeconds(last_thumb_aux_joy_time_, now);
      if (dexhand_thumb_aux_joy_timeout_s_ > 0.0 &&
          sample_gap_s >= dexhand_thumb_aux_joy_timeout_s_)
      {
        markThumbAuxInputTimedOut();
      }
      else if (sample_gap_s >= dexhand_thumb_aux_max_sample_gap_s_)
      {
        // A discontinuous sample stream cannot prove that the button remained
        // physically held throughout the missing interval.  Cancel only the
        // in-progress gesture and preserve the already latched output.
        left_thumb_aux_button_.handleInputTimeout();
        right_thumb_aux_button_.handleInputTimeout();
        ROS_WARN_STREAM_THROTTLE(
            2.0,
            "Pico Joy sample gap " << sample_gap_s
                                    << "s interrupted dexhand thumb gesture; wait for release");
      }
    }

    last_thumb_aux_joy_time_ = now;
    have_thumb_aux_joy_time_ = true;
    left_thumb_aux_button_.update(left_x_pressed, now);
    right_thumb_aux_button_.update(right_a_pressed, now);
    thumb_aux_input_timed_out_ = false;
  }

  bool updateThumbAuxInputTimeout(const SteadyClock::time_point now)
  {
    if (!have_thumb_aux_joy_time_ || thumb_aux_input_timed_out_ ||
        dexhand_thumb_aux_joy_timeout_s_ <= 0.0)
    {
      return false;
    }

    if (elapsedSeconds(last_thumb_aux_joy_time_, now) < dexhand_thumb_aux_joy_timeout_s_)
      return false;

    markThumbAuxInputTimedOut();
    return true;
  }

  void markThumbAuxInputTimedOut()
  {
    left_thumb_aux_button_.handleInputTimeout();
    right_thumb_aux_button_.handleInputTimeout();
    thumb_aux_input_timed_out_ = true;
  }

  void logThumbAuxTransitions(const bool left_was_active, const bool right_was_active)
  {
    const bool left_is_active = left_thumb_aux_button_.active();
    const bool right_is_active = right_thumb_aux_button_.active();
    if (left_was_active != left_is_active)
      ROS_INFO_STREAM("Pico left thumb auxiliary " << (left_is_active ? "active" : "released"));
    if (right_was_active != right_is_active)
      ROS_INFO_STREAM("Pico right thumb auxiliary " << (right_is_active ? "active" : "released"));
  }

  uint8_t triggerToDexhandCommand(const double trigger,
                                  double& filtered_value,
                                  bool& have_filtered_value)
  {
    const double normalized = trigger < dexhand_trigger_deadzone_ ? 0.0 : trigger;
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
  double claw_trigger_threshold_ = 0.5;
  double open_position_ = 10.0;
  double close_position_ = 90.0;
  double velocity_ = 90.0;
  double effort_ = 1.0;
  double publish_rate_hz_ = 10.0;
  double dexhand_command_min_ = 0.0;
  double dexhand_command_max_ = 100.0;
  double dexhand_trigger_deadzone_ = 0.02;
  double dexhand_smoothing_alpha_ = 0.35;
  double gait_a_short_press_max_s_ = 0.5;
  double dexhand_thumb_aux_long_press_s_ = 0.8;
  double dexhand_thumb_aux_inactive_command_ = 0.0;
  double dexhand_thumb_aux_active_command_ = 100.0;
  double dexhand_thumb_aux_joy_timeout_s_ = 2.0;
  double dexhand_thumb_aux_max_sample_gap_s_ = 0.5;
  uint8_t dexhand_thumb_aux_inactive_cmd_ = 0;
  uint8_t dexhand_thumb_aux_active_cmd_ = 100;
  ToggleLongPressButton left_thumb_aux_button_;
  ToggleLongPressButton right_thumb_aux_button_;
  SteadyClock::time_point last_thumb_aux_joy_time_{};
  bool have_thumb_aux_joy_time_ = false;
  bool thumb_aux_input_timed_out_ = false;
  bool have_joy_ = false;
  bool left_closed_ = false;
  bool right_closed_ = false;
  bool have_filtered_left_flex_ = false;
  bool have_filtered_right_flex_ = false;
  double left_trigger_ = 0.0;
  double right_trigger_ = 0.0;
  double filtered_left_flex_ = 0.0;
  double filtered_right_flex_ = 0.0;
};

enum class PicoHeadControlMode
{
  FIXED,
  AUTO_TRACK_ACTIVE,
  FIXED_MAIN_HAND,
  VR_FOLLOW,
};

const char* headControlModeName(const PicoHeadControlMode mode)
{
  switch (mode)
  {
    case PicoHeadControlMode::FIXED:
      return "fixed";
    case PicoHeadControlMode::AUTO_TRACK_ACTIVE:
      return "auto_track_active";
    case PicoHeadControlMode::FIXED_MAIN_HAND:
      return "fixed_main_hand";
    case PicoHeadControlMode::VR_FOLLOW:
      return "vr_follow";
  }
  return "vr_follow";
}

bool parseHeadControlMode(const std::string& value, PicoHeadControlMode& mode)
{
  const std::string normalized = normalizeEndEffectorType(value);
  if (normalized == "vr_follow")
  {
    mode = PicoHeadControlMode::VR_FOLLOW;
    return true;
  }
  if (normalized == "auto_track_active")
  {
    mode = PicoHeadControlMode::AUTO_TRACK_ACTIVE;
    return true;
  }
  if (normalized == "fixed")
  {
    mode = PicoHeadControlMode::FIXED;
    return true;
  }
  if (normalized == "fixed_main" || normalized == "fixed_main_hand")
  {
    mode = PicoHeadControlMode::FIXED_MAIN_HAND;
    return true;
  }
  return false;
}

bool parseFixedMainHand(const std::string& value, kuavo_xsense_gmr::ActiveHand& hand)
{
  const std::string normalized = normalizeEndEffectorType(value);
  if (normalized == "left")
  {
    hand = kuavo_xsense_gmr::ActiveHand::LEFT;
    return true;
  }
  if (normalized == "right")
  {
    hand = kuavo_xsense_gmr::ActiveHand::RIGHT;
    return true;
  }
  return false;
}

enum class TfLookupStatus
{
  VALID,
  UNAVAILABLE,
  STALE
};

class XsensePicoHeadController
{
public:
  using ActiveHand = kuavo_xsense_gmr::ActiveHand;
  using Position = kuavo_xsense_gmr::ActiveHandSelector::Position;

  XsensePicoHeadController(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : private_nh_(private_nh),
      tf_listener_(ros::Duration(10.0), false),
      yaw_limit_(readLimitParam(private_nh, "head_yaw_limit", -50.0, 50.0)),
      pitch_limit_(readLimitParam(private_nh, "head_pitch_limit", -25.0, 30.0)),
      head_pitch_offset_limit_(readLimitParam(private_nh, "head_pitch_offset_limit", -20.0, 20.0)),
      head_yaw_offset_limit_(readLimitParam(private_nh, "head_yaw_offset_limit", -20.0, 20.0))
  {
    private_nh.param<std::string>("body_matrices_topic", body_matrices_topic_, "/robot_body_matrices");
    private_nh.param<std::string>("head_output_topic", head_output_topic_, "/robot_head_motion_data");
    private_nh.param<double>("head_publish_hz", publish_hz_, 50.0);
    private_nh.param<double>("head_stale_timeout_s", stale_timeout_s_, 0.5);
    private_nh.param<double>("head_smoothing_factor", smoothing_factor_, 0.2);
    private_nh.param<double>("head_yaw_sign", yaw_sign_, 1.0);
    private_nh.param<double>("head_pitch_sign", pitch_sign_, 1.0);
    private_nh.param<std::string>("head_reference_frame", reference_frame_, "waist_yaw_link");
    private_nh.param<std::string>("head_origin_frame", head_origin_frame_, "zhead_1_link");
    private_nh.param<std::string>("head_left_motion_frame", left_motion_frame_, "zarm_l5_link");
    private_nh.param<std::string>("head_right_motion_frame", right_motion_frame_, "zarm_r5_link");
    private_nh.param<std::string>("head_left_target_frame", left_target_frame_, "zarm_l7_end_effector");
    private_nh.param<std::string>("head_right_target_frame", right_target_frame_, "zarm_r7_end_effector");
    private_nh.param<double>("head_auto_tf_hold_timeout_s", auto_tf_hold_timeout_s_, 0.5);
    private_nh.param<double>("head_auto_smoothing_tau_s", auto_smoothing_tau_s_, 0.12);
    private_nh.param<double>("head_yaw_rate_limit_dps", yaw_rate_limit_dps_, 60.0);
    private_nh.param<double>("head_pitch_rate_limit_dps", pitch_rate_limit_dps_, 45.0);

    kuavo_xsense_gmr::ActiveHandSelectorConfig selector_config;
    private_nh.param<double>("head_activity_window_s", selector_config.window_s, 0.20);
    private_nh.param<double>("head_activity_enter_speed_mps", selector_config.enter_speed_mps, 0.08);
    private_nh.param<double>("head_activity_exit_speed_mps", selector_config.exit_speed_mps, 0.04);
    private_nh.param<double>("head_activity_dominance_margin_mps",
                             selector_config.dominance_margin_mps,
                             0.05);
    private_nh.param<double>("head_activity_dominance_ratio", selector_config.dominance_ratio, 1.5);
    private_nh.param<double>("head_activity_acquire_confirm_s", selector_config.acquire_confirm_s, 0.25);
    private_nh.param<double>("head_activity_switch_confirm_s", selector_config.switch_confirm_s, 0.25);
    private_nh.param<double>("head_activity_both_active_confirm_s",
                             selector_config.both_active_confirm_s,
                             0.45);
    private_nh.param<double>("head_activity_min_hold_s", selector_config.min_hold_s, 0.75);
    private_nh.param<double>("head_activity_max_sample_gap_s", selector_config.max_sample_gap_s, 0.10);
    private_nh.param<double>("head_activity_max_valid_speed_mps",
                             selector_config.max_valid_speed_mps,
                             3.0);

    publish_hz_ = std::max(1.0, publish_hz_);
    stale_timeout_s_ = std::max(0.0, stale_timeout_s_);
    smoothing_factor_ = clamp(smoothing_factor_, 0.0, 1.0);
    yaw_sign_ = yaw_sign_ >= 0.0 ? 1.0 : -1.0;
    pitch_sign_ = pitch_sign_ >= 0.0 ? 1.0 : -1.0;

    std::string selector_error;
    if (!selector_config.validate(selector_error))
      throw std::invalid_argument("invalid head active-hand selector parameters: " + selector_error);
    if (!std::isfinite(auto_tf_hold_timeout_s_) || auto_tf_hold_timeout_s_ <= 0.0)
      throw std::invalid_argument("head_auto_tf_hold_timeout_s must be finite and > 0");
    if (!std::isfinite(auto_smoothing_tau_s_) || auto_smoothing_tau_s_ <= 0.0)
      throw std::invalid_argument("head_auto_smoothing_tau_s must be finite and > 0");
    if (!std::isfinite(yaw_rate_limit_dps_) || yaw_rate_limit_dps_ <= 0.0 ||
        !std::isfinite(pitch_rate_limit_dps_) || pitch_rate_limit_dps_ <= 0.0)
    {
      throw std::invalid_argument("head yaw/pitch rate limits must be finite and > 0");
    }
    if (reference_frame_.empty() || head_origin_frame_.empty() || left_motion_frame_.empty() ||
        right_motion_frame_.empty() || left_target_frame_.empty() || right_target_frame_.empty())
    {
      throw std::invalid_argument("head TF frame parameters must not be empty");
    }
    active_hand_selector_.configure(selector_config);

    // --- 头部 pitch 偏移量微调参�? ---
    private_nh.param<bool>("head_pitch_offset_enabled", head_pitch_offset_enabled_, true);
    private_nh.param<double>("head_pitch_offset_rate_dps", head_pitch_offset_rate_dps_, 3.0);
    private_nh.param<double>("head_pitch_offset_trigger_threshold",
                             head_pitch_offset_trigger_threshold_, 0.7);
    private_nh.param<double>("head_pitch_offset_stick_threshold",
                             head_pitch_offset_stick_threshold_, 0.7);
    private_nh.param<double>("head_pitch_offset_pitch_sign",
                             head_pitch_offset_pitch_sign_, 1.0);
    private_nh.param<double>("head_yaw_offset_rate_dps", head_yaw_offset_rate_dps_, 3.0);
    private_nh.param<double>("head_yaw_offset_yaw_sign", head_yaw_offset_yaw_sign_, 1.0);
    private_nh.param<double>("head_offset_zero_stick_threshold",
                             head_offset_zero_stick_threshold_, 0.8);
    private_nh.param<double>("head_offset_joy_timeout_s", head_offset_joy_timeout_s_, 0.5);

    if (!std::isfinite(head_pitch_offset_rate_dps_) || head_pitch_offset_rate_dps_ <= 0.0)
      throw std::invalid_argument("head_pitch_offset_rate_dps must be finite and > 0");
    if (!std::isfinite(head_yaw_offset_rate_dps_) || head_yaw_offset_rate_dps_ <= 0.0)
      throw std::invalid_argument("head_yaw_offset_rate_dps must be finite and > 0");
    if (!std::isfinite(head_pitch_offset_limit_[0]) ||
        !std::isfinite(head_pitch_offset_limit_[1]) ||
        !std::isfinite(head_yaw_offset_limit_[0]) ||
        !std::isfinite(head_yaw_offset_limit_[1]))
    {
      throw std::invalid_argument("head pitch/yaw offset limits must be finite");
    }
    if (!std::isfinite(head_pitch_offset_trigger_threshold_) ||
        !std::isfinite(head_pitch_offset_stick_threshold_) ||
        !std::isfinite(head_offset_zero_stick_threshold_))
    {
      throw std::invalid_argument("head offset thresholds must be finite");
    }
    head_pitch_offset_trigger_threshold_ =
        clamp(head_pitch_offset_trigger_threshold_, 0.0, 1.0);
    head_pitch_offset_stick_threshold_ =
        clamp(head_pitch_offset_stick_threshold_, 0.0, 1.0);
    head_offset_zero_stick_threshold_ =
        clamp(head_offset_zero_stick_threshold_, 0.0, 1.0);
    if (!std::isfinite(head_pitch_offset_pitch_sign_) || head_pitch_offset_pitch_sign_ == 0.0)
      throw std::invalid_argument("head_pitch_offset_pitch_sign must be finite and non-zero");
    if (!std::isfinite(head_yaw_offset_yaw_sign_) || head_yaw_offset_yaw_sign_ == 0.0)
      throw std::invalid_argument("head_yaw_offset_yaw_sign must be finite and non-zero");
    if (!std::isfinite(head_offset_joy_timeout_s_) || head_offset_joy_timeout_s_ < 0.0)
      throw std::invalid_argument("head_offset_joy_timeout_s must be finite and >= 0");

    // joy 订阅（与 PicoEndEffectorController 共用同一话题�?
    {
      std::string joy_topic;
      private_nh.param<std::string>("joy_topic", joy_topic, "/pico/joy");
      joy_sub_ = nh.subscribe(joy_topic, 20, &XsensePicoHeadController::joyCallback, this);
    }

    std::string requested_mode;
    private_nh.param<std::string>("head_control_mode", requested_mode, "vr_follow");
    if (!parseHeadControlMode(requested_mode, mode_))
    {
      throw std::invalid_argument("invalid head_control_mode '" + requested_mode +
                                  "'; expected fixed, auto_track_active, fixed_main_hand or vr_follow");
    }

    std::string requested_fixed_main_hand;
    private_nh.param<std::string>("head_fixed_main_hand", requested_fixed_main_hand, "right");
    if (!parseFixedMainHand(requested_fixed_main_hand, fixed_main_hand_))
    {
      throw std::invalid_argument("invalid head_fixed_main_hand '" + requested_fixed_main_hand +
                                  "'; expected left or right");
    }

    head_pub_ = nh.advertise<kuavo_msgs::robotHeadMotionData>(head_output_topic_, 10);
    body_matrices_sub_ = nh.subscribe(body_matrices_topic_,
                                      20,
                                      &XsensePicoHeadController::bodyMatricesCallback,
                                      this);
    active_hand_pub_ = private_nh.advertise<std_msgs::String>("active_hand", 1, true);
    motion_score_pub_ =
        private_nh.advertise<std_msgs::Float64MultiArray>("active_hand_motion_score", 10);
    mode_service_ = private_nh.advertiseService("set_head_control_mode",
                                                &XsensePicoHeadController::setHeadControlMode,
                                                this);
    publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_hz_),
                                    &XsensePicoHeadController::publishTimerCallback,
                                    this);

    enterMode(mode_);
    private_nh_.setParam("head_control_mode", headControlModeName(mode_));
    private_nh_.setParam("head_fixed_main_hand",
                         kuavo_xsense_gmr::activeHandName(fixed_main_hand_));
    publishActiveHand(true);

    ROS_INFO_STREAM("Pico head control enabled: mode=" << headControlModeName(mode_)
                                                        << ", output=" << head_output_topic_
                                                        << ", publish_hz=" << publish_hz_
                                                        << ", yaw_limit=[" << yaw_limit_[0]
                                                        << ", " << yaw_limit_[1]
                                                        << "], pitch_limit=[" << pitch_limit_[0]
                                                        << ", " << pitch_limit_[1] << "]");
    ROS_INFO_STREAM("Pico fixed-main-hand selection: "
                    << kuavo_xsense_gmr::activeHandName(fixed_main_hand_));
    ROS_INFO_STREAM("Pico auto-track frames: reference=" << reference_frame_
                                                          << ", origin=" << head_origin_frame_
                                                          << ", motion=[" << left_motion_frame_
                                                          << ", " << right_motion_frame_
                                                          << "], target=[" << left_target_frame_
                                                          << ", " << right_target_frame_ << "]");
    ROS_INFO_STREAM("Pico auto-track selector: window=" << selector_config.window_s
                                                         << "s, enter=" << selector_config.enter_speed_mps
                                                         << "m/s, exit=" << selector_config.exit_speed_mps
                                                         << "m/s, margin="
                                                         << selector_config.dominance_margin_mps
                                                         << "m/s, ratio=" << selector_config.dominance_ratio
                                                         << ", acquire=" << selector_config.acquire_confirm_s
                                                         << "s, switch=" << selector_config.switch_confirm_s
                                                         << "s, both_active="
                                                         << selector_config.both_active_confirm_s
                                                         << "s, min_hold=" << selector_config.min_hold_s << "s");
    ROS_INFO_STREAM("Pico head offset: enabled=" << (head_pitch_offset_enabled_ ? "true" : "false")
                    << ", pitch_rate=" << head_pitch_offset_rate_dps_ << "dps"
                    << ", pitch_limit=[" << head_pitch_offset_limit_[0] << ", "
                    << head_pitch_offset_limit_[1] << "]"
                    << ", pitch_sign=" << head_pitch_offset_pitch_sign_
                    << ", yaw_rate=" << head_yaw_offset_rate_dps_ << "dps"
                    << ", yaw_limit=[" << head_yaw_offset_limit_[0] << ", "
                    << head_yaw_offset_limit_[1] << "]"
                    << ", yaw_sign=" << head_yaw_offset_yaw_sign_
                    << ", trigger_threshold=" << head_pitch_offset_trigger_threshold_
                    << ", stick_threshold=" << head_pitch_offset_stick_threshold_
                    << ", zero_stick_threshold=" << head_offset_zero_stick_threshold_
                    << ", joy_timeout=" << head_offset_joy_timeout_s_ << "s");
  }

private:
  void joyCallback(const kuavo_msgs::JoySticks::ConstPtr& joy)
  {
    if (!joy)
      return;

    joy_left_trigger_ = clampFinite(joy->left_trigger, 0.0, 1.0);
    joy_left_grip_ = clampFinite(joy->left_grip, 0.0, 1.0);
    joy_right_trigger_ = clampFinite(joy->right_trigger, 0.0, 1.0);
    joy_right_grip_ = clampFinite(joy->right_grip, 0.0, 1.0);
    joy_left_x_ = clampFinite(joy->left_x, -1.0, 1.0);
    joy_left_y_ = clampFinite(joy->left_y, -1.0, 1.0);
    joy_right_x_ = clampFinite(joy->right_x, -1.0, 1.0);
    joy_right_y_ = clampFinite(joy->right_y, -1.0, 1.0);
    last_offset_joy_time_ = SteadyClock::now();
    have_joy_ = true;
  }

  void stopHeadOffsetAdjustment()
  {
    if (!offset_active_)
      return;
    offset_active_ = false;
    ROS_INFO_STREAM("Pico head offset stopped, holding at pitch=" << pitch_offset_
                                                                  << "deg, yaw="
                                                                  << yaw_offset_ << "deg");
  }

  void updateHeadOffset(const bool have_valid_output_dt, const double output_dt)
  {
    const SteadyClock::time_point now = SteadyClock::now();
    const bool joy_timed_out =
        have_joy_ && head_offset_joy_timeout_s_ > 0.0 &&
        elapsedSeconds(last_offset_joy_time_, now) >= head_offset_joy_timeout_s_;
    if (!head_pitch_offset_enabled_ || !have_joy_ || joy_timed_out ||
        !have_valid_output_dt)
    {
      if (joy_timed_out)
        ROS_WARN_THROTTLE(2.0, "Pico head offset input timed out; hold current offsets");
      stopHeadOffsetAdjustment();
      return;
    }

    const bool all_triggers_pressed =
        joy_left_trigger_ >= head_pitch_offset_trigger_threshold_ &&
        joy_left_grip_ >= head_pitch_offset_trigger_threshold_ &&
        joy_right_trigger_ >= head_pitch_offset_trigger_threshold_ &&
        joy_right_grip_ >= head_pitch_offset_trigger_threshold_;

    if (!all_triggers_pressed)
    {
      stopHeadOffsetAdjustment();
      return;
    }

    const double left_abs_x = std::abs(joy_left_x_);
    const double left_abs_y = std::abs(joy_left_y_);
    const double right_abs_x = std::abs(joy_right_x_);
    const double right_abs_y = std::abs(joy_right_y_);
    const bool left_main_x = left_abs_x > left_abs_y;
    const bool left_main_y = left_abs_y > left_abs_x;
    const bool right_main_x = right_abs_x > right_abs_y;
    const bool right_main_y = right_abs_y > right_abs_x;

    // Four triggers plus the specified outward cross resets both offsets.
    // The opposite cross is intentionally left unassigned.
    if (left_main_x && right_main_x &&
        joy_left_x_ > head_offset_zero_stick_threshold_ &&
        joy_right_x_ < -head_offset_zero_stick_threshold_)
    {
      if (pitch_offset_ != 0.0 || yaw_offset_ != 0.0)
      {
        ROS_INFO_STREAM("Pico head offset reset to zero: pitch=" << pitch_offset_
                                                                  << "deg, yaw="
                                                                  << yaw_offset_ << "deg");
        pitch_offset_ = 0.0;
        yaw_offset_ = 0.0;
      }
      offset_active_ = false;
      return;
    }

    bool any_active = false;

    if (left_main_y && right_main_y)
    {
      const bool both_forward =
          joy_left_y_ > head_pitch_offset_stick_threshold_ &&
          joy_right_y_ > head_pitch_offset_stick_threshold_;
      const bool both_backward =
          joy_left_y_ < -head_pitch_offset_stick_threshold_ &&
          joy_right_y_ < -head_pitch_offset_stick_threshold_;
      if (both_forward || both_backward)
      {
        const double direction = both_forward ? -head_pitch_offset_pitch_sign_
                                              : head_pitch_offset_pitch_sign_;
        pitch_offset_ += direction * head_pitch_offset_rate_dps_ * output_dt;
        pitch_offset_ = clamp(pitch_offset_,
                              head_pitch_offset_limit_[0],
                              head_pitch_offset_limit_[1]);
        any_active = true;
      }
    }

    if (left_main_x && right_main_x)
    {
      const bool both_left =
          joy_left_x_ < -head_pitch_offset_stick_threshold_ &&
          joy_right_x_ < -head_pitch_offset_stick_threshold_;
      const bool both_right =
          joy_left_x_ > head_pitch_offset_stick_threshold_ &&
          joy_right_x_ > head_pitch_offset_stick_threshold_;
      if (both_left || both_right)
      {
        const double direction = both_right ? head_yaw_offset_yaw_sign_
                                            : -head_yaw_offset_yaw_sign_;
        yaw_offset_ += direction * head_yaw_offset_rate_dps_ * output_dt;
        yaw_offset_ = clamp(yaw_offset_,
                            head_yaw_offset_limit_[0],
                            head_yaw_offset_limit_[1]);
        any_active = true;
      }
    }

    if (any_active && !offset_active_)
    {
      offset_active_ = true;
      ROS_INFO_STREAM("Pico head offset active: pitch=" << pitch_offset_
                                                         << "deg, yaw=" << yaw_offset_
                                                         << "deg");
    }
    else if (!any_active)
    {
      stopHeadOffsetAdjustment();
    }
  }

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

    // vr_follow 只关心头显相对骨盆的姿态，避免人体整体转身导致机器人头部误跟随世界朝向�?
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

    if (!have_valid_vr_pose_)
    {
      vr_current_yaw_ = target_yaw;
      vr_current_pitch_ = target_pitch;
      have_valid_vr_pose_ = true;
    }
    else
    {
      vr_current_yaw_ =
          (1.0 - smoothing_factor_) * vr_current_yaw_ + smoothing_factor_ * target_yaw;
      vr_current_pitch_ =
          (1.0 - smoothing_factor_) * vr_current_pitch_ + smoothing_factor_ * target_pitch;
    }

    last_valid_vr_time_ = ros::Time::now();
    waiting_for_fresh_vr_pose_ = false;
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    double output_dt = 0.0;
    const bool have_valid_output_dt = updateOutputClock(now, output_dt);

    // 更新头部偏移量（所有模式通用，在模式分发之前执行�?
    updateHeadOffset(have_valid_output_dt, output_dt);

    switch (mode_)
    {
      case PicoHeadControlMode::FIXED:
        updateAndPublishFixed(have_valid_output_dt, output_dt);
        break;
      case PicoHeadControlMode::AUTO_TRACK_ACTIVE:
        updateAndPublishAutoTrack(now, have_valid_output_dt, output_dt);
        break;
      case PicoHeadControlMode::FIXED_MAIN_HAND:
        updateAndPublishFixedMainHand(now, have_valid_output_dt, output_dt);
        break;
      case PicoHeadControlMode::VR_FOLLOW:
        publishVrFollow(now, have_valid_output_dt, output_dt);
        break;
    }
  }

  void publishVrFollow(const ros::Time& now,
                       const bool have_valid_output_dt,
                       const double output_dt)
  {
    if (!have_valid_vr_pose_)
      return;

    const double vr_pose_age_s = (now - last_valid_vr_time_).toSec();
    if (!std::isfinite(vr_pose_age_s) || vr_pose_age_s < 0.0)
    {
      waiting_for_fresh_vr_pose_ = true;
      ROS_WARN_THROTTLE(
          2.0,
          "Pico head control detected ROS time rollback; wait for a fresh /robot_body_matrices frame");
      return;
    }

    if (waiting_for_fresh_vr_pose_ ||
        (stale_timeout_s_ > 0.0 && vr_pose_age_s > stale_timeout_s_))
    {
      ROS_WARN_THROTTLE(2.0, "Pico head control has no fresh /robot_body_matrices, stop publishing head command");
      return;
    }

    if (transitioning_to_vr_ && have_published_command_)
    {
      if (!have_valid_output_dt)
      {
        publishHeadCommand(command_yaw_, command_pitch_);
        return;
      }
      command_yaw_ = moveTowards(command_yaw_, vr_current_yaw_, yaw_rate_limit_dps_ * output_dt);
      command_pitch_ =
          moveTowards(command_pitch_, vr_current_pitch_, pitch_rate_limit_dps_ * output_dt);
      command_yaw_ = clamp(command_yaw_, yaw_limit_[0], yaw_limit_[1]);
      command_pitch_ = clamp(command_pitch_, pitch_limit_[0], pitch_limit_[1]);
      publishHeadCommand(command_yaw_, command_pitch_);
      if (std::abs(command_yaw_ - vr_current_yaw_) < 1e-6 &&
          std::abs(command_pitch_ - vr_current_pitch_) < 1e-6)
      {
        transitioning_to_vr_ = false;
        ROS_INFO("Pico head transition to vr_follow completed");
      }
      return;
    }

    // 默认 vr_follow 路径保持原行为：callback alpha 滤波，timer 仅按 50 Hz 发布�?
    publishHeadCommand(vr_current_yaw_, vr_current_pitch_);
  }

  void updateAndPublishFixed(const bool have_valid_output_dt, const double output_dt)
  {
    const double target_yaw = clamp(0.0, yaw_limit_[0], yaw_limit_[1]);
    const double target_pitch = clamp(0.0, pitch_limit_[0], pitch_limit_[1]);
    if (!have_valid_output_dt && !have_published_command_)
    {
      non_vr_filtered_yaw_ = target_yaw;
      non_vr_filtered_pitch_ = target_pitch;
      publishHeadCommand(target_yaw, target_pitch);
      return;
    }
    filterAndPublishNonVrTarget(target_yaw,
                               target_pitch,
                               have_valid_output_dt,
                               output_dt);
  }

  void updateAndPublishFixedMainHand(const ros::Time& now,
                                     const bool have_valid_output_dt,
                                     const double output_dt)
  {
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    const TfLookupStatus tracking_status =
        lookupTrackingTarget(fixed_main_hand_, now, target_yaw, target_pitch);
    if (tracking_status != TfLookupStatus::VALID)
    {
      handleFixedMainTrackingUnavailable(now, tracking_status == TfLookupStatus::STALE);
      return;
    }

    if (tracking_missing_ || fixed_main_output_suppressed_)
      ROS_INFO("Pico fixed-main-hand tracking TF recovered");
    tracking_missing_ = false;
    fixed_main_output_suppressed_ = false;
    filterAndPublishNonVrTarget(target_yaw,
                               target_pitch,
                               have_valid_output_dt,
                               output_dt);
  }

  void updateAndPublishAutoTrack(const ros::Time& now,
                                 const bool have_valid_output_dt,
                                 const double output_dt)
  {
    Position left_motion{{0.0, 0.0, 0.0}};
    Position right_motion{{0.0, 0.0, 0.0}};
    ros::Time selector_stamp;
    const TfLookupStatus motion_status = lookupPositionPair(left_motion_frame_,
                                                             right_motion_frame_,
                                                             now,
                                                             selector_stamp,
                                                             left_motion,
                                                             right_motion);

    const ActiveHand owner_before = active_hand_selector_.owner();
    const ActiveHand candidate_before = active_hand_selector_.candidateHand();
    const kuavo_xsense_gmr::CandidateReason reason_before =
        active_hand_selector_.candidateReason();

    // 先确认当前主手仍可跟踪。当前主手目标断流期间必须冻结选择器，
    // 否则另一只手的候选可能在“头部实际无法跟踪”的时间里悄悄成熟�?
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    bool have_tracking_target = false;
    if (owner_before != ActiveHand::NONE)
    {
      const TfLookupStatus tracking_status =
          lookupTrackingTarget(owner_before, now, target_yaw, target_pitch);
      if (tracking_status != TfLookupStatus::VALID)
      {
        handleTrackingUnavailable(now, tracking_status == TfLookupStatus::STALE);
        return;
      }

      have_tracking_target = true;
      if (tracking_missing_)
      {
        // 短断流恢复后保留主手，但重新 warmup，并从恢复时刻重新计算最短驻留�?
        active_hand_selector_.reset(false);
        tracking_missing_ = false;
        ROS_INFO("Pico auto-track current-hand tracking TF recovered; rebuild motion window");
      }
    }

    // owner 切换要在新主�? target 验证后才提交。保留更新前快照，避免挑战手
    // 只有 motion TF、没�? target TF 时破坏仍可跟踪的当前主手�?
    const kuavo_xsense_gmr::ActiveHandSelector selector_before_update =
        active_hand_selector_;
    if (motion_status == TfLookupStatus::VALID)
    {
      const kuavo_xsense_gmr::SelectorUpdateResult result =
          active_hand_selector_.update(selector_stamp.toSec(), left_motion, right_motion);
      logSelectorReset(result);
      publishMotionScores();
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "Pico auto-track freezes active-hand comparison: motion TF pair is "
                                   << (motion_status == TfLookupStatus::STALE ? "stale" : "unavailable"));
    }

    ActiveHand owner = active_hand_selector_.owner();

    // 首次获取或左右切换后，必须先验证“新主手”的 tracking �? B�?
    // 验证失败则原子回滚选择器，既不发布瞬�? owner，也不让非主�? target
    // 缺失阻断当前主手的正常跟踪�?
    if (owner != owner_before)
    {
      const ActiveHand proposed_owner = owner;
      double proposed_target_yaw = 0.0;
      double proposed_target_pitch = 0.0;
      const TfLookupStatus proposed_status =
          lookupTrackingTarget(proposed_owner, now, proposed_target_yaw, proposed_target_pitch);
      if (proposed_status != TfLookupStatus::VALID)
      {
        active_hand_selector_ = selector_before_update;
        owner = owner_before;
        ROS_WARN_STREAM_THROTTLE(
            2.0,
            "Pico auto-track rejects active-hand change to "
                << kuavo_xsense_gmr::activeHandName(proposed_owner)
                << ": proposed target TF is unavailable");
      }
      else
      {
        target_yaw = proposed_target_yaw;
        target_pitch = proposed_target_pitch;
        have_tracking_target = true;
      }
    }

    logSelectorTransitions(owner_before, candidate_before, reason_before);
    if (owner != owner_before)
      publishActiveHand(false);

    if (owner == ActiveHand::NONE)
    {
      tracking_missing_ = false;
      // motion TF 有效时，warmup/unselected 保持进入自动模式前的方向�?
      // tracking 超时后的 NONE 则必须继续停发，直到重新获取可跟踪的新主手�?
      if (!auto_output_suppressed_ && motion_status == TfLookupStatus::VALID &&
          have_published_command_)
      {
        publishHeadCommand(command_yaw_, command_pitch_);
      }
      return;
    }

    if (!have_tracking_target)
    {
      const TfLookupStatus tracking_status =
          lookupTrackingTarget(owner, now, target_yaw, target_pitch);
      if (tracking_status != TfLookupStatus::VALID)
      {
        handleTrackingUnavailable(now, tracking_status == TfLookupStatus::STALE);
        return;
      }
    }

    tracking_missing_ = false;
    auto_output_suppressed_ = false;
    filterAndPublishNonVrTarget(target_yaw,
                               target_pitch,
                               have_valid_output_dt,
                               output_dt);
  }

  void filterAndPublishNonVrTarget(const double target_yaw,
                                   const double target_pitch,
                                   const bool have_valid_output_dt,
                                   const double output_dt)
  {
    if (!have_valid_output_dt)
    {
      if (have_published_command_)
        publishHeadCommand(command_yaw_, command_pitch_);
      return;
    }

    const double limited_target_yaw = clamp(target_yaw, yaw_limit_[0], yaw_limit_[1]);
    const double limited_target_pitch = clamp(target_pitch, pitch_limit_[0], pitch_limit_[1]);
    const double alpha = 1.0 - std::exp(-output_dt / auto_smoothing_tau_s_);
    non_vr_filtered_yaw_ += alpha * (limited_target_yaw - non_vr_filtered_yaw_);
    non_vr_filtered_pitch_ += alpha * (limited_target_pitch - non_vr_filtered_pitch_);
    command_yaw_ =
        moveTowards(command_yaw_, non_vr_filtered_yaw_, yaw_rate_limit_dps_ * output_dt);
    command_pitch_ =
        moveTowards(command_pitch_, non_vr_filtered_pitch_, pitch_rate_limit_dps_ * output_dt);
    command_yaw_ = clamp(command_yaw_, yaw_limit_[0], yaw_limit_[1]);
    command_pitch_ = clamp(command_pitch_, pitch_limit_[0], pitch_limit_[1]);
    publishHeadCommand(command_yaw_, command_pitch_);
  }

  TfLookupStatus lookupTrackingTarget(const ActiveHand owner,
                                      const ros::Time& now,
                                      double& target_yaw,
                                      double& target_pitch)
  {
    if (owner != ActiveHand::LEFT && owner != ActiveHand::RIGHT)
      return TfLookupStatus::UNAVAILABLE;

    const std::string& target_frame =
        owner == ActiveHand::LEFT ? left_target_frame_ : right_target_frame_;
    Position head_origin{{0.0, 0.0, 0.0}};
    Position hand_target{{0.0, 0.0, 0.0}};
    ros::Time tracking_stamp;
    const TfLookupStatus status = lookupPositionPair(head_origin_frame_,
                                                      target_frame,
                                                      now,
                                                      tracking_stamp,
                                                      head_origin,
                                                      hand_target);
    if (status != TfLookupStatus::VALID)
      return status;

    if (!calculateLookAtTarget(head_origin, hand_target, target_yaw, target_pitch))
    {
      ROS_WARN_THROTTLE(2.0, "Pico hand-track skips invalid head-to-hand geometry");
      return TfLookupStatus::UNAVAILABLE;
    }
    return TfLookupStatus::VALID;
  }

  TfLookupStatus lookupPositionPair(const std::string& first_frame,
                                    const std::string& second_frame,
                                    const ros::Time& now,
                                    ros::Time& sample_stamp,
                                    Position& first_position,
                                    Position& second_position)
  {
    ros::Time first_latest;
    ros::Time second_latest;
    if (!getLatestTime(first_frame, first_latest) || !getLatestTime(second_frame, second_latest))
      return TfLookupStatus::UNAVAILABLE;

    if (first_frame == reference_frame_)
      sample_stamp = second_latest;
    else if (second_frame == reference_frame_)
      sample_stamp = first_latest;
    else
      sample_stamp = first_latest < second_latest ? first_latest : second_latest;

    // TF1 对纯静态链会用 Time(0) 表示“任意时刻均有效”。一侧为静态链�?
    // 使用另一侧的动态时间；两侧均为静态链时以当前时刻查询�?
    if (sample_stamp.isZero())
    {
      if (!first_latest.isZero())
        sample_stamp = first_latest;
      else if (!second_latest.isZero())
        sample_stamp = second_latest;
      else
        sample_stamp = now;
    }

    const double sample_age_s = (now - sample_stamp).toSec();
    if (!std::isfinite(sample_age_s) || sample_age_s < -0.05)
      return TfLookupStatus::UNAVAILABLE;
    if (sample_age_s > auto_tf_hold_timeout_s_)
      return TfLookupStatus::STALE;

    try
    {
      if (!lookupPosition(first_frame, sample_stamp, first_position) ||
          !lookupPosition(second_frame, sample_stamp, second_position))
      {
        return TfLookupStatus::UNAVAILABLE;
      }
    }
    catch (const tf::TransformException& exception)
    {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "Pico hand-track TF lookup failed at " << sample_stamp
                                                                       << ": " << exception.what());
      return TfLookupStatus::UNAVAILABLE;
    }
    return TfLookupStatus::VALID;
  }

  bool getLatestTime(const std::string& frame, ros::Time& latest_time)
  {
    if (frame == reference_frame_)
    {
      latest_time = ros::Time::now();
      return true;
    }

    std::string error;
    const int result =
        tf_listener_.getLatestCommonTime(reference_frame_, frame, latest_time, &error);
    if (result != tf::NO_ERROR)
    {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "Pico hand-track cannot resolve TF " << reference_frame_ << " -> "
                                                                     << frame << ": " << error);
      return false;
    }
    return true;
  }

  bool lookupPosition(const std::string& frame,
                      const ros::Time& stamp,
                      Position& position)
  {
    if (frame == reference_frame_)
    {
      position = Position{{0.0, 0.0, 0.0}};
      return true;
    }

    tf::StampedTransform transform;
    tf_listener_.lookupTransform(reference_frame_, frame, stamp, transform);
    const tf::Vector3& origin = transform.getOrigin();
    position = Position{{origin.x(), origin.y(), origin.z()}};
    return std::all_of(position.begin(), position.end(), [](const double value) {
      return std::isfinite(value);
    });
  }

  bool calculateLookAtTarget(const Position& head_origin,
                             const Position& hand_target,
                             double& yaw_deg,
                             double& pitch_deg) const
  {
    const double dx = hand_target[0] - head_origin[0];
    const double dy = hand_target[1] - head_origin[1];
    const double dz = hand_target[2] - head_origin[2];
    const double distance_xy = std::sqrt(dx * dx + dy * dy);
    const double distance = std::sqrt(distance_xy * distance_xy + dz * dz);
    if (!std::isfinite(distance) || distance < 1e-6)
      return false;

    yaw_deg = radToDeg(std::atan2(dy, dx));
    pitch_deg = -radToDeg(std::atan2(dz, distance_xy));
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg))
      return false;
    yaw_deg = clamp(yaw_deg, yaw_limit_[0], yaw_limit_[1]);
    pitch_deg = clamp(pitch_deg, pitch_limit_[0], pitch_limit_[1]);
    return true;
  }

  void handleTrackingUnavailable(const ros::Time& now, const bool already_stale)
  {
    if (!tracking_missing_)
    {
      tracking_missing_ = true;
      tracking_missing_since_ = now;
    }
    double missing_elapsed_s = (now - tracking_missing_since_).toSec();
    if (!std::isfinite(missing_elapsed_s) || missing_elapsed_s < 0.0)
    {
      ROS_WARN_THROTTLE(2.0,
                        "Pico auto-track tracking hold clock reset; restart missing timeout");
      tracking_missing_since_ = now;
      missing_elapsed_s = 0.0;
    }
    const bool timed_out = already_stale || missing_elapsed_s > auto_tf_hold_timeout_s_;
    if (timed_out)
    {
      ROS_ERROR_THROTTLE(2.0,
                         "Pico auto-track current-hand tracking TF timed out; clear owner and stop publishing");
      active_hand_selector_.reset(true);
      tracking_missing_ = false;
      auto_output_suppressed_ = true;
      publishActiveHand(false);
      publishMotionScores();
      return;
    }

    ROS_WARN_THROTTLE(2.0,
                      "Pico auto-track current-hand tracking TF unavailable; hold last head command");
    if (!auto_output_suppressed_ && have_published_command_)
      publishHeadCommand(command_yaw_, command_pitch_);
  }

  void handleFixedMainTrackingUnavailable(const ros::Time& now, const bool already_stale)
  {
    if (!tracking_missing_)
    {
      tracking_missing_ = true;
      tracking_missing_since_ = now;
    }
    double missing_elapsed_s = (now - tracking_missing_since_).toSec();
    if (!std::isfinite(missing_elapsed_s) || missing_elapsed_s < 0.0)
    {
      ROS_WARN_THROTTLE(
          2.0,
          "Pico fixed-main-hand tracking hold clock reset; restart missing timeout");
      tracking_missing_since_ = now;
      missing_elapsed_s = 0.0;
    }

    const bool timed_out = already_stale || missing_elapsed_s > auto_tf_hold_timeout_s_;
    if (timed_out)
    {
      ROS_ERROR_STREAM_THROTTLE(
          2.0,
          "Pico fixed-main-hand "
              << kuavo_xsense_gmr::activeHandName(fixed_main_hand_)
              << " tracking TF timed out; keep selected hand and stop publishing");
      fixed_main_output_suppressed_ = true;
      return;
    }

    ROS_WARN_STREAM_THROTTLE(
        2.0,
        "Pico fixed-main-hand "
            << kuavo_xsense_gmr::activeHandName(fixed_main_hand_)
            << " tracking TF unavailable; hold last head command");
    if (!fixed_main_output_suppressed_ && have_published_command_)
      publishHeadCommand(command_yaw_, command_pitch_);
  }

  bool setHeadControlMode(kuavo_msgs::SetHeadControlMode::Request& request,
                          kuavo_msgs::SetHeadControlMode::Response& response)
  {
    PicoHeadControlMode requested_mode = PicoHeadControlMode::VR_FOLLOW;
    if (!parseHeadControlMode(request.mode, requested_mode))
    {
      response.success = false;
      response.message = "invalid mode '" + request.mode +
                         "'; expected fixed, auto_track_active, fixed_main_hand or vr_follow";
      response.current_mode = headControlModeName(mode_);
      return true;
    }

    ActiveHand requested_fixed_main_hand = fixed_main_hand_;
    if (requested_mode == PicoHeadControlMode::FIXED_MAIN_HAND &&
        !parseFixedMainHand(request.fixed_hand, requested_fixed_main_hand))
    {
      response.success = false;
      response.message = "fixed_main_hand requires fixed_hand=left or right";
      response.current_mode = headControlModeName(mode_);
      return true;
    }

    const bool fixed_hand_changed =
        requested_mode == PicoHeadControlMode::FIXED_MAIN_HAND &&
        requested_fixed_main_hand != fixed_main_hand_;
    if (requested_mode == mode_ && !fixed_hand_changed)
    {
      response.success = true;
      response.message = "head control mode is already " + std::string(headControlModeName(mode_));
      if (mode_ == PicoHeadControlMode::FIXED_MAIN_HAND)
      {
        response.message += " with " +
                            std::string(kuavo_xsense_gmr::activeHandName(fixed_main_hand_));
      }
      response.current_mode = headControlModeName(mode_);
      return true;
    }

    const PicoHeadControlMode previous_mode = mode_;
    const ActiveHand previous_fixed_main_hand = fixed_main_hand_;
    if (requested_mode == PicoHeadControlMode::FIXED_MAIN_HAND)
      fixed_main_hand_ = requested_fixed_main_hand;
    mode_ = requested_mode;
    enterMode(mode_);
    private_nh_.setParam("head_control_mode", headControlModeName(mode_));
    private_nh_.setParam("head_fixed_main_hand",
                         kuavo_xsense_gmr::activeHandName(fixed_main_hand_));

    if (previous_mode == mode_ && fixed_hand_changed)
    {
      ROS_INFO_STREAM("Pico fixed-main-hand selection changed: "
                      << kuavo_xsense_gmr::activeHandName(previous_fixed_main_hand) << " -> "
                      << kuavo_xsense_gmr::activeHandName(fixed_main_hand_));
    }
    else
    {
      ROS_INFO_STREAM("Pico head control mode changed: " << headControlModeName(previous_mode)
                                                          << " -> " << headControlModeName(mode_));
    }
    response.success = true;
    response.message = "head control mode set to " + std::string(headControlModeName(mode_));
    if (mode_ == PicoHeadControlMode::FIXED_MAIN_HAND)
    {
      response.message += " with " +
                          std::string(kuavo_xsense_gmr::activeHandName(fixed_main_hand_));
    }
    response.current_mode = headControlModeName(mode_);
    return true;
  }

  void enterMode(const PicoHeadControlMode mode)
  {
    switch (mode)
    {
      case PicoHeadControlMode::FIXED:
        enterFixedMode();
        break;
      case PicoHeadControlMode::AUTO_TRACK_ACTIVE:
        enterAutoMode();
        break;
      case PicoHeadControlMode::FIXED_MAIN_HAND:
        enterFixedMainHandMode();
        break;
      case PicoHeadControlMode::VR_FOLLOW:
        enterVrMode();
        break;
    }
  }

  void prepareNonVrMode()
  {
    active_hand_selector_.reset(true);
    tracking_missing_ = false;
    auto_output_suppressed_ = false;
    fixed_main_output_suppressed_ = false;
    transitioning_to_vr_ = false;
    non_vr_filtered_yaw_ = have_published_command_ ? command_yaw_ : 0.0;
    non_vr_filtered_pitch_ = have_published_command_ ? command_pitch_ : 0.0;
    if (!have_published_command_)
    {
      command_yaw_ = 0.0;
      command_pitch_ = 0.0;
    }
    publishActiveHand(false);
    publishZeroMotionScores();
  }

  void enterFixedMode()
  {
    prepareNonVrMode();
  }

  void enterAutoMode()
  {
    prepareNonVrMode();
  }

  void enterFixedMainHandMode()
  {
    prepareNonVrMode();
  }

  void enterVrMode()
  {
    active_hand_selector_.reset(true);
    tracking_missing_ = false;
    auto_output_suppressed_ = false;
    fixed_main_output_suppressed_ = false;
    publishActiveHand(false);
    publishZeroMotionScores();
    if (have_published_command_)
    {
      // 等待下一帧新�? Pico 数据，再从最后基础命令限速过渡；
      // 保留�? offset 会在发布时只叠加一次�?
      vr_current_yaw_ = command_yaw_;
      vr_current_pitch_ = command_pitch_;
      have_valid_vr_pose_ = true;
      waiting_for_fresh_vr_pose_ = true;
      transitioning_to_vr_ = true;
    }
    else
    {
      have_valid_vr_pose_ = false;
      waiting_for_fresh_vr_pose_ = false;
      transitioning_to_vr_ = false;
    }
  }

  bool updateOutputClock(const ros::Time& now, double& dt)
  {
    if (!have_output_tick_time_)
    {
      last_output_tick_time_ = now;
      have_output_tick_time_ = true;
      return false;
    }

    dt = (now - last_output_tick_time_).toSec();
    last_output_tick_time_ = now;
    if (!std::isfinite(dt) || dt <= 0.0)
    {
      ROS_WARN_THROTTLE(2.0, "Pico head output clock moved backwards or did not advance; hold command");
      return false;
    }
    const double max_control_dt = 2.0 / publish_hz_;
    if (dt > max_control_dt)
    {
      ROS_WARN_THROTTLE(2.0, "Pico head output clock jumped forward; limit this control step");
      dt = max_control_dt;
    }
    return true;
  }

  static double moveTowards(const double current,
                            const double target,
                            const double max_step)
  {
    if (target > current)
      return std::min(target, current + max_step);
    return std::max(target, current - max_step);
  }

  void publishHeadCommand(const double yaw, const double pitch)
  {
    // Offset is applied only to the outgoing message.  command_yaw_/pitch_
    // retain the unoffset base command, otherwise hold/filter paths that feed
    // them back into this method would accumulate the same offset every tick.
    const double final_yaw = clamp(yaw + yaw_offset_,
                                   yaw_limit_[0],
                                   yaw_limit_[1]);
    const double final_pitch = clamp(pitch + pitch_offset_,
                                     pitch_limit_[0],
                                     pitch_limit_[1]);

    kuavo_msgs::robotHeadMotionData msg;
    // robotHeadMotionData 约定单位是度：joint_data[0]=yaw，joint_data[1]=pitch�?
    msg.joint_data = {final_yaw, final_pitch};
    head_pub_.publish(msg);
    command_yaw_ = yaw;
    command_pitch_ = pitch;
    have_published_command_ = true;
  }

  void publishActiveHand(const bool force)
  {
    const ActiveHand owner = diagnosticActiveHand();
    if (!force && owner == last_published_active_hand_)
      return;
    std_msgs::String msg;
    msg.data = kuavo_xsense_gmr::activeHandName(owner);
    active_hand_pub_.publish(msg);
    last_published_active_hand_ = owner;
  }

  ActiveHand diagnosticActiveHand() const
  {
    switch (mode_)
    {
      case PicoHeadControlMode::AUTO_TRACK_ACTIVE:
        return active_hand_selector_.owner();
      case PicoHeadControlMode::FIXED_MAIN_HAND:
        return fixed_main_hand_;
      case PicoHeadControlMode::FIXED:
      case PicoHeadControlMode::VR_FOLLOW:
        return ActiveHand::NONE;
    }
    return ActiveHand::NONE;
  }

  void publishMotionScores()
  {
    std_msgs::Float64MultiArray msg;
    msg.data = {active_hand_selector_.leftScoreMps(),
                active_hand_selector_.rightScoreMps()};
    motion_score_pub_.publish(msg);
  }

  void publishZeroMotionScores()
  {
    std_msgs::Float64MultiArray msg;
    msg.data = {0.0, 0.0};
    motion_score_pub_.publish(msg);
  }

  void logSelectorReset(const kuavo_xsense_gmr::SelectorUpdateResult result)
  {
    switch (result)
    {
      case kuavo_xsense_gmr::SelectorUpdateResult::RESET_INVALID:
        ROS_WARN_THROTTLE(2.0, "Pico auto-track reset motion window: invalid synchronized sample");
        break;
      case kuavo_xsense_gmr::SelectorUpdateResult::RESET_TIME_JUMP:
        ROS_WARN_THROTTLE(2.0, "Pico auto-track reset motion window: TF time jump or sample gap");
        break;
      case kuavo_xsense_gmr::SelectorUpdateResult::RESET_OVERSPEED:
        ROS_WARN_THROTTLE(2.0, "Pico auto-track reset motion window: implausible hand speed");
        break;
      case kuavo_xsense_gmr::SelectorUpdateResult::SEEDED:
      case kuavo_xsense_gmr::SelectorUpdateResult::UPDATED:
      case kuavo_xsense_gmr::SelectorUpdateResult::DUPLICATE:
        break;
    }
  }

  void logSelectorTransitions(const ActiveHand owner_before,
                              const ActiveHand candidate_before,
                              const kuavo_xsense_gmr::CandidateReason reason_before)
  {
    const ActiveHand owner_after = active_hand_selector_.owner();
    if (owner_before != owner_after)
    {
      ROS_INFO_STREAM("Pico auto-track active hand changed: "
                      << kuavo_xsense_gmr::activeHandName(owner_before) << " -> "
                      << kuavo_xsense_gmr::activeHandName(owner_after));
    }

    const ActiveHand candidate_after = active_hand_selector_.candidateHand();
    const kuavo_xsense_gmr::CandidateReason reason_after =
        active_hand_selector_.candidateReason();
    if (candidate_before != candidate_after || reason_before != reason_after)
    {
      ROS_INFO_STREAM_THROTTLE(1.0,
                               "Pico auto-track candidate: hand="
                                   << kuavo_xsense_gmr::activeHandName(candidate_after)
                                   << ", reason="
                                   << kuavo_xsense_gmr::candidateReasonName(reason_after));
    }
  }

  ros::NodeHandle private_nh_;
  ros::Subscriber body_matrices_sub_;
  ros::Publisher head_pub_;
  ros::Publisher active_hand_pub_;
  ros::Publisher motion_score_pub_;
  ros::ServiceServer mode_service_;
  ros::Timer publish_timer_;
  tf::TransformListener tf_listener_;
  kuavo_xsense_gmr::ActiveHandSelector active_hand_selector_;

  std::string body_matrices_topic_;
  std::string head_output_topic_;
  std::string reference_frame_;
  std::string head_origin_frame_;
  std::string left_motion_frame_;
  std::string right_motion_frame_;
  std::string left_target_frame_;
  std::string right_target_frame_;
  PicoHeadControlMode mode_ = PicoHeadControlMode::VR_FOLLOW;
  ActiveHand fixed_main_hand_ = ActiveHand::RIGHT;
  double publish_hz_ = 50.0;
  double stale_timeout_s_ = 0.5;
  double smoothing_factor_ = 0.2;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;
  double auto_tf_hold_timeout_s_ = 0.5;
  double auto_smoothing_tau_s_ = 0.12;
  double yaw_rate_limit_dps_ = 60.0;
  double pitch_rate_limit_dps_ = 45.0;
  std::vector<double> yaw_limit_{-30.0, 30.0};
  std::vector<double> pitch_limit_{-25.0, 25.0};
  bool have_valid_vr_pose_ = false;
  bool waiting_for_fresh_vr_pose_ = false;
  bool transitioning_to_vr_ = false;
  bool have_published_command_ = false;
  bool tracking_missing_ = false;
  bool auto_output_suppressed_ = false;
  bool fixed_main_output_suppressed_ = false;
  bool have_output_tick_time_ = false;
  double vr_current_yaw_ = 0.0;
  double vr_current_pitch_ = 0.0;
  double command_yaw_ = 0.0;
  double command_pitch_ = 0.0;
  double non_vr_filtered_yaw_ = 0.0;
  double non_vr_filtered_pitch_ = 0.0;
  ros::Time last_valid_vr_time_;
  ros::Time tracking_missing_since_;
  ros::Time last_output_tick_time_;
  ActiveHand last_published_active_hand_ = ActiveHand::NONE;

  // --- 头部 pitch/yaw 偏移量微�? ---
  bool head_pitch_offset_enabled_ = true;
  double pitch_offset_ = 0.0;
  double yaw_offset_ = 0.0;
  double head_pitch_offset_rate_dps_ = 3.0;
  std::vector<double> head_pitch_offset_limit_{-20.0, 20.0};
  double head_pitch_offset_trigger_threshold_ = 0.7;
  double head_pitch_offset_stick_threshold_ = 0.7;
  double head_pitch_offset_pitch_sign_ = 1.0;  // 正负=方向，绝对�?=速度倍率
  double head_yaw_offset_rate_dps_ = 3.0;
  std::vector<double> head_yaw_offset_limit_{-20.0, 20.0};
  double head_yaw_offset_yaw_sign_ = 1.0;  // 正负=方向，绝对�?=速度倍率
  double head_offset_zero_stick_threshold_ = 0.8;
  double head_offset_joy_timeout_s_ = 0.5;

  // joy 订阅与缓存状�?
  ros::Subscriber joy_sub_;
  bool have_joy_ = false;
  SteadyClock::time_point last_offset_joy_time_{};
  double joy_left_trigger_ = 0.0;
  double joy_left_grip_ = 0.0;
  double joy_right_trigger_ = 0.0;
  double joy_right_grip_ = 0.0;
  double joy_left_x_ = 0.0;
  double joy_left_y_ = 0.0;
  double joy_right_x_ = 0.0;
  double joy_right_y_ = 0.0;
  bool offset_active_ = false;
};

class XsensePicoControlNode
{
public:
  XsensePicoControlNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<bool>("enable_head_control", enable_head_control_, false);
    private_nh.param<bool>("enable_claw_control", enable_claw_control_, false);

    if (enable_head_control_)
      head_controller_.reset(new XsensePicoHeadController(nh, private_nh));

    if (enable_claw_control_)
    {
      const EndEffectorSelection selection = resolveEndEffectorSelection(private_nh);
      if (selection.source == "launch_explicit" &&
          selection.mode != EndEffectorControlMode::INVALID)
      {
        ROS_WARN_STREAM("Using explicit Pico end-effector type from launch: "
                        << selection.normalized_type
                        << "; /kuavo_configuration will not determine the end-effector mode");
      }
      else if (selection.source == "kuavo_configuration" &&
               selection.mode != EndEffectorControlMode::INVALID)
      {
        ROS_INFO_STREAM("Loaded EndEffectorType from /kuavo_configuration: ["
                        << selection.normalized_type << ", " << selection.normalized_type << "]");
      }

      if (selection.mode == EndEffectorControlMode::CLAW ||
          selection.mode == EndEffectorControlMode::DEXHAND)
      {
        ROS_INFO_STREAM("Pico end-effector mode selected: "
                        << endEffectorModeName(selection.mode)
                        << ", source=" << selection.source
                        << ", type=" << selection.normalized_type);
        end_effector_controller_.reset(
            new PicoEndEffectorController(nh, private_nh, selection.normalized_type));
      }
      else if (selection.mode == EndEffectorControlMode::NONE)
      {
        ROS_WARN_STREAM("Pico end-effector control remains disabled: source="
                        << selection.source << ", type=none");
      }
      else
      {
        if (selection.source == "kuavo_configuration")
        {
          ROS_ERROR_STREAM(selection.error
                           << "; Pico end-effector control was not started. "
                              "Use a complete system launch or pass pico_end_effector_type "
                              "explicitly when launching xsense_udp_bvh_to_gmr.launch");
        }
        else
        {
          ROS_ERROR_STREAM(selection.error
                           << "; Pico end-effector control was not started");
        }
      }
    }
    const char* end_effector_status = !enable_claw_control_
                                          ? "disabled"
                                          : (end_effector_controller_ ? "enabled" : "not_started");
    ROS_INFO_STREAM("Pico control node started: head="
                    << (enable_head_control_ ? "enabled" : "disabled")
                    << ", end_effector=" << end_effector_status);
  }

private:
  bool enable_head_control_ = false;
  bool enable_claw_control_ = false;
  std::unique_ptr<PicoEndEffectorController> end_effector_controller_;
  std::unique_ptr<XsensePicoHeadController> head_controller_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xsense_pico_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  try
  {
    XsensePicoControlNode node(nh, private_nh);
    (void)node;  // keep node alive for callbacks while spinning
    ros::spin();
    return 0;
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Failed to start xsense_pico_control: " << exception.what());
    return 1;
  }
}
