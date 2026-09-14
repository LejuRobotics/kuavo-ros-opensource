#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <kuavo_msgs/JoySticks.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace
{

double clampFinite(const double value, const double low, const double high)
{
  if (!std::isfinite(value))
    return 0.0;
  return std::max(low, std::min(high, value));
}

class PicoJoyConverter
{
public:
  PicoJoyConverter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("input_topic", input_topic_, "/pico/joy");
    private_nh.param<std::string>("gamepad_input_topic", gamepad_input_topic_, "/joy");
    private_nh.param<std::string>("output_topic", output_topic_, "/pico/joy_converted");
    private_nh.param<double>("gamepad_axis_deadzone", gamepad_axis_deadzone_, 0.05);
    private_nh.param<double>("gamepad_trigger_pressed_threshold",
                             gamepad_trigger_pressed_threshold_,
                             -0.5);
    private_nh.param<double>("gamepad_input_timeout_s",
                             gamepad_input_timeout_s_,
                             0.2);
    private_nh.param<double>("controller_switch_grip_threshold",
                             controller_switch_grip_threshold_,
                             0.5);
    private_nh.param<double>("calibration_trigger_threshold",
                             calibration_trigger_threshold_,
                             0.5);
    private_nh.param<bool>("suppress_controller_switch_buttons",
                           suppress_controller_switch_buttons_,
                           true);
    private_nh.param<bool>("suppress_calibration_buttons",
                           suppress_calibration_buttons_,
                           true);
    private_nh.param<bool>("control_node_enabled", control_node_enabled_, true);
    private_nh.param<bool>("head_control_enabled", head_control_enabled_, false);
    private_nh.param<bool>("head_offset_enabled", head_offset_enabled_, true);
    private_nh.param<double>("head_offset_trigger_threshold",
                             head_offset_trigger_threshold_,
                             0.7);
    private_nh.param<double>("head_offset_release_threshold",
                             head_offset_release_threshold_,
                             0.5);
    private_nh.param<double>("head_offset_stick_center_deadzone",
                             head_offset_stick_center_deadzone_,
                             0.05);
    private_nh.param<int>("head_offset_center_stable_frames",
                          head_offset_center_stable_frames_,
                          2);
    if (!private_nh.getParam("gait_a_short_press_max_s", gait_a_short_press_max_s_))
    {
      private_nh.param<double>("gait_a_long_press_s", gait_a_short_press_max_s_, 0.5);
      if (private_nh.hasParam("gait_a_long_press_s"))
      {
        ROS_WARN("~gait_a_long_press_s is deprecated; use ~gait_a_short_press_max_s");
      }
    }
    private_nh.param<double>("gait_a_long_press_min_s", gait_a_long_press_min_s_, 0.8);
    private_nh.param<double>("input_timeout_s", input_timeout_s_, 0.5);
    private_nh.param<double>("neutral_publish_hz", neutral_publish_hz_, 20.0);

    if (!std::isfinite(controller_switch_grip_threshold_))
      throw std::invalid_argument("controller_switch_grip_threshold must be finite");
    if (!std::isfinite(calibration_trigger_threshold_))
      throw std::invalid_argument("calibration_trigger_threshold must be finite");
    if (!std::isfinite(head_offset_trigger_threshold_))
      throw std::invalid_argument("head_offset_trigger_threshold must be finite");
    if (!std::isfinite(head_offset_release_threshold_))
      throw std::invalid_argument("head_offset_release_threshold must be finite");
    if (!std::isfinite(head_offset_stick_center_deadzone_) ||
        head_offset_stick_center_deadzone_ <= 0.0)
    {
      throw std::invalid_argument(
          "head_offset_stick_center_deadzone must be finite and > 0");
    }
    if (head_offset_center_stable_frames_ < 1)
      throw std::invalid_argument("head_offset_center_stable_frames must be >= 1");
    if (!std::isfinite(gait_a_short_press_max_s_) || gait_a_short_press_max_s_ <= 0.0)
      throw std::invalid_argument("gait_a_short_press_max_s must be finite and > 0");
    if (!std::isfinite(gait_a_long_press_min_s_) || gait_a_long_press_min_s_ <= 0.0)
      throw std::invalid_argument("gait_a_long_press_min_s must be finite and > 0");
    if (gait_a_short_press_max_s_ >= gait_a_long_press_min_s_)
    {
      throw std::invalid_argument(
          "gait_a_short_press_max_s must be < gait_a_long_press_min_s");
    }
    if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0)
      throw std::invalid_argument("input_timeout_s must be finite and > 0");
    if (!std::isfinite(gamepad_axis_deadzone_) || gamepad_axis_deadzone_ <= 0.0 ||
        gamepad_axis_deadzone_ >= 1.0)
    {
      throw std::invalid_argument("gamepad_axis_deadzone must be finite and in (0, 1)");
    }
    if (!std::isfinite(gamepad_trigger_pressed_threshold_) ||
        gamepad_trigger_pressed_threshold_ <= -1.0 ||
        gamepad_trigger_pressed_threshold_ > 0.0)
    {
      throw std::invalid_argument(
          "gamepad_trigger_pressed_threshold must be finite and in (-1, 0]");
    }
    if (!std::isfinite(gamepad_input_timeout_s_) || gamepad_input_timeout_s_ <= 0.0)
      throw std::invalid_argument("gamepad_input_timeout_s must be finite and > 0");
    if (!std::isfinite(neutral_publish_hz_) || neutral_publish_hz_ <= 0.0)
      throw std::invalid_argument("neutral_publish_hz must be finite and > 0");
    controller_switch_grip_threshold_ =
        std::max(0.0, std::min(1.0, controller_switch_grip_threshold_));
    calibration_trigger_threshold_ =
        std::max(0.0, std::min(1.0, calibration_trigger_threshold_));
    head_offset_trigger_threshold_ =
        std::max(0.0, std::min(1.0, head_offset_trigger_threshold_));
    head_offset_release_threshold_ =
        std::max(0.0, std::min(1.0, head_offset_release_threshold_));
    head_offset_stick_center_deadzone_ =
        std::max(0.0, std::min(1.0, head_offset_stick_center_deadzone_));
    if (head_offset_release_threshold_ >= head_offset_trigger_threshold_)
    {
      throw std::invalid_argument(
          "head_offset_release_threshold must be < head_offset_trigger_threshold");
    }

    const std::string resolved_input_topic = nh.resolveName(input_topic_);
    const std::string resolved_gamepad_topic = nh.resolveName(gamepad_input_topic_);
    const std::string resolved_output_topic = nh.resolveName(output_topic_);
    if (resolved_input_topic == resolved_gamepad_topic ||
        resolved_input_topic == resolved_output_topic ||
        resolved_gamepad_topic == resolved_output_topic)
    {
      throw std::invalid_argument(
          "input_topic, gamepad_input_topic, and output_topic must resolve to distinct names");
    }

    joy_pub_ = nh.advertise<sensor_msgs::Joy>(output_topic_, 10);
    pico_joy_sub_ =
        nh.subscribe(input_topic_, 10, &PicoJoyConverter::picoJoyCallback, this);
    gamepad_joy_sub_ = nh.subscribe(gamepad_input_topic_,
                                    10,
                                    &PicoJoyConverter::gamepadJoyCallback,
                                    this);
    // Stale-input protection must keep running even when /use_sim_time is
    // enabled but MuJoCo's /clock is paused or has not started yet.
    neutral_timer_ = nh.createWallTimer(ros::WallDuration(1.0 / neutral_publish_hz_),
                                        &PicoJoyConverter::neutralTimerCallback,
                                        this);
    ROS_INFO_STREAM(
        "Pico/gamepad joy selector: Pico="
        << input_topic_ << ", gamepad=" << gamepad_input_topic_ << " -> " << output_topic_
        << ", gamepad axis deadzone=" << gamepad_axis_deadzone_
        << ", gamepad trigger threshold=" << gamepad_trigger_pressed_threshold_
        << ", gamepad timeout=" << gamepad_input_timeout_s_ << "s"
        << ", controller-combo suppression="
        << (suppress_controller_switch_buttons_ ? "enabled" : "disabled")
        << ", calibration-combo suppression="
        << (suppress_calibration_buttons_ ? "enabled" : "disabled")
        << ", head-offset axes suppression="
        << ((control_node_enabled_ && head_control_enabled_ && head_offset_enabled_)
                ? "enabled"
                : "disabled")
        << ", head-offset release threshold=" << head_offset_release_threshold_
        << ", stick center deadzone=" << head_offset_stick_center_deadzone_
        << ", center stable frames=" << head_offset_center_stable_frames_
        << ", gait A short press max=" << gait_a_short_press_max_s_ << "s"
        << ", gait A reserved long press min=" << gait_a_long_press_min_s_ << "s"
        << ", input_timeout=" << input_timeout_s_ << "s");
  }

private:
  static constexpr std::size_t kBt2AxisCount = 8;
  static constexpr std::size_t kBt2ButtonCount = 11;
  static constexpr std::size_t kBt2ProButtonCount = 16;
  static constexpr double kAxisRangeTolerance = 1e-3;

  static sensor_msgs::Joy makeNeutralJoy()
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.axes.assign(kBt2AxisCount, 0.0F);
    joy.buttons.assign(kBt2ButtonCount, 0);
    return joy;
  }

  bool validateGamepadJoy(const sensor_msgs::Joy& gamepad) const
  {
    if (gamepad.axes.size() != kBt2AxisCount ||
        (gamepad.buttons.size() != kBt2ButtonCount &&
         gamepad.buttons.size() != kBt2ProButtonCount))
    {
      ROS_WARN_THROTTLE(2.0,
                        "Ignore gamepad Joy with unexpected layout: axes=%zu, buttons=%zu; "
                        "Pico AMP fusion supports BT2 8/11 or BT2Pro 8/16",
                        gamepad.axes.size(),
                        gamepad.buttons.size());
      return false;
    }

    for (const float axis : gamepad.axes)
    {
      if (!std::isfinite(axis) ||
          axis < -1.0F - static_cast<float>(kAxisRangeTolerance) ||
          axis > 1.0F + static_cast<float>(kAxisRangeTolerance))
      {
        ROS_WARN_THROTTLE(2.0, "Ignore gamepad Joy with a non-finite/out-of-range axis");
        return false;
      }
    }
    for (const int button : gamepad.buttons)
    {
      if (button != 0 && button != 1)
      {
        ROS_WARN_THROTTLE(2.0, "Ignore gamepad Joy with a button value other than 0/1");
        return false;
      }
    }
    return true;
  }

  sensor_msgs::Joy normalizeGamepadJoy(const sensor_msgs::Joy& gamepad) const
  {
    sensor_msgs::Joy output = makeNeutralJoy();
    output.header = gamepad.header;
    output.header.stamp = ros::Time::now();

    if (gamepad.buttons.size() == kBt2ButtonCount)
    {
      output.axes = gamepad.axes;
      output.buttons = gamepad.buttons;
    }
    else
    {
      // Keep /pico/joy_converted in the canonical BT2 8-axis/11-button
      // layout.  BT2Pro has the same semantic controls at different physical
      // indices; copying or truncating its arrays would turn LB into BACK and
      // mix the right stick with LT/RT.
      constexpr std::size_t bt2pro_axis_indices[kBt2AxisCount] = {
          0, 1, 5, 2, 3, 4, 6, 7};
      constexpr std::size_t bt2pro_button_indices[kBt2ButtonCount] = {
          0, 1, 3, 4, 6, 7, 10, 11, 12, 13, 14};

      for (std::size_t output_index = 0; output_index < kBt2AxisCount;
           ++output_index)
      {
        output.axes[output_index] = gamepad.axes[bt2pro_axis_indices[output_index]];
      }
      for (std::size_t output_index = 0; output_index < kBt2ButtonCount;
           ++output_index)
      {
        output.buttons[output_index] =
            gamepad.buttons[bt2pro_button_indices[output_index]];
      }
    }

    for (float& axis : output.axes)
      axis = std::max(-1.0F, std::min(1.0F, axis));
    return output;
  }

  bool gamepadHasIntent(const sensor_msgs::Joy& gamepad) const
  {
    if (std::any_of(gamepad.buttons.begin(), gamepad.buttons.end(),
                    [](const int button) { return button != 0; }))
    {
      return true;
    }

    // BT2 axes: left stick 0/1, LT 2, right stick 3/4, RT 5,
    // D-pad 6/7.  LT/RT rest at either 0 or +1 depending on driver state and
    // are pressed towards -1, so they need semantic handling rather than an
    // absolute-value deadzone.
    constexpr std::size_t motion_axes[] = {0, 1, 3, 4, 6, 7};
    for (const std::size_t index : motion_axes)
    {
      if (std::abs(gamepad.axes[index]) > gamepad_axis_deadzone_)
        return true;
    }
    return gamepad.axes[2] < gamepad_trigger_pressed_threshold_ ||
           gamepad.axes[5] < gamepad_trigger_pressed_threshold_;
  }

  bool gamepadOwnsOutput(const std::chrono::steady_clock::time_point& now) const
  {
    return have_gamepad_input_ && gamepad_active_ &&
           std::chrono::duration<double>(now - last_gamepad_input_time_).count() <
               gamepad_input_timeout_s_;
  }

  bool expireStaleGamepad(const std::chrono::steady_clock::time_point& now)
  {
    if (!have_gamepad_input_ ||
        std::chrono::duration<double>(now - last_gamepad_input_time_).count() <
            gamepad_input_timeout_s_)
    {
      return false;
    }

    have_gamepad_input_ = false;
    if (!gamepad_active_)
      return false;

    gamepad_active_ = false;
    joy_pub_.publish(makeNeutralJoy());
    ROS_WARN_THROTTLE(
        5.0,
        "Active gamepad input is stale; release override and publish neutral AMP Joy");
    return true;
  }

  void gamepadJoyCallback(const sensor_msgs::Joy::ConstPtr& gamepad)
  {
    if (!gamepad)
      return;

    if (!validateGamepadJoy(*gamepad))
    {
      have_gamepad_input_ = false;
      if (gamepad_active_)
      {
        gamepad_active_ = false;
        joy_pub_.publish(makeNeutralJoy());
        ROS_WARN("Invalid active gamepad frame; release override and publish neutral Joy");
      }
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool was_active = gamepad_active_;
    const sensor_msgs::Joy normalized_gamepad = normalizeGamepadJoy(*gamepad);
    last_gamepad_input_time_ = now;
    have_gamepad_input_ = true;
    gamepad_active_ = gamepadHasIntent(normalized_gamepad);

    if (gamepad_active_)
    {
      joy_pub_.publish(normalized_gamepad);
      if (!was_active)
      {
        ROS_INFO("Gamepad input active (%s); gamepad takes Pico AMP Joy priority",
                 gamepad->buttons.size() == kBt2ProButtonCount ? "BT2Pro 8/16"
                                                               : "BT2 8/11");
      }
      return;
    }

    if (was_active)
    {
      // Do not replay a cached Pico one-shot when the gamepad is released.
      // Emit one neutral handoff frame; the next fresh Pico callback resumes
      // Pico control, normally within one Pico publishing period.
      joy_pub_.publish(makeNeutralJoy());
      ROS_INFO("Gamepad returned to neutral; hand AMP Joy priority back to Pico");
    }
  }

  void picoJoyCallback(const kuavo_msgs::JoySticks::ConstPtr& pico)
  {
    if (!pico)
      return;

    const auto now = std::chrono::steady_clock::now();
    // The WallTimer normally expires a stale gamepad.  Repeat the check here
    // so a Pico callback arriving between the timeout boundary and the next
    // timer tick cannot skip the neutral handoff frame.
    const bool gamepad_timeout_handoff = expireStaleGamepad(now);
    if (have_pico_input_ &&
        std::chrono::duration<double>(now - last_pico_input_time_).count() >=
            input_timeout_s_)
    {
      resetTransientInputState();
      ROS_WARN_THROTTLE(
          2.0,
          "Pico Joy sample gap interrupted pending AMP button gestures; wait for release");
    }
    last_pico_input_time_ = now;
    have_pico_input_ = true;

    sensor_msgs::Joy joy = makeNeutralJoy();

    // HumanoidAutoGaitJoyCommandNodeWithVel interprets the bt2 axes as
    // [lateral, forward, LT, yaw, height, RT, dpad_lr, dpad_fb].  Match the
    // established Pico cmd_vel signs: forward=left_y, lateral=-left_x,
    // yaw=-right_x.  Auxiliary axes remain neutral because this bridge is for
    // AMP walking; forwarding Pico triggers/right_y would also enter the
    // gamepad's head, waist, or height-control paths.
    const double left_trigger = clampFinite(pico->left_trigger, 0.0, 1.0);
    const double left_grip = clampFinite(pico->left_grip, 0.0, 1.0);
    const double right_trigger = clampFinite(pico->right_trigger, 0.0, 1.0);
    const double right_grip = clampFinite(pico->right_grip, 0.0, 1.0);
    const double left_x = clampFinite(pico->left_x, -1.0, 1.0);
    const double left_y = clampFinite(pico->left_y, -1.0, 1.0);
    const double right_x = clampFinite(pico->right_x, -1.0, 1.0);
    const double right_y = clampFinite(pico->right_y, -1.0, 1.0);
    const bool head_offset_modifier_inputs_valid =
        std::isfinite(pico->left_trigger) && std::isfinite(pico->left_grip) &&
        std::isfinite(pico->right_trigger) && std::isfinite(pico->right_grip);
    const bool stick_inputs_valid =
        std::isfinite(pico->left_x) && std::isfinite(pico->left_y) &&
        std::isfinite(pico->right_x) && std::isfinite(pico->right_y);
    const bool head_offset_protection_enabled =
        control_node_enabled_ && head_control_enabled_ && head_offset_enabled_;
    const bool head_offset_modifier_pressed =
        head_offset_protection_enabled && head_offset_modifier_inputs_valid &&
        left_trigger >= head_offset_trigger_threshold_ &&
        left_grip >= head_offset_trigger_threshold_ &&
        right_trigger >= head_offset_trigger_threshold_ &&
        right_grip >= head_offset_trigger_threshold_;

    if (head_offset_modifier_pressed && !head_offset_walk_interlock_active_)
    {
      head_offset_walk_interlock_active_ = true;
      head_offset_centered_frames_ = 0;
      ROS_INFO("Pico AMP walking interlock engaged by head-offset modifier");
    }

    // When head-offset protection is enabled, malformed modifier or stick
    // input is not safe to interpret as an AMP walking command.  Fail closed
    // for this frame even if the interlock was not already active.
    const bool head_offset_input_invalid =
        head_offset_protection_enabled &&
        (!head_offset_modifier_inputs_valid || !stick_inputs_valid);
    const bool suppress_walking_axes =
        head_offset_walk_interlock_active_ || head_offset_input_invalid;
    if (head_offset_walk_interlock_active_)
    {
      const bool head_offset_modifier_released =
          head_offset_modifier_inputs_valid &&
          (left_trigger < head_offset_release_threshold_ ||
           left_grip < head_offset_release_threshold_ ||
           right_trigger < head_offset_release_threshold_ ||
           right_grip < head_offset_release_threshold_);
      const bool sticks_centered =
          stick_inputs_valid &&
          std::abs(left_x) < head_offset_stick_center_deadzone_ &&
          std::abs(left_y) < head_offset_stick_center_deadzone_ &&
          std::abs(right_x) < head_offset_stick_center_deadzone_ &&
          std::abs(right_y) < head_offset_stick_center_deadzone_;

      if (head_offset_modifier_released && sticks_centered)
        ++head_offset_centered_frames_;
      else
        head_offset_centered_frames_ = 0;

      if (head_offset_centered_frames_ >= head_offset_center_stable_frames_)
      {
        head_offset_walk_interlock_active_ = false;
        head_offset_centered_frames_ = 0;
        ROS_INFO("Pico AMP walking interlock released after stable centered input");
      }
    }

    if (!suppress_walking_axes)
    {
      joy.axes[0] = static_cast<float>(-left_x);
      joy.axes[1] = static_cast<float>(left_y);
      joy.axes[3] = static_cast<float>(-right_x);
    }

    // X/Y deliberately remain unmapped.  A/B gait actions are suppressed for
    // their Pico combo for the entire physical button press, so releasing the
    // modifier first cannot leak a delayed gait edge.
    const bool controller_modifier_pressed =
        clampFinite(pico->right_grip, 0.0, 1.0) >= controller_switch_grip_threshold_;
    const bool calibration_modifier_pressed =
        clampFinite(pico->left_trigger, 0.0, 1.0) >= calibration_trigger_threshold_ ||
        clampFinite(pico->right_trigger, 0.0, 1.0) >= calibration_trigger_threshold_;
    const bool a_pressed = pico->right_first_button_pressed;
    const bool b_pressed = pico->right_second_button_pressed;
    bool a_short_press = false;

    if (!a_gesture_armed_)
    {
      if (!a_pressed)
      {
        a_gesture_armed_ = true;
        previous_a_pressed_ = false;
        suppress_a_until_release_ = false;
        have_a_press_start_time_ = false;
      }
    }
    else if (a_pressed && !previous_a_pressed_)
    {
      a_press_start_time_ = now;
      have_a_press_start_time_ = true;
    }
    if (a_gesture_armed_ && a_pressed && suppress_controller_switch_buttons_ &&
        controller_modifier_pressed)
    {
      suppress_a_until_release_ = true;
    }

    if (a_gesture_armed_ && !a_pressed)
    {
      if (previous_a_pressed_ && have_a_press_start_time_ && !suppress_a_until_release_)
      {
        const double held_s =
            std::chrono::duration<double>(now - a_press_start_time_).count();
        a_short_press = held_s < gait_a_short_press_max_s_;
      }
      suppress_a_until_release_ = false;
      have_a_press_start_time_ = false;
    }

    if (!b_pressed)
      suppress_b_until_release_ = false;
    else if ((suppress_controller_switch_buttons_ && controller_modifier_pressed) ||
             (suppress_calibration_buttons_ && calibration_modifier_pressed))
      suppress_b_until_release_ = true;

    // A short press selects stance on release.  Holding A for the configured
    // long-press duration is reserved for the existing right-thumb auxiliary
    // action and therefore must not also select a gait.
    joy.buttons[0] = static_cast<int>(a_short_press);
    joy.buttons[1] = static_cast<int>(b_pressed && !suppress_b_until_release_);
    previous_a_pressed_ = a_gesture_armed_ && a_pressed;

    if (!gamepad_timeout_handoff && !gamepadOwnsOutput(now))
      joy_pub_.publish(joy);
  }

  void neutralTimerCallback(const ros::WallTimerEvent&)
  {
    const auto now = std::chrono::steady_clock::now();
    const bool gamepad_timeout_handoff = expireStaleGamepad(now);

    const bool pico_input_stale =
        !have_pico_input_ ||
        std::chrono::duration<double>(now - last_pico_input_time_).count() >=
            input_timeout_s_;
    if (!pico_input_stale)
      return;

    // Discard any incomplete button gesture.  If the stream resumes while a
    // button is still physically held, keep it suppressed until release; a
    // reconnect must never synthesize a short-press gait command.
    have_pico_input_ = false;
    resetTransientInputState();

    if (gamepad_timeout_handoff || gamepadOwnsOutput(now))
      return;

    // AutoGait caches the last Joy command.  Continue publishing a neutral
    // frame when Pico data is absent so switching to this source, or losing
    // the Pico stream while walking, cannot leave an old non-zero command.
    joy_pub_.publish(makeNeutralJoy());
    ROS_WARN_THROTTLE(5.0, "Pico Joy input is stale; publishing neutral AMP Joy");
  }

  void resetTransientInputState()
  {
    head_offset_centered_frames_ = 0;
    a_gesture_armed_ = false;
    previous_a_pressed_ = true;
    have_a_press_start_time_ = false;
    suppress_a_until_release_ = true;
    suppress_b_until_release_ = true;
  }

  ros::Publisher joy_pub_;
  ros::Subscriber pico_joy_sub_;
  ros::Subscriber gamepad_joy_sub_;
  ros::WallTimer neutral_timer_;
  std::string input_topic_;
  std::string gamepad_input_topic_;
  std::string output_topic_;
  double controller_switch_grip_threshold_ = 0.5;
  double calibration_trigger_threshold_ = 0.5;
  double head_offset_trigger_threshold_ = 0.7;
  double head_offset_release_threshold_ = 0.5;
  double head_offset_stick_center_deadzone_ = 0.05;
  int head_offset_center_stable_frames_ = 2;
  int head_offset_centered_frames_ = 0;
  double gait_a_short_press_max_s_ = 0.5;
  double gait_a_long_press_min_s_ = 0.8;
  double input_timeout_s_ = 0.5;
  double gamepad_axis_deadzone_ = 0.05;
  double gamepad_trigger_pressed_threshold_ = -0.5;
  double gamepad_input_timeout_s_ = 0.2;
  double neutral_publish_hz_ = 20.0;
  bool suppress_controller_switch_buttons_ = true;
  bool suppress_calibration_buttons_ = true;
  bool control_node_enabled_ = true;
  bool head_control_enabled_ = false;
  bool head_offset_enabled_ = true;
  bool head_offset_walk_interlock_active_ = false;
  bool have_pico_input_ = false;
  std::chrono::steady_clock::time_point last_pico_input_time_;
  bool have_gamepad_input_ = false;
  bool gamepad_active_ = false;
  std::chrono::steady_clock::time_point last_gamepad_input_time_;
  bool previous_a_pressed_ = false;
  bool a_gesture_armed_ = false;
  bool have_a_press_start_time_ = false;
  std::chrono::steady_clock::time_point a_press_start_time_;
  bool suppress_a_until_release_ = false;
  bool suppress_b_until_release_ = false;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pico_joy_converter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try
  {
    PicoJoyConverter converter(nh, private_nh);
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Failed to start Pico joy converter: " << exception.what());
    return 1;
  }
  return 0;
}
