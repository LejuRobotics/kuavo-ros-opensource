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

class PicoGmrJoyConverter
{
public:
  PicoGmrJoyConverter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("input_topic", input_topic_, "/pico/joy");
    private_nh.param<std::string>("g12_input_topic", g12_input_topic_, "/joy");
    private_nh.param<std::string>("output_topic", output_topic_, "/pico/joy_converted");
    private_nh.param<double>("g12_axis_deadzone", g12_axis_deadzone_, 0.05);
    private_nh.param<double>("g12_input_timeout_s", g12_input_timeout_s_, 0.2);
    private_nh.param<double>("input_timeout_s", input_timeout_s_, 0.5);
    private_nh.param<double>("neutral_publish_hz", neutral_publish_hz_, 20.0);

    if (!std::isfinite(g12_axis_deadzone_) || g12_axis_deadzone_ <= 0.0 ||
        g12_axis_deadzone_ >= 1.0)
    {
      throw std::invalid_argument("g12_axis_deadzone must be finite and in (0, 1)");
    }
    if (!std::isfinite(g12_input_timeout_s_) || g12_input_timeout_s_ <= 0.0)
      throw std::invalid_argument("g12_input_timeout_s must be finite and > 0");
    if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0)
      throw std::invalid_argument("input_timeout_s must be finite and > 0");
    if (!std::isfinite(neutral_publish_hz_) || neutral_publish_hz_ <= 0.0)
      throw std::invalid_argument("neutral_publish_hz must be finite and > 0");

    const std::string resolved_input_topic = nh.resolveName(input_topic_);
    const std::string resolved_g12_topic = nh.resolveName(g12_input_topic_);
    const std::string resolved_output_topic = nh.resolveName(output_topic_);
    if (resolved_input_topic == resolved_g12_topic ||
        resolved_input_topic == resolved_output_topic ||
        resolved_g12_topic == resolved_output_topic)
    {
      throw std::invalid_argument(
          "input_topic, g12_input_topic, and output_topic must resolve to distinct names");
    }

    joy_pub_ = nh.advertise<sensor_msgs::Joy>(output_topic_, 10);
    pico_joy_sub_ =
        nh.subscribe(input_topic_, 10, &PicoGmrJoyConverter::picoJoyCallback, this);
    g12_joy_sub_ =
        nh.subscribe(g12_input_topic_, 10, &PicoGmrJoyConverter::g12JoyCallback, this);
    // Use wall time so stale-input protection continues while simulation time
    // is paused or before a /clock publisher becomes available.
    neutral_timer_ = nh.createWallTimer(ros::WallDuration(1.0 / neutral_publish_hz_),
                                        &PicoGmrJoyConverter::neutralTimerCallback,
                                        this);

    ROS_INFO_STREAM("Pico GMR AMP joy selector: Pico="
                    << input_topic_ << ", G12=" << g12_input_topic_ << " -> "
                    << output_topic_ << ", G12 deadzone=" << g12_axis_deadzone_
                    << ", G12 timeout=" << g12_input_timeout_s_ << "s, Pico timeout="
                    << input_timeout_s_ << "s, neutral rate=" << neutral_publish_hz_
                    << "Hz");
  }

private:
  static constexpr std::size_t kCanonicalAxisCount = 8;
  static constexpr std::size_t kCanonicalButtonCount = 11;
  static constexpr double kAxisRangeTolerance = 1e-3;

  static sensor_msgs::Joy makeNeutralJoy()
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.axes.assign(kCanonicalAxisCount, 0.0F);
    joy.buttons.assign(kCanonicalButtonCount, 0);
    return joy;
  }

  bool validateG12Joy(const sensor_msgs::Joy& g12) const
  {
    if (g12.axes.size() != kCanonicalAxisCount ||
        g12.buttons.size() != kCanonicalButtonCount)
    {
      ROS_WARN_THROTTLE(2.0,
                        "Ignore G12 Joy with unexpected layout: axes=%zu, buttons=%zu; "
                        "expected canonical 8/11",
                        g12.axes.size(),
                        g12.buttons.size());
      return false;
    }

    for (const float axis : g12.axes)
    {
      if (!std::isfinite(axis) ||
          axis < -1.0F - static_cast<float>(kAxisRangeTolerance) ||
          axis > 1.0F + static_cast<float>(kAxisRangeTolerance))
      {
        ROS_WARN_THROTTLE(2.0, "Ignore G12 Joy with a non-finite/out-of-range axis");
        return false;
      }
    }
    for (const int button : g12.buttons)
    {
      if (button != 0 && button != 1)
      {
        ROS_WARN_THROTTLE(2.0, "Ignore G12 Joy with a button value other than 0/1");
        return false;
      }
    }
    return true;
  }

  sensor_msgs::Joy normalizeG12Joy(const sensor_msgs::Joy& g12) const
  {
    sensor_msgs::Joy output = g12;
    output.header.stamp = ros::Time::now();
    for (float& axis : output.axes)
      axis = std::max(-1.0F, std::min(1.0F, axis));
    return output;
  }

  bool g12HasIntent(const sensor_msgs::Joy& g12) const
  {
    // The biped G12 publisher emits canonical 8-axis/11-button Joy frames.
    // Any deliberate G12 input takes the whole AMP Joy frame so a G12 button
    // pulse cannot be combined with simultaneous Pico motion. Wheel-arm G12
    // mode has different persistent-button semantics and is outside this AMP
    // walking bridge's supported scope.
    for (const int button : g12.buttons)
    {
      if (button != 0)
        return true;
    }
    for (const float axis : g12.axes)
    {
      if (std::abs(axis) > g12_axis_deadzone_)
        return true;
    }
    return false;
  }

  bool g12OwnsOutput(const std::chrono::steady_clock::time_point& now) const
  {
    return have_g12_input_ && g12_active_ &&
           std::chrono::duration<double>(now - last_g12_input_time_).count() <
               g12_input_timeout_s_;
  }

  bool expireStaleG12(const std::chrono::steady_clock::time_point& now)
  {
    if (!have_g12_input_ ||
        std::chrono::duration<double>(now - last_g12_input_time_).count() <
            g12_input_timeout_s_)
    {
      return false;
    }

    have_g12_input_ = false;
    if (!g12_active_)
      return false;

    g12_active_ = false;
    joy_pub_.publish(makeNeutralJoy());
    ROS_WARN_THROTTLE(
        5.0,
        "Active G12 input is stale; release priority and publish neutral AMP Joy");
    return true;
  }

  bool readPicoWalkingAxes(const kuavo_msgs::JoySticks& pico,
                           float& lateral,
                           float& forward,
                           float& yaw) const
  {
    const double values[] = {pico.left_x, pico.left_y, pico.right_x};
    for (const double value : values)
    {
      if (!std::isfinite(value) || value < -1.0 - kAxisRangeTolerance ||
          value > 1.0 + kAxisRangeTolerance)
      {
        ROS_WARN_THROTTLE(
            2.0,
            "Invalid Pico walking axis; publish neutral instead of forwarding motion");
        return false;
      }
    }

    const double left_x =
        std::max(-1.0, std::min(1.0, static_cast<double>(pico.left_x)));
    const double left_y =
        std::max(-1.0, std::min(1.0, static_cast<double>(pico.left_y)));
    const double right_x =
        std::max(-1.0, std::min(1.0, static_cast<double>(pico.right_x)));
    lateral = static_cast<float>(-left_x);
    forward = static_cast<float>(left_y);
    yaw = static_cast<float>(-right_x);
    return true;
  }

  void g12JoyCallback(const sensor_msgs::Joy::ConstPtr& g12)
  {
    if (!g12)
      return;

    if (!validateG12Joy(*g12))
    {
      have_g12_input_ = false;
      if (g12_active_)
      {
        g12_active_ = false;
        joy_pub_.publish(makeNeutralJoy());
        ROS_WARN("Invalid active G12 frame; release priority and publish neutral Joy");
      }
      return;
    }

    const bool was_active = g12_active_;
    const sensor_msgs::Joy normalized_g12 = normalizeG12Joy(*g12);
    last_g12_input_time_ = std::chrono::steady_clock::now();
    have_g12_input_ = true;
    g12_active_ = g12HasIntent(normalized_g12);

    if (g12_active_)
    {
      joy_pub_.publish(normalized_g12);
      if (!was_active)
        ROS_INFO("G12 input active; G12 takes Pico AMP Joy priority");
      return;
    }

    if (was_active)
    {
      // Do not replay cached Pico input during ownership handoff. Emit one
      // neutral frame; the next fresh Pico callback resumes Pico walking.
      joy_pub_.publish(makeNeutralJoy());
      ROS_INFO("G12 sticks returned to neutral; hand AMP walking priority to Pico");
    }
  }

  void picoJoyCallback(const kuavo_msgs::JoySticks::ConstPtr& pico)
  {
    if (!pico)
      return;

    const auto now = std::chrono::steady_clock::now();
    // Repeat timeout handling here so a Pico callback arriving just after the
    // G12 timeout cannot skip the neutral handoff before Pico resumes.
    const bool g12_timeout_handoff = expireStaleG12(now);

    sensor_msgs::Joy joy = makeNeutralJoy();
    float lateral = 0.0F;
    float forward = 0.0F;
    float yaw = 0.0F;
    if (readPicoWalkingAxes(*pico, lateral, forward, yaw))
    {
      joy.axes[0] = lateral;
      joy.axes[1] = forward;
      joy.axes[3] = yaw;
      last_pico_input_time_ = now;
      have_pico_input_ = true;
    }
    else
    {
      have_pico_input_ = false;
    }

    if (!g12_timeout_handoff && !g12OwnsOutput(now))
      joy_pub_.publish(joy);
  }

  void neutralTimerCallback(const ros::WallTimerEvent&)
  {
    const auto now = std::chrono::steady_clock::now();
    const bool g12_timeout_handoff = expireStaleG12(now);

    const bool pico_input_stale =
        !have_pico_input_ ||
        std::chrono::duration<double>(now - last_pico_input_time_).count() >=
            input_timeout_s_;
    if (!pico_input_stale)
      return;

    have_pico_input_ = false;
    if (g12_timeout_handoff || g12OwnsOutput(now))
      return;

    // AutoGait caches its last Joy command. Continue sending neutral frames
    // while Pico is absent so losing the stream cannot preserve old motion.
    joy_pub_.publish(makeNeutralJoy());
    ROS_WARN_THROTTLE(5.0, "Pico Joy input is stale; publishing neutral AMP Joy");
  }

  ros::Publisher joy_pub_;
  ros::Subscriber pico_joy_sub_;
  ros::Subscriber g12_joy_sub_;
  ros::WallTimer neutral_timer_;

  std::string input_topic_;
  std::string g12_input_topic_;
  std::string output_topic_;
  double g12_axis_deadzone_ = 0.05;
  double g12_input_timeout_s_ = 0.2;
  double input_timeout_s_ = 0.5;
  double neutral_publish_hz_ = 20.0;

  bool have_pico_input_ = false;
  std::chrono::steady_clock::time_point last_pico_input_time_;
  bool have_g12_input_ = false;
  bool g12_active_ = false;
  std::chrono::steady_clock::time_point last_g12_input_time_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pico_gmr_joy_converter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try
  {
    PicoGmrJoyConverter converter(nh, private_nh);
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("Failed to start Pico GMR joy converter: %s", exception.what());
    return 1;
  }
  return 0;
}
