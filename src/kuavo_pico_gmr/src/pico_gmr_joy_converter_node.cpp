#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <kuavo_pico_gmr/AmpJoyRoute.h>
#include <kuavo_msgs/JoySticks.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace
{

class PicoGmrJoyConverter
{
public:
  PicoGmrJoyConverter(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh)
  {
    private_nh.param<std::string>("input_topic", input_topic_, "/pico/joy");
    private_nh.param<std::string>("g12_input_topic", g12_input_topic_, "/joy");
    private_nh.param<std::string>("output_topic", output_topic_, "/pico/joy_converted");
    private_nh.param<std::string>(
        "initial_joy_layout", initial_joy_layout_mode_, "auto");
    private_nh.param<std::string>("prepare_route_service",
                                  prepare_route_service_,
                                  "/pico_gmr_joy_converter/prepare_amp_route");
    private_nh.param<std::string>(
        "route_subscriber_node",
        route_subscriber_node_,
        "/humanoid_joy_control_auto_gait_with_vel");
    private_nh.param<double>("g12_axis_deadzone", g12_axis_deadzone_, 0.05);
    private_nh.param<double>(
        "g12_trigger_pressed_threshold", g12_trigger_pressed_threshold_, -0.5);
    private_nh.param<double>("g12_input_timeout_s", g12_input_timeout_s_, 0.2);
    private_nh.param<double>("input_timeout_s", input_timeout_s_, 0.5);
    private_nh.param<double>("neutral_publish_hz", neutral_publish_hz_, 20.0);

    if (!std::isfinite(g12_axis_deadzone_) || g12_axis_deadzone_ <= 0.0 ||
        g12_axis_deadzone_ >= 1.0)
    {
      throw std::invalid_argument("g12_axis_deadzone must be finite and in (0, 1)");
    }
    if (!std::isfinite(g12_trigger_pressed_threshold_) ||
        g12_trigger_pressed_threshold_ <= -1.0 ||
        g12_trigger_pressed_threshold_ > 0.0)
    {
      throw std::invalid_argument(
          "g12_trigger_pressed_threshold must be finite and in (-1, 0]");
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

    configureInitialLayout(nh);
    // A converter restart can occur while AutoGait is already subscribed to
    // the converted topic.  Start every button in release-required state so a
    // button held across that restart cannot be turned into a new edge by the
    // first neutral frame from this process.  A normal valid neutral /joy
    // frame clears these non-X gates immediately.
    resetButtonReleaseGates(true);
    route_subscriber_node_ = nh.resolveName(route_subscriber_node_);

    joy_pub_ = nh.advertise<sensor_msgs::Joy>(
        output_topic_,
        10,
        boost::bind(&PicoGmrJoyConverter::joySubscriberConnected,
                    this,
                    boost::placeholders::_1),
        boost::bind(&PicoGmrJoyConverter::joySubscriberDisconnected,
                    this,
                    boost::placeholders::_1));
    prepare_route_server_ = nh.advertiseService(
        prepare_route_service_, &PicoGmrJoyConverter::prepareAmpRouteCallback, this);
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
                    << input_topic_ << ", local Joy=" << g12_input_topic_ << " -> "
                    << output_topic_ << ", initial layout mode="
                    << initial_joy_layout_mode_
                    << ", initial output=" << layoutName(output_layout_)
                    << (layout_locked_ ? " (explicitly locked)"
                                       : " (provisional until first valid local frame)")
                    << ", axis deadzone=" << g12_axis_deadzone_
                    << ", trigger threshold=" << g12_trigger_pressed_threshold_
                    << ", routed subscriber=" << route_subscriber_node_
                    << ", local timeout=" << g12_input_timeout_s_
                    << "s, Pico timeout=" << input_timeout_s_
                    << "s, neutral rate=" << neutral_publish_hz_ << "Hz");
  }

private:
  enum class JoyLayout
  {
    kAxes8Buttons11,
    kAxes8Buttons16,
  };

  static constexpr std::size_t kAxisCount = 8;
  static constexpr std::size_t kButtonCount11 = 11;
  static constexpr std::size_t kButtonCount16 = 16;
  static constexpr double kAxisRangeTolerance = 1e-3;

  static std::string lowercase(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
    return value;
  }

  static const char* layoutName(const JoyLayout layout)
  {
    return layout == JoyLayout::kAxes8Buttons16 ? "8/16" : "8/11";
  }

  static std::size_t buttonCountForLayout(const JoyLayout layout)
  {
    return layout == JoyLayout::kAxes8Buttons16 ? kButtonCount16 : kButtonCount11;
  }

  static JoyLayout layoutForButtonCount(const std::size_t button_count)
  {
    return button_count == kButtonCount16 ? JoyLayout::kAxes8Buttons16
                                          : JoyLayout::kAxes8Buttons11;
  }

  void configureInitialLayout(ros::NodeHandle& nh)
  {
    initial_joy_layout_mode_ = lowercase(initial_joy_layout_mode_);
    if (initial_joy_layout_mode_ == "bt2" || initial_joy_layout_mode_ == "g12" ||
        initial_joy_layout_mode_ == "8/11" || initial_joy_layout_mode_ == "11")
    {
      output_layout_ = JoyLayout::kAxes8Buttons11;
      layout_locked_ = true;
      return;
    }
    if (initial_joy_layout_mode_ == "bt2pro" ||
        initial_joy_layout_mode_ == "8/16" || initial_joy_layout_mode_ == "16")
    {
      output_layout_ = JoyLayout::kAxes8Buttons16;
      layout_locked_ = true;
      return;
    }
    if (initial_joy_layout_mode_ != "auto")
    {
      throw std::invalid_argument(
          "initial_joy_layout must be auto, bt2/g12/11/8/11, or bt2pro/16/8/16");
    }

    // A neutral output is required before the first local /joy frame arrives.
    // Use the launch-selected joystick type as a provisional layout, then let
    // the first complete valid /joy frame become authoritative for this node
    // process.  Once the first complete valid frame is observed the layout is
    // locked for the lifetime of this process.  Changing physical controller
    // layouts therefore requires restarting this node; timeout and malformed
    // data never change the lock.
    std::string joystick_type;
    nh.param<std::string>("/joystick_type", joystick_type, "");
    output_layout_ = lowercase(joystick_type) == "bt2pro"
                         ? JoyLayout::kAxes8Buttons16
                         : JoyLayout::kAxes8Buttons11;
  }

  sensor_msgs::Joy makeNeutralJoy() const
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.axes.assign(kAxisCount, 0.0F);
    joy.buttons.assign(buttonCountForLayout(output_layout_), 0);
    return joy;
  }

  void publishOutput(const sensor_msgs::Joy& joy)
  {
    // Once /set_joy_topic starts creating the target TCP connection, the
    // connection-specific callback below must put two neutral frames first on
    // that connection.  Do not let an ordinary broadcast overtake that
    // baseline while the subscriber callback is still queued.
    if (amp_route_prepare_active_ && !amp_route_connection_ready_)
      return;
    joy_pub_.publish(joy);
  }

  void resetButtonReleaseGates(const bool conservative)
  {
    route_button_release_gates_.assign(
        buttonCountForLayout(output_layout_), conservative);
  }

  void invalidateLocalRouteSequence()
  {
    // An invalid/stale frame cannot prove that a previously held button was
    // released.  Require one valid released sample for every button
    // before allowing another edge through the newly routed connection.
    resetButtonReleaseGates(true);
  }

  bool validateLocalJoy(const sensor_msgs::Joy& local_joy) const
  {
    if (local_joy.axes.size() != kAxisCount ||
        (local_joy.buttons.size() != kButtonCount11 &&
         local_joy.buttons.size() != kButtonCount16))
    {
      ROS_WARN_THROTTLE(2.0,
                        "Ignore local Joy with unexpected layout: axes=%zu, "
                        "buttons=%zu; expected 8/11 or 8/16",
                        local_joy.axes.size(),
                        local_joy.buttons.size());
      return false;
    }

    for (const float axis : local_joy.axes)
    {
      if (!std::isfinite(axis) ||
          axis < -1.0F - static_cast<float>(kAxisRangeTolerance) ||
          axis > 1.0F + static_cast<float>(kAxisRangeTolerance))
      {
        ROS_WARN_THROTTLE(
            2.0, "Ignore local Joy with a non-finite/out-of-range axis");
        return false;
      }
    }
    for (const int button : local_joy.buttons)
    {
      if (button != 0 && button != 1)
      {
        ROS_WARN_THROTTLE(
            2.0, "Ignore local Joy with a button value other than 0/1");
        return false;
      }
    }
    return true;
  }

  sensor_msgs::Joy sanitizeLocalJoy(const sensor_msgs::Joy& local_joy) const
  {
    sensor_msgs::Joy output = local_joy;
    output.header.stamp = ros::Time::now();
    for (float& axis : output.axes)
      axis = std::max(-1.0F, std::min(1.0F, axis));
    return output;
  }

  bool localJoyHasIntent(const sensor_msgs::Joy& local_joy) const
  {
    // Any deliberate local button input owns the whole AMP Joy frame.  Intent
    // is evaluated in the locked physical layout; no field-wise fusion is
    // allowed, so a local button can never combine with simultaneous Pico
    // motion.
    for (const int button : local_joy.buttons)
    {
      if (button != 0)
        return true;
    }

    const bool is_bt2pro = local_joy.buttons.size() == kButtonCount16;
    const std::size_t motion_axes_11[] = {0, 1, 3, 4, 6, 7};
    const std::size_t motion_axes_16[] = {0, 1, 2, 3, 6, 7};
    const std::size_t* motion_axes = is_bt2pro ? motion_axes_16 : motion_axes_11;
    for (std::size_t index = 0; index < 6; ++index)
    {
      if (std::abs(local_joy.axes[motion_axes[index]]) > g12_axis_deadzone_)
        return true;
    }

    // BT2/BT2Pro trigger axes may rest at 0 or +1 and move toward -1 when
    // pressed.  abs(axis)>deadzone would therefore make a neutral controller
    // permanently suppress Pico.
    const std::size_t left_trigger = is_bt2pro ? 5 : 2;
    const std::size_t right_trigger = is_bt2pro ? 4 : 5;
    return local_joy.axes[left_trigger] < g12_trigger_pressed_threshold_ ||
           local_joy.axes[right_trigger] < g12_trigger_pressed_threshold_;
  }

  bool selectLayoutForValidLocalJoy(const sensor_msgs::Joy& local_joy)
  {
    const JoyLayout incoming_layout = layoutForButtonCount(local_joy.buttons.size());
    if (layout_locked_)
    {
      if (incoming_layout == output_layout_)
        return false;

      // Alternating 8/11 and 8/16 frames most often indicate multiple /joy
      // publishers.  Never let that change the output schema at runtime.  If
      // the locked source owned motion, fail closed in the locked layout.
      have_g12_input_ = false;
      invalidateLocalRouteSequence();
      if (g12_active_)
      {
        g12_active_ = false;
        publishOutput(makeNeutralJoy());
      }
      ROS_WARN_THROTTLE(
          2.0,
          "Ignore local Joy layout %s because AMP output is locked to %s; "
          "restart the converter when changing controller layouts",
          layoutName(incoming_layout),
          layoutName(output_layout_));
      return true;
    }

    if (incoming_layout == output_layout_)
    {
      layout_locked_ = true;
      ROS_INFO("Local Joy layout locked to %s from the first valid /joy frame",
               layoutName(output_layout_));
      return false;
    }

    // Correct a wrong auto-detected provisional layout exactly once.  Stop
    // motion using the provisional map before changing the message size.
    // AutoGait intentionally drops the first frame that changes button count,
    // so send the new-layout neutral twice and consume the determining frame.
    const JoyLayout old_layout = output_layout_;
    joy_pub_.publish(makeNeutralJoy());
    output_layout_ = incoming_layout;
    layout_locked_ = true;
    have_g12_input_ = false;
    g12_active_ = false;
    have_pico_input_ = false;
    resetButtonReleaseGates(true);
    joy_pub_.publish(makeNeutralJoy());
    joy_pub_.publish(makeNeutralJoy());
    ROS_WARN("Auto layout corrected from provisional %s to %s; published "
             "old/new neutral layout barriers, locked the output, and consumed "
             "the determining frame",
             layoutName(old_layout),
             layoutName(output_layout_));
    return true;
  }

  bool applyRouteButtonReleaseGates(sensor_msgs::Joy& local_joy)
  {
    bool gated = false;
    if (route_button_release_gates_.size() != local_joy.buttons.size())
      resetButtonReleaseGates(true);
    for (std::size_t index = 0; index < local_joy.buttons.size(); ++index)
    {
      if (!route_button_release_gates_[index])
        continue;

      // The target connection receives an all-zero baseline before this
      // process reconnects its own /joy subscription.  Every frame handled
      // here therefore belongs to the new input generation.  A released
      // sample arms the button; a button held across the route/restart stays
      // suppressed until the operator physically releases it.
      if (local_joy.buttons[index] == 0)
        route_button_release_gates_[index] = false;
      else
      {
        local_joy.buttons[index] = 0;
        gated = true;
      }
    }
    return gated;
  }

  bool routeTokenMatches(const kuavo_pico_gmr::AmpJoyRoute::Request& request) const
  {
    return request.manager_epoch == route_manager_epoch_ &&
           request.route_generation == route_generation_;
  }

  void fillRouteResponse(kuavo_pico_gmr::AmpJoyRoute::Response& response,
                         const bool success,
                         const std::string& message) const
  {
    response.success = success;
    response.ready = amp_route_prepare_active_ && amp_route_connection_ready_;
    response.target_connected = route_subscriber_connection_count_ > 0;
    response.local_barrier_established = route_local_barrier_established_;
    response.output_button_count =
        static_cast<uint8_t>(buttonCountForLayout(output_layout_));
    response.manager_epoch = route_manager_epoch_;
    response.route_generation = route_generation_;
    response.message = message;
  }

  void clearInputOwnersForRouteBarrier()
  {
    have_g12_input_ = false;
    g12_active_ = false;
    have_pico_input_ = false;
    resetButtonReleaseGates(true);
  }

  void rebuildInputSubscriptionsAfterBaseline()
  {
    // This is the causal barrier for local button edges and Pico motion.  It
    // runs only after the exact AutoGait connection has received its neutral
    // baseline.  ROS discards callbacks queued for an old Subscription when
    // shutdown() is called; callbacks on both replacement connections are
    // therefore newer than the target baseline, without relying on timestamps
    // shared by different TCP connections (G12 Joy headers may be zero).
    g12_joy_sub_.shutdown();
    pico_joy_sub_.shutdown();
    clearInputOwnersForRouteBarrier();
    g12_joy_sub_ = nh_.subscribe(
        g12_input_topic_, 10, &PicoGmrJoyConverter::g12JoyCallback, this);
    pico_joy_sub_ =
        nh_.subscribe(input_topic_, 10, &PicoGmrJoyConverter::picoJoyCallback, this);
    route_local_barrier_established_ = true;
  }

  void establishRouteForNewConnection(
      const ros::SingleSubscriberPublisher& subscriber)
  {
    // AutoGait consumes a size-changing first frame only to reload its layout;
    // the second neutral establishes the actual all-buttons-released baseline.
    subscriber.publish(makeNeutralJoy());
    subscriber.publish(makeNeutralJoy());
    rebuildInputSubscriptionsAfterBaseline();
    amp_route_connection_ready_ = true;
    ROS_INFO("AMP Joy route generation %llu is ready on new AutoGait connection "
             "with %s output",
             static_cast<unsigned long long>(route_generation_),
             layoutName(output_layout_));
  }

  bool prepareAmpRouteCallback(kuavo_pico_gmr::AmpJoyRoute::Request& request,
                               kuavo_pico_gmr::AmpJoyRoute::Response& response)
  {
    if (request.manager_epoch.empty() || request.route_generation == 0)
    {
      fillRouteResponse(response, false, "manager_epoch and route_generation are required");
      return true;
    }

    if (request.action == kuavo_pico_gmr::AmpJoyRoute::Request::ACTION_STATUS)
    {
      if (!routeTokenMatches(request))
        fillRouteResponse(response, false, "route transaction token does not match");
      else if (route_transaction_aborted_)
        fillRouteResponse(response, false, "route transaction was aborted");
      else
        fillRouteResponse(response, true,
                          amp_route_connection_ready_ ? "route ready"
                                                      : "waiting for target connection");
      return true;
    }

    if (request.action == kuavo_pico_gmr::AmpJoyRoute::Request::ACTION_ABORT)
    {
      if (!routeTokenMatches(request))
      {
        fillRouteResponse(response, false, "route transaction token does not match");
        return true;
      }
      amp_route_prepare_active_ = false;
      amp_route_connection_ready_ = false;
      route_local_barrier_established_ = false;
      route_transaction_aborted_ = true;
      clearInputOwnersForRouteBarrier();
      joy_pub_.publish(makeNeutralJoy());
      fillRouteResponse(response, true, "route transaction aborted");
      return true;
    }

    if (request.action != kuavo_pico_gmr::AmpJoyRoute::Request::ACTION_PREPARE)
    {
      fillRouteResponse(response, false, "unknown route action");
      return true;
    }
    if (request.reason < kuavo_pico_gmr::AmpJoyRoute::Request::REASON_CONTROLLER_EVENT ||
        request.reason > kuavo_pico_gmr::AmpJoyRoute::Request::REASON_CONVERTER_RECOVERY)
    {
      fillRouteResponse(response, false, "unknown route reason");
      return true;
    }

    if (routeTokenMatches(request))
    {
      if (route_transaction_aborted_)
        fillRouteResponse(response, false, "route transaction was already aborted");
      else
        fillRouteResponse(response, true,
                          amp_route_connection_ready_ ? "idempotent prepare; route ready"
                                                      : "idempotent prepare; waiting for target");
      return true;
    }
    if (route_subscriber_connection_count_ > 0)
    {
      // A new transaction must start from a confirmed raw-/joy route.  The
      // manager performs that preflight before PREPARE, so a remaining target
      // link is an old asynchronous disconnect, never this generation's ACK.
      // Wait for it to disappear instead of allowing STATUS to observe a
      // stale connection as ready.
      fillRouteResponse(
          response, false, "previous target connection has not disconnected yet");
      return true;
    }
    if (request.manager_epoch == route_manager_epoch_ &&
        request.route_generation < route_generation_)
    {
      fillRouteResponse(response, false, "stale route generation");
      return true;
    }

    route_manager_epoch_ = request.manager_epoch;
    route_generation_ = request.route_generation;
    route_reason_ = request.reason;
    route_controller_event_revision_ = request.controller_event_revision;
    route_controller_event_stamp_ = request.controller_event_stamp;
    route_from_controller_ = request.from_controller;
    route_to_controller_ = request.to_controller;
    route_transaction_aborted_ = false;
    amp_route_prepare_active_ = true;
    amp_route_connection_ready_ = false;
    route_local_barrier_established_ = false;
    clearInputOwnersForRouteBarrier();

    // Stop any old converted-topic consumer before the subscription changes.
    // The normal output path remains frozen until the target baseline and the
    // replacement /joy subscription have both been established.
    joy_pub_.publish(makeNeutralJoy());
    // Even if an old converted-topic connection still exists, do not declare
    // it ready for this generation.  AutoGait's /set_joy_topic callback always
    // shuts down and recreates its subscriber; only that new connection is
    // allowed to establish this token's neutral/input-generation barrier.
    fillRouteResponse(response, true, "route prepared; waiting for new target connection");
    ROS_INFO("Prepared AMP Joy route epoch=%s generation=%llu reason=%u "
             "event_revision=%llu controller=%s->%s",
             route_manager_epoch_.c_str(),
             static_cast<unsigned long long>(route_generation_),
             static_cast<unsigned int>(route_reason_),
             static_cast<unsigned long long>(route_controller_event_revision_),
             route_from_controller_.c_str(),
             route_to_controller_.c_str());
    return true;
  }

  void joySubscriberConnected(const ros::SingleSubscriberPublisher& subscriber)
  {
    if (subscriber.getSubscriberName() != route_subscriber_node_)
      return;

    ++route_subscriber_connection_count_;
    if (amp_route_prepare_active_ && !route_transaction_aborted_)
    {
      establishRouteForNewConnection(subscriber);
      return;
    }

    // Be conservative if AutoGait is manually routed or reconnects before the
    // manager has prepared a transaction.  This protects held buttons while
    // the manager inventory/reconciliation catches up.
    subscriber.publish(makeNeutralJoy());
    subscriber.publish(makeNeutralJoy());
    rebuildInputSubscriptionsAfterBaseline();
    ROS_WARN("AutoGait connected to converted Joy without an active route "
             "transaction; sent a neutral baseline and kept buttons gated");
  }

  void joySubscriberDisconnected(
      const ros::SingleSubscriberPublisher& subscriber)
  {
    if (subscriber.getSubscriberName() != route_subscriber_node_)
      return;

    if (route_subscriber_connection_count_ > 0)
      --route_subscriber_connection_count_;
    if (amp_route_prepare_active_ && route_subscriber_connection_count_ == 0)
    {
      amp_route_connection_ready_ = false;
      route_local_barrier_established_ = false;
      clearInputOwnersForRouteBarrier();
      ROS_WARN("AutoGait disconnected from converted Joy; freezing AMP output "
               "until the target connection is re-established");
    }
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
    // A controller that disappears while a button is held must not create a
    // fresh rising edge merely by reconnecting with that same button held.
    // Require a complete valid release after every local-input timeout.
    invalidateLocalRouteSequence();
    if (!g12_active_)
      return false;

    g12_active_ = false;
    publishOutput(makeNeutralJoy());
    ROS_WARN_THROTTLE(
        5.0,
        "Active local Joy is stale; release priority and publish neutral AMP Joy");
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

  void g12JoyCallback(const sensor_msgs::Joy::ConstPtr& local_joy)
  {
    if (!local_joy)
      return;

    if (!validateLocalJoy(*local_joy))
    {
      have_g12_input_ = false;
      invalidateLocalRouteSequence();
      if (g12_active_)
      {
        g12_active_ = false;
        publishOutput(makeNeutralJoy());
        ROS_WARN(
            "Invalid/mismatched active local Joy frame; release priority and "
            "publish neutral Joy");
      }
      return;
    }

    if (selectLayoutForValidLocalJoy(*local_joy))
      return;

    const bool was_active = g12_active_;
    sensor_msgs::Joy sanitized_local_joy = sanitizeLocalJoy(*local_joy);
    const auto local_input_time = std::chrono::steady_clock::now();
    // Ownership follows the physical input, while the published copy may
    // suppress buttons held across the neutral route baseline.  A held button
    // still keeps local full-frame priority over Pico until physical release.
    const bool physical_local_intent = localJoyHasIntent(sanitized_local_joy);
    const bool button_was_gated =
        applyRouteButtonReleaseGates(sanitized_local_joy);
    last_g12_input_time_ = local_input_time;
    have_g12_input_ = true;
    g12_active_ = physical_local_intent;

    if (g12_active_)
    {
      publishOutput(sanitized_local_joy);
      if (!was_active)
        ROS_INFO("Local Joy input active; local controller takes Pico AMP Joy priority");
      return;
    }

    if (button_was_gated)
    {
      // Repeat the sanitized frame while a held button is gated.  Axes remain
      // intact and own the complete frame, while no button edge can be created
      // until a post-baseline released sample arms that button.
      publishOutput(sanitized_local_joy);
      return;
    }

    if (was_active)
    {
      // Do not replay cached Pico input during ownership handoff. Emit one
      // neutral frame; the next fresh Pico callback resumes Pico walking.
      publishOutput(makeNeutralJoy());
      ROS_INFO("Local Joy returned to neutral; hand AMP walking priority to Pico");
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
      joy.axes[output_layout_ == JoyLayout::kAxes8Buttons16 ? 2 : 3] = yaw;
      last_pico_input_time_ = now;
      have_pico_input_ = true;
    }
    else
    {
      have_pico_input_ = false;
    }

    if (!g12_timeout_handoff && !g12OwnsOutput(now))
      publishOutput(joy);
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
    publishOutput(makeNeutralJoy());
    ROS_WARN_THROTTLE(5.0, "Pico Joy input is stale; publishing neutral AMP Joy");
  }

  ros::NodeHandle nh_;
  ros::Publisher joy_pub_;
  ros::Subscriber pico_joy_sub_;
  ros::Subscriber g12_joy_sub_;
  ros::ServiceServer prepare_route_server_;
  ros::WallTimer neutral_timer_;

  std::string input_topic_;
  std::string g12_input_topic_;
  std::string output_topic_;
  std::string initial_joy_layout_mode_ = "auto";
  std::string prepare_route_service_;
  std::string route_subscriber_node_;
  double g12_axis_deadzone_ = 0.05;
  double g12_trigger_pressed_threshold_ = -0.5;
  double g12_input_timeout_s_ = 0.2;
  double input_timeout_s_ = 0.5;
  double neutral_publish_hz_ = 20.0;

  bool have_pico_input_ = false;
  std::chrono::steady_clock::time_point last_pico_input_time_;
  JoyLayout output_layout_ = JoyLayout::kAxes8Buttons11;
  bool layout_locked_ = false;
  bool amp_route_prepare_active_ = false;
  bool amp_route_connection_ready_ = false;
  bool route_local_barrier_established_ = false;
  bool route_transaction_aborted_ = false;
  std::size_t route_subscriber_connection_count_ = 0;
  std::string route_manager_epoch_;
  uint64_t route_generation_ = 0;
  uint8_t route_reason_ = 0;
  uint64_t route_controller_event_revision_ = 0;
  ros::Time route_controller_event_stamp_;
  std::string route_from_controller_;
  std::string route_to_controller_;
  std::vector<bool> route_button_release_gates_;
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
