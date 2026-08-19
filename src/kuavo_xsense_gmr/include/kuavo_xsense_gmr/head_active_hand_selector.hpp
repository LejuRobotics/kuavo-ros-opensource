#ifndef KUAVO_XSENSE_GMR_HEAD_ACTIVE_HAND_SELECTOR_HPP_
#define KUAVO_XSENSE_GMR_HEAD_ACTIVE_HAND_SELECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace kuavo_xsense_gmr
{

enum class ActiveHand
{
  NONE,
  LEFT,
  RIGHT
};

enum class SelectorUpdateResult
{
  SEEDED,
  UPDATED,
  DUPLICATE,
  RESET_INVALID,
  RESET_TIME_JUMP,
  RESET_OVERSPEED
};

enum class CandidateReason
{
  NONE,
  ACQUIRE,
  CURRENT_QUIET,
  BOTH_ACTIVE
};

inline const char* activeHandName(const ActiveHand hand)
{
  switch (hand)
  {
    case ActiveHand::LEFT:
      return "left";
    case ActiveHand::RIGHT:
      return "right";
    case ActiveHand::NONE:
      return "none";
  }
  return "none";
}

inline const char* candidateReasonName(const CandidateReason reason)
{
  switch (reason)
  {
    case CandidateReason::ACQUIRE:
      return "acquire";
    case CandidateReason::CURRENT_QUIET:
      return "current_quiet";
    case CandidateReason::BOTH_ACTIVE:
      return "both_active";
    case CandidateReason::NONE:
      return "none";
  }
  return "none";
}

struct ActiveHandSelectorConfig
{
  double window_s = 0.20;
  double enter_speed_mps = 0.08;
  double exit_speed_mps = 0.04;
  double dominance_margin_mps = 0.05;
  double dominance_ratio = 1.5;
  double acquire_confirm_s = 0.25;
  double switch_confirm_s = 0.25;
  double both_active_confirm_s = 0.45;
  double min_hold_s = 0.75;
  double max_sample_gap_s = 0.10;
  double max_valid_speed_mps = 3.0;

  bool validate(std::string& error) const
  {
    if (!(window_s > 0.0))
      error = "window_s must be > 0";
    else if (!(enter_speed_mps > exit_speed_mps && exit_speed_mps >= 0.0))
      error = "enter_speed_mps must be > exit_speed_mps >= 0";
    else if (!(dominance_margin_mps >= 0.0))
      error = "dominance_margin_mps must be >= 0";
    else if (!(dominance_ratio >= 1.0))
      error = "dominance_ratio must be >= 1";
    else if (!(acquire_confirm_s >= 0.0 && switch_confirm_s >= 0.0 &&
               both_active_confirm_s >= 0.0 && min_hold_s >= 0.0))
      error = "confirmation and minimum hold times must be >= 0";
    else if (!(max_sample_gap_s > 0.0))
      error = "max_sample_gap_s must be > 0";
    else if (!(max_valid_speed_mps > enter_speed_mps))
      error = "max_valid_speed_mps must be > enter_speed_mps";
    else
    {
      const std::array<double, 11> values = {
          window_s,
          enter_speed_mps,
          exit_speed_mps,
          dominance_margin_mps,
          dominance_ratio,
          acquire_confirm_s,
          switch_confirm_s,
          both_active_confirm_s,
          min_hold_s,
          max_sample_gap_s,
          max_valid_speed_mps,
      };
      if (std::all_of(values.begin(), values.end(), [](const double value) {
            return std::isfinite(value);
          }))
      {
        error.clear();
        return true;
      }
      error = "all selector parameters must be finite";
    }
    return false;
  }
};

class ActiveHandSelector
{
public:
  using Position = std::array<double, 3>;

  ActiveHandSelector() = default;

  explicit ActiveHandSelector(const ActiveHandSelectorConfig& config)
    : config_(config)
  {
  }

  void configure(const ActiveHandSelectorConfig& config)
  {
    config_ = config;
    reset(true);
  }

  SelectorUpdateResult update(const double stamp_s,
                              const Position& left_position,
                              const Position& right_position)
  {
    if (!std::isfinite(stamp_s) || stamp_s <= 0.0 ||
        !isFinite(left_position) || !isFinite(right_position))
    {
      reset(false);
      return SelectorUpdateResult::RESET_INVALID;
    }

    if (!have_previous_sample_)
    {
      seedSample(stamp_s, left_position, right_position);
      return SelectorUpdateResult::SEEDED;
    }

    const double dt = stamp_s - previous_stamp_s_;
    if (std::abs(dt) <= kTimeEpsilon)
      return SelectorUpdateResult::DUPLICATE;

    if (dt < 0.0 || dt > config_.max_sample_gap_s)
    {
      reset(false);
      seedSample(stamp_s, left_position, right_position);
      return SelectorUpdateResult::RESET_TIME_JUMP;
    }

    const double left_speed = distance(left_position, previous_left_position_) / dt;
    const double right_speed = distance(right_position, previous_right_position_) / dt;
    if (!std::isfinite(left_speed) || !std::isfinite(right_speed))
    {
      reset(false);
      seedSample(stamp_s, left_position, right_position);
      return SelectorUpdateResult::RESET_INVALID;
    }
    if (left_speed > config_.max_valid_speed_mps ||
        right_speed > config_.max_valid_speed_mps)
    {
      reset(false);
      seedSample(stamp_s, left_position, right_position);
      return SelectorUpdateResult::RESET_OVERSPEED;
    }

    previous_stamp_s_ = stamp_s;
    previous_left_position_ = left_position;
    previous_right_position_ = right_position;
    appendSpeed(left_speed_samples_, stamp_s, left_speed);
    appendSpeed(right_speed_samples_, stamp_s, right_speed);

    motion_window_ready_ =
        stamp_s - warmup_start_stamp_s_ + kTimeEpsilon >= config_.window_s;
    if (!motion_window_ready_)
      return SelectorUpdateResult::UPDATED;

    left_score_mps_ = median(left_speed_samples_);
    right_score_mps_ = median(right_speed_samples_);
    updateSelection(stamp_s);
    return SelectorUpdateResult::UPDATED;
  }

  void reset(const bool clear_owner)
  {
    have_previous_sample_ = false;
    motion_window_ready_ = false;
    previous_stamp_s_ = 0.0;
    warmup_start_stamp_s_ = 0.0;
    left_speed_samples_.clear();
    right_speed_samples_.clear();
    left_score_mps_ = 0.0;
    right_score_mps_ = 0.0;
    clearCandidate();
    selected_since_valid_ = false;
    if (clear_owner)
      owner_ = ActiveHand::NONE;
  }

  ActiveHand owner() const
  {
    return owner_;
  }

  ActiveHand candidateHand() const
  {
    return candidate_hand_;
  }

  CandidateReason candidateReason() const
  {
    return candidate_reason_;
  }

  bool motionWindowReady() const
  {
    return motion_window_ready_;
  }

  double leftScoreMps() const
  {
    return left_score_mps_;
  }

  double rightScoreMps() const
  {
    return right_score_mps_;
  }

private:
  using TimedSpeed = std::pair<double, double>;

  static constexpr double kTimeEpsilon = 1e-9;

  static bool isFinite(const Position& position)
  {
    return std::all_of(position.begin(), position.end(), [](const double value) {
      return std::isfinite(value);
    });
  }

  static double distance(const Position& lhs, const Position& rhs)
  {
    const double dx = lhs[0] - rhs[0];
    const double dy = lhs[1] - rhs[1];
    const double dz = lhs[2] - rhs[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void seedSample(const double stamp_s,
                  const Position& left_position,
                  const Position& right_position)
  {
    have_previous_sample_ = true;
    previous_stamp_s_ = stamp_s;
    warmup_start_stamp_s_ = stamp_s;
    previous_left_position_ = left_position;
    previous_right_position_ = right_position;
    if (owner_ != ActiveHand::NONE)
    {
      selected_since_stamp_s_ = stamp_s;
      selected_since_valid_ = true;
    }
  }

  void appendSpeed(std::deque<TimedSpeed>& samples,
                   const double stamp_s,
                   const double speed)
  {
    samples.emplace_back(stamp_s, speed);
    const double oldest_allowed = stamp_s - config_.window_s;
    while (!samples.empty() && samples.front().first < oldest_allowed - kTimeEpsilon)
      samples.pop_front();
  }

  static double median(const std::deque<TimedSpeed>& samples)
  {
    if (samples.empty())
      return 0.0;

    std::vector<double> values;
    values.reserve(samples.size());
    for (const TimedSpeed& sample : samples)
      values.push_back(sample.second);
    std::sort(values.begin(), values.end());

    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1)
      return values[middle];
    return 0.5 * (values[middle - 1] + values[middle]);
  }

  bool dominates(const double challenger_score, const double other_score) const
  {
    return challenger_score >= config_.enter_speed_mps &&
           challenger_score - other_score >= config_.dominance_margin_mps &&
           challenger_score >= config_.dominance_ratio * other_score;
  }

  void updateSelection(const double stamp_s)
  {
    if (owner_ == ActiveHand::NONE)
    {
      const bool left_dominates = dominates(left_score_mps_, right_score_mps_);
      const bool right_dominates = dominates(right_score_mps_, left_score_mps_);
      if (left_dominates == right_dominates)
      {
        clearCandidate();
        return;
      }
      updateCandidate(left_dominates ? ActiveHand::LEFT : ActiveHand::RIGHT,
                      CandidateReason::ACQUIRE,
                      stamp_s,
                      config_.acquire_confirm_s);
      return;
    }

    if (!selected_since_valid_)
    {
      selected_since_stamp_s_ = stamp_s;
      selected_since_valid_ = true;
      clearCandidate();
      return;
    }
    if (stamp_s - selected_since_stamp_s_ + kTimeEpsilon < config_.min_hold_s)
    {
      clearCandidate();
      return;
    }

    const bool owner_is_left = owner_ == ActiveHand::LEFT;
    const double owner_score = owner_is_left ? left_score_mps_ : right_score_mps_;
    const double challenger_score = owner_is_left ? right_score_mps_ : left_score_mps_;
    if (!dominates(challenger_score, owner_score))
    {
      clearCandidate();
      return;
    }

    const CandidateReason reason = owner_score <= config_.exit_speed_mps
                                       ? CandidateReason::CURRENT_QUIET
                                       : CandidateReason::BOTH_ACTIVE;
    const double confirm_s = reason == CandidateReason::CURRENT_QUIET
                                 ? config_.switch_confirm_s
                                 : config_.both_active_confirm_s;
    updateCandidate(owner_is_left ? ActiveHand::RIGHT : ActiveHand::LEFT,
                    reason,
                    stamp_s,
                    confirm_s);
  }

  void updateCandidate(const ActiveHand hand,
                       const CandidateReason reason,
                       const double stamp_s,
                       const double confirm_s)
  {
    if (candidate_hand_ != hand || candidate_reason_ != reason || !candidate_since_valid_)
    {
      candidate_hand_ = hand;
      candidate_reason_ = reason;
      candidate_since_stamp_s_ = stamp_s;
      candidate_since_valid_ = true;
    }

    if (stamp_s - candidate_since_stamp_s_ + kTimeEpsilon < confirm_s)
      return;

    owner_ = hand;
    selected_since_stamp_s_ = stamp_s;
    selected_since_valid_ = true;
    clearCandidate();
  }

  void clearCandidate()
  {
    candidate_hand_ = ActiveHand::NONE;
    candidate_reason_ = CandidateReason::NONE;
    candidate_since_stamp_s_ = 0.0;
    candidate_since_valid_ = false;
  }

  ActiveHandSelectorConfig config_;
  ActiveHand owner_ = ActiveHand::NONE;
  ActiveHand candidate_hand_ = ActiveHand::NONE;
  CandidateReason candidate_reason_ = CandidateReason::NONE;
  bool have_previous_sample_ = false;
  bool motion_window_ready_ = false;
  bool selected_since_valid_ = false;
  bool candidate_since_valid_ = false;
  double previous_stamp_s_ = 0.0;
  double warmup_start_stamp_s_ = 0.0;
  double selected_since_stamp_s_ = 0.0;
  double candidate_since_stamp_s_ = 0.0;
  double left_score_mps_ = 0.0;
  double right_score_mps_ = 0.0;
  Position previous_left_position_{{0.0, 0.0, 0.0}};
  Position previous_right_position_{{0.0, 0.0, 0.0}};
  std::deque<TimedSpeed> left_speed_samples_;
  std::deque<TimedSpeed> right_speed_samples_;
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_HEAD_ACTIVE_HAND_SELECTOR_HPP_
