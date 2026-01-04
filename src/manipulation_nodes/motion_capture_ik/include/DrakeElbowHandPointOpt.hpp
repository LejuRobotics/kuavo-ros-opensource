#pragma once

#include <drake/common/symbolic/expression.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include <Eigen/Dense>

#include <stdexcept>
#include <utility>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdarg>

namespace HighlyDynamic {

/**
 * Weight configuration structure for DrakeVelocityIKSolver.
 * Defines the cost weights for tracking and smoothness terms.
 */
struct DrakeVelocityIKWeightConfig {
  const char* name;  // Configuration name
  double q11;        // Weight for ||P1 - P1_fixed||^2
  double q12;        // Weight for ||P1 - P1_ref||^2
  double q2;         // Weight for ||P2 - P2_ref||^2
  double qv1;        // Weight for ||P1 - P1_prev||^2 (smoothness)
  double qv2;        // Weight for ||P2 - P2_prev||^2 (smoothness)
};

/**
 * Predefined weight configurations for DrakeVelocityIKSolver.
 * These can be used directly when initializing solver instances.
 *
 * Usage example:
 *   DrakeVelocityIKSolver solver(p0, l1, l2);
 *   solver.setWeights(DrakeVelocityIKWeights::HandPriority);
 */
namespace DrakeVelocityIKWeights {
// Pure tracking without smoothness
inline constexpr DrakeVelocityIKWeightConfig PureTracking{"PureTracking", 0.2, 0.8, 1.0, 0.0, 0.0};

// Balanced tracking with moderate smoothness
inline constexpr DrakeVelocityIKWeightConfig Balanced{"Balanced", 0.2, 0.8, 1.0, 0.1, 0.1};

// Hand priority: prioritize hand (P2) tracking over elbow (P1)
inline constexpr DrakeVelocityIKWeightConfig HandPriority{"HandPriority", 0.5, 0.5, 500.0, 0.05, 0.001};

// Elbow priority: prioritize elbow (P1) tracking over hand (P2)
inline constexpr DrakeVelocityIKWeightConfig ElbowPriority{"ElbowPriority", 2.0, 8.0, 1.0, 0.1, 0.05};

// Light smoothness: moderate tracking with light smoothness
inline constexpr DrakeVelocityIKWeightConfig LightSmooth{"LightSmooth", 0.4, 3.6, 2.0, 0.02, 0.02};

// Aggressive tracking: high tracking weights with minimal smoothness
inline constexpr DrakeVelocityIKWeightConfig Aggressive{"Aggressive", 2.0, 8.0, 10.0, 0.01, 0.01};
}  // namespace DrakeVelocityIKWeights

/**
 * Drake-based IK solver for a 2-link arm (elbow + hand) using a custom nonlinear program.
 *
 * Decision variables:
 * - u1, u2 in R^3 : unit direction vectors for each link
 * Constraints:
 * - u1ᵀu1 = 1, u2ᵀu2 = 1  (nonconvex equality constraints)
 * Forward kinematics:
 * - P1 = P0 + l1 * u1
 * - P2 = P0 + l1 * u1 + l2 * u2
 * Cost:
 * - Tracking: ||P1 - P1_fixed||^2_{Q11} + ||P1 - P1_ref||^2_{Q12} + ||P2 - P2_ref||^2_{Q2}
 * - Smoothness: ||P1 - P1_prev||^2_{Qv1} + ||P2 - P2_prev||^2_{Qv2}
 */
class DrakeVelocityIKSolver final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Construct the solver with fixed base point and link lengths.
   *
   * @param [in] p0: Base point P0.
   * @param [in] link1Length: Link length l1.
   * @param [in] link2Length: Link length l2.
   */
  explicit DrakeVelocityIKSolver(const Eigen::Vector3d& p0,
                                 double link1Length,
                                 double link2Length,
                                 const Eigen::Vector3d& p2LowerBound,
                                 const Eigen::Vector3d& p2UpperBound)
      : p0_(p0),
        link1Length_(link1Length),
        link2Length_(link2Length),
        p2LowerBound_(p2LowerBound),  // [NEW] Init lower bound
        p2UpperBound_(p2UpperBound)   // [NEW] Init upper bound
  {
    // print p2 bounds
    std::printf("p2LowerBound: [%f, %f, %f]\n", p2LowerBound_.x(), p2LowerBound_.y(), p2LowerBound_.z());
    std::printf("p2UpperBound: [%f, %f, %f]\n", p2UpperBound_.x(), p2UpperBound_.y(), p2UpperBound_.z());

    if (!(link1Length_ > 0.0) || !(link2Length_ > 0.0)) {
      throw std::invalid_argument("DrakeVelocityIKSolver: link lengths must be > 0.");
    }
    // Default weights.
    Q11_.setIdentity();
    Q12_.setIdentity();
    Q2_.setIdentity();
    Qv1_.setIdentity();
    Qv2_.setIdentity();

    // Initialize previous solution to unit vectors.
    u1Prev_ = Eigen::Vector3d::UnitX();
    u2Prev_ = Eigen::Vector3d::UnitX();
  }

  ~DrakeVelocityIKSolver() = default;

  /**
   * [NEW] Set bounds for p2 (hand position).
   * Defines a box constraint lb <= p2 <= ub.
   */
  void setP2Bounds(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
    p2LowerBound_ = lb;
    p2UpperBound_ = ub;
  }
  /**
   * Set cost weights.
   *
   * @param [in] Q11: Weight for ||P1 - P1_fixed||^2.
   * @param [in] Q12: Weight for ||P1 - P1_ref||^2.
   * @param [in] Q2:  Weight for ||P2 - P2_ref||^2.
   * @param [in] Qv1: Weight for ||P1 - P1_prev||^2.
   * @param [in] Qv2: Weight for ||P2 - P2_prev||^2.
   */
  void setWeights(const Eigen::Matrix3d& Q11,
                  const Eigen::Matrix3d& Q12,
                  const Eigen::Matrix3d& Q2,
                  const Eigen::Matrix3d& Qv1,
                  const Eigen::Matrix3d& Qv2) {
    Q11_ = Q11;
    Q12_ = Q12;
    Q2_ = Q2;
    Qv1_ = Qv1;
    Qv2_ = Qv2;
  }

  /**
   * Set cost weights from a predefined configuration.
   *
   * @param [in] config: Predefined weight configuration (e.g., DrakeVelocityIKWeights::HandPriority).
   */
  void setWeights(const DrakeVelocityIKWeightConfig& config) {
    setWeights(config.q11 * Eigen::Matrix3d::Identity(),
               config.q12 * Eigen::Matrix3d::Identity(),
               config.q2 * Eigen::Matrix3d::Identity(),
               config.qv1 * Eigen::Matrix3d::Identity(),
               config.qv2 * Eigen::Matrix3d::Identity());
  }

  /**
   * Solve the IK and return (P1, P2).
   *
   * Warm-starts from the previous solution (u1Prev_, u2Prev_) and updates them on success.
   *
   * @param [in] p1Ref: Reference point for elbow P1.
   * @param [in] p1Fixed: Fixed/anchor point for elbow P1.
   * @param [in] p2Ref: Reference point for hand P2.
   * @return pair<P1, P2>.
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> solve(const Eigen::Vector3d& p1Ref,
                                                    const Eigen::Vector3d& p1Fixed,
                                                    const Eigen::Vector3d& p2Ref) {
    // // ONLY FOR DEBUG
    // printConfigTable(p1Ref, p1Fixed, p2Ref);

    using drake::symbolic::Expression;

    drake::solvers::MathematicalProgram prog;

    // Decision variables.
    const auto u1 = prog.NewContinuousVariables(3, "u1");
    const auto u2 = prog.NewContinuousVariables(3, "u2");
    const auto p1 = prog.NewContinuousVariables(3, "p1");
    const auto p2 = prog.NewContinuousVariables(3, "p2");

    // [NEW] Add Bounding Box Constraint for p2
    // Ensures: p2LowerBound_ <= p2 <= p2UpperBound_ (element-wise)
    prog.AddBoundingBoxConstraint(p2LowerBound_, p2UpperBound_, p2);

    // [NEW][HARD-CODED] Lower bound on p2 XY-plane projection norm:
    // Require ||p2_xy|| >= 0.2 m  <=>  p2_x^2 + p2_y^2 >= 0.04
    // Note: This is a nonconvex constraint (reverse of a convex quadratic), but matches the requirement.
    constexpr double kMinP2XyNorm = 0.2;  // [m]
    constexpr double kMinP2XyNormSquared = kMinP2XyNorm * kMinP2XyNorm;
    const Expression p2XyNormSquared = p2(0) * p2(0) + 0.81 * p2(1) * p2(1);
    prog.AddConstraint(p2XyNormSquared >= kMinP2XyNormSquared);

    // [NEW][HARD-CODED] Lower bound on p1 y-component squared:
    // Require p1_y^2 >= 0.3^2 = 0.09
    constexpr double kMinP1YSquared = 0.22 * 0.22;  // 0.09
    const Expression p1YSquared = p1(1) * p1(1);
    prog.AddConstraint(p1YSquared >= kMinP1YSquared);

    // Unit-norm constraints (nonconvex equalities).
    // Manually expand dot product to avoid Eigen<Variable>.dot() template issue.
    const Expression u1DotU1 = u1(0) * u1(0) + u1(1) * u1(1) + u1(2) * u1(2);
    const Expression u2DotU2 = u2(0) * u2(0) + u2(1) * u2(1) + u2(2) * u2(2);
    prog.AddConstraint(u1DotU1 == 1.0);
    prog.AddConstraint(u2DotU2 == 1.0);

    // Forward kinematics constraints:
    // p1 = p0 + l1 * u1
    // p2 = p0 + l1 * u1 + l2 * u2
    const Eigen::Matrix<Expression, 3, 1> fkP1 = p0_.cast<Expression>() + link1Length_ * u1.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> fkP2 =
        p0_.cast<Expression>() + link1Length_ * u1.cast<Expression>() + link2Length_ * u2.cast<Expression>();
    prog.AddLinearEqualityConstraint(p1.cast<Expression>() - fkP1, Eigen::Vector3d::Zero());
    prog.AddLinearEqualityConstraint(p2.cast<Expression>() - fkP2, Eigen::Vector3d::Zero());

    // Previous-frame FK targets.
    const Eigen::Vector3d p1Prev = p0_ + link1Length_ * u1Prev_;
    const Eigen::Vector3d p2Prev = p0_ + link1Length_ * u1Prev_ + link2Length_ * u2Prev_;

    // Tracking costs.
    prog.AddQuadraticErrorCost(Q11_, p1Fixed, p1);
    prog.AddQuadraticErrorCost(Q12_, p1Ref, p1);
    prog.AddQuadraticErrorCost(Q2_, p2Ref, p2);

    // Smoothness costs.
    prog.AddQuadraticErrorCost(Qv1_, p1Prev, p1);
    prog.AddQuadraticErrorCost(Qv2_, p2Prev, p2);

    // Warm-start from previous solution.
    prog.SetInitialGuess(u1, u1Prev_);
    prog.SetInitialGuess(u2, u2Prev_);
    prog.SetInitialGuess(p1, p1Prev);
    prog.SetInitialGuess(p2, p2Prev);

    const auto result = drake::solvers::Solve(prog);
    if (!result.is_success()) {
      // Fail-safe: do not update previous solution; return previous FK.
      return {p1Prev, p2Prev};
    }

    Eigen::Vector3d u1Sol = result.GetSolution(u1);
    Eigen::Vector3d u2Sol = result.GetSolution(u2);

    // Normalize defensively (solver tolerances may drift).
    const double u1Norm = u1Sol.norm();
    const double u2Norm = u2Sol.norm();
    if (u1Norm > 0.0) {
      u1Sol /= u1Norm;
    }
    if (u2Norm > 0.0) {
      u2Sol /= u2Norm;
    }

    u1Prev_ = u1Sol;
    u2Prev_ = u2Sol;

    const Eigen::Vector3d p1Sol = result.GetSolution(p1);
    const Eigen::Vector3d p2Sol = result.GetSolution(p2);
    return {p1Sol, p2Sol};
  }

 private:
  // model constants
  Eigen::Vector3d p0_;
  double link1Length_;
  double link2Length_;

  // weights
  Eigen::Matrix3d Q11_;
  Eigen::Matrix3d Q12_;
  Eigen::Matrix3d Q2_;
  Eigen::Matrix3d Qv1_;
  Eigen::Matrix3d Qv2_;

  // previous solution
  Eigen::Vector3d u1Prev_;
  Eigen::Vector3d u2Prev_;

  Eigen::Vector3d p2LowerBound_;
  Eigen::Vector3d p2UpperBound_;

  // ONLY FOR DEBUG
  /**
   * Print configuration table in ASCII format.
   */
  void printConfigTable(const Eigen::Vector3d& p1Ref,
                        const Eigen::Vector3d& p1Fixed,
                        const Eigen::Vector3d& p2Ref) const {
    char buffer[512];

    std::printf("+==================================================================================+\n");
    std::printf("|                    DrakeVelocityIKSolver Configuration Table                    |\n");
    std::printf("+==================================================================================+\n");

    // Print geometry parameters
    std::printf("| Geometry Parameters:\n");
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]", "p0 (base point)", p0_.x(), p0_.y(), p0_.z());
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6f", "l1 (link1 length)", link1Length_);
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6f", "l2 (link2 length)", link2Length_);
    std::printf("%s\n", buffer);

    // Print weight parameters (extract diagonal values since weights are scalar * Identity)
    std::printf("+==================================================================================+\n");
    std::printf("| Weight Parameters:\n");
    double q11 = Q11_(0, 0);
    double q12 = Q12_(0, 0);
    double q2 = Q2_(0, 0);
    double qv1 = Qv1_(0, 0);
    double qv2 = Qv2_(0, 0);

    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "q11 (P1 fixed weight)", q11);
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "q12 (P1 ref weight)", q12);
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "q2 (P2 ref weight)", q2);
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "qv1 (P1 smoothness weight)", qv1);
    std::printf("%s\n", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "qv2 (P2 smoothness weight)", qv2);
    std::printf("%s\n", buffer);

    // Print previous solution state
    std::printf("+==================================================================================+\n");
    std::printf("| Previous Solution State:\n");
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]",
             "u1Prev",
             u1Prev_.x(),
             u1Prev_.y(),
             u1Prev_.z());
    std::printf("%s\n", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]",
             "u2Prev",
             u2Prev_.x(),
             u2Prev_.y(),
             u2Prev_.z());
    std::printf("%s\n", buffer);

    // Print input parameters
    std::printf("+==================================================================================+\n");
    std::printf("| Input Parameters:\n");
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]",
             "p1Ref (elbow ref)",
             p1Ref.x(),
             p1Ref.y(),
             p1Ref.z());
    std::printf("%s\n", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]",
             "p1Fixed (elbow fixed)",
             p1Fixed.x(),
             p1Fixed.y(),
             p1Fixed.z());
    std::printf("%s\n", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | [%-8.6f, %-8.6f, %-8.6f]",
             "p2Ref (hand ref)",
             p2Ref.x(),
             p2Ref.y(),
             p2Ref.z());
    std::printf("%s\n", buffer);

    std::printf("+==================================================================================+\n");
    std::fflush(stdout);
  }
};

/**
 * Test suite for DrakeVelocityIKSolver.
 * Encapsulates all test cases including noisy closed-loop test and dynamic test.
 */
class DrakeVelocityIKTestSuite final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Constructor with solver parameters.
   *
   * @param [in] p0: Base point P0.
   * @param [in] l1: Link length l1.
   * @param [in] l2: Link length l2.
   */
  explicit DrakeVelocityIKTestSuite(const Eigen::Vector3d& p0, double l1, double l2) : p0_(p0), l1_(l1), l2_(l2) {}

  /**
   * Run all tests.
   * This is the main interface function that executes all test cases.
   */
  void test() {
    testNoisyClosedLoop();
    testHandPriorityDynamic();
  }

 private:
  // Helper function for formatted output
  __attribute__((format(printf, 1, 2))) static void logInfo(const char* format, ...) {
    std::printf("[INFO] ");
    va_list args;
    va_start(args, format);
    std::vprintf(format, args);
    va_end(args);
    std::printf("\n");
    std::fflush(stdout);
  }

  // Helper function for warning output (throttled)
  __attribute__((format(printf, 1, 2))) static void logWarn(const char* format, ...) {
    static size_t warnCount = 0;
    warnCount++;
    // Simple throttling: output every 100 warnings
    if (warnCount % 100 == 0) {
      std::printf("[WARN] ");
      va_list args;
      va_start(args, format);
      std::vprintf(format, args);
      va_end(args);
      std::printf("\n");
      std::fflush(stdout);
    }
  }

 private:
  // Test parameters
  Eigen::Vector3d p0_;
  double l1_;
  double l2_;

  // Statistics structure
  struct ConfigStats {
    std::string name;
    std::vector<double> p1Errors;
    std::vector<double> p2Errors;
    std::vector<double> l1Violations;
    std::vector<double> l2Violations;
    size_t successCount = 0;
    size_t failCount = 0;
  };

  // Helper function to calculate statistics
  static std::tuple<double, double, double, double> calcStats(const std::vector<double>& data) {
    if (data.empty()) return std::make_tuple(0.0, 0.0, 0.0, 0.0);
    double sum = 0, sumSq = 0, minV = 1e9, maxV = -1e9;
    for (double v : data) {
      sum += v;
      sumSq += v * v;
      minV = std::min(minV, v);
      maxV = std::max(maxV, v);
    }
    double mean = sum / data.size();
    double stddev = std::sqrt(sumSq / data.size() - mean * mean);
    return std::make_tuple(mean, stddev, minV, maxV);
  }

  /**
   * Noisy closed-loop test with multiple weight configurations.
   */
  void testNoisyClosedLoop() {
    // Static variables to maintain state across calls
    static std::mt19937 rng(std::random_device{}());
    static size_t totalTrials = 0;
    static size_t frameCount = 0;

    // Test parameters
    constexpr double posNoiseSigma = 0.02;    // Position noise standard deviation (m)
    constexpr double angleNoiseRange = 0.3;   // Angle noise range (rad)
    constexpr int trialsPerConfig = 5;        // Trials per config per frame
    constexpr int statsReportInterval = 100;  // Report every N frames

    // Noise distributions
    std::normal_distribution<double> posNoise(0.0, posNoiseSigma);
    std::uniform_real_distribution<double> angleNoise(-angleNoiseRange, angleNoiseRange);

    // Weight configurations (using predefined configs from namespace)
    static const std::vector<DrakeVelocityIKWeightConfig> configs = {
        DrakeVelocityIKWeights::PureTracking,
        DrakeVelocityIKWeights::Balanced,
        DrakeVelocityIKWeights::HandPriority,
        DrakeVelocityIKWeights::ElbowPriority,
        DrakeVelocityIKWeights::LightSmooth,
        DrakeVelocityIKWeights::Aggressive,
    };

    // Statistics storage
    static std::vector<ConfigStats> allStats(configs.size());

    // Initialize statistics names
    static bool statsInitialized = false;
    if (!statsInitialized) {
      statsInitialized = true;
      for (size_t i = 0; i < configs.size(); ++i) {
        allStats[i].name = configs[i].name;
      }
      logInfo("========== DrakeVelocityIKSolver Noisy Closed-Loop Test Started ==========");
      logInfo("Noise params: posNoiseSigma=%.3f m, angleNoiseRange=%.2f rad", posNoiseSigma, angleNoiseRange);
      logInfo("Test params: trialsPerConfig=%d, statsReportInterval=%d frames", trialsPerConfig, statsReportInterval);
      logInfo("Configs: %zu weight sets", configs.size());
      logInfo("==========================================================================");
    }

    // Run tests for each configuration
    for (size_t cfgIdx = 0; cfgIdx < configs.size(); ++cfgIdx) {
      const auto& cfg = configs[cfgIdx];
      auto& stats = allStats[cfgIdx];

      for (int trial = 0; trial < trialsPerConfig; ++trial) {
        try {
          // Generate random direction vectors
          const double theta1 = angleNoise(rng);      // u1 rotation around Z
          const double theta2 = angleNoise(rng);      // u2 rotation around Z
          const double phi1 = angleNoise(rng) * 0.5;  // u1 elevation
          const double phi2 = angleNoise(rng) * 0.5;  // u2 elevation

          // Base directions: u1 = +X, u2 = +Y
          Eigen::Vector3d u1Base(std::cos(theta1) * std::cos(phi1), std::sin(theta1) * std::cos(phi1), std::sin(phi1));
          Eigen::Vector3d u2Base(std::sin(theta2) * std::cos(phi2), std::cos(theta2) * std::cos(phi2), std::sin(phi2));
          u1Base.normalize();
          u2Base.normalize();

          // Calculate ideal target points
          const Eigen::Vector3d p1Ideal = p0_ + l1_ * u1Base;
          const Eigen::Vector3d p2Ideal = p1Ideal + l2_ * u2Base;

          // Add position noise
          Eigen::Vector3d p1Noise(posNoise(rng), posNoise(rng), posNoise(rng));
          Eigen::Vector3d p2Noise(posNoise(rng), posNoise(rng), posNoise(rng));

          const Eigen::Vector3d p1Ref = p1Ideal + p1Noise;
          const Eigen::Vector3d p1Fixed = p1Ideal + Eigen::Vector3d(posNoise(rng), posNoise(rng), posNoise(rng));
          const Eigen::Vector3d p2Ref = p2Ideal + p2Noise;

          // Create solver and set weights
          DrakeVelocityIKSolver testSolver(p0_, l1_, l2_, -Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
          testSolver.setWeights(cfg.q11 * Eigen::Matrix3d::Identity(),
                                cfg.q12 * Eigen::Matrix3d::Identity(),
                                cfg.q2 * Eigen::Matrix3d::Identity(),
                                cfg.qv1 * Eigen::Matrix3d::Identity(),
                                cfg.qv2 * Eigen::Matrix3d::Identity());

          const auto [p1Sol, p2Sol] = testSolver.solve(p1Ref, p1Fixed, p2Ref);

          // Calculate errors
          const double p1Err = (p1Sol - p1Ref).norm();
          const double p2Err = (p2Sol - p2Ref).norm();
          const double l1Actual = (p1Sol - p0_).norm();
          const double l2Actual = (p2Sol - p1Sol).norm();
          const double l1Violation = std::abs(l1Actual - l1_);
          const double l2Violation = std::abs(l2Actual - l2_);

          // Record statistics
          stats.p1Errors.push_back(p1Err);
          stats.p2Errors.push_back(p2Err);
          stats.l1Violations.push_back(l1Violation);
          stats.l2Violations.push_back(l2Violation);
          stats.successCount++;
          totalTrials++;

        } catch (const std::exception& e) {
          stats.failCount++;
          logWarn("[%s] Trial failed: %s", cfg.name, e.what());
        }
      }
    }

    frameCount++;

    // Periodic statistics report
    if (frameCount % statsReportInterval == 0) {
      logInfo("==================== Statistics Report (Frame %zu, Trials %zu) ====================",
              frameCount,
              totalTrials);
      logInfo("Noise: sigma=%.3f m, angleRange=%.2f rad | Seed=%zu", posNoiseSigma, angleNoiseRange, totalTrials);

      for (size_t cfgIdx = 0; cfgIdx < configs.size(); ++cfgIdx) {
        const auto& cfg = configs[cfgIdx];
        const auto& stats = allStats[cfgIdx];

        if (stats.p1Errors.empty()) continue;

        auto [p1Mean, p1Std, p1Min, p1Max] = calcStats(stats.p1Errors);
        auto [p2Mean, p2Std, p2Min, p2Max] = calcStats(stats.p2Errors);
        auto [l1VioMean, l1VioStd, l1VioMin, l1VioMax] = calcStats(stats.l1Violations);
        auto [l2VioMean, l2VioStd, l2VioMin, l2VioMax] = calcStats(stats.l2Violations);

        logInfo("[%zu] %s (Q11=%.1f,Q12=%.1f,Q2=%.1f,Qv1=%.2f,Qv2=%.2f) n=%zu",
                cfgIdx + 1,
                cfg.name,
                cfg.q11,
                cfg.q12,
                cfg.q2,
                cfg.qv1,
                cfg.qv2,
                stats.p1Errors.size());
        logInfo("    P1 Error: mean=%.4f std=%.4f min=%.4f max=%.4f (m)", p1Mean, p1Std, p1Min, p1Max);
        logInfo("    P2 Error: mean=%.4f std=%.4f min=%.4f max=%.4f (m)", p2Mean, p2Std, p2Min, p2Max);
        logInfo("    L1 Violation: mean=%.6f std=%.6f (m)", l1VioMean, l1VioStd);
        logInfo("    L2 Violation: mean=%.6f std=%.6f (m)", l2VioMean, l2VioStd);
        logInfo("    Success: %zu, Fail: %zu", stats.successCount, stats.failCount);
      }
      logInfo("===================================================================================");
    }
  }

  /**
   * Hand priority dynamic test: elbow extension + hand distance sinusoidal variation.
   */
  void testHandPriorityDynamic() {
    // Static variables to maintain state
    static size_t dynamicTestFrameCount = 0;
    static bool dynamicTestInitialized = false;
    static std::vector<double> dynamicP1Errors;
    static std::vector<double> dynamicP2Errors;
    static std::vector<double> dynamicL1Violations;
    static std::vector<double> dynamicL2Violations;
    static std::vector<double> dynamicL2TargetDistances;
    static std::vector<double> dynamicL2ActualDistances;

    // Test parameters
    constexpr double elbowExtension = 0.1;  // Elbow extension beyond l1 (m)
    constexpr double handAmplitude = 0.3;   // Hand distance variation amplitude (m)
    constexpr double timeScale = 0.1;       // Time scaling factor
    constexpr int reportInterval = 50;      // Report every N frames

    if (!dynamicTestInitialized) {
      dynamicTestInitialized = true;
      logInfo("========== HandPriority Dynamic Test Started ==========");
      logInfo("Test params: elbowExtension=%.2f m, handAmplitude=%.2f m, timeScale=%.2f",
              elbowExtension,
              handAmplitude,
              timeScale);
      logInfo("Constraint: ||p1-p0|| = l1+%.2f = %.2f m (超出正常范围)", elbowExtension, l1_ + elbowExtension);
      logInfo("Constraint: ||p2-p1|| = l2 + %.2f*sin(t) = [%.2f, %.2f] m",
              handAmplitude,
              l2_ - handAmplitude,
              l2_ + handAmplitude);
      logInfo("Weight config: HandPriority (Q11=0.5, Q12=0.5, Q2=5.0, Qv1=0.05, Qv2=0.1)");
      logInfo("======================================================");
    }

    try {
      // Calculate time
      const double t = dynamicTestFrameCount * timeScale;
      const double handDistance = l2_ + handAmplitude * std::sin(t);

      // Target directions: u1 = +X, u2 = +Y
      const Eigen::Vector3d u1Dir = Eigen::Vector3d::UnitX();
      const Eigen::Vector3d u2Dir = Eigen::Vector3d::UnitY();

      // Calculate target positions
      const Eigen::Vector3d p1Ref = p0_ + (l1_ + elbowExtension) * u1Dir;
      const Eigen::Vector3d p1Fixed = p1Ref;

      // p2 target: distance from p1 is l2+0.3*sin(t), direction is +Y
      const Eigen::Vector3d p2Ref = p1Ref + handDistance * u2Dir;

      // Use hand priority configuration
      DrakeVelocityIKSolver testSolver(p0_, l1_, l2_, -Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
      testSolver.setWeights(0.5 * Eigen::Matrix3d::Identity(),   // Q11
                            0.5 * Eigen::Matrix3d::Identity(),   // Q12
                            5.0 * Eigen::Matrix3d::Identity(),   // Q2 (hand priority)
                            0.05 * Eigen::Matrix3d::Identity(),  // Qv1
                            0.1 * Eigen::Matrix3d::Identity());  // Qv2

      const auto [p1Sol, p2Sol] = testSolver.solve(p1Ref, p1Fixed, p2Ref);

      // Calculate actual distances and errors
      const double l1Actual = (p1Sol - p0_).norm();
      const double l2Actual = (p2Sol - p1Sol).norm();
      const double p1Err = (p1Sol - p1Ref).norm();
      const double p2Err = (p2Sol - p2Ref).norm();
      const double l1Violation = std::abs(l1Actual - l1_);
      const double l2Violation = std::abs(l2Actual - l2_);

      // Record statistics
      dynamicP1Errors.push_back(p1Err);
      dynamicP2Errors.push_back(p2Err);
      dynamicL1Violations.push_back(l1Violation);
      dynamicL2Violations.push_back(l2Violation);
      dynamicL2TargetDistances.push_back(handDistance);
      dynamicL2ActualDistances.push_back(l2Actual);

      dynamicTestFrameCount++;

      // Periodic statistics report
      if (dynamicTestFrameCount % reportInterval == 0) {
        auto [p1Mean, p1Std, p1Min, p1Max] = calcStats(dynamicP1Errors);
        auto [p2Mean, p2Std, p2Min, p2Max] = calcStats(dynamicP2Errors);
        auto [l1VioMean, l1VioStd, l1VioMin, l1VioMax] = calcStats(dynamicL1Violations);
        auto [l2VioMean, l2VioStd, l2VioMin, l2VioMax] = calcStats(dynamicL2Violations);
        auto [l2TargetMean, l2TargetStd, l2TargetMin, l2TargetMax] = calcStats(dynamicL2TargetDistances);
        auto [l2ActualMean, l2ActualStd, l2ActualMin, l2ActualMax] = calcStats(dynamicL2ActualDistances);

        logInfo("========== HandPriority Dynamic Test Report (Frame %zu) ==========", dynamicTestFrameCount);
        logInfo("Current: t=%.2f, targetHandDist=%.4f m, actualHandDist=%.4f m", t, handDistance, l2Actual);
        logInfo("Current: p1Ref=[%.3f,%.3f,%.3f], p1Sol=[%.3f,%.3f,%.3f]",
                p1Ref.x(),
                p1Ref.y(),
                p1Ref.z(),
                p1Sol.x(),
                p1Sol.y(),
                p1Sol.z());
        logInfo("Current: p2Ref=[%.3f,%.3f,%.3f], p2Sol=[%.3f,%.3f,%.3f]",
                p2Ref.x(),
                p2Ref.y(),
                p2Ref.z(),
                p2Sol.x(),
                p2Sol.y(),
                p2Sol.z());
        logInfo("Statistics (n=%zu):", dynamicP1Errors.size());
        logInfo("    P1 Error: mean=%.4f std=%.4f min=%.4f max=%.4f (m)", p1Mean, p1Std, p1Min, p1Max);
        logInfo("    P2 Error: mean=%.4f std=%.4f min=%.4f max=%.4f (m)", p2Mean, p2Std, p2Min, p2Max);
        logInfo("    L1 Violation: mean=%.6f std=%.6f min=%.6f max=%.6f (m)", l1VioMean, l1VioStd, l1VioMin, l1VioMax);
        logInfo("    L2 Violation: mean=%.6f std=%.6f min=%.6f max=%.6f (m)", l2VioMean, l2VioStd, l2VioMin, l2VioMax);
        logInfo("    Hand Distance: target=%.4f±%.4f [%.4f,%.4f], actual=%.4f±%.4f [%.4f,%.4f] (m)",
                l2TargetMean,
                l2TargetStd,
                l2TargetMin,
                l2TargetMax,
                l2ActualMean,
                l2ActualStd,
                l2ActualMin,
                l2ActualMax);
        logInfo("    Hand Distance Tracking Error: mean=%.4f m", l2VioMean);
        logInfo("==================================================================");
      }

    } catch (const std::exception& e) {
      logWarn("[HandPriority Dynamic Test] Exception: %s", e.what());
    }
  }
};

}  // namespace HighlyDynamic
