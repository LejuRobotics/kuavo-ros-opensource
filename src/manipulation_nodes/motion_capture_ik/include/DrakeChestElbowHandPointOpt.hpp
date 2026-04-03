#pragma once

#include <drake/common/symbolic/expression.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <leju_utils/define.hpp>

namespace HighlyDynamic {

struct DrakeChestElbowHandWeightConfig {
  const char* name = "Default";
  Eigen::Vector3d q0 = Eigen::Vector3d::Ones();
  Eigen::Vector3d qv0 = Eigen::Vector3d::Ones() * 0.1;

  // chest orientation tracking/smoothness (optional, quaternion form):
  //   wq0  * ||vec(q_ref^* ⊗ q_opt)||^2   +   wqv0 * ||vec(q_prev^* ⊗ q_opt)||^2
  // Now supports independent weights for x, y, z components
  Eigen::Vector3d wq0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d wqv0 = Eigen::Vector3d::Zero();

  // arm point tracking
  double q1 = 0.8;  // ||p1 - p1_ref||^2
  double q2 = 1.0;  // ||p2 - p2_ref||^2

  // arm point smoothness
  double qv1 = 0.1;  // ||p1 - p1_prev||^2
  double qv2 = 0.1;  // ||p2 - p2_prev||^2

  // acceleration cost weights
  double qa0 = 5.0e-3;  // chest
  double qa1 = 5.0e-3;  // elbow
  double qa2 = 5.0e-3;  // hand

  // acceleration dt: false = use measured intervals from steady_clock stamps; true = h1=h2=accFixedDtSec
  bool accUseFixedDt = false;
  double accFixedDtSec = 0.01;
};

struct DrakeChestElbowHandBoundsConfig {
  Eigen::Vector3d chestLb = -Eigen::Vector3d::Ones() * 1e3;
  Eigen::Vector3d chestUb = Eigen::Vector3d::Ones() * 1e3;

  Eigen::Vector3d leftHandLb = -Eigen::Vector3d::Ones() * 1e3;
  Eigen::Vector3d leftHandUb = Eigen::Vector3d::Ones() * 1e3;
  Eigen::Vector3d rightHandLb = -Eigen::Vector3d::Ones() * 1e3;
  Eigen::Vector3d rightHandUb = Eigen::Vector3d::Ones() * 1e3;

  // chest yaw/pitch hard bounds (radians)
  double yawLb = -M_PI;
  double yawUb = M_PI;
  double pitchLb = -M_PI / 2.0;
  double pitchUb = M_PI / 2.0;

  // Hard-coded constraint parameters (now configurable)
  // Lower bound on p2 (hand) XY-plane projection norm: ||p2_xy|| >= minP2XyNorm
  // Constraint: p2_x^2 + p2XyNormYWeight * p2_y^2 >= minP2XyNorm^2
  double minP2XyNorm = 0.2;       // [m] minimum XY-plane projection norm for hand position
  double p2XyNormYWeight = 0.81;  // weight coefficient for p2_y in the XY norm constraint

  // Lower bound on p1 (elbow) XY-plane projection norm: ||p1_xy|| >= minP1XyNorm
  // Constraint: p1_x^2 + p1XyNormYWeight * p1_y^2 >= minP1XyNorm^2
  double minP1XyNorm = 0.22;      // [m] minimum XY-plane projection norm for elbow position
  double p1XyNormYWeight = 0.81;  // weight coefficient for p1_y in the XY norm constraint
};

struct DrakeChestElbowHandPrevFilterConfig {
  // alpha in [0, 1], new_prev = alpha * new + (1 - alpha) * old
  double pChest = 1.0;
  double u1Left = 1.0;
  double u2Left = 1.0;
  double u1Right = 1.0;
  double u2Right = 1.0;
  double pLeftElbow = 1.0;
  double pLeftHand = 1.0;
  double pRightElbow = 1.0;
  double pRightHand = 1.0;
};

struct DrakeChestElbowHandSolution {
  bool success = false;

  Eigen::Vector3d pChest = Eigen::Vector3d::Zero();
  Eigen::Quaterniond qChest = Eigen::Quaterniond::Identity();

  Eigen::Vector3d pLeftShoulder = Eigen::Vector3d::Zero();
  Eigen::Vector3d pRightShoulder = Eigen::Vector3d::Zero();

  Eigen::Vector3d pLeftElbow = Eigen::Vector3d::Zero();
  Eigen::Vector3d pLeftHand = Eigen::Vector3d::Zero();
  Eigen::Vector3d pRightElbow = Eigen::Vector3d::Zero();
  Eigen::Vector3d pRightHand = Eigen::Vector3d::Zero();
  int64_t solveTimeUs = 0;
};

class DrakeChestElbowHandPointOptSolver final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DrakeChestElbowHandPointOptSolver(const Eigen::Vector3d& vClsInChest,
                                             const Eigen::Vector3d& vCrsInChest,
                                             double link1Length,
                                             double link2Length,
                                             const UpperBodyPoseList* initFkResult = nullptr)
      : vClsInChest_(vClsInChest), vCrsInChest_(vCrsInChest), link1Length_(link1Length), link2Length_(link2Length) {
    if (!(link1Length_ > 0.0) || !(link2Length_ > 0.0)) {
      throw std::invalid_argument("DrakeChestElbowHandPointOptSolver: link lengths must be > 0.");
    }

    setWeights(DrakeChestElbowHandWeightConfig{});
    setBounds(DrakeChestElbowHandBoundsConfig{});
    setPrevFilterConfig(DrakeChestElbowHandPrevFilterConfig{});

    // 如果提供了 FK 结果，使用它来初始化 prev 状态
    if (initFkResult) {
      initFromFkResult(*initFkResult);
    } else {
      pChestPrev_.setZero();
      qPrev_ = Eigen::Quaterniond::Identity();
      u1LeftPrev_ = -Eigen::Vector3d::UnitZ();
      u2LeftPrev_ = -Eigen::Vector3d::UnitZ();
      u1RightPrev_ = -Eigen::Vector3d::UnitZ();
      u2RightPrev_ = -Eigen::Vector3d::UnitZ();
      pLeftElbowPrev_.setZero();
      pLeftHandPrev_.setZero();
      pRightElbowPrev_.setZero();
      pRightHandPrev_.setZero();

      syncCachedPointsFromCachedState();  // 使用连杆几何约束进行fk计算，得到初始位置
    }
  }

  ~DrakeChestElbowHandPointOptSolver() = default;

  void setWeights(const DrakeChestElbowHandWeightConfig& config) {
    weightConfig_ = config;
    accChestWeight_ = std::max(0.0, config.qa0);
    accElbowWeight_ = std::max(0.0, config.qa1);
    accHandWeight_ = std::max(0.0, config.qa2);
    accUseFixedDt_ = config.accUseFixedDt;
    {
      double fd = config.accFixedDtSec;
      if (!std::isfinite(fd) || !(fd > 0.0)) {
        fd = 0.01;
      }
      accFixedDtSec_ = std::clamp(fd, accDtMinSec_, accDtMaxSec_);
    }
  }

  void setBounds(const DrakeChestElbowHandBoundsConfig& config) { boundsConfig_ = config; }

  void setPrevFilterConfig(const DrakeChestElbowHandPrevFilterConfig& config) {
    prevFilterConfig_ = config;
    prevFilterConfig_.pChest = clampAlpha(prevFilterConfig_.pChest);
    prevFilterConfig_.u1Left = clampAlpha(prevFilterConfig_.u1Left);
    prevFilterConfig_.u2Left = clampAlpha(prevFilterConfig_.u2Left);
    prevFilterConfig_.u1Right = clampAlpha(prevFilterConfig_.u1Right);
    prevFilterConfig_.u2Right = clampAlpha(prevFilterConfig_.u2Right);
    prevFilterConfig_.pLeftElbow = clampAlpha(prevFilterConfig_.pLeftElbow);
    prevFilterConfig_.pLeftHand = clampAlpha(prevFilterConfig_.pLeftHand);
    prevFilterConfig_.pRightElbow = clampAlpha(prevFilterConfig_.pRightElbow);
    prevFilterConfig_.pRightHand = clampAlpha(prevFilterConfig_.pRightHand);
  }

  DrakeChestElbowHandSolution solve(const Eigen::Vector3d& pChestRef,
                                    const Eigen::Quaterniond& qChestRef,  // unit quaternion, yaw/pitch-only; roll=0
                                    const Eigen::Vector3d& pLeftElbowRef,
                                    const Eigen::Vector3d& pLeftHandRef,
                                    const Eigen::Vector3d& pRightElbowRef,
                                    const Eigen::Vector3d& pRightHandRef) {
    using drake::symbolic::Expression;

    syncCachedPointsFromCachedState();  // 防御性初始化，确保cached points与cached state一致

    const double solveStampSec = steadyClockNowSec();

    drake::solvers::MathematicalProgram prog;

    // Decision variables.
    const auto pChest = prog.NewContinuousVariables(3, "p_chest");
    const auto yaw = prog.NewContinuousVariables(1, "yaw")(0);
    const auto pitch = prog.NewContinuousVariables(1, "pitch")(0);

    const auto u1Left = prog.NewContinuousVariables(3, "u1_left");
    const auto u2Left = prog.NewContinuousVariables(3, "u2_left");
    const auto u1Right = prog.NewContinuousVariables(3, "u1_right");
    const auto u2Right = prog.NewContinuousVariables(3, "u2_right");

    const auto p1Left = prog.NewContinuousVariables(3, "p1_left");
    const auto p2Left = prog.NewContinuousVariables(3, "p2_left");
    const auto p1Right = prog.NewContinuousVariables(3, "p1_right");
    const auto p2Right = prog.NewContinuousVariables(3, "p2_right");

    // -----------------------------
    // Bounds constraints (box)
    // -----------------------------
    prog.AddBoundingBoxConstraint(boundsConfig_.chestLb, boundsConfig_.chestUb, pChest);           // chest box
    prog.AddBoundingBoxConstraint(boundsConfig_.leftHandLb, boundsConfig_.leftHandUb, p2Left);     // left hand box
    prog.AddBoundingBoxConstraint(boundsConfig_.rightHandLb, boundsConfig_.rightHandUb, p2Right);  // right hand box
    prog.AddBoundingBoxConstraint(boundsConfig_.yawLb, boundsConfig_.yawUb, yaw);                  // yaw 1 dim limit
    prog.AddBoundingBoxConstraint(boundsConfig_.pitchLb, boundsConfig_.pitchUb, pitch);            // pitch 1 dim limit

    // -----------------------------
    // Unit-norm constraints (nonconvex)
    // -----------------------------
    const Expression u1LeftDot = u1Left(0) * u1Left(0) + u1Left(1) * u1Left(1) + u1Left(2) * u1Left(2);
    const Expression u2LeftDot = u2Left(0) * u2Left(0) + u2Left(1) * u2Left(1) + u2Left(2) * u2Left(2);
    const Expression u1RightDot = u1Right(0) * u1Right(0) + u1Right(1) * u1Right(1) + u1Right(2) * u1Right(2);
    const Expression u2RightDot = u2Right(0) * u2Right(0) + u2Right(1) * u2Right(1) + u2Right(2) * u2Right(2);
    prog.AddConstraint(u1LeftDot == 1.0);
    prog.AddConstraint(u2LeftDot == 1.0);
    prog.AddConstraint(u1RightDot == 1.0);
    prog.AddConstraint(u2RightDot == 1.0);

    // -----------------------------
    // q_opt(yaw,pitch) := AngleAxis(yaw, z) ⊗ AngleAxis(pitch, y)
    // -----------------------------
    const Eigen::Matrix<Expression, 4, 1> qOpt = quatZYYawPitchSymbolic(yaw, pitch);

    const Eigen::Matrix<Expression, 3, 1> pChestExpr = pChest.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> vLeftShoulderWorldExpr = quatRotateVectorSymbolic(qOpt, vClsInChest_);
    const Eigen::Matrix<Expression, 3, 1> vRightShoulderWorldExpr = quatRotateVectorSymbolic(qOpt, vCrsInChest_);
    const Eigen::Matrix<Expression, 3, 1> pLeftShoulderExpr = pChestExpr + vLeftShoulderWorldExpr;
    const Eigen::Matrix<Expression, 3, 1> pRightShoulderExpr = pChestExpr + vRightShoulderWorldExpr;

    // -----------------------------
    // FK equality constraints
    // -----------------------------
    const Eigen::Matrix<Expression, 3, 1> fkP1Left = pLeftShoulderExpr + link1Length_ * u1Left.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> fkP2Left =
        pLeftShoulderExpr + link1Length_ * u1Left.cast<Expression>() + link2Length_ * u2Left.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> fkP1Right = pRightShoulderExpr + link1Length_ * u1Right.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> fkP2Right =
        pRightShoulderExpr + link1Length_ * u1Right.cast<Expression>() + link2Length_ * u2Right.cast<Expression>();

    for (int i = 0; i < 3; ++i) {
      prog.AddConstraint(p1Left(i) - fkP1Left(i) == 0.0);
      prog.AddConstraint(p2Left(i) - fkP2Left(i) == 0.0);
      prog.AddConstraint(p1Right(i) - fkP1Right(i) == 0.0);
      prog.AddConstraint(p2Right(i) - fkP2Right(i) == 0.0);
    }

    // -----------------------------
    // Costs
    // -----------------------------
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // chest position tracking + smoothness
    const Eigen::Matrix3d q0Weight = weightConfig_.q0.asDiagonal();
    const Eigen::Matrix3d qv0Weight = weightConfig_.qv0.asDiagonal();
    prog.AddQuadraticErrorCost(q0Weight, pChestRef, pChest);
    prog.AddQuadraticErrorCost(qv0Weight, pChestPrev_, pChest);

    // chest orientation tracking + smoothness (with independent x,y,z weights)
    const auto qVec0 = quatRelativeVector(qChestRef, qOpt);
    prog.AddCost(weightConfig_.wq0.x() * qVec0(0) * qVec0(0) + weightConfig_.wq0.y() * qVec0(1) * qVec0(1) +
                 weightConfig_.wq0.z() * qVec0(2) * qVec0(2));

    const auto qVec_v0 = quatRelativeVector(qPrev_, qOpt);
    prog.AddCost(weightConfig_.wqv0.x() * qVec_v0(0) * qVec_v0(0) + weightConfig_.wqv0.y() * qVec_v0(1) * qVec_v0(1) +
                 weightConfig_.wqv0.z() * qVec_v0(2) * qVec_v0(2));

    // keep elbows close to previous chest center
    prog.AddQuadraticErrorCost(weightConfig_.qv1 * I3, pChestPrev_, p1Left);
    prog.AddQuadraticErrorCost(weightConfig_.qv1 * I3, pChestPrev_, p1Right);

    // left arm tracking
    prog.AddQuadraticErrorCost(weightConfig_.q1 * I3, pLeftElbowRef, p1Left);
    prog.AddQuadraticErrorCost(weightConfig_.q2 * I3, pLeftHandRef, p2Left);

    // left arm smoothness
    prog.AddQuadraticErrorCost(weightConfig_.qv1 * I3, pLeftElbowPrev_, p1Left);
    prog.AddQuadraticErrorCost(weightConfig_.qv2 * I3, pLeftHandPrev_, p2Left);

    // right arm tracking
    prog.AddQuadraticErrorCost(weightConfig_.q1 * I3, pRightElbowRef, p1Right);
    prog.AddQuadraticErrorCost(weightConfig_.q2 * I3, pRightHandRef, p2Right);

    // right arm smoothness
    prog.AddQuadraticErrorCost(weightConfig_.qv1 * I3, pRightElbowPrev_, p1Right);
    prog.AddQuadraticErrorCost(weightConfig_.qv2 * I3, pRightHandPrev_, p2Right);

    const SolverStateSample* tMinus2 = nullptr;
    const SolverStateSample* tMinus1 = nullptr;
    double h1 = 0.0;
    double h2 = 0.0;
    const char* accDisableReason = "startup";
    const bool accEnabled = getAccContext(solveStampSec, tMinus2, tMinus1, h1, h2, accDisableReason);
    if (accEnabled) {
      const Eigen::Matrix<Expression, 3, 1> aChest = buildAccelerationExpr(pChest.cast<Expression>(), tMinus1->pChest, tMinus2->pChest, h1, h2);
      const Eigen::Matrix<Expression, 3, 1> aP1Left = buildAccelerationExpr(p1Left.cast<Expression>(), tMinus1->p1Left, tMinus2->p1Left, h1, h2);
      const Eigen::Matrix<Expression, 3, 1> aP2Left = buildAccelerationExpr(p2Left.cast<Expression>(), tMinus1->p2Left, tMinus2->p2Left, h1, h2);
      const Eigen::Matrix<Expression, 3, 1> aP1Right = buildAccelerationExpr(p1Right.cast<Expression>(), tMinus1->p1Right, tMinus2->p1Right, h1, h2);
      const Eigen::Matrix<Expression, 3, 1> aP2Right = buildAccelerationExpr(p2Right.cast<Expression>(), tMinus1->p2Right, tMinus2->p2Right, h1, h2);

      prog.AddCost(accChestWeight_ * aChest.dot(aChest));
      prog.AddCost(accElbowWeight_ * aP1Left.dot(aP1Left));
      prog.AddCost(accHandWeight_ * aP2Left.dot(aP2Left));
      prog.AddCost(accElbowWeight_ * aP1Right.dot(aP1Right));
      prog.AddCost(accHandWeight_ * aP2Right.dot(aP2Right));
    }

    // -----------------------------
    // Configurable constraints
    // -----------------------------
    const auto [yawRef, pitchRefUnused] = yawPitchFromQuatZYYawPitch(qChestRef);
    const Expression p1LeftRelX = p1Left(0) - pChest(0);
    const Expression p1LeftRelY = p1Left(1) - pChest(1);
    const Expression p1RightRelX = p1Right(0) - pChest(0);
    const Expression p1RightRelY = p1Right(1) - pChest(1);

    const Expression p1LeftRelBodyY = -std::sin(yawRef) * p1LeftRelX + std::cos(yawRef) * p1LeftRelY;
    const Expression p1RightRelBodyY = -std::sin(yawRef) * p1RightRelX + std::cos(yawRef) * p1RightRelY;
    prog.AddConstraint(p1LeftRelBodyY >= boundsConfig_.minP1XyNorm);
    prog.AddConstraint(p1RightRelBodyY <= -boundsConfig_.minP1XyNorm);

    const Expression p2LeftRelX = p2Left(0) - pChest(0);
    const Expression p2LeftRelY = p2Left(1) - pChest(1);
    const Expression p2RightRelX = p2Right(0) - pChest(0);
    const Expression p2RightRelY = p2Right(1) - pChest(1);

    const Expression p2LeftRelBodyX = std::cos(yawRef) * p2LeftRelX + std::sin(yawRef) * p2LeftRelY;
    const Expression p2RightRelBodyX = std::cos(yawRef) * p2RightRelX + std::sin(yawRef) * p2RightRelY;
    prog.AddConstraint(p2LeftRelBodyX >= -0.03);
    prog.AddConstraint(p2RightRelBodyX >= -0.03);

    prog.AddConstraint(p2Left(2) <= pChest(2) + 0.25);
    prog.AddConstraint(p2Right(2) <= pChest(2) + 0.25);
    // -----------------------------
    // Warm-start from cached previous solution
    // -----------------------------
    const auto [yawPrev, pitchPrev] = yawPitchFromQuatZYYawPitch(qPrev_);
    prog.SetInitialGuess(pChest, pChestPrev_);
    prog.SetInitialGuess(yaw, yawPrev);
    prog.SetInitialGuess(pitch, pitchPrev);
    prog.SetInitialGuess(u1Left, u1LeftPrev_);
    prog.SetInitialGuess(u2Left, u2LeftPrev_);
    prog.SetInitialGuess(u1Right, u1RightPrev_);
    prog.SetInitialGuess(u2Right, u2RightPrev_);
    prog.SetInitialGuess(p1Left, pLeftElbowPrev_);
    prog.SetInitialGuess(p2Left, pLeftHandPrev_);
    prog.SetInitialGuess(p1Right, pRightElbowPrev_);
    prog.SetInitialGuess(p2Right, pRightHandPrev_);

    // 求解前的时间记录
    const auto tStart = std::chrono::high_resolution_clock::now();
    const auto result = drake::solvers::Solve(prog);
    const auto tEnd = std::chrono::high_resolution_clock::now();
    // 计算求解耗时，单位为us
    const auto solveTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tStart).count();
    if (!result.is_success()) {
      // Fail-safe: do not update cached state.
      auto failSol = makeSolutionFromCache(false);
      failSol.solveTimeUs = solveTimeUs;
      lastAccEnabled_ = accEnabled;
      lastAccDisableReason_ = accEnabled ? "enabled" : accDisableReason;
      lastAccCostValue_ = 0.0;
      lastAccH1_ = h1;
      lastAccH2_ = h2;
      emitAccDebugLog();
      return failSol;
    }

    // Read solution.
    DrakeChestElbowHandSolution sol;
    sol.success = true;
    sol.solveTimeUs = solveTimeUs;
    sol.pChest = result.GetSolution(pChest);
    const double yawSol = result.GetSolution(yaw);
    const double pitchSol = result.GetSolution(pitch);
    sol.qChest = quatZYYawPitchDouble(yawSol, pitchSol);
    sol.pLeftElbow = result.GetSolution(p1Left);
    sol.pLeftHand = result.GetSolution(p2Left);
    sol.pRightElbow = result.GetSolution(p1Right);
    sol.pRightHand = result.GetSolution(p2Right);

    // Update cached direction vectors (defensively normalized).
    const Eigen::Vector3d u1LeftNew = normalizeSafe(result.GetSolution(u1Left));
    const Eigen::Vector3d u2LeftNew = normalizeSafe(result.GetSolution(u2Left));
    const Eigen::Vector3d u1RightNew = normalizeSafe(result.GetSolution(u1Right));
    const Eigen::Vector3d u2RightNew = normalizeSafe(result.GetSolution(u2Right));
    u1LeftPrev_ = normalizeSafe(applyAlphaFilter(u1LeftNew, u1LeftPrev_, prevFilterConfig_.u1Left));
    u2LeftPrev_ = normalizeSafe(applyAlphaFilter(u2LeftNew, u2LeftPrev_, prevFilterConfig_.u2Left));
    u1RightPrev_ = normalizeSafe(applyAlphaFilter(u1RightNew, u1RightPrev_, prevFilterConfig_.u1Right));
    u2RightPrev_ = normalizeSafe(applyAlphaFilter(u2RightNew, u2RightPrev_, prevFilterConfig_.u2Right));

    // Update cached chest pose and points.
    pChestPrev_ = applyAlphaFilter(sol.pChest, pChestPrev_, prevFilterConfig_.pChest);
    qPrev_ = sol.qChest.normalized();
    syncCachedPointsFromCachedState();

    // Derived shoulders (double evaluation for output convenience).
    const Eigen::Matrix3d RDouble = sol.qChest.toRotationMatrix();
    sol.pLeftShoulder = sol.pChest + RDouble * vClsInChest_;
    sol.pRightShoulder = sol.pChest + RDouble * vCrsInChest_;

    if (accEnabled) {
      lastAccCostValue_ = computeAccelerationCost(sol, *tMinus2, *tMinus1, h1, h2);
    } else {
      lastAccCostValue_ = 0.0;
    }
    lastAccEnabled_ = accEnabled;
    lastAccDisableReason_ = accEnabled ? "enabled" : accDisableReason;
    lastAccH1_ = h1;
    lastAccH2_ = h2;
    emitAccDebugLog();

    pushHistorySample(makeSolverStateSample(sol, solveStampSec));

    return sol;
  }

 private:
  struct SolverStateSample {
    double stamp = 0.0;
    Eigen::Vector3d pChest = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1Left = Eigen::Vector3d::Zero();
    Eigen::Vector3d p2Left = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1Right = Eigen::Vector3d::Zero();
    Eigen::Vector3d p2Right = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qChest = Eigen::Quaterniond::Identity();
  };

  static double steadyClockNowSec() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
  }

  static Eigen::Vector3d computeAccelerationVector(const Eigen::Vector3d& xNow,
                                                   const Eigen::Vector3d& xPrev1,
                                                   const Eigen::Vector3d& xPrev2,
                                                   double h1,
                                                   double h2) {
    const double scale = 2.0 / (h1 + h2);
    return scale * ((xNow - xPrev1) / h2 - (xPrev1 - xPrev2) / h1);
  }

  static Eigen::Matrix<drake::symbolic::Expression, 3, 1> buildAccelerationExpr(
      const Eigen::Matrix<drake::symbolic::Expression, 3, 1>& xNow,
      const Eigen::Vector3d& xPrev1,
      const Eigen::Vector3d& xPrev2,
      double h1,
      double h2) {
    using drake::symbolic::Expression;
    const Expression scale = 2.0 / (h1 + h2);
    const Eigen::Matrix<Expression, 3, 1> xPrev1Expr = xPrev1.cast<Expression>();
    const Eigen::Matrix<Expression, 3, 1> xPrev2Expr = xPrev2.cast<Expression>();
    return scale * ((xNow - xPrev1Expr) / h2 - (xPrev1Expr - xPrev2Expr) / h1);
  }

  static SolverStateSample makeSolverStateSample(const DrakeChestElbowHandSolution& sol, double stampSec) {
    SolverStateSample sample;
    sample.stamp = stampSec;
    sample.pChest = sol.pChest;
    sample.p1Left = sol.pLeftElbow;
    sample.p2Left = sol.pLeftHand;
    sample.p1Right = sol.pRightElbow;
    sample.p2Right = sol.pRightHand;
    sample.qChest = sol.qChest;
    return sample;
  }

  void emitAccDebugLog() const {
    if (!accDebugPrint_) {
      return;
    }
    std::cout << "[DrakeChestElbowHandPointOptSolver] acc_enabled=" << (lastAccEnabled_ ? "true" : "false")
              << ", fixed_dt=" << (accUseFixedDt_ ? "true" : "false")
              << ", h1=" << lastAccH1_ << ", h2=" << lastAccH2_ << ", j_acc=" << lastAccCostValue_
              << ", reason=" << lastAccDisableReason_ << std::endl;
  }

  void clearHistorySample() { historySample2_.clear(); }

  void pushHistorySample(const SolverStateSample& sample) {
    historySample2_.push_back(sample);
    if (historySample2_.size() > kHistorySize) {
      historySample2_.pop_front();
    }
  }

  bool getAccContext(double stampSecNow,
                     const SolverStateSample*& tMinus2,
                     const SolverStateSample*& tMinus1,
                     double& h1,
                     double& h2,
                     const char*& disableReason) {
    tMinus2 = nullptr;
    tMinus1 = nullptr;
    h1 = 0.0;
    h2 = 0.0;
    disableReason = "startup";
    if (historySample2_.size() < kHistorySize) {
      return false;
    }

    const SolverStateSample& sampleTMinus2 = historySample2_[0];
    const SolverStateSample& sampleTMinus1 = historySample2_[1];
    if (!std::isfinite(sampleTMinus2.stamp) || !std::isfinite(sampleTMinus1.stamp)) {
      clearHistorySample();
      disableReason = "nonfinite_stamp";
      return false;
    }

    const double rawH1 = sampleTMinus1.stamp - sampleTMinus2.stamp;
    const double rawH2 = stampSecNow - sampleTMinus1.stamp;
    if (!(rawH1 > 0.0) || !(rawH2 > 0.0)) {
      disableReason = "non_monotonic";
      return false;
    }

    if (accUseFixedDt_) {
      h1 = accFixedDtSec_;
      h2 = accFixedDtSec_;
      if (!std::isfinite(h1) || !std::isfinite(h2) || !(h1 > 0.0) || !(h2 > 0.0)) {
        disableReason = "nonfinite_dt";
        return false;
      }
      tMinus2 = &sampleTMinus2;
      tMinus1 = &sampleTMinus1;
      disableReason = "enabled_fixed_dt";
      return true;
    }

    if (rawH1 > accStaleThresholdSec_ || rawH2 > accStaleThresholdSec_) {
      clearHistorySample();
      disableReason = "stale";
      return false;
    }

    h1 = std::clamp(rawH1, accDtMinSec_, accDtMaxSec_);
    h2 = std::clamp(rawH2, accDtMinSec_, accDtMaxSec_);
    if (!std::isfinite(h1) || !std::isfinite(h2) || !(h1 > 0.0) || !(h2 > 0.0)) {
      disableReason = "nonfinite_dt";
      return false;
    }

    tMinus2 = &sampleTMinus2;
    tMinus1 = &sampleTMinus1;
    disableReason = "enabled_actual_dt";
    return true;
  }

  double computeAccelerationCost(const DrakeChestElbowHandSolution& sol,
                                 const SolverStateSample& tMinus2,
                                 const SolverStateSample& tMinus1,
                                 double h1,
                                 double h2) const {
    const Eigen::Vector3d aChest = computeAccelerationVector(sol.pChest, tMinus1.pChest, tMinus2.pChest, h1, h2);
    const Eigen::Vector3d aP1Left = computeAccelerationVector(sol.pLeftElbow, tMinus1.p1Left, tMinus2.p1Left, h1, h2);
    const Eigen::Vector3d aP2Left = computeAccelerationVector(sol.pLeftHand, tMinus1.p2Left, tMinus2.p2Left, h1, h2);
    const Eigen::Vector3d aP1Right = computeAccelerationVector(sol.pRightElbow, tMinus1.p1Right, tMinus2.p1Right, h1, h2);
    const Eigen::Vector3d aP2Right = computeAccelerationVector(sol.pRightHand, tMinus1.p2Right, tMinus2.p2Right, h1, h2);

    return accChestWeight_ * aChest.squaredNorm() + accElbowWeight_ * aP1Left.squaredNorm() +
           accHandWeight_ * aP2Left.squaredNorm() + accElbowWeight_ * aP1Right.squaredNorm() +
           accHandWeight_ * aP2Right.squaredNorm();
  }

  static Eigen::Vector3d normalizeSafe(const Eigen::Vector3d& v) {
    const double n = v.norm();
    if (n > 0.0) {
      return v / n;
    }
    return Eigen::Vector3d::UnitX();
  }

  static Eigen::Vector3d applyAlphaFilter(const Eigen::Vector3d& newValue,
                                          const Eigen::Vector3d& oldValue,
                                          double alpha) {
    return alpha * newValue + (1.0 - alpha) * oldValue;
  }

  static double clampAlpha(double alpha) { return std::max(0.0, std::min(1.0, alpha)); }

  static Eigen::Quaterniond quatZYYawPitchDouble(double yaw, double pitch) {
    const double cy2 = std::cos(yaw * 0.5);
    const double sy2 = std::sin(yaw * 0.5);
    const double cp2 = std::cos(pitch * 0.5);
    const double sp2 = std::sin(pitch * 0.5);
    // q = qz(yaw) ⊗ qy(pitch), with q = [w,x,y,z]
    return Eigen::Quaterniond(cy2 * cp2,   // w
                              -sy2 * sp2,  // x
                              cy2 * sp2,   // y
                              sy2 * cp2    // z
    );
  }

  static Eigen::Matrix<drake::symbolic::Expression, 4, 1> quatZYYawPitchSymbolic(
      const drake::symbolic::Expression& yaw,
      const drake::symbolic::Expression& pitch) {
    using drake::symbolic::cos;
    using drake::symbolic::Expression;
    using drake::symbolic::sin;

    const Expression cy2 = cos(yaw * 0.5);
    const Expression sy2 = sin(yaw * 0.5);
    const Expression cp2 = cos(pitch * 0.5);
    const Expression sp2 = sin(pitch * 0.5);

    Eigen::Matrix<Expression, 4, 1> q;
    q << cy2 * cp2,  // w
        -sy2 * sp2,  // x
        cy2 * sp2,   // y
        sy2 * cp2;   // z
    return q;
  }

  // Rotate a constant vector v (in chest frame) by a symbolic quaternion q (w,x,y,z).
  // v_world = q ⊙ v. Uses the numerically stable "t = 2*(q_vec × v)" form:
  //   v' = v + w*t + (q_vec × t)
  static Eigen::Matrix<drake::symbolic::Expression, 3, 1> quatRotateVectorSymbolic(
      const Eigen::Matrix<drake::symbolic::Expression, 4, 1>& q,
      const Eigen::Vector3d& v) {
    using drake::symbolic::Expression;

    const Expression w = q(0);
    const Expression x = q(1);
    const Expression y = q(2);
    const Expression z = q(3);

    const Expression vx = v.x();
    const Expression vy = v.y();
    const Expression vz = v.z();

    // t = 2 * (q_vec × v)
    const Expression tx = 2.0 * (y * vz - z * vy);
    const Expression ty = 2.0 * (z * vx - x * vz);
    const Expression tz = 2.0 * (x * vy - y * vx);

    // q_vec × t
    const Expression cx = y * tz - z * ty;
    const Expression cy = z * tx - x * tz;
    const Expression cz = x * ty - y * tx;

    Eigen::Matrix<Expression, 3, 1> vRot;
    vRot << vx + w * tx + cx,  //
        vy + w * ty + cy,      //
        vz + w * tz + cz;
    return vRot;
  }

  // Compute vec(q_err), where q_err = qRef^* ⊗ qOpt.
  // Returns the vector part [x, y, z] of the quaternion error.
  static Eigen::Matrix<drake::symbolic::Expression, 3, 1> quatRelativeVector(
      const Eigen::Quaterniond& qRefIn,
      const Eigen::Matrix<drake::symbolic::Expression, 4, 1>& qOpt) {
    using drake::symbolic::Expression;

    const Eigen::Quaterniond qRef = qRefIn.normalized();

    // q1 = qRef^* = [w, -x, -y, -z]
    const double w1 = qRef.w();
    const double x1 = -qRef.x();
    const double y1 = -qRef.y();
    const double z1 = -qRef.z();

    // q2 = qOpt = [w, x, y, z]
    const Expression w2 = qOpt(0);
    const Expression x2 = qOpt(1);
    const Expression y2 = qOpt(2);
    const Expression z2 = qOpt(3);

    // q = q1 ⊗ q2
    const Expression x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    const Expression y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    const Expression z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

    Eigen::Matrix<Expression, 3, 1> vec;
    vec << x, y, z;
    return vec;
  }

  static std::pair<double, double> yawPitchFromQuatZYYawPitch(const Eigen::Quaterniond& qIn) {
    // q is assumed to represent yaw/pitch-only rotation: q = qz(yaw) ⊗ qy(pitch).
    // Robust factorization:
    //   yaw = 2*atan2(z, w), then qy = qz(yaw)^{-1} ⊗ q, pitch = 2*atan2(qy.y, qy.w).
    Eigen::Quaterniond q = qIn.normalized();
    const double yaw = 2.0 * std::atan2(q.z(), q.w());

    const double cy2 = std::cos(yaw * 0.5);
    const double sy2 = std::sin(yaw * 0.5);
    const Eigen::Quaterniond qzInv(cy2, 0.0, 0.0, -sy2);
    const Eigen::Quaterniond qy = qzInv * q;

    const double pitch = 2.0 * std::atan2(qy.y(), qy.w());
    return {yaw, pitch};
  }

  void initFromFkResult(const UpperBodyPoseList& fkResult) {
    // 从 FK 结果初始化 prev 状态
    pChestPrev_ = fkResult[BODY][WAIST].p;
    qPrev_ = fkResult[BODY][WAIST].q.normalized();

    pLeftElbowPrev_ = fkResult[LEFT][ELBOW].p;
    pLeftHandPrev_ = fkResult[LEFT][HAND].p;
    pRightElbowPrev_ = fkResult[RIGHT][ELBOW].p;
    pRightHandPrev_ = fkResult[RIGHT][HAND].p;

    // 从位置反推方向向量
    const Eigen::Matrix3d R = qPrev_.toRotationMatrix();
    const Eigen::Vector3d pLeftShoulder = pChestPrev_ + R * vClsInChest_;
    const Eigen::Vector3d pRightShoulder = pChestPrev_ + R * vCrsInChest_;

    // 左臂方向向量
    const Eigen::Vector3d vecShoulderToElbowLeft = pLeftElbowPrev_ - pLeftShoulder;
    u1LeftPrev_ = normalizeSafe(vecShoulderToElbowLeft);
    const Eigen::Vector3d vecElbowToHandLeft = pLeftHandPrev_ - pLeftElbowPrev_;
    u2LeftPrev_ = normalizeSafe(vecElbowToHandLeft);

    // 右臂方向向量
    const Eigen::Vector3d vecShoulderToElbowRight = pRightElbowPrev_ - pRightShoulder;
    u1RightPrev_ = normalizeSafe(vecShoulderToElbowRight);
    const Eigen::Vector3d vecElbowToHandRight = pRightHandPrev_ - pRightElbowPrev_;
    u2RightPrev_ = normalizeSafe(vecElbowToHandRight);
  }

  void syncCachedPointsFromCachedState() {
    // FK from cached pose and direction vectors; this is the authoritative "previous"
    // reference used for warm-start and smoothness costs.
    const Eigen::Matrix3d R = qPrev_.normalized().toRotationMatrix();
    const Eigen::Vector3d pLeftShoulder = pChestPrev_ + R * vClsInChest_;
    const Eigen::Vector3d pRightShoulder = pChestPrev_ + R * vCrsInChest_;

    const Eigen::Vector3d pLeftElbowNew = pLeftShoulder + link1Length_ * u1LeftPrev_;
    const Eigen::Vector3d pLeftHandNew = pLeftElbowNew + link2Length_ * u2LeftPrev_;
    const Eigen::Vector3d pRightElbowNew = pRightShoulder + link1Length_ * u1RightPrev_;
    const Eigen::Vector3d pRightHandNew = pRightElbowNew + link2Length_ * u2RightPrev_;

    pLeftElbowPrev_ = applyAlphaFilter(pLeftElbowNew, pLeftElbowPrev_, prevFilterConfig_.pLeftElbow);
    pLeftHandPrev_ = applyAlphaFilter(pLeftHandNew, pLeftHandPrev_, prevFilterConfig_.pLeftHand);
    pRightElbowPrev_ = applyAlphaFilter(pRightElbowNew, pRightElbowPrev_, prevFilterConfig_.pRightElbow);
    pRightHandPrev_ = applyAlphaFilter(pRightHandNew, pRightHandPrev_, prevFilterConfig_.pRightHand);
  }

  DrakeChestElbowHandSolution makeSolutionFromCache(bool success) const {
    DrakeChestElbowHandSolution sol;
    sol.success = success;
    sol.pChest = pChestPrev_;
    sol.qChest = qPrev_;
    sol.pLeftElbow = pLeftElbowPrev_;
    sol.pLeftHand = pLeftHandPrev_;
    sol.pRightElbow = pRightElbowPrev_;
    sol.pRightHand = pRightHandPrev_;

    const Eigen::Matrix3d RDouble = sol.qChest.toRotationMatrix();
    sol.pLeftShoulder = sol.pChest + RDouble * vClsInChest_;
    sol.pRightShoulder = sol.pChest + RDouble * vCrsInChest_;
    return sol;
  }

 private:
  // model constants
  Eigen::Vector3d vClsInChest_;
  Eigen::Vector3d vCrsInChest_;
  double link1Length_;
  double link2Length_;

  // runtime configs
  DrakeChestElbowHandWeightConfig weightConfig_;
  DrakeChestElbowHandBoundsConfig boundsConfig_;
  DrakeChestElbowHandPrevFilterConfig prevFilterConfig_;

  // acceleration cost internal configuration
  static constexpr size_t kHistorySize = 2;
  bool accDebugPrint_ = false;
  double accChestWeight_ = 5.0e-3;
  double accElbowWeight_ = 5.0e-3;
  double accHandWeight_ = 5.0e-3;
  double accDtMinSec_ = 1.0e-4;
  double accDtMaxSec_ = 0.1;
  double accStaleThresholdSec_ = 0.3;
  bool accUseFixedDt_ = false;
  double accFixedDtSec_ = 0.01;

  // cached previous solution (warm-start + smoothness reference)
  Eigen::Vector3d pChestPrev_;
  Eigen::Quaterniond qPrev_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d u1LeftPrev_;
  Eigen::Vector3d u2LeftPrev_;
  Eigen::Vector3d u1RightPrev_;
  Eigen::Vector3d u2RightPrev_;

  Eigen::Vector3d pLeftElbowPrev_;
  Eigen::Vector3d pLeftHandPrev_;
  Eigen::Vector3d pRightElbowPrev_;
  Eigen::Vector3d pRightHandPrev_;

  // two-sample history for non-uniform acceleration cost
  std::deque<SolverStateSample> historySample2_;

  // lightweight observability for debugging
  bool lastAccEnabled_ = false;
  const char* lastAccDisableReason_ = "startup";
  double lastAccH1_ = 0.0;
  double lastAccH2_ = 0.0;
  double lastAccCostValue_ = 0.0;
};

}  // namespace HighlyDynamic