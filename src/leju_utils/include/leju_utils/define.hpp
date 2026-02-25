#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <kuavo_msgs/twoArmHandPoseCmd.h>

struct PoseData {
  Eigen::Matrix3d rotation_matrix;  // Direct rotation matrix
  Eigen::Vector3d position;         // Position vector

  PoseData() {
    rotation_matrix = Eigen::Matrix3d::Identity();
    position = Eigen::Vector3d::Zero();
  }

  PoseData(const Eigen::Matrix3d& rot_mat, const Eigen::Vector3d& pos) : rotation_matrix(rot_mat), position(pos) {}
};

enum class ArmIdx { LEFT = 0, RIGHT = 1, BOTH = 2 };

enum class EndEffectorType { QIANGNAO = 0, QIANGNAO_TOUCH = 1, REVO2 = 2, LEJUCLAW = 3 };

// 判断是否为手部末端执行器类型
inline bool isHandEndEffectorType(EndEffectorType type) {
  return type == EndEffectorType::QIANGNAO || type == EndEffectorType::QIANGNAO_TOUCH || type == EndEffectorType::REVO2;
}

// 判断是否为夹爪末端执行器类型
inline bool isClawEndEffectorType(EndEffectorType type) { return type == EndEffectorType::LEJUCLAW; }

// 字符串转EndEffectorType枚举
inline EndEffectorType stringToEndEffectorType(const std::string& typeStr) {
  if (typeStr == "qiangnao") {
    return EndEffectorType::QIANGNAO;
  } else if (typeStr == "qiangnao_touch") {
    return EndEffectorType::QIANGNAO_TOUCH;
  } else if (typeStr == "revo2") {
    return EndEffectorType::REVO2;
  } else if (typeStr == "lejuclaw") {
    return EndEffectorType::LEJUCLAW;
  } else {
    return EndEffectorType::QIANGNAO;  // 默认返回QIANGNAO
  }
}

#define POSE_INDEX_LEFT_HAND 4        // 左手 - 对应Python bone_names[4] "LeftHandPalm"
#define POSE_INDEX_LEFT_ELBOW 1       // 左肘 - 对应Python bone_names[1] "LeftArmLower"
#define POSE_INDEX_RIGHT_HAND 5       // 右手 - 对应Python bone_names[5] "RightHandPalm"
#define POSE_INDEX_RIGHT_ELBOW 3      // 右肘 - 对应Python bone_names[3] "RightArmLower"
#define POSE_INDEX_CHEST 23           // 胸部 - 对应Python bone_names[23] "Chest"
#define POSE_INDEX_LEFT_ARM_UPPER 0   // 左上臂 - 对应Python bone_names[0] "LeftArmUpper"
#define POSE_INDEX_RIGHT_ARM_UPPER 2  // 右上臂 - 对应Python bone_names[2] "RightArmUpper"

#define POSE_DATA_LIST_INDEX_CHEST 0        // 胸部 - 固定值
#define POSE_DATA_LIST_INDEX_LEFT_HAND 1    // 左手
#define POSE_DATA_LIST_INDEX_RIGHT_HAND 2   // 右手
#define POSE_DATA_LIST_INDEX_LEFT_ELBOW 3   // 左肘
#define POSE_DATA_LIST_INDEX_RIGHT_ELBOW 4  // 右肘
#define POSE_DATA_LIST_SIZE 5

struct TwoStageIKParameters {
  std::vector<std::string> ikConstraintFrameNames;
  double constraintTolerance = 1.0e-8;
  double solverTolerance = 1.0e-6;
  int maxSolverIterations = 1000;
  ArmIdx controlArmIndex = ArmIdx::LEFT;

  TwoStageIKParameters() = default;

  TwoStageIKParameters(const std::vector<std::string>& frameNames,
                       double constraintTol = 1.0e-8,
                       double solverTol = 1.0e-6,
                       int maxIterations = 1000,
                       ArmIdx armIndex = ArmIdx::LEFT)
      : ikConstraintFrameNames(frameNames),
        constraintTolerance(constraintTol),
        solverTolerance(solverTol),
        maxSolverIterations(maxIterations),
        controlArmIndex(armIndex) {}

  bool operator==(const TwoStageIKParameters& other) const {
    return ikConstraintFrameNames == other.ikConstraintFrameNames && constraintTolerance == other.constraintTolerance &&
           solverTolerance == other.solverTolerance && maxSolverIterations == other.maxSolverIterations &&
           controlArmIndex == other.controlArmIndex;
  }

  bool operator!=(const TwoStageIKParameters& other) const { return !(*this == other); }
};

namespace HighlyDynamic {
enum class HandSide { LEFT, RIGHT, BOTH };

inline HandSide intToHandSide(int value) {
  if (value < 0 || value > 2) throw std::invalid_argument("Invalid value for HandSide");
  return static_cast<HandSide>(value);
}

struct HeadBodyPose {
  double head_pitch = 0.0;
  double head_yaw = 0.0;
  double body_yaw = 0.0;
  double body_x = 0.0;
  double body_y = 0.0;
  double body_roll = 0.0;
  double body_pitch = 6.0 * M_PI / 180.0;
  double body_height = 0.74;
};

typedef std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> FramePoseVec;

struct IKParams {
  // snopt params
  double major_optimality_tol{1e-3};
  double major_feasibility_tol{1e-3};
  double minor_feasibility_tol{1e-3};
  double major_iterations_limit{100};
  // constraint and cost params
  double oritation_constraint_tol{1e-3};
  double pos_constraint_tol{1e-3};  // work when pos_cost_weight > 0.0
  double pos_cost_weight{100};      // NOT work if pos_cost_weight <= 0.0
};

struct IkCmd {
  Eigen::Vector3d pos_xyz;        // hand pos
  Eigen::Quaterniond quat;        // hand quaternion
  Eigen::Vector3d elbow_pos_xyz;  // elbow pos, only for motion capture
  Eigen::VectorXd joint_angles;   // joint angles, could as initial guess
  // IKParams ik_params; // solver params

  bool is_elbow_pos_valid() const {
    bool invalid = true;
    for (int i = 0; i < 3; ++i)  // if all three values are close to zero, it's invalid
      invalid &= (elbow_pos_xyz[i] >= -1e3 && elbow_pos_xyz[i] <= 1e3);
    return !invalid;
  }
};

struct IKSolveResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool isSuccess = false;                      ///< 求解是否成功
  Eigen::VectorXd solution;                    ///< 求解得到的关节角向量
  std::chrono::milliseconds solveDuration{0};  ///< 求解耗时
  std::string solverLog;

  IKSolveResult(const Eigen::VectorXd& sol, const std::chrono::milliseconds& duration)
      : isSuccess(true), solution(sol), solveDuration(duration), solverLog("ok") {}

  IKSolveResult(int nq, const std::string& errorMsg = "")
      : isSuccess(false), solution(Eigen::VectorXd::Zero(nq)), solveDuration(0), solverLog(errorMsg) {}

  IKSolveResult() = default;
};

}  // namespace HighlyDynamic

struct ArmPose {
  Eigen::Vector3d position;       // 位置向量
  Eigen::Quaterniond quaternion;  // 四元数表示的方向

  ArmPose() {
    position = Eigen::Vector3d::Zero();
    quaternion = Eigen::Quaterniond::Identity();
  }

  ArmPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat) : position(pos), quaternion(quat) {}

  bool isValid() const {
    return position.allFinite() && quaternion.coeffs().allFinite() && std::abs(quaternion.norm() - 1.0) < 1e-4;
  }
};

struct ArmData {
  Eigen::Quaterniond& handQuatInW;  // 手部四元数引用
  ArmPose& handPose;                // 手部位姿引用
  ArmPose& elbowPose;               // 肘部位姿引用
  int elbowIndex;                   // 肘部索引
  int shoulderIndex;                // 肩部索引
  int handIndex;                    // 手部索引

  ArmData(Eigen::Quaterniond& handQuat, ArmPose& hand, ArmPose& elbow, int elbowIdx, int shoulderIdx, int handIdx)
      : handQuatInW(handQuat),
        handPose(hand),
        elbowPose(elbow),
        elbowIndex(elbowIdx),
        shoulderIndex(shoulderIdx),
        handIndex(handIdx) {}
};

struct IKSolverConfig {
  double constraintTolerance;
  double solverTolerance;
  int maxIterations;
  ArmIdx controlArmIndex;
  bool isWeldBaseLink;

  // Joint limit configuration
  bool useJointLimits;               // Whether to enable application-level joint limits
  Eigen::VectorXd jointLowerBounds;  // Joint lower bounds (radians)
  Eigen::VectorXd jointUpperBounds;  // Joint upper bounds (radians)

  IKSolverConfig()
      : constraintTolerance(1e-8),
        solverTolerance(1e-6),
        maxIterations(1000),
        controlArmIndex(ArmIdx::BOTH),
        isWeldBaseLink(true),
        useJointLimits(false),  // Disabled by default (follow Python behavior)
        jointLowerBounds(Eigen::VectorXd()),
        jointUpperBounds(Eigen::VectorXd()) {}

  IKSolverConfig(double constraintTol, double solverTol, int maxIter, ArmIdx armIdx, bool weldBase)
      : constraintTolerance(constraintTol),
        solverTolerance(solverTol),
        maxIterations(maxIter),
        controlArmIndex(armIdx),
        isWeldBaseLink(weldBase),
        useJointLimits(false),
        jointLowerBounds(Eigen::VectorXd()),
        jointUpperBounds(Eigen::VectorXd()) {}
};

namespace Quest3ArmInfoTransformerConfig {
// 机器人手臂参数 (根据Python实时信息硬编码)
constexpr double UPPER_ARM_LENGTH = 0.1646;   // 上臂长度 (cm) - 从Python获取
constexpr double LOWER_ARM_LENGTH = 0.36229;  // 下臂长度 (cm) - 从Python获取
constexpr double TOTAL_ARM_LENGTH = 0.53;     // 总臂长 (cm) - 从Python获取
constexpr double SHOULDER_WIDTH = 0.150;      // 肩宽 (m) - 从Python获取

// 基座偏移参数 (根据Python实时信息硬编码)
constexpr double BASE_SHOULDER_X_BIAS = 0.000;  // 基座到肩部的X偏移 (m) - 从Python获取
constexpr double BASE_SHOULDER_Y_BIAS = 0.150;  // 基座到肩部的Y偏移 (m) - 从Python获取
constexpr double BASE_SHOULDER_Z_BIAS = 0.420;  // 基座到肩部的Z偏移 (m) -与biped_v3_arm.urdf一致

// 手臂长度测量参数 (根据Python实时信息硬编码)
constexpr bool MEASURE_ARM_LENGTH = true;  // 是否测量手臂长度 - 从Python获取
constexpr int ARM_LENGTH_NUM = 30;         // 手臂长度测量样本数量 - 从Python获取

// 控制参数 (根据Python实时信息硬编码)
constexpr bool CONTROL_TORSO = false;                       // 是否控制躯干 - 从Python获取
constexpr bool PREDICT_GESTURE = false;                     // 是否预测手势 - 从Python获取
constexpr const char* HAND_REFERENCE_MODE = "thumb_index";  // 手部参考模式 - 从Python获取

// 左臂平均测量值 (根据Python实时信息硬编码)
constexpr double LEFT_AVG_UPPER_ARM_LENGTH = 0.22;  // 左臂平均上臂长度 (cm) - 从Python获取
constexpr double LEFT_AVG_LOWER_ARM_LENGTH = 0.26;  // 左臂平均下臂长度 (cm) - 从Python获取
constexpr double LEFT_AVG_TOTAL_ARM_LENGTH = 0.48;  // 左臂平均总臂长 (cm) - 从Python获取

// 右臂平均测量值 (根据Python实时信息硬编码)
constexpr double RIGHT_AVG_UPPER_ARM_LENGTH = 0.23;  // 右臂平均上臂长度 (cm) - 从Python获取
constexpr double RIGHT_AVG_LOWER_ARM_LENGTH = 0.26;  // 右臂平均下臂长度 (cm) - 从Python获取
constexpr double RIGHT_AVG_TOTAL_ARM_LENGTH = 0.49;  // 右臂平均总臂长 (cm) - 从Python获取

// 缩放比例 (根据Python实时信息硬编码)
constexpr double LEFT_UPPER_ARM_RATIO = 0.7401;   // 左臂上臂缩放比例 - 从Python获取
constexpr double LEFT_LOWER_ARM_RATIO = 1.3983;   // 左臂下臂缩放比例 - 从Python获取
constexpr double RIGHT_UPPER_ARM_RATIO = 0.7295;  // 右臂上臂缩放比例 - 从Python获取
constexpr double RIGHT_LOWER_ARM_RATIO = 1.3805;  // 右臂下臂缩放比例 - 从Python获取
}  // namespace Quest3ArmInfoTransformerConfig

/**
 * @brief 设置手部位姿信息（模板函数，类型无关）
 * @tparam PoseInfoT 位姿信息类型，需要有position和orientation成员
 *                   position需要有x,y,z成员，orientation需要有x,y,z,w成员
 * @param poseInfo 输出的位姿信息对象
 * @param armPose 输入的手臂位姿数据
 */
template <typename PoseInfoT>
inline void setHandPoseInfo(PoseInfoT& poseInfo, const ArmPose& armPose) {
  if (armPose.isValid()) {
    poseInfo.position.x = armPose.position.x();
    poseInfo.position.y = armPose.position.y();
    poseInfo.position.z = armPose.position.z();
    poseInfo.orientation.x = armPose.quaternion.x();
    poseInfo.orientation.y = armPose.quaternion.y();
    poseInfo.orientation.z = armPose.quaternion.z();
    poseInfo.orientation.w = armPose.quaternion.w();
  } else {
    poseInfo = PoseInfoT();
  }
}

/**
 * @brief 设置肘部位姿信息（模板函数，类型无关）
 * @tparam PoseInfoT 位姿信息类型，需要有position和orientation成员
 *                   position需要有x,y,z成员，orientation需要有x,y,z,w成员
 * @param poseInfo 输出的位姿信息对象
 * @param armPose 输入的手臂位姿数据
 * @note 肘部没有四元数信息，使用默认值 - 参考Python版本
 */
template <typename PoseInfoT>
inline void setElbowPoseInfo(PoseInfoT& poseInfo, const ArmPose& armPose) {
  if (armPose.isValid()) {
    poseInfo.position.x = armPose.position.x();
    poseInfo.position.y = armPose.position.y();
    poseInfo.position.z = armPose.position.z();
    // 肘部没有四元数信息，使用默认值 - 参考Python版本
    poseInfo.orientation.x = 0.0;
    poseInfo.orientation.y = 0.0;
    poseInfo.orientation.z = 0.0;
    poseInfo.orientation.w = 1.0;
  } else {
    poseInfo = PoseInfoT();
  }
}

inline Eigen::Vector3d deepCopyVector3d(const Eigen::Vector3d& source) {
  return Eigen::Vector3d(source.x(), source.y(), source.z());
}

inline bool validatePoseData(const std::vector<PoseData>& poseDataList) {
  for (size_t i = 0; i < poseDataList.size(); ++i) {
    const auto& pos = poseDataList[i].position;

    // 检查数据是否异常
    bool hasNaN = pos.hasNaN();
    bool hasInf = !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2));
    bool isZero = pos.isZero(1e-10);
    bool isExtreme = (pos.array().abs() > 1e6).any();

    if (hasNaN) {
      std::cerr << "⚠️  位置 " << i << " 包含 NaN 值!" << std::endl;
    }
    if (hasInf) {
      std::cerr << "⚠️  位置 " << i << " 包含无穷大值!" << std::endl;
    }
    if (isZero) {
      std::cout << "⚠️  位置 " << i << " 为零向量" << std::endl;
    }
    if (isExtreme) {
      std::cerr << "⚠️  位置 " << i << " 包含极值!" << std::endl;
    }
  }

  // 检查关键位置数据是否有效
  for (size_t i = 0; i < poseDataList.size(); ++i) {
    const auto& pos = poseDataList[i].position;
    bool hasInf = !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2));
    if (pos.hasNaN() || hasInf || (pos.array().abs() > 1e6).any()) {
      std::cerr << "✗ 检测到无效的位置数据，无法进行IK求解!" << std::endl;
      std::cerr << "请检查 poseDataList 的数据源是否正确初始化。" << std::endl;
      return false;
    }
  }

  return true;
}

inline Eigen::VectorXd limitAngle(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& jointLowerBounds,
                                  const Eigen::VectorXd& jointUpperBounds,
                                  bool hasJointLimits) {
  if (!hasJointLimits) {
    return q;  // Return original if no limits available
  }

  Eigen::VectorXd qLimited = q;
  for (int i = 0; i < q.size() && i < jointLowerBounds.size() && i < jointUpperBounds.size(); ++i) {
    qLimited(i) = std::max(jointLowerBounds(i), std::min(q(i), jointUpperBounds(i)));
  }

  return qLimited;
}

inline Eigen::VectorXd limitJointAngleByVelocity(const Eigen::VectorXd& qLast,
                                                 const Eigen::VectorXd& qNow,
                                                 double velLimitDeg = 720.0,
                                                 double controllerDt = 0.01,
                                                 double firstJointAngleLimitDeg = 120.0) {
  if (qLast.size() != qNow.size()) {
    std::cerr << "Error: qLast and qNow size mismatch in limitJointAngleByVelocity" << std::endl;
    return qNow;
  }

  Eigen::VectorXd qLimited = qNow;
  int size = qNow.size();

  double aglLimit = controllerDt * velLimitDeg * M_PI / 180.0;
  double firstJointAngleLimit = controllerDt * firstJointAngleLimitDeg * M_PI / 180.0;
  int singleArmDof = qLast.size() / 2;

  for (int i = 0; i < size; ++i) {
    qLimited(i) = std::max(qLast(i) - aglLimit, std::min(qNow(i), qLast(i) + aglLimit));

    if (i == 0) {
      qLimited(i) = std::max(qLast(i) - firstJointAngleLimit, std::min(qNow(i), qLast(i) + firstJointAngleLimit));
    } else if (i == singleArmDof) {  // Right arm first joint (r_arm_pitch)
      qLimited(i) = std::max(qLast(i) - firstJointAngleLimit, std::min(qNow(i), qLast(i) + firstJointAngleLimit));
    }
  }

  return qLimited;
}

/**
 * @brief 创建并填充twoArmHandPoseCmd消息
 * @param leftHandPose 左手位姿
 * @param rightHandPose 右手位姿
 * @param leftElbowPose 左手肘部位姿
 * @param rightElbowPose 右手肘部位姿
 * @return 填充好的twoArmHandPoseCmd消息
 */
inline kuavo_msgs::twoArmHandPoseCmd createEefPoseMessage(const ArmPose& leftHandPose,
                                                          const ArmPose& rightHandPose,
                                                          const ArmPose& leftElbowPose,
                                                          const ArmPose& rightElbowPose) {
  kuavo_msgs::twoArmHandPoseCmd eef_pose_msg;

  // 设置消息头
  eef_pose_msg.hand_poses.header.stamp = ros::Time::now();
  eef_pose_msg.hand_poses.header.frame_id = "base_link";

  // 设置左手位姿数据
  eef_pose_msg.hand_poses.left_pose.pos_xyz[0] = leftHandPose.position.x();
  eef_pose_msg.hand_poses.left_pose.pos_xyz[1] = leftHandPose.position.y();
  eef_pose_msg.hand_poses.left_pose.pos_xyz[2] = leftHandPose.position.z();

  eef_pose_msg.hand_poses.left_pose.quat_xyzw[0] = leftHandPose.quaternion.x();
  eef_pose_msg.hand_poses.left_pose.quat_xyzw[1] = leftHandPose.quaternion.y();
  eef_pose_msg.hand_poses.left_pose.quat_xyzw[2] = leftHandPose.quaternion.z();
  eef_pose_msg.hand_poses.left_pose.quat_xyzw[3] = leftHandPose.quaternion.w();

  // 设置左手肘部位置
  eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz[0] = leftElbowPose.position.x();
  eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz[1] = leftElbowPose.position.y();
  eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz[2] = leftElbowPose.position.z();

  // 设置右手位姿数据
  eef_pose_msg.hand_poses.right_pose.pos_xyz[0] = rightHandPose.position.x();
  eef_pose_msg.hand_poses.right_pose.pos_xyz[1] = rightHandPose.position.y();
  eef_pose_msg.hand_poses.right_pose.pos_xyz[2] = rightHandPose.position.z();

  eef_pose_msg.hand_poses.right_pose.quat_xyzw[0] = rightHandPose.quaternion.x();
  eef_pose_msg.hand_poses.right_pose.quat_xyzw[1] = rightHandPose.quaternion.y();
  eef_pose_msg.hand_poses.right_pose.quat_xyzw[2] = rightHandPose.quaternion.z();
  eef_pose_msg.hand_poses.right_pose.quat_xyzw[3] = rightHandPose.quaternion.w();

  // 设置右手肘部位置
  eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz[0] = rightElbowPose.position.x();
  eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz[1] = rightElbowPose.position.y();
  eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz[2] = rightElbowPose.position.z();

  // 设置IK求解参数 (参考Python文件中的参数设置)
  eef_pose_msg.use_custom_ik_param = true;
  eef_pose_msg.joint_angles_as_q0 = false;
  eef_pose_msg.frame = 0;  // 保持当前坐标系

  // 设置IK求解器参数
  eef_pose_msg.ik_param.major_optimality_tol = 9e-3;
  eef_pose_msg.ik_param.major_feasibility_tol = 9e-3;
  eef_pose_msg.ik_param.minor_feasibility_tol = 9e-3;
  eef_pose_msg.ik_param.major_iterations_limit = 50;
  eef_pose_msg.ik_param.oritation_constraint_tol = 9e-3;
  eef_pose_msg.ik_param.pos_constraint_tol = 9e-3;
  eef_pose_msg.ik_param.pos_cost_weight = 10.0;

  return eef_pose_msg;
}

/**
 * @brief 以表格样式打印四个位姿数据
 * @param leftHandPose 左手位姿
 * @param rightHandPose 右手位姿
 * @param leftElbowPose 左肘位姿
 * @param rightElbowPose 右肘位姿
 * @param debugPrint 是否启用调试打印
 */
inline void printPoseDataTable(const ArmPose& leftHandPose,
                               const ArmPose& rightHandPose,
                               const ArmPose& leftElbowPose,
                               const ArmPose& rightElbowPose,
                               bool debugPrint = true) {
  if (!debugPrint) {
    return;
  }

  std::cout << "\n" << std::string(80, '=') << std::endl;
  std::cout << "                    POSE DATA TABLE" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  // 表头
  std::cout << std::left << std::setw(12) << "Pose Type" << std::setw(8) << "X" << std::setw(10) << "Y" << std::setw(10)
            << "Z" << std::setw(8) << "QX" << std::setw(10) << "QY" << std::setw(10) << "QZ" << std::setw(8) << "QW"
            << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  // 打印左手数据
  std::cout << std::left << std::setw(12) << "Left Hand" << std::fixed << std::setprecision(3) << std::setw(8)
            << leftHandPose.position.x() << std::setw(10) << leftHandPose.position.y() << std::setw(10)
            << leftHandPose.position.z() << std::setw(8) << leftHandPose.quaternion.x() << std::setw(10)
            << leftHandPose.quaternion.y() << std::setw(10) << leftHandPose.quaternion.z() << std::setw(8)
            << leftHandPose.quaternion.w() << std::endl;

  // 打印右手数据
  std::cout << std::left << std::setw(12) << "Right Hand" << std::fixed << std::setprecision(3) << std::setw(8)
            << rightHandPose.position.x() << std::setw(10) << rightHandPose.position.y() << std::setw(10)
            << rightHandPose.position.z() << std::setw(8) << rightHandPose.quaternion.x() << std::setw(10)
            << rightHandPose.quaternion.y() << std::setw(10) << rightHandPose.quaternion.z() << std::setw(8)
            << rightHandPose.quaternion.w() << std::endl;

  // 打印左肘数据
  std::cout << std::left << std::setw(12) << "Left Elbow" << std::fixed << std::setprecision(3) << std::setw(8)
            << leftElbowPose.position.x() << std::setw(10) << leftElbowPose.position.y() << std::setw(10)
            << leftElbowPose.position.z() << std::setw(8) << leftElbowPose.quaternion.x() << std::setw(10)
            << leftElbowPose.quaternion.y() << std::setw(10) << leftElbowPose.quaternion.z() << std::setw(8)
            << leftElbowPose.quaternion.w() << std::endl;

  // 打印右肘数据
  std::cout << std::left << std::setw(12) << "Right Elbow" << std::fixed << std::setprecision(3) << std::setw(8)
            << rightElbowPose.position.x() << std::setw(10) << rightElbowPose.position.y() << std::setw(10)
            << rightElbowPose.position.z() << std::setw(8) << rightElbowPose.quaternion.x() << std::setw(10)
            << rightElbowPose.quaternion.y() << std::setw(10) << rightElbowPose.quaternion.z() << std::setw(8)
            << rightElbowPose.quaternion.w() << std::endl;

  std::cout << std::string(80, '=') << std::endl;
  std::cout << std::endl;
}