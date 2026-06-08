#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
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
  bool isHandEndEffectorType = (type == EndEffectorType::QIANGNAO || type == EndEffectorType::QIANGNAO_TOUCH || type == EndEffectorType::REVO2);
  // if (isHandEndEffectorType) {
  //   std::cout << "isHandEndEffectorType: " << static_cast<int>(type) << std::endl;
  //   std::cout << "QIANGNAO: " << static_cast<int>(EndEffectorType::QIANGNAO) << std::endl;
  //   std::cout << "QIANGNAO_TOUCH: " << static_cast<int>(EndEffectorType::QIANGNAO_TOUCH) << std::endl;
  //   std::cout << "REVO2: " << static_cast<int>(EndEffectorType::REVO2) << std::endl;
  //   std::cout << "LEJUCLAW: " << static_cast<int>(EndEffectorType::LEJUCLAW) << std::endl;
  // }

  return isHandEndEffectorType;
}

inline bool isClawEndEffectorType(EndEffectorType type) {
  bool isClawEndEffectorType = (type == EndEffectorType::LEJUCLAW);
  // if (isClawEndEffectorType) {
  //   std::cout << "isClawEndEffectorType: " << static_cast<int>(type) << std::endl;
  //   std::cout << "LEJUCLAW: " << static_cast<int>(EndEffectorType::LEJUCLAW) << std::endl;
  // }

  return isClawEndEffectorType;
}

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
#define POSE_DATA_LIST_SIZE_PLUS 13  // add [l_link6_pos, r_link6_pos, l_vir_thumb_pos, r_vir_thumb_pos]

#define POSE_DATA_LIST_INDEX_LEFT_LINK6 5           // 左link6
#define POSE_DATA_LIST_INDEX_RIGHT_LINK6 6          // 右link6
#define POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB 7   // 左虚拟拇指
#define POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB 8  // 右虚拟拇指
#define POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR 9    // 左end_effector
#define POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR 10  // 右end_effector
#define POSE_DATA_LIST_INDEX_LEFT_SHOULDER 11       // 左肩
#define POSE_DATA_LIST_INDEX_RIGHT_SHOULDER 12      // 右肩

struct TwoStageIKParameters {
  std::vector<std::string> ikConstraintFrameNames;
  double constraintTolerance = 1.0e-6;
  double solverTolerance = 1.0e-5;
  int maxSolverIterations = 2000;
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
enum class KuavoArmCtrlMode { ARM_FIXED = 0, AUTO_SWING = 1, EXTERNAL_CONTROL = 2 };
enum class MpcRefUpdateMode { DISABLED_ARM = 0, ENABLED_ARM = 1 };
enum class IncrementalMpcCtrlMode { NO_CONTROL = 0, ARM_ONLY = 1, BASE_ONLY = 2, BASE_ARM = 3, ERROR = -1 };
enum class ControlMode { NONE = 0, INCREMENTAL = 1 };
enum class VRHandControlType { NONE = 0, LEFT_HAND = 1, TWO_HAND = 2 };

// 枚举：左右侧和身体索引（用于 UpperBodyPoseList）
enum class UpperBodySide : int { LEFT = 0, RIGHT = 1, BODY = 2 };

// 枚举：身体部位索引（用于 UpperBodyPoseList）
enum class UpperBodyPart : int { SHOULDER = 0, HAND = 1, ELBOW = 2, WRIST = 3, WAIST = 4 };

// 便捷别名：简化访问方式，支持 data[LEFT][SHOULDER].p
constexpr UpperBodySide LEFT = UpperBodySide::LEFT;
constexpr UpperBodySide RIGHT = UpperBodySide::RIGHT;
constexpr UpperBodySide BODY = UpperBodySide::BODY;
constexpr UpperBodyPart SHOULDER = UpperBodyPart::SHOULDER;
constexpr UpperBodyPart HAND = UpperBodyPart::HAND;
constexpr UpperBodyPart ELBOW = UpperBodyPart::ELBOW;
constexpr UpperBodyPart WRIST = UpperBodyPart::WRIST;
constexpr UpperBodyPart WAIST = UpperBodyPart::WAIST;

/**
 * @brief 将一个位姿从世界坐标系转换到指定的参考坐标系中
 * 
 * @param refQuatW 参考坐标系在世界坐标系下的姿态 (e.g., chestQuatW)
 * @param refPosW 参考坐标系在世界坐标系下的位置 (e.g., chestPosW)
 * @param targetQuatW 目标物体在世界坐标系下的姿态 (e.g., handQuatW)
 * @param targetPosW 目标物体在世界坐标系下的位置 (e.g., handPosW)
 * @return std::pair<Quaterniond, Vector3d> 目标物体在参考坐标系下的位姿 (姿态, 位置)
 */
 inline std::pair<Eigen::Quaterniond, Eigen::Vector3d> transformPose(
  const Eigen::Quaterniond& refQuatW, 
  const Eigen::Vector3d& refPosW, 
  const Eigen::Quaterniond& targetQuatW, 
  const Eigen::Vector3d& targetPosW) 
{
  // 计算从世界坐标系 (W) 到参考坐标系 (C, Chest) 的旋转
  // 这是从参考坐标系到世界坐标系旋转的逆
  // 对于单位四元数，其逆等于其共轭
  Eigen::Quaterniond quatW_C = refQuatW.inverse(); 

  // 1. 计算目标在参考坐标系下的姿态 (handQuatC)
  // q_H_C = q_W_C * q_H_W
  Eigen::Quaterniond targetQuatC = quatW_C * targetQuatW;

  // 2. 计算目标在参考坐标系下的位置 (handPosC)
  // 首先，计算从参考坐标系原点指向目标物体原点的向量在世界坐标系下的表示
  Eigen::Vector3d vec_C_H_in_W = targetPosW - refPosW;
  // 然后，将这个向量旋转到参考坐标系中
  // p_H_C = q_W_C * (p_H_W - p_C_W)
  Eigen::Vector3d targetPosC = quatW_C * vec_C_H_in_W;

  return std::make_pair(targetQuatC, targetPosC);
}

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

// 位姿结构体：包含位置和四元数
struct Pose {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

// 上身位姿列表：统一使用 data，支持 data[LEFT][SHOULDER].p 和 data[BODY][WAIST].p 访问方式
// 也可以使用完整枚举：data[UpperBodySide::LEFT][UpperBodyPart::SHOULDER].p
struct UpperBodyPoseList {
  Pose data[3][5];  // [LEFT/RIGHT/BODY][SHOULDER/HAND/LINK4/LINK6/WAIST]，允许冗余

  UpperBodyPoseList() {
    // 初始化所有位姿（允许冗余）
    for (int side = 0; side < 3; ++side) {
      for (int part = 0; part < 5; ++part) {
        data[side][part].p = Eigen::Vector3d::Zero();
        data[side][part].q = Eigen::Quaterniond::Identity();
      }
    }
  }

  // 嵌套类用于支持二维索引访问
  struct SideAccessor {
    Pose* row;
    SideAccessor(Pose* r) : row(r) {}
    Pose& operator[](UpperBodyPart part) { return row[static_cast<int>(part)]; }
    const Pose& operator[](UpperBodyPart part) const { return row[static_cast<int>(part)]; }
  };

  // 返回 SideAccessor 以支持链式访问
  SideAccessor operator[](UpperBodySide side) { return SideAccessor(data[static_cast<int>(side)]); }

  const SideAccessor operator[](UpperBodySide side) const {
    return SideAccessor(const_cast<Pose*>(data[static_cast<int>(side)]));
  }
};

inline VRHandControlType intToVRHandControlType(int value) {
  if (value < 0 || value > 2) {
    std::cerr << "Warning: Invalid VRHandControlType value: " << value << ", using NONE as default" << std::endl;
    return VRHandControlType::NONE;
  }
  return static_cast<VRHandControlType>(value);
}

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

  ArmPose(const std::string& robotModel, bool isLeftHand) {
    if (robotModel == "kuavo_45") {
      if (isLeftHand) {
        // 左手默认位姿: 0.073, 0.25, -0.26, 0, 0, 0, 1
        position << 0.073, 0.25, -0.26;
        quaternion = Eigen::Quaterniond::Identity();
      } else {
        // 右手默认位姿: 0.073, -0.25, -0.26, 0, 0, 0, 1
        position << 0.073, -0.25, -0.26;
        quaternion = Eigen::Quaterniond::Identity();
      }
    } else {
      // 其他型号：初始化为零向量和单位四元数
      position = Eigen::Vector3d::Zero();
      quaternion = Eigen::Quaterniond::Identity();
    }
  }

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
        maxIterations(3000),
        controlArmIndex(ArmIdx::BOTH),
        isWeldBaseLink(true),
        useJointLimits(true),  // Disabled by default (follow Python behavior)
        jointLowerBounds(Eigen::VectorXd()),
        jointUpperBounds(Eigen::VectorXd()) {}

  IKSolverConfig(double constraintTol, double solverTol, int maxIter, ArmIdx armIdx, bool weldBase)
      : constraintTolerance(constraintTol),
        solverTolerance(solverTol),
        maxIterations(maxIter),
        controlArmIndex(armIdx),
        isWeldBaseLink(weldBase),
        useJointLimits(true),
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

inline Eigen::Quaterniond limitQuaternionAngleEulerZYX(const Eigen::Quaterniond& q_input,
                                                       const Eigen::Vector3d& zyxLimits,
                                                       const Eigen::Vector3d scale = Eigen::Vector3d::Ones()) {
  // 1. 归一化并处理双倍覆盖 (Double Cover)
  // 确保 w >= 0，保证我们在“最短路径”半球面上进行分解
  Eigen::Quaterniond q = q_input.normalized();
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }

  // 2. 手动分解欧拉角 (Z-Y-X 顺序)
  // 使用 atan2 确保角度范围在 [-PI, PI]，这是限幅算法正常工作的基础
  // 公式参考：Standard 3-2-1 Euler Angles extraction from Quaternion
  double yaw_z = 0.0;
  double pitch_y = 0.0;
  double roll_x = 0.0;

  // --- Roll (X-axis rotation) ---
  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll_x = std::atan2(sinr_cosp, cosr_cosp);

  // --- Pitch (Y-axis rotation) ---
  // 警告：Pitch 接近 90 度时存在奇点 (Gimbal Lock)，但在你的 ±60 度限位下是安全的。
  // 我们必须 clamp 输入值，因为浮点误差可能导致值略微超出 [-1, 1]，使 asin 返回 NaN。
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1.0) {
    // 极少见情况，直接根据符号给 PI/2
    pitch_y = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch_y = std::asin(sinp);
  }

  // --- Yaw (Z-axis rotation) ---
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw_z = std::atan2(siny_cosp, cosy_cosp);

  // 3. 核心限幅逻辑 (Box Limiting)
  // 针对每个轴独立进行限幅
  yaw_z = std::clamp(yaw_z * scale[0], -zyxLimits[0], zyxLimits[0]);
  pitch_y = std::clamp(pitch_y * scale[1], -zyxLimits[1], zyxLimits[1]);
  roll_x = std::clamp(roll_x * scale[2], -zyxLimits[2], zyxLimits[2]);
  roll_x = std::clamp(roll_x, -zyxLimits[2], zyxLimits[2]);

  // 4. 重构四元数
  // 顺序必须与分解顺序一致：Yaw(Z) -> Pitch(Y) -> Roll(X)
  // 在 Eigen 中，q = q_yaw * q_pitch * q_roll 代表先转 Roll，再 Pitch，再 Yaw (从局部坐标系看)
  // 或者理解为 R_total = R_z * R_y * R_x
  Eigen::AngleAxisd rollAngle(roll_x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_z, Eigen::Vector3d::UnitZ());

  // 乘法顺序：Z * Y * X
  return (yawAngle * pitchAngle * rollAngle).normalized();
}

/**
 * @brief 四元数转欧拉角 (Z-Y-X 顺序)，不做限幅/clip
 * @param q_input 输入四元数
 * @return 欧拉角向量 (yaw_z, pitch_y, roll_x)，单位: rad
 * @note 返回角度范围主要由 atan2/asin 决定：yaw/roll \in [-pi, pi]，pitch \in [-pi/2, pi/2]
 */
inline Eigen::Vector3d quaternionToEulerZYXNoClip(const Eigen::Quaterniond& q_input) {
  // 1. 归一化并处理双倍覆盖 (Double Cover)：确保 w >= 0，使用“最短路径”半球面
  Eigen::Quaterniond q = q_input.normalized();
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }

  // 2. 手动分解欧拉角 (Z-Y-X / 3-2-1)
  double yaw_z = 0.0;
  double pitch_y = 0.0;
  double roll_x = 0.0;

  // --- Roll (X-axis rotation) ---
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll_x = std::atan2(sinr_cosp, cosr_cosp);

  // --- Pitch (Y-axis rotation) ---
  // 数值安全：浮点误差可能导致略微超出 [-1, 1]，使 asin 返回 NaN
  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1.0) {
    pitch_y = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch_y = std::asin(sinp);
  }

  // --- Yaw (Z-axis rotation) ---
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw_z = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(yaw_z, pitch_y, roll_x);
}

/**
 * @brief 欧拉角转四元数 (Z-Y-X 顺序)，不做限幅/clip
 * @param euler_zyx 欧拉角向量 (yaw_z, pitch_y, roll_x)，单位: rad
 * @return 对应四元数（单位四元数，且做 double-cover 处理使 w >= 0）
 * @note 约定与 quaternionToEulerZYXNoClip() 保持一致：R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
inline Eigen::Quaterniond eulerZYXToQuaternionNoClip(const Eigen::Vector3d& euler_zyx) {
  const double yaw_z = euler_zyx[0];
  const double pitch_y = euler_zyx[1];
  const double roll_x = euler_zyx[2];

  const Eigen::AngleAxisd rollAngle(roll_x, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitchAngle(pitch_y, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yawAngle(yaw_z, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = (yawAngle * pitchAngle * rollAngle).normalized();
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }
  return q;
}

/**
 * @brief 传入两个四元数，分别转欧拉角(ZYX)，逐轴选择“幅值更小”的欧拉角分量，重组为新四元数返回
 * @param q_a 四元数 A
 * @param q_b 四元数 B
 * @return 新四元数（单位四元数，且做 double-cover 处理使 w >= 0）
 * @note
 * - 欧拉角定义与 quaternionToEulerZYXNoClip()/eulerZYXToQuaternionNoClip() 一致：返回/输入为 (yaw, pitch, roll)
 * - “最小”按绝对值比较：对每个轴选择 abs(angle) 更小的那个分量
 */
inline Eigen::Quaterniond pickMinAbsEulerComponentsZYXToQuaternion(const Eigen::Quaterniond& q_a,
                                                                   const Eigen::Quaterniond& q_b) {
  const Eigen::Vector3d e_a = quaternionToEulerZYXNoClip(q_a);
  const Eigen::Vector3d e_b = quaternionToEulerZYXNoClip(q_b);

  Eigen::Vector3d e_out;
  for (int i = 0; i < 3; ++i) {
    e_out[i] = (std::abs(e_a[i]) <= std::abs(e_b[i])) ? e_a[i] : e_b[i];
  }

  return eulerZYXToQuaternionNoClip(e_out);
}

/**
 * @brief 将Quest3的位姿转换为机器人的位姿
 * @tparam QuaternionT 四元数类型，需要有w, x, y, z成员
 * @param orientation 输入的四元数
 * @return 转换后的Eigen四元数
 */
template <typename QuaternionT>
inline Eigen::Quaterniond transformQuestPoseTORobotPose(const QuaternionT& orientation) {
  Eigen::Quaterniond qCurrent = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).conjugate() *
                                Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);

  Eigen::Quaterniond qRemap = Eigen::Quaterniond(qCurrent.w(),  //
                                                 qCurrent.z(),  // x
                                                 qCurrent.x(),
                                                 qCurrent.y());  // z x →z

  Eigen::Quaterniond pitchOffset(Eigen::AngleAxisd(-M_PI / 12, Eigen::Vector3d::UnitY()));
  qRemap = pitchOffset * qRemap;

  return qRemap;
}

inline Eigen::Quaterniond limitQuaternionAngle(const Eigen::Quaterniond& q_input, double max_angle_rad) {
  Eigen::Quaterniond q = q_input.normalized();

  // 1. Shortest Path (双倍覆盖处理)
  // 确保 w >= 0，这样旋转角度永远在 [-180, 180] 之间
  // 如果 w < 0，说明这个四元数表示的是转了 > 180 度 (长路径)，
  // 或者仅仅是符号翻转。取反不会改变旋转姿态，但能保证后续角度计算正确。
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }

  // 2. 计算旋转角度
  // q.w() = cos(theta / 2)  =>  theta = 2 * acos(w)
  // 此时因为 w >= 0，所以 theta 在 [0, 180] 之间
  double angle = 2.0 * std::acos(std::clamp(q.w(), -1.0, 1.0));

  // 3. 避免极小角度除零风险
  if (std::abs(angle) < 1e-6) {
    return q;
  }

  // 4. 限幅 (Clamping)
  // 如果角度超过限制，使用 Slerp 插值截断到边界
  if (angle > max_angle_rad) {
    // 计算截断比例： t * angle = max_angle  =>  t = max_angle / angle
    double t = max_angle_rad / angle;
    // 从 Identity (0度) 向 q (目标度) 插值 t
    return Eigen::Quaterniond::Identity().slerp(t, q).normalized();
  }

  return q;
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

inline kuavo_msgs::twoArmHandPoseCmd createTwoArmHandPoseCmd(const Eigen::Vector3d& leftHandPos,
                                                             const Eigen::Vector3d& rightHandPos,
                                                             const Eigen::Quaterniond& leftHandQuat,
                                                             const Eigen::Quaterniond& rightHandQuat,
                                                             const Eigen::Vector3d& leftElbowPos,
                                                             const Eigen::Vector3d& rightElbowPos,
                                                             const HighlyDynamic::IKParams& ikParams) {
  kuavo_msgs::twoArmHandPoseCmd two_arm_hand_pose_msg;

  // 设置消息头
  two_arm_hand_pose_msg.hand_poses.header.stamp = ros::Time::now();
  two_arm_hand_pose_msg.hand_poses.header.frame_id = "base_link";

  // 设置左手位姿数据

  two_arm_hand_pose_msg.hand_poses.left_pose.pos_xyz[0] = leftHandPos.x();
  two_arm_hand_pose_msg.hand_poses.left_pose.pos_xyz[1] = leftHandPos.y();
  two_arm_hand_pose_msg.hand_poses.left_pose.pos_xyz[2] = leftHandPos.z();

  two_arm_hand_pose_msg.hand_poses.left_pose.quat_xyzw[0] = leftHandQuat.x();
  two_arm_hand_pose_msg.hand_poses.left_pose.quat_xyzw[1] = leftHandQuat.y();
  two_arm_hand_pose_msg.hand_poses.left_pose.quat_xyzw[2] = leftHandQuat.z();
  two_arm_hand_pose_msg.hand_poses.left_pose.quat_xyzw[3] = leftHandQuat.w();

  // 设置左手肘部位置
  two_arm_hand_pose_msg.hand_poses.left_pose.elbow_pos_xyz[0] = leftElbowPos.x();
  two_arm_hand_pose_msg.hand_poses.left_pose.elbow_pos_xyz[1] = leftElbowPos.y();
  two_arm_hand_pose_msg.hand_poses.left_pose.elbow_pos_xyz[2] = leftElbowPos.z();

  two_arm_hand_pose_msg.hand_poses.right_pose.pos_xyz[0] = rightHandPos.x();
  two_arm_hand_pose_msg.hand_poses.right_pose.pos_xyz[1] = rightHandPos.y();
  two_arm_hand_pose_msg.hand_poses.right_pose.pos_xyz[2] = rightHandPos.z();

  two_arm_hand_pose_msg.hand_poses.right_pose.quat_xyzw[0] = rightHandQuat.x();
  two_arm_hand_pose_msg.hand_poses.right_pose.quat_xyzw[1] = rightHandQuat.y();
  two_arm_hand_pose_msg.hand_poses.right_pose.quat_xyzw[2] = rightHandQuat.z();
  two_arm_hand_pose_msg.hand_poses.right_pose.quat_xyzw[3] = rightHandQuat.w();

  // 设置右手肘部位置
  two_arm_hand_pose_msg.hand_poses.right_pose.elbow_pos_xyz[0] = rightElbowPos.x();  // 默认模式时，与hand_x一致
  two_arm_hand_pose_msg.hand_poses.right_pose.elbow_pos_xyz[1] = rightElbowPos.y();  // 默认模式时，与hand_y一致
  two_arm_hand_pose_msg.hand_poses.right_pose.elbow_pos_xyz[2] = rightElbowPos.z();  // 默认模式时，与hand_z/2一致

  // 设置IK求解参数 (参考Python文件中的参数设置)
  two_arm_hand_pose_msg.use_custom_ik_param = true;
  two_arm_hand_pose_msg.joint_angles_as_q0 = false;
  two_arm_hand_pose_msg.frame = 0;  // 保持当前坐标系

  // 设置IK求解器参数
  two_arm_hand_pose_msg.ik_param.major_optimality_tol = ikParams.major_optimality_tol;
  two_arm_hand_pose_msg.ik_param.major_feasibility_tol = ikParams.major_feasibility_tol;
  two_arm_hand_pose_msg.ik_param.minor_feasibility_tol = ikParams.minor_feasibility_tol;
  two_arm_hand_pose_msg.ik_param.major_iterations_limit = ikParams.major_iterations_limit;
  two_arm_hand_pose_msg.ik_param.oritation_constraint_tol = ikParams.oritation_constraint_tol;
  two_arm_hand_pose_msg.ik_param.pos_constraint_tol = ikParams.pos_constraint_tol;
  two_arm_hand_pose_msg.ik_param.pos_cost_weight = ikParams.pos_cost_weight;

  return two_arm_hand_pose_msg;
}

inline bool setDefaultSrvIkParams(HighlyDynamic::IKParams& ikParams) {
  ikParams.major_optimality_tol = 9e-3;
  ikParams.major_feasibility_tol = 9e-3;
  ikParams.minor_feasibility_tol = 9e-3;
  ikParams.major_iterations_limit = 50;
  ikParams.oritation_constraint_tol = 9e-3;
  ikParams.pos_constraint_tol = 9e-3;
  ikParams.pos_cost_weight = 10.0;

  return true;
}

inline bool checkPositionError(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double threshold) {
  return ((p1 - p2).norm() - 1e-4) < threshold;
}

class HumanArmMoveDetector {
 public:
  explicit HumanArmMoveDetector(bool enableDebugPrint = false)
      : debugPrint_(enableDebugPrint),
        isFirstFrame_(true),
        hasHumanArmMoved_(false),
        prevLeftHandPosition_(Eigen::Vector3d::Zero()),
        prevRightHandPosition_(Eigen::Vector3d::Zero()) {}

  void reset() {
    isFirstFrame_ = true;
    hasHumanArmMoved_ = false;
    prevLeftHandPosition_.setZero();
    prevRightHandPosition_.setZero();
  }

  bool detectMovement(const Eigen::Vector3d& currentLeftHandPos,
                      const Eigen::Vector3d& currentRightHandPos,
                      double threshold = 0.01) {
    // 如果已经检测到移动，直接返回true，不再重复检测
    if (hasHumanArmMoved_) {
      return true;
    }

    // 如果是第一帧，记录当前位置并返回false
    if (isFirstFrame_) {
      prevLeftHandPosition_ = currentLeftHandPos;
      prevRightHandPosition_ = currentRightHandPos;
      isFirstFrame_ = false;
      return false;
    }

    // 计算左右手位置的norm误差
    double leftHandNormError = (currentLeftHandPos - prevLeftHandPosition_).norm();
    double rightHandNormError = (currentRightHandPos - prevRightHandPosition_).norm();

    // 更新上一帧位置
    prevLeftHandPosition_ = currentLeftHandPos;
    prevRightHandPosition_ = currentRightHandPos;

    // 检查是否有任一手的移动超过阈值
    bool hasMovement = (leftHandNormError > threshold) || (rightHandNormError > threshold);

    // 更新移动状态
    if (hasMovement) {
      hasHumanArmMoved_ = true;
    }

    // 可选：添加调试信息
    if (debugPrint_ && hasMovement) {
      std::cout << "Human arm movement detected - Left: " << std::fixed << std::setprecision(4) << leftHandNormError
                << ", Right: " << rightHandNormError << ", Threshold: " << threshold << std::endl;
    }

    return hasMovement;
  }

  bool hasHumanArmMoved() const { return hasHumanArmMoved_; }

 private:
  bool debugPrint_;                        ///< 调试打印开关
  bool isFirstFrame_;                      ///< 是否为第一帧数据
  bool hasHumanArmMoved_;                  ///< 记录是否检测到人体手臂移动
  Eigen::Vector3d prevLeftHandPosition_;   ///< 上一帧左手位置
  Eigen::Vector3d prevRightHandPosition_;  ///< 上一帧右手位置
};

// ============== 四元数验证函数 ==============

// 验证ROS geometry_msgs::Quaternion类型的四元数
template <typename QuaternionT>
inline bool validateROSQuaternion(const QuaternionT& quat, const std::string& context = "", bool printWarning = true) {
  double quatNorm = sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);

  if (quatNorm < 1e-6 || !std::isfinite(quatNorm)) {
    if (printWarning) {
      std::cerr << "⚠️ [" << context << "] Invalid quaternion norm: " << quatNorm << std::endl;
    }
    return false;
  }
  return true;
}

// 验证并归一化ROS四元数，返回旋转矩阵
template <typename QuaternionT>
inline Eigen::Matrix3d validateAndConvertROSQuaternion(const QuaternionT& quat, const std::string& context = "") {
  if (!validateROSQuaternion(quat, context)) {
    return Eigen::Matrix3d::Identity();  // 返回单位矩阵作为默认值
  }

  Eigen::Quaterniond eigenQuat(quat.w, quat.x, quat.y, quat.z);
  eigenQuat.normalize();
  return eigenQuat.toRotationMatrix();
}

// 验证Eigen四元数
inline bool validateEigenQuaternion(const Eigen::Quaterniond& quat,
                                    const std::string& context = "",
                                    bool printWarning = true) {
  if (!quat.coeffs().allFinite() || std::abs(quat.norm() - 1.0) > 1e-4) {
    if (printWarning) {
      std::cerr << "⚠️ [" << context << "] Invalid Eigen quaternion" << std::endl;
    }
    return false;
  }
  return true;
}

// ============== 数值验证函数 ==============

// 验证单个Vector3d
inline bool validateVector3d(const Eigen::Vector3d& vec,
                             const std::string& context = "",
                             double maxValue = 1e6,
                             bool allowZero = true,
                             bool printWarning = true) {
  bool hasNaN = vec.hasNaN();
  bool hasInf = !vec.allFinite();
  bool isZero = vec.isZero(1e-10);
  bool isExtreme = (vec.array().abs() > maxValue).any();

  if (printWarning) {
    if (hasNaN) std::cerr << "⚠️ [" << context << "] Vector contains NaN!" << std::endl;
    if (hasInf) std::cerr << "⚠️ [" << context << "] Vector contains infinite value!" << std::endl;
    if (isZero && !allowZero) std::cout << "⚠️ [" << context << "] Vector is zero!" << std::endl;
    if (isExtreme) std::cerr << "⚠️ [" << context << "] Vector contains extreme value!" << std::endl;
  }

  return !hasNaN && !hasInf && !isExtreme && (allowZero || !isZero);
}

// 验证VectorXd
inline bool validateVectorXd(const Eigen::VectorXd& vec,
                             const std::string& context = "",
                             double maxValue = 1e6,
                             bool printWarning = true) {
  for (int i = 0; i < vec.size(); ++i) {
    if (!std::isfinite(vec(i)) || std::abs(vec(i)) > maxValue) {
      if (printWarning) {
        std::cerr << "⚠️ [" << context << "] Invalid value at index " << i << ": " << vec(i) << std::endl;
      }
      return false;
    }
  }
  return true;
}

// 验证ROS Point
template <typename PointT>
inline bool validateROSPoint(const PointT& point, const std::string& context = "", bool printWarning = true) {
  Eigen::Vector3d vec(point.x, point.y, point.z);
  return validateVector3d(vec, context, 1e6, true, printWarning);
}

// ============== 位姿安全检查函数 ==============

// 计算四元数角度差异
inline double quaternionAngleDifference(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
  double dotProduct = std::abs(q1.dot(q2));
  dotProduct = std::clamp(dotProduct, 0.0, 1.0);
  return 2.0 * std::acos(dotProduct);
}

// 验证索引范围
inline bool validateIndex(size_t index,
                          size_t containerSize,
                          const std::string& context = "",
                          bool printWarning = true) {
  if (index >= containerSize) {
    if (printWarning) {
      std::cerr << "⚠️ [" << context << "] Index out of range: " << index << " >= " << containerSize << std::endl;
    }
    return false;
  }
  return true;
}

// 验证关节角度限制
inline bool validateJointLimits(const Eigen::VectorXd& jointAngles,
                                const Eigen::VectorXd& lowerLimits,
                                const Eigen::VectorXd& upperLimits,
                                const std::string& context = "Joint Limits",
                                bool printWarning = true) {
  if (jointAngles.size() != lowerLimits.size() || jointAngles.size() != upperLimits.size()) {
    if (printWarning) {
      std::cerr << "⚠️ [" << context << "] Size mismatch in joint limits!" << std::endl;
    }
    return false;
  }

  for (int i = 0; i < jointAngles.size(); ++i) {
    if (jointAngles(i) < lowerLimits(i) || jointAngles(i) > upperLimits(i)) {
      if (printWarning) {
        std::cerr << "⚠️ [" << context << "] Joint " << i << " out of limits: " << jointAngles(i) << " not in ["
                  << lowerLimits(i) << ", " << upperLimits(i) << "]" << std::endl;
      }
      return false;
    }
  }

  return true;
}

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

// 位置约束函数：通过球体约束限制位置
inline void clipPositionBySphere(Eigen::Vector3d& position,
                                 const Eigen::Vector3d& center,
                                 double sphereRadius,
                                 double minReachableDistance) {
  const double epsilon = 1e-6;  // 避免除以零的小值

  // 对位置进行球约束
  Eigen::Vector3d positionToCenter = position - center;
  double distance = positionToCenter.norm();
  if (distance > epsilon) {  // 避免除以零
    if (distance > sphereRadius) {
      position = center + positionToCenter.normalized() * sphereRadius;
    } else if (distance < minReachableDistance) {
      position = center + positionToCenter.normalized() * minReachableDistance;
    }
  }
}

// 位置约束函数：通过边界框约束限制位置
inline void clipPositionByBox(Eigen::Vector3d& position,
                              const Eigen::Vector3d& boxMinBound,
                              const Eigen::Vector3d& boxMaxBound) {
  // 对坐标进行边界约束（限制在 [min, max] 范围内）
  if (position.x() < boxMinBound.x()) {
    position.x() = boxMinBound.x();
  } else if (position.x() > boxMaxBound.x()) {
    position.x() = boxMaxBound.x();
  }

  if (position.y() < boxMinBound.y()) {
    position.y() = boxMinBound.y();
  } else if (position.y() > boxMaxBound.y()) {
    position.y() = boxMaxBound.y();
  }

  if (position.z() < boxMinBound.z()) {
    position.z() = boxMinBound.z();
  } else if (position.z() > boxMaxBound.z()) {
    position.z() = boxMaxBound.z();
  }
}

inline void clipPositionByChestMidline(Eigen::Vector3d& position, bool isLeftHand, double chestOffset) {
  if (isLeftHand) {
    // 左手：y 轴最大值不超过 0 - chestOffset
    if (position.y() < chestOffset) {
      position.y() = chestOffset;
    }
  } else {
    // 右手：y 轴最小值不小于 0 + chestOffset
    if (position.y() > -chestOffset) {
      position.y() = -chestOffset;
    }
  }
}

// 位置约束函数：通过圆柱体约束限制位置（仅约束xOy平面投影，z不约束）
// 参考clipPositionBySphere的下界约束逻辑，不约束上界
inline void clipPositionByCylinder(Eigen::Vector3d& position,
                                   const Eigen::Vector3d& center,
                                   double minReachableDistance) {
  const double epsilon = 1e-6;  // 避免除以零的小值

  // 计算从center到position的向量
  Eigen::Vector3d positionToCenter = position - center;

  // 提取xOy平面投影向量（z分量设为0）
  Eigen::Vector2d projectionXY(positionToCenter.x(), positionToCenter.y());
  double projectionDistance = projectionXY.norm();

  if (projectionDistance > epsilon) {  // 避免除以零
    // 仅约束下界：如果投影距离小于minReachableDistance，则缩放到minReachableDistance
    if (projectionDistance < minReachableDistance) {
      // 计算缩放后的xOy平面投影向量
      Eigen::Vector2d scaledProjection = projectionXY.normalized() * minReachableDistance;

      // 更新position：保持z坐标不变，只更新x和y坐标
      position.x() = center.x() + scaledProjection.x();
      position.y() = center.y() + scaledProjection.y();
      // z坐标保持不变：position.z() = position.z()
    }
    // 不约束上界：如果投影距离大于minReachableDistance，不做任何处理
  }
}

// 位置约束函数：裁剪位置的下边界（x轴下界和z轴上界）
inline void clipBoxBackCeiling(Eigen::Vector3d& position,
                               const Eigen::Vector3d& boxMinBound,
                               const Eigen::Vector3d& boxMaxBound) {
  if (position.x() < boxMinBound.x()) position.x() = boxMinBound.x();
  if (position.z() > boxMaxBound.z()) position.z() = boxMaxBound.z();
}

inline void clipPositionByElbowDistance(Eigen::Vector3d& handPos,
                                        const Eigen::Vector3d& elbowPos,
                                        double minDistance,
                                        double maxDistance) {
  Eigen::Vector3d handToElbow = handPos - elbowPos;
  double squaredDistance = handToElbow.squaredNorm();

  // 使用更小的阈值，基于平方距离
  const double epsilonSquared = 1e-12;  // 对应 epsilon = 1e-6 的平方

  if (squaredDistance <= epsilonSquared) {
    // 如果距离非常小（接近零），保持原位置不变，避免数值不稳定
    return;
  }

  double distance = std::sqrt(squaredDistance);

  // 定义平滑过渡区域的宽度（边界附近的百分比）
  // 允许在过渡区域内轻微超出约束，但会平滑地拉回
  const double transitionRatio = 0.1;                                 // 10% 的过渡区域
  const double minSoftBound = minDistance * (1.0 - transitionRatio);  // 软下界
  const double maxSoftBound = maxDistance * (1.0 + transitionRatio);  // 软上界

  // 平滑插值函数：smoothstep (3t^2 - 2t^3)，在 [0,1] 区间内平滑过渡
  // 提供 C1 连续性（一阶导数连续）
  auto smoothstep = [](double t) {
    t = std::max(0.0, std::min(1.0, t));  // 限制在 [0,1]
    return t * t * (3.0 - 2.0 * t);
  };

  double targetDistance = distance;

  if (distance < minDistance) {
    if (distance <= minSoftBound) {
      // 超出软下界：完全约束到最小值
      targetDistance = minDistance;
    } else {
      // 在过渡区域内：平滑插值，允许轻微超出 minDistance
      // t = 0 在 minSoftBound，t = 1 在 minDistance
      double t = (distance - minSoftBound) / (minDistance - minSoftBound);
      double smoothFactor = smoothstep(t);
      targetDistance = minSoftBound + (minDistance - minSoftBound) * smoothFactor;
    }
  } else if (distance > maxDistance) {
    if (distance >= maxSoftBound) {
      // 超出软上界：完全约束到最大值
      targetDistance = maxDistance;
    } else {
      // 在过渡区域内：平滑插值，允许轻微超出 maxDistance
      // t = 0 在 maxDistance，t = 1 在 maxSoftBound
      double t = (distance - maxDistance) / (maxSoftBound - maxDistance);
      double smoothFactor = smoothstep(t);
      // 反向插值：距离越大，越接近 maxDistance
      targetDistance = maxDistance + (maxSoftBound - maxDistance) * (1.0 - smoothFactor);
    }
  } else {
    // 在 [minDistance, maxDistance] 范围内：完全允许，不做调整
    return;
  }

  // 使用稳定的缩放方法
  double scale = targetDistance / distance;
  handPos = elbowPos + handToElbow * scale;
}

// 位置约束函数：同时裁剪左右手位置的下边界（x轴下界和z轴上界）
inline void clipBoxBackCeilingBothHands(Eigen::Vector3d& leftHandPos,
                                        Eigen::Vector3d& rightHandPos,
                                        const Eigen::Vector3d& boxMinBound,
                                        const Eigen::Vector3d& boxMaxBound) {
  clipBoxBackCeiling(leftHandPos, boxMinBound, boxMaxBound);
  clipBoxBackCeiling(rightHandPos, boxMinBound, boxMaxBound);
}

// 位置约束函数：同时通过球体约束限制左右手位置
inline void clipPositionBySphereBothHands(Eigen::Vector3d& leftHandPos,
                                          Eigen::Vector3d& rightHandPos,
                                          const Eigen::Vector3d& leftCenter,
                                          const Eigen::Vector3d& rightCenter,
                                          double sphereRadius,
                                          double minReachableDistance) {
  clipPositionBySphere(leftHandPos, leftCenter, sphereRadius, minReachableDistance);
  clipPositionBySphere(rightHandPos, rightCenter, sphereRadius, minReachableDistance);
}

// 位置约束函数：同时通过圆柱体约束限制左右手位置
inline void clipPositionByCylinderBothHands(Eigen::Vector3d& leftHandPos,
                                            Eigen::Vector3d& rightHandPos,
                                            const Eigen::Vector3d& leftCenter,
                                            const Eigen::Vector3d& rightCenter,
                                            double minReachableDistance) {
  clipPositionByCylinder(leftHandPos, leftCenter, minReachableDistance);
  clipPositionByCylinder(rightHandPos, rightCenter, minReachableDistance);
}

// 位置约束函数：同时通过胸部中线约束限制左右手位置
inline void clipPositionByChestMidlineBothHands(Eigen::Vector3d& leftHandPos,
                                                Eigen::Vector3d& rightHandPos,
                                                double chestOffset) {
  clipPositionByChestMidline(leftHandPos, true, chestOffset);    // 左手：y <= -chestOffset
  clipPositionByChestMidline(rightHandPos, false, chestOffset);  // 右手：y >= chestOffset
}

// 位置约束函数：综合应用所有约束条件限制左右手位置
// 依次应用：球体约束、圆柱体约束、边界框约束、胸部中线约束
inline void clipHandPositionsByAllConstraints(Eigen::Vector3d& leftHandPos,
                                              Eigen::Vector3d& rightHandPos,
                                              const Eigen::Vector3d& leftShoulderPos,
                                              const Eigen::Vector3d& rightShoulderPos,
                                              double sphereRadiusLimit,
                                              double minReachableDistance,
                                              const Eigen::Vector3d& leftCylinderCenter,
                                              const Eigen::Vector3d& rightCylinderCenter,
                                              double cylinderMinReachableDistance,
                                              const Eigen::Vector3d& boxMinBound,
                                              const Eigen::Vector3d& boxMaxBound,
                                              double chestOffsetY,
                                              const Eigen::Vector3d& currentLeftElbowPos,
                                              const Eigen::Vector3d& currentRightElbowPos,
                                              double elbowMinDistance,
                                              double elbowMaxDistance) {
  // 1. 球体约束：限制手部位置在肩部球体范围内
  clipPositionBySphereBothHands(
      leftHandPos, rightHandPos, leftShoulderPos, rightShoulderPos, sphereRadiusLimit, minReachableDistance);

  // 2. 圆柱体约束：限制手部位置在xOy平面的投影距离
  clipPositionByCylinderBothHands(
      leftHandPos, rightHandPos, leftCylinderCenter, rightCylinderCenter, cylinderMinReachableDistance);

  // 3. 边界框约束：限制x轴下界和z轴上界
  clipBoxBackCeilingBothHands(leftHandPos, rightHandPos, boxMinBound, boxMaxBound);

  // 4. 胸部中线约束：防止左右手过中线
  clipPositionByChestMidlineBothHands(leftHandPos, rightHandPos, chestOffsetY);

  clipPositionByCylinderBothHands(
      leftHandPos, rightHandPos, leftCylinderCenter, rightCylinderCenter, cylinderMinReachableDistance);

  //   clipPositionByElbowDistance(leftHandPos, currentLeftElbowPos, elbowMinDistance, elbowMaxDistance);
  //   clipPositionByElbowDistance(rightHandPos, currentRightElbowPos, elbowMinDistance, elbowMaxDistance);
}

// ============== 指针检查宏函数 ==============

/**
 * @brief 检查指针是否为空，如果为空则打印警告并返回（用于void函数）
 * @param ptr 要检查的指针
 * @param context 上下文信息（通常是函数名）
 * @param objName 对象名称（用于日志输出）
 * @note 使用此宏需要确保已包含 <ros/ros.h> 头文件
 */
#define CHECK_PTR_AND_RETURN_VOID(ptr, context, objName)      \
  if ((ptr) == nullptr) {                                     \
    ROS_WARN("[%s] %s is not initialized", context, objName); \
    return;                                                   \
  }

// ============== 参数读取和打印宏函数 ==============
/**
 * @brief 读取 ROS 参数并打印（浮点数）
 * @param nodeHandle ROS NodeHandle 对象
 * @param paramPath 参数路径（如 "/ik_ros_uni_cpp_node/quest3/fhan_r_joint"）
 * @param var 变量引用（用于存储读取的参数值）
 * @param defaultValue 默认值
 * @param precision 打印精度（默认2位）
 * @note 使用路径作为 tag 进行打印，格式为：paramPath=value
 */
#define PARAM_AND_PRINT_FLOAT(nodeHandle, paramPath, var, defaultValue, precision) \
  do {                                                                             \
    (nodeHandle).param((paramPath), (var), (defaultValue));                        \
    std::cout << (paramPath) << "=";                                               \
    std::cout << std::fixed << std::setprecision((precision));                     \
    std::cout << (var) << std::endl;                                               \
  } while (0)

/**
 * @brief 读取 ROS 参数并打印（Vector3d），带默认值和警告
 * @param nodeHandle ROS NodeHandle 对象
 * @param paramPath 参数路径
 * @param var 变量引用（用于存储读取的参数值）
 * @param defaultValue 默认值（Eigen::Vector3d）
 * @param precision 打印精度（默认2位）
 * @param warnTag 警告标签（用于 ROS_WARN，如 "[Quest3IkIncrementalROS]"）
 * @note 如果参数加载失败，会使用默认值并打印警告
 */
#define PARAM_AND_PRINT_VECTOR3D(nodeHandle, paramPath, var, defaultValue, precision, warnTag) \
  do {                                                                                         \
    std::vector<double> temp_vec;                                                              \
    if ((nodeHandle).getParam((paramPath), temp_vec) && temp_vec.size() == 3) {                \
      (var) = Eigen::Vector3d(temp_vec[0], temp_vec[1], temp_vec[2]);                          \
    } else {                                                                                   \
      (var) = (defaultValue);                                                                  \
      ROS_WARN("%s Failed to load %s, using default: [%.2f, %.2f, %.2f]",                      \
               (warnTag),                                                                      \
               (paramPath),                                                                    \
               (var).x(),                                                                      \
               (var).y(),                                                                      \
               (var).z());                                                                     \
    }                                                                                          \
    std::cout << (paramPath) << "=[";                                                          \
    std::cout << std::fixed << std::setprecision((precision));                                 \
    std::cout << (var).x() << ", " << (var).y() << ", " << (var).z() << "]" << std::endl;      \
  } while (0)
