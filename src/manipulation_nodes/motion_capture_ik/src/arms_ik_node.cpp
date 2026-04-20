#include <iostream>
#include <Eigen/Core>
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>
#include <optional>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/package.h>
#include <fstream>

#include "plantIK.h"
#include "motion_capture_ik/package_path.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/utils.hpp"
#include "motion_capture_ik/AnalyticArmIk.hpp"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"

// // msgs
// #include "motion_capture_ik/twoArmHandPoseCmd.h"
// #include "motion_capture_ik/twoArmHandPose.h"
// #include "motion_capture_ik/headBodyPose.h"
// // srv
// #include "motion_capture_ik/twoArmHandPoseCmdSrv.h"
// #include "motion_capture_ik/fkSrv.h"

#include "kuavo_msgs/twoArmHandPoseCmd.h"
#include "kuavo_msgs/twoArmHandPose.h"
#include "kuavo_msgs/headBodyPose.h"

#include "kuavo_msgs/twoArmHandPoseCmdSrv.h"
#include "kuavo_msgs/fkSrv.h"
#include "kuavo_msgs/twoArmHandPoseCmdFreeSrv.h"
#include "kuavo_msgs/twoArmHandPoseFree.h"
#include "kuavo_msgs/sensorsData.h"
#include <mutex>


namespace
{
    const double TO_RADIAN = M_PI / 180.0;
    const double TO_DEGREE = 180.0 / M_PI;
};

namespace HighlyDynamic{
struct IkCmd
{
    Eigen::Vector3d pos_xyz; // hand pos
    Eigen::Quaterniond quat; // hand quaternion
    Eigen::Vector3d elbow_pos_xyz; // elbow pos, only for motion capture
    Eigen::VectorXd joint_angles; // joint angles, could as initial guess
    // IKParams ik_params; // solver params

    bool is_elbow_pos_valid() const
    {
        bool invalid = true;
        for(int i=0; i<3; ++i)// if all three values are close to zero, it's invalid
            invalid &= (elbow_pos_xyz[i] >= -1e3 && elbow_pos_xyz[i] <= 1e3);
        return !invalid;
    }
};

class ArmsIKNode
{
    public:
        ArmsIKNode(ros::NodeHandle& nh, const std::string& model_path
            , std::vector<std::string> end_frames_name
            , std::vector<std::string> shoulder_frame_names
            , Eigen::Vector3d custom_eef_frame_pos=Eigen::Vector3d::Zero()
            , HighlyDynamic::HandSide hand_side=HighlyDynamic::HandSide::LEFT
            , int single_arm_num=7
        )
        : nh_(nh)
        , single_arm_num_(single_arm_num)
        , hand_side_(hand_side)
        , shoulder_frame_names_(shoulder_frame_names)
        {
            // 从ROS参数读取腰部关节信息
            int mpc_waist_dof = 0;
            if (nh_.hasParam("/mpc/mpcWaistDof"))
            {
                nh_.getParam("/mpc/mpcWaistDof", mpc_waist_dof);
            }
            has_waist_joint_ = (mpc_waist_dof > 0);
            if (has_waist_joint_)
            {
                waist_joint_index_ = 12;
            }
            else
            {
                waist_joint_index_ = -1;
            }
            const double dt = 0.001;
            const std::vector<std::string> custom_eef_frame_names{"eef_left", "eef_right"};
            
            drake::systems::DiagramBuilder<double> builder;
            plant_ptr_ = builder.AddSystem<drake::multibody::MultibodyPlant>(dt);
            drake::multibody::Parser(plant_ptr_).AddModelFromFile(model_path);
            std::cout << "original frame_name: " << std::endl;
            for(auto& frame_name : end_frames_name)
            {
                std::cout << " " << frame_name << std::endl;
            }
            plant_ptr_->WeldFrames(plant_ptr_->world_frame(), plant_ptr_->GetFrameByName(end_frames_name[0]));
            // custom eef frame
            plant_ptr_->AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
                custom_eef_frame_names[0], plant_ptr_->GetFrameByName(end_frames_name[1]),
                drake::math::RigidTransform<double>(drake::math::RotationMatrix<double>::Identity(), custom_eef_frame_pos)));
            plant_ptr_->AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
                custom_eef_frame_names[1], plant_ptr_->GetFrameByName(end_frames_name[2]),
                drake::math::RigidTransform<double>(drake::math::RotationMatrix<double>::Identity(), custom_eef_frame_pos)));
            // 替换原始eef name
            end_frames_name[1] = custom_eef_frame_names[0];
            end_frames_name[2] = custom_eef_frame_names[1];
            plant_ptr_->Finalize();

            diagram_ptr_ = builder.Build();
            diagram_context_ptr_ = diagram_ptr_->CreateDefaultContext();
            plant_context_ptr_ = &diagram_ptr_->GetMutableSubsystemContext(*plant_ptr_, diagram_context_ptr_.get());

            q0_ = plant_ptr_->GetPositions(*plant_context_ptr_);

            ik_ = HighlyDynamic::CoMIK(plant_ptr_, end_frames_name);
            // 保存end_frames_name用于伪逆求解
            end_frames_name_ = end_frames_name;

            // ros
            sub_ik_cmd_ = nh_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/ik/two_arm_hand_pose_cmd", 10, &ArmsIKNode::ik_cmd_callback, this);
            if (has_waist_joint_)
            {
                sensor_data_raw_sub_ = nh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &ArmsIKNode::sensor_data_raw_callback, this);
            }

            joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 10);
            time_cost_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/ik/debug/time_cost", 10);
            ik_result_pub_ = nh_.advertise<kuavo_msgs::twoArmHandPose>("/ik/result", 10);
            ik_result_free_pub_ = nh_.advertise<kuavo_msgs::twoArmHandPoseFree>("/ik/result_free", 10);
            head_body_pose_pub_ = nh_.advertise<kuavo_msgs::headBodyPose>("/kuavo_head_body_orientation", 10);
            // srv
            // 初始化服务服务器
            ik_server_ = nh_.advertiseService("/ik/two_arm_hand_pose_cmd_srv", &ArmsIKNode::handleServiceRequest, this);
            ik_server_muli_refer_ = nh_.advertiseService("/ik/two_arm_hand_pose_cmd_srv_muli_refer", &ArmsIKNode::handleServiceRequest_multiple_refer_q, this);
            ik_free_server_ = nh_.advertiseService("/ik/two_arm_hand_pose_cmd_free_srv", &ArmsIKNode::free_handleServiceRequest, this);
            fk_server_ = nh_.advertiseService("/ik/fk_srv", &ArmsIKNode::handleFKServiceRequest, this);
            // solver params
            ik_solve_params_.major_optimality_tol = 9e-3;
            ik_solve_params_.major_feasibility_tol = 9e-3;
            ik_solve_params_.minor_feasibility_tol = 9e-3;

            ik_solve_params_.major_iterations_limit = 50;
            ik_solve_params_.oritation_constraint_tol = 19e-3;
            ik_solve_params_.pos_constraint_tol = 9e-3;
            ik_solve_params_.pos_cost_weight = 10;
            // default constraint mode: pos soft + ori hard (01 -> 1)
            ik_solve_params_.constraint_mode = 0;
            // q0 for ik
            ik_cmd_left_.joint_angles = Eigen::VectorXd::Zero(single_arm_num_);
            ik_cmd_right_.joint_angles = Eigen::VectorXd::Zero(single_arm_num_);
            // ros param
            std::cout << "print_ik_info: " << print_ik_info_ << std::endl;
        }

    private:
        struct SolutionMetrics
        {
            // -------------------- 位置（左右取最差） --------------------
            double pos_err_mm{std::numeric_limits<double>::infinity()};    // mm
            double pos_err_norm{std::numeric_limits<double>::infinity()};  // 无量纲：pos_err_mm / pos_threshold_mm

            // -------------------- 姿态（左右取最差） --------------------
            double ori_angle_deg{std::numeric_limits<double>::infinity()};       // deg（整体角误差）
            double roll_deg{std::numeric_limits<double>::infinity()};            // deg（分量）
            double pitch_deg{std::numeric_limits<double>::infinity()};           // deg（分量）
            double yaw_deg{std::numeric_limits<double>::infinity()};             // deg（分量）
            double ori_weighted_score{std::numeric_limits<double>::infinity()};  // deg：0.1*pitch + 0.2*roll + 0.3*yaw
            double ori_err_norm{std::numeric_limits<double>::infinity()};        // 无量纲：ori_weighted_score / ori_threshold_deg

            // -------------------- 综合评分（越小越好） --------------------
            double total_score{std::numeric_limits<double>::infinity()};  // 无量纲：0.4*pos_norm + 0.6*ori_norm
        };

        struct SolutionCandidate
        {
            Eigen::VectorXd q;
            std::string reference_name;
            SolutionMetrics metrics;
        };

        static double wrapDeg(double deg)
        {
            // 角度包裹到 [-180, 180]
            return std::remainder(deg, 360.0);
        }

        static double quatAngleDeg(const Eigen::Quaterniond &q_des, const Eigen::Quaterniond &q_act)
        {
            // 两个四元数之间的最短角距离
            Eigen::Quaterniond dq = q_des.conjugate() * q_act;
            dq.normalize();
            const double w = std::clamp(std::abs(dq.w()), 0.0, 1.0);
            return 2.0 * std::acos(w) * TO_DEGREE;
        }

        static Eigen::Vector3d rotvecErrorDeg(const Eigen::Quaterniond &q_des, const Eigen::Quaterniond &q_act)
        {
            // 基于四元数的姿态误差计算，避免欧拉角分支跳变
            // 返回 abs(旋转向量)（单位：deg；x/y/z 对应刚体小角度误差分量）
            Eigen::Quaterniond dq = q_des.conjugate() * q_act;
            dq.normalize();

            // 保证取最短旋转（避免由于四元数符号导致角度 > pi）
            if (dq.w() < 0.0)
                dq.coeffs() *= -1.0;

            Eigen::AngleAxisd aa(dq);
            // 极小角度时 Eigen 的轴可能不稳定；AngleAxis 通常能处理，但仍需防 NaN
            Eigen::Vector3d rotvec_rad = aa.axis() * aa.angle();  // 弧度
            if (rotvec_rad.hasNaN())
                rotvec_rad.setZero();

            Eigen::Vector3d rotvec_deg = rotvec_rad * TO_DEGREE;
            return rotvec_deg.cwiseAbs();
        }

        // 调试辅助：打印 link7 与 eef 两个帧之间的固定变换（左右手）
        // 用于确认 IK 约束目标到底是 `eef_*` 还是 `zarm_*7_link`，
        // 以及解释“姿态很准但位置残差很大”等现象
        void printLink7ToEefFixedOffsetsOnce()
        {
            if (!print_ik_info_)
                return;

            auto findFrameInstance =
                [&](const std::string &frame_name,
                    drake::multibody::ModelInstanceIndex &out_instance) -> bool
            {
                const int n = plant_ptr_->num_model_instances();
                for (int i = 0; i < n; ++i)
                {
                    const drake::multibody::ModelInstanceIndex mi(i);
                    if (plant_ptr_->HasFrameNamed(frame_name, mi))
                    {
                        out_instance = mi;
                        return true;
                    }
                }
                return false;
            };

            drake::multibody::ModelInstanceIndex mi_l7(0), mi_r7(0), mi_eef_l(0), mi_eef_r(0);
            const bool has_l7 = findFrameInstance("zarm_l7_link", mi_l7);
            const bool has_r7 = findFrameInstance("zarm_r7_link", mi_r7);
            const bool has_eef_l = findFrameInstance(end_frames_name_[1], mi_eef_l);
            const bool has_eef_r = findFrameInstance(end_frames_name_[2], mi_eef_r);

            if (!(has_l7 && has_r7 && has_eef_l && has_eef_r))
            {
                std::cout << "[frame-check][warn] cannot resolve frames:"
                          << " has_l7=" << has_l7
                          << " has_r7=" << has_r7
                          << " has_eef_l=" << has_eef_l << "(" << end_frames_name_[1] << ")"
                          << " has_eef_r=" << has_eef_r << "(" << end_frames_name_[2] << ")"
                          << std::endl;
                return;
            }

            const auto &l7 = plant_ptr_->GetFrameByName("zarm_l7_link", mi_l7);
            const auto &r7 = plant_ptr_->GetFrameByName("zarm_r7_link", mi_r7);
            const auto &eef_l = plant_ptr_->GetFrameByName(end_frames_name_[1], mi_eef_l);
            const auto &eef_r = plant_ptr_->GetFrameByName(end_frames_name_[2], mi_eef_r);

            const auto X_W_l7 = l7.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_r7 = r7.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_eef_l = eef_l.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_eef_r = eef_r.CalcPoseInWorld(*plant_context_ptr_);

            const auto X_l7_eef = X_W_l7.inverse() * X_W_eef_l;
            const auto X_r7_eef = X_W_r7.inverse() * X_W_eef_r;

            auto rotAngleDegFromR = [&](const Eigen::Matrix3d &R) -> double
            {
                Eigen::AngleAxisd aa(R);
                double a = aa.angle();
                if (!std::isfinite(a))
                    return 0.0;
                if (a > M_PI)
                    a -= 2.0 * M_PI;
                return std::abs(a) * TO_DEGREE;
            };

            const Eigen::Vector3d t_l = X_l7_eef.translation();
            const Eigen::Vector3d t_r = X_r7_eef.translation();
            const double rot_l_deg = rotAngleDegFromR(X_l7_eef.rotation().matrix());
            const double rot_r_deg = rotAngleDegFromR(X_r7_eef.rotation().matrix());

            std::cout << "[frame-check]"
                      << " link7->" << end_frames_name_[1]
                      << " t_mm=(" << std::fixed << std::setprecision(1)
                      << t_l.x() * 1000.0 << ", " << t_l.y() * 1000.0 << ", " << t_l.z() * 1000.0
                      << "), |t|=" << t_l.norm() * 1000.0 << "mm"
                      << ", rot=" << std::fixed << std::setprecision(3) << rot_l_deg << "deg"
                      << " || link7->" << end_frames_name_[2]
                      << " t_mm=(" << std::fixed << std::setprecision(1)
                      << t_r.x() * 1000.0 << ", " << t_r.y() * 1000.0 << ", " << t_r.z() * 1000.0
                      << "), |t|=" << t_r.norm() * 1000.0 << "mm"
                      << ", rot=" << std::fixed << std::setprecision(3) << rot_r_deg << "deg"
                      << std::endl;
        }

        // 函数1：判断“当前解是否已经足够好，可以直接返回”
        static bool shouldEarlyExit(const SolutionMetrics &m,
                                    double pos_thresh_mm = 1.5,
                                    double ori_thresh_deg = 3.0)
        {
            return (m.pos_err_mm < pos_thresh_mm) && (m.ori_angle_deg < ori_thresh_deg);
        }

        static SolutionMetrics evaluateSolutionMetrics(HighlyDynamic::CoMIK& ik,
                                                       const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>& desired_pose_vec,
                                                       const Eigen::VectorXd& q)
        {
            // desired_pose_vec[1] = 左手，[2] = 右手
            SolutionMetrics m;
            // 归一化时使用与 early-exit 相同的阈值
            constexpr double kPosThreshMm = 1.5;
            constexpr double kOriThreshDeg = 3.0;

            auto [left_pos, left_quat] = ik.FK(q, HighlyDynamic::HandSide::LEFT);
            auto [right_pos, right_quat] = ik.FK(q, HighlyDynamic::HandSide::RIGHT);

            const double left_pos_mm = (left_pos - desired_pose_vec[1].second).norm() * 1000.0;
            const double right_pos_mm = (right_pos - desired_pose_vec[2].second).norm() * 1000.0;
            m.pos_err_mm = std::max(left_pos_mm, right_pos_mm);
            m.pos_err_norm = m.pos_err_mm / kPosThreshMm;

            const double left_ori_deg = quatAngleDeg(desired_pose_vec[1].first, left_quat);
            const double right_ori_deg = quatAngleDeg(desired_pose_vec[2].first, right_quat);
            m.ori_angle_deg = std::max(left_ori_deg, right_ori_deg);

            const Eigen::Vector3d left_rotvec_deg = rotvecErrorDeg(desired_pose_vec[1].first, left_quat);
            const Eigen::Vector3d right_rotvec_deg = rotvecErrorDeg(desired_pose_vec[2].first, right_quat);

            // 取左右手“最差”的误差作为整体评价（更保守）
            // NOTE：为兼容日志保留 roll/pitch/yaw 字段，但其含义已变为
            // 四元数 log 对应的旋转向量分量 (x,y,z)，不再是欧拉角
            m.roll_deg = std::max(left_rotvec_deg[0], right_rotvec_deg[0]);   // x
            m.pitch_deg = std::max(left_rotvec_deg[1], right_rotvec_deg[1]);  // y
            m.yaw_deg = std::max(left_rotvec_deg[2], right_rotvec_deg[2]);    // z

            // 用户给的权重：pitch 0.1, roll 0.2, yaw 0.3（总和 0.6）
            m.ori_weighted_score = 0.1 * m.pitch_deg + 0.2 * m.roll_deg + 0.3 * m.yaw_deg;
            m.ori_err_norm = m.ori_weighted_score / kOriThreshDeg;
            // 总分：位置 0.4 + 姿态 0.6（无量纲，越小越好）
            m.total_score = 0.4 * m.pos_err_norm + 0.6 * m.ori_err_norm;
            return m;
        }

        static std::string formatSolutionMetrics(const SolutionCandidate &cand)
        {
            const auto &m = cand.metrics;
            std::ostringstream oss;
            oss << cand.reference_name
                << " | pos=" << std::fixed << std::setprecision(3) << m.pos_err_mm << "mm"
                << " (n=" << std::fixed << std::setprecision(3) << m.pos_err_norm << ")"
                << ", ori=" << std::fixed << std::setprecision(3) << m.ori_angle_deg << "deg"
                // NOTE：此处 rpy 已不再是欧拉角，而是四元数 log 的旋转向量分量 (x,y,z)，单位：deg
                << ", rpy=("
                << std::fixed << std::setprecision(3) << m.roll_deg << ", "
                << std::fixed << std::setprecision(3) << m.pitch_deg << ", "
                << std::fixed << std::setprecision(3) << m.yaw_deg << ")deg"
                << ", ori_w=" << std::fixed << std::setprecision(3) << m.ori_weighted_score
                << " (n=" << std::fixed << std::setprecision(3) << m.ori_err_norm << ")"
                << ", score=" << std::fixed << std::setprecision(3) << m.total_score;
            return oss.str();
        }

        // 检查“实际FK”与“期望位姿”的误差；当误差过大时打印日志（用于排查seed/局部最优/坐标系问题）。
        //
        // 阈值：
        // - 位置误差：5mm
        // - 姿态误差：60deg（四元数最短角）
        void checkDesiredPoseErrorAndLog(const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>& pose_vec,
            const Eigen::VectorXd& q, const std::string& context)
        {
            if (pose_vec.size() <= 2)
                return;

            constexpr double kPosThreshM = 0.005;   // 5mm
            constexpr double kOriThreshDeg = 60.0;  // 60deg

            const auto [left_pos, left_quat] = ik_.FK(q, HighlyDynamic::HandSide::LEFT);
            const auto [right_pos, right_quat] = ik_.FK(q, HighlyDynamic::HandSide::RIGHT);

            const double left_pos_err_m = (left_pos - pose_vec[1].second).norm();
            const double right_pos_err_m = (right_pos - pose_vec[2].second).norm();
            const double left_ori_err_deg = quatAngleDeg(pose_vec[1].first, left_quat);
            const double right_ori_err_deg = quatAngleDeg(pose_vec[2].first, right_quat);

            const double pos_err_max_m = std::max(left_pos_err_m, right_pos_err_m);
            const double ori_err_max_deg = std::max(left_ori_err_deg, right_ori_err_deg);

            if (pos_err_max_m <= kPosThreshM && ori_err_max_deg <= kOriThreshDeg)
                return;

            std::ostringstream oss;
            oss << "[pose-error][" << context << "] "
                << "pos_thresh=5mm, ori_thresh=60deg | "
                << "pos_mm(L,R,max)=("
                << std::fixed << std::setprecision(3)
                << left_pos_err_m * 1000.0 << ", "
                << right_pos_err_m * 1000.0 << ", "
                << pos_err_max_m * 1000.0
                << "), ori_deg(L,R,max)=("
                << std::fixed << std::setprecision(3)
                << left_ori_err_deg << ", "
                << right_ori_err_deg << ", "
                << ori_err_max_deg
                << ")";
            ROS_WARN_STREAM(oss.str());
        }

        // 函数2：在所有参考都求完后，按“模式 + 打分系统”选最优解（分数越低越好）
        static size_t pickBestSolutionIndex(const std::vector<SolutionCandidate>& cands,
                                            const HighlyDynamic::IKParams& params)
        {
            auto better = [&](const SolutionCandidate &a, const SolutionCandidate &b) -> bool
            {
                // 统一规则：只按打分系统评判（总分越低越好）
                if (a.metrics.total_score != b.metrics.total_score)
                    return a.metrics.total_score < b.metrics.total_score;

                // 次级规则：优先位置误差更小
                if (a.metrics.pos_err_mm != b.metrics.pos_err_mm)
                    return a.metrics.pos_err_mm < b.metrics.pos_err_mm;

                // 次级规则：再比较整体姿态角误差更小
                if (a.metrics.ori_angle_deg != b.metrics.ori_angle_deg)
                    return a.metrics.ori_angle_deg < b.metrics.ori_angle_deg;

                return false;
            };

            size_t best = 0;
            for (size_t i = 1; i < cands.size(); ++i)
            {
                if (better(cands[i], cands[best]))
                    best = i;
            }
            return best;
        }

    public:
        void run()
        {
            ros::Rate rate(1000);
            while(ros::ok()) 
            {
                ros::spinOnce();
                if(!recived_cmd_)
                {
                    rate.sleep();
                    // std::cout << "Waiting for command..." << std::endl;
                    continue;
                }
                auto loop_start = std::chrono::high_resolution_clock::now();
                std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                    {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                    {ik_cmd_left_.quat, ik_cmd_left_.pos_xyz},
                    {ik_cmd_right_.quat, ik_cmd_right_.pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_left_.elbow_pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_right_.elbow_pos_xyz}
                    };
                // std::cout << "ik_cmd_left_.elbow_pos_xyz: " << ik_cmd_left_.elbow_pos_xyz.transpose() << std::endl;
                // std::cout << "ik_cmd_right_.elbow_pos_xyz: " << ik_cmd_right_.elbow_pos_xyz.transpose() << std::endl;
                Eigen::VectorXd q;
                // std::cout << std::fixed << std::setprecision(5) << "q0: " << q0_.head(single_arm_num_).transpose() << std::endl;
                if(use_ik_cmd_q0_)
                {
                    q0_ << ik_cmd_left_.joint_angles, ik_cmd_right_.joint_angles;
                    std::cout << std::fixed << std::setprecision(3) << "Left: " << q0_.head(single_arm_num_).transpose()
                                            << ", Right: " << q0_.tail(single_arm_num_).transpose() << std::endl;
                }
                auto start = std::chrono::high_resolution_clock::now();
                checkInWorkspace(pose_vec[1].second, pose_vec[2].second);
                bool result = solveWithBinarySearch(pose_vec, q0_, q);
                std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
                // std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;

                if(result && recived__new_cmd_)//接收到新的命令时才发布手臂控制话题
                {
                    ++ik_success_count_;
                    // std::cout << "IK success" << std::endl;
                    // std::cout << std::fixed << std::setprecision(5) << "q:  " << q.head(single_arm_num_).transpose() << std::endl;
                    q0_ = Eigen::VectorXd(q);
                    // 发布JointState消息
                    sensor_msgs::JointState joint_state;
                    joint_state.header.stamp = ros::Time::now();
                    const int start_idx = q.size() - 2*single_arm_num_;
                    for (int j = 0; j < single_arm_num_ * 2; ++j)
                    {
                        joint_state.name.push_back("joint_" + std::to_string(j+1));
                        joint_state.position.push_back(TO_DEGREE*q[start_idx+j]);
                    }
                    // 如果只控制单手，则将其他手臂的关节位置设置为0
                    if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                        std::fill(joint_state.position.begin() + single_arm_num_, joint_state.position.begin() + single_arm_num_ * 2, 0);
                    else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                        std::fill(joint_state.position.begin() + 0, joint_state.position.begin() + single_arm_num_, 0);
                    joint_pub_.publish(joint_state);
                    if(start_idx > 0)//包含躯干
                    {
                        kuavo_msgs::headBodyPose head_body_pose_msg;
                        head_body_pose_msg.body_height = q[0];
                        head_body_pose_msg.body_yaw = q[1];
                        head_body_pose_msg.body_pitch = q[2];
                        head_body_pose_msg.body_roll = q[3];
                        // TODO: body height
                        head_body_pose_pub_.publish(head_body_pose_msg);
                    }
                    publish_ik_result_info(q);
                }
                if(print_ik_info_)
                    printIkResultInfo(ik_cmd_left_, ik_cmd_right_, q, result);
                // else
                //     std::cout << "IK failed" << std::endl;
                rate.sleep();

                std::chrono::duration<double, std::milli> loop_elapsed = std::chrono::high_resolution_clock::now() - loop_start;
                // time cost
                std_msgs::Float32MultiArray time_cost_msg;
                time_cost_msg.data.push_back(loop_elapsed.count());
                time_cost_msg.data.push_back(elapsed.count());
                time_cost_pub_.publish(time_cost_msg);
                if(recived__new_cmd_)
                {
                    ++ik_count_;
                    // print status
                    std::cout << "\rLoop count: " << ik_count_ 
                            << ", sucess count: " << ik_success_count_
                            << ", IK success rate: " << std::fixed << std::setprecision(1) << 100 * ik_success_count_ / static_cast<double>(ik_count_)
                            << "%, time-cost: " << std::fixed << std::setprecision(1) << (elapsed.count()) << " ms.";
                    std::cout.flush();
                    if(ik_count_ >= std::numeric_limits<long long int>::max())
                    {
                        ik_count_ = 0;
                        ik_success_count_ = 0;
                    }
                }
                recived__new_cmd_ = false;
            }
        }

        double getWorkSpaceRadius()
        {
            auto X_bSl = plant_ptr_->GetFrameByName(shoulder_frame_names_[0]).CalcPoseInWorld(*plant_context_ptr_);
            auto X_bEl = plant_ptr_->GetFrameByName("eef_left").CalcPoseInWorld(*plant_context_ptr_);
            Eigen::Vector3d p_SlEl = X_bEl.translation() - X_bSl.translation();
            return p_SlEl.norm();
        }

        Eigen::Vector3d transPosFrameToShoulder(const Eigen::Vector3d& p_bE, HighlyDynamic::HandSide hand_side)
        {
            auto p_bSl = plant_ptr_->GetFrameByName(shoulder_frame_names_[0]).CalcPoseInWorld(*plant_context_ptr_).translation();
            auto p_bSr = plant_ptr_->GetFrameByName(shoulder_frame_names_[1]).CalcPoseInWorld(*plant_context_ptr_).translation();
            auto p_bS = (hand_side == HighlyDynamic::HandSide::LEFT) ? p_bSl : p_bSr;
            auto p_SE = p_bE - p_bS;
            return p_SE;
        }
        bool checkInWorkspace(const Eigen::Vector3d& p_bE, HighlyDynamic::HandSide hand_side)
        {
            auto p_SE = transPosFrameToShoulder(p_bE, hand_side);
            double radius = getWorkSpaceRadius();
            return p_SE.norm() <= radius;
        }
        bool checkInWorkspace(const Eigen::Vector3d& p_bEl, const Eigen::Vector3d& p_bEr)
        {
            if(!checkInWorkspace(p_bEl, HighlyDynamic::HandSide::LEFT))
            {
                ROS_ERROR_STREAM("Left hand out of workspace!");
                return false;
            }
            if(!checkInWorkspace(p_bEr, HighlyDynamic::HandSide::RIGHT))
            {
                ROS_ERROR_STREAM("Right hand out of workspace!");
                return false;
            }
            return true;
        }

    private:
        template <typename IkParamMsgT>
        void applyIkParamsFromMsg(const IkParamMsgT& p)
        {
            ik_solve_params_.major_optimality_tol = p.major_optimality_tol;
            ik_solve_params_.major_feasibility_tol = p.major_feasibility_tol;
            ik_solve_params_.minor_feasibility_tol = p.minor_feasibility_tol;
            ik_solve_params_.major_iterations_limit = p.major_iterations_limit;

            ik_solve_params_.oritation_constraint_tol = p.oritation_constraint_tol;
            ik_solve_params_.pos_constraint_tol = p.pos_constraint_tol;
            ik_solve_params_.pos_cost_weight = p.pos_cost_weight;
            ik_solve_params_.constraint_mode = p.constraint_mode;
        }

        template <typename HandPosesT>
        void fillIkCmdFromHandPoses(const HandPosesT& hand_poses)
        {
            // left
            ik_cmd_left_.pos_xyz << hand_poses.left_pose.pos_xyz[0],
                hand_poses.left_pose.pos_xyz[1],
                hand_poses.left_pose.pos_xyz[2];
            ik_cmd_left_.quat = Eigen::Quaterniond(hand_poses.left_pose.quat_xyzw[3],
                                                  hand_poses.left_pose.quat_xyzw[0],
                                                  hand_poses.left_pose.quat_xyzw[1],
                                                  hand_poses.left_pose.quat_xyzw[2]);
            ik_cmd_left_.elbow_pos_xyz << hand_poses.left_pose.elbow_pos_xyz[0],
                hand_poses.left_pose.elbow_pos_xyz[1],
                hand_poses.left_pose.elbow_pos_xyz[2];

            // right
            ik_cmd_right_.pos_xyz << hand_poses.right_pose.pos_xyz[0],
                hand_poses.right_pose.pos_xyz[1],
                hand_poses.right_pose.pos_xyz[2];
            ik_cmd_right_.quat = Eigen::Quaterniond(hand_poses.right_pose.quat_xyzw[3],
                                                   hand_poses.right_pose.quat_xyzw[0],
                                                   hand_poses.right_pose.quat_xyzw[1],
                                                   hand_poses.right_pose.quat_xyzw[2]);
            ik_cmd_right_.elbow_pos_xyz << hand_poses.right_pose.elbow_pos_xyz[0],
                hand_poses.right_pose.elbow_pos_xyz[1],
                hand_poses.right_pose.elbow_pos_xyz[2];
        }

        template <typename HandPosesT>
        void copyArmJointAnglesIfEnabled(const HandPosesT& hand_poses)
        {
            if (!use_ik_cmd_q0_)
                return;
            for (int i = 0; i < single_arm_num_; ++i)
            {
                ik_cmd_left_.joint_angles[i] = hand_poses.left_pose.joint_angles[i];
                ik_cmd_right_.joint_angles[i] = hand_poses.right_pose.joint_angles[i];
            }
        }

        template <typename CmdT>
        void applyTwoArmCmdCommon(const CmdT& cmd)
        {
            auto& hand_poses = cmd.hand_poses;
            fillIkCmdFromHandPoses(hand_poses);

            use_ik_cmd_q0_ = cmd.joint_angles_as_q0;
            copyArmJointAnglesIfEnabled(hand_poses);

            if (cmd.use_custom_ik_param)
                applyIkParamsFromMsg(cmd.ik_param);
        }

        Eigen::VectorXd buildUserQ0FromCmd(const Eigen::VectorXd& base_q0) const
        {
            Eigen::VectorXd user_q0 = base_q0;
            // 是否使用用户 q0 以成员标志位为准。
            if (use_ik_cmd_q0_)
            {
                // 只覆盖手臂关节那一段，保留 torso/腰等其它关节的 q0
                const int start_idx = static_cast<int>(user_q0.size()) - 2 * single_arm_num_;
                if (start_idx >= 0 && user_q0.size() >= 2 * single_arm_num_)
                {
                    user_q0.segment(start_idx, single_arm_num_) = ik_cmd_left_.joint_angles;
                    user_q0.segment(start_idx + single_arm_num_, single_arm_num_) = ik_cmd_right_.joint_angles;
                }
            }
            return user_q0;
        }

        void sensor_data_raw_callback(const kuavo_msgs::sensorsData::ConstPtr& msg)
        {
            if (has_waist_joint_ && waist_joint_index_ >= 0 && 
                waist_joint_index_ < static_cast<int>(msg->joint_data.joint_q.size()))
            {
                std::lock_guard<std::mutex> lock(waist_yaw_mutex_);
                current_waist_yaw_ = msg->joint_data.joint_q[waist_joint_index_];
            }
        }

        std::pair<Eigen::Vector3d, Eigen::Quaterniond> transformPoseByWaistYaw(
            const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
        {
            if (!has_waist_joint_)
            {
                return std::make_pair(pos, quat);
            }

            std::lock_guard<std::mutex> lock(waist_yaw_mutex_);
            double waist_yaw = current_waist_yaw_;
            
            // 构建绕Z轴的旋转矩阵（负的腰部角度）
            Eigen::Matrix3d R = Eigen::AngleAxisd(-waist_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            
            // 变换位置
            Eigen::Vector3d pos_transformed = R * pos;
            
            // 变换四元数
            Eigen::Quaterniond R_quat(R);
            Eigen::Quaterniond quat_transformed = R_quat * quat;
            quat_transformed.normalize();
            
            return std::make_pair(pos_transformed, quat_transformed);
        }

        void ik_cmd_callback(const kuavo_msgs::twoArmHandPoseCmd::ConstPtr& msg)
        {
            applyTwoArmCmdCommon(*msg);
            if (!recived_cmd_)
                recived_cmd_ = true;
            recived__new_cmd_ = true;
        }

        kuavo_msgs::twoArmHandPose publish_ik_result_info(const Eigen::VectorXd& q)
        {
            kuavo_msgs::twoArmHandPose msg;
            const int start_idx = q.size() - 2*single_arm_num_;
            for (size_t i = 0; i < single_arm_num_; i++)
            {
                msg.left_pose.joint_angles[i] = q[i + start_idx];
                msg.right_pose.joint_angles[i] = q[i + single_arm_num_ + start_idx];
            }
            auto [left_pos, left_quat] = ik_.FK(q, HighlyDynamic::HandSide::LEFT);
            auto [right_pos, right_quat] = ik_.FK(q, HighlyDynamic::HandSide::RIGHT);
            for(int i = 0; i < 3; i++)
            {
                msg.left_pose.pos_xyz[i] = left_pos[i];
                msg.right_pose.pos_xyz[i] = right_pos[i];
            }
            for(int i = 0; i < 4; i++)
            {
                msg.left_pose.quat_xyzw[i] = left_quat.coeffs()[i]; //coeffs() return order: xyzw
                msg.right_pose.quat_xyzw[i] = right_quat.coeffs()[i];
            }
            ik_result_pub_.publish(msg);
            return msg;
        }

        kuavo_msgs::twoArmHandPoseFree publish_ik_result_info_free(const Eigen::VectorXd& q)
        {
            kuavo_msgs::twoArmHandPoseFree msg;
            const int start_idx = q.size() - 2*single_arm_num_;
            msg.left_pose.joint_angles.resize(single_arm_num_);
            msg.right_pose.joint_angles.resize(single_arm_num_);
            for (size_t i = 0; i < single_arm_num_; i++)
            {
                msg.left_pose.joint_angles[i] = q[i + start_idx];
                msg.right_pose.joint_angles[i] = q[i + single_arm_num_ + start_idx];
            }
            auto [left_pos, left_quat] = ik_.FK(q, HighlyDynamic::HandSide::LEFT);
            auto [right_pos, right_quat] = ik_.FK(q, HighlyDynamic::HandSide::RIGHT);
            for(int i = 0; i < 3; i++)
            {
                msg.left_pose.pos_xyz[i] = left_pos[i];
                msg.right_pose.pos_xyz[i] = right_pos[i];
            }
            for(int i = 0; i < 4; i++)
            {
                msg.left_pose.quat_xyzw[i] = left_quat.coeffs()[i]; //coeffs() return order: xyzw
                msg.right_pose.quat_xyzw[i] = right_quat.coeffs()[i];
            }
            ik_result_free_pub_.publish(msg);
            return msg;
        }

        void printIkResultInfo(const IkCmd& cmd_l, const IkCmd& cmd_r, const Eigen::VectorXd& result_q, bool result)
        {
            if(!result)
            {
                std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
                std::cerr << "\n++++++++++++++++++++ IK FAILED!!! ++++++++++++++++++++";
                std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
                return;
            }
            std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
            std::cerr << "\n+++++++++++++++++++ IK RESULT INFO +++++++++++++++++++";
            std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
            auto [left_pos, left_quat] = ik_.FK(result_q, HighlyDynamic::HandSide::LEFT);
            auto [right_pos, right_quat] = ik_.FK(result_q, HighlyDynamic::HandSide::RIGHT);
            std::cerr << "Eef cmd: \n" << std::fixed << std::setprecision(3) << 
                "  Left pos: " << cmd_l.pos_xyz.transpose() << std::endl << 
                "  Left quat: " << cmd_l.quat.coeffs().transpose() << std::endl << 
                "  Right pos: " << cmd_r.pos_xyz.transpose() << std::endl << 
                "  Right quat: " << cmd_r.quat.coeffs().transpose() << std::endl;

            std::cerr << "Result:\n" << 
                "  q: " << std::fixed << std::setprecision(3) << result_q.transpose() << std::endl;
            std::cerr << "  Left eef pos: "  << std::fixed << std::setprecision(3) << left_pos.transpose() 
                << ", Left eef quat: " << std::fixed << std::setprecision(3) << left_quat.coeffs().transpose() << std::endl;
            std::cerr << "  Left pos error: " << (left_pos - cmd_l.pos_xyz).transpose() 
                << ", error norm: " << 1000.0*(left_pos - cmd_l.pos_xyz).norm() << " mm." << std::endl;

            std::cerr << "  Right eef pos: " << std::fixed << std::setprecision(3) << right_pos.transpose() 
                << ", Right eef quat: " << std::fixed << std::setprecision(3) << right_quat.coeffs().transpose() << std::endl;
            std::cerr << "  Right pos error: " << (right_pos - cmd_r.pos_xyz).transpose() 
                << ", error norm: " << 1000.0*(right_pos - cmd_r.pos_xyz).norm() << " mm." << std::endl;
            std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
        }

        // 处理服务请求的回调函数
        bool handleServiceRequest(kuavo_msgs::twoArmHandPoseCmdSrv::Request &req,
                                kuavo_msgs::twoArmHandPoseCmdSrv::Response &res) 
        {
            const auto &cmd = req.twoArmHandPoseCmdRequest;
            applyTwoArmCmdCommon(cmd);
            // q0 seed：默认用“上一帧 q0_”；如果用户显式提供 joint_angles_as_q0，则仅覆盖手臂段
            const Eigen::VectorXd q0_seed = buildUserQ0FromCmd(q0_);
            auto start = std::chrono::high_resolution_clock::now();
            
            // 如果有腰部关节，对位姿进行变换
            Eigen::Quaterniond left_quat = ik_cmd_left_.quat;
            Eigen::Vector3d left_pos = ik_cmd_left_.pos_xyz;
            Eigen::Quaterniond right_quat = ik_cmd_right_.quat;
            Eigen::Vector3d right_pos = ik_cmd_right_.pos_xyz;
            Eigen::Vector3d left_elbow_pos = ik_cmd_left_.elbow_pos_xyz;
            Eigen::Vector3d right_elbow_pos = ik_cmd_right_.elbow_pos_xyz;
            
            if (has_waist_joint_)
            {
                auto [left_pos_t, left_quat_t] = transformPoseByWaistYaw(left_pos, left_quat);
                auto [right_pos_t, right_quat_t] = transformPoseByWaistYaw(right_pos, right_quat);
                auto [left_elbow_pos_t, _] = transformPoseByWaistYaw(left_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
                auto [right_elbow_pos_t, __] = transformPoseByWaistYaw(right_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
                
                left_pos = left_pos_t;
                left_quat = left_quat_t;
                right_pos = right_pos_t;
                right_quat = right_quat_t;
                left_elbow_pos = left_elbow_pos_t;
                right_elbow_pos = right_elbow_pos_t;
            }
            
            std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                    {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                    {left_quat, left_pos},
                    {right_quat, right_pos},
                    {Eigen::Quaterniond(1, 0, 0, 0), left_elbow_pos},
                    {Eigen::Quaterniond(1, 0, 0, 0), right_elbow_pos}
                    };
            Eigen::VectorXd q;
            checkInWorkspace(pose_vec[1].second, pose_vec[2].second);
            // bool result = ik_.solve(pose_vec, q0_, q, ik_solve_params_);
            bool result = solveWithBinarySearch(pose_vec, q0_seed, q);
            std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
            // std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;
            res.success = false;
            res.with_torso = false;
            res.time_cost = elapsed.count();
            if(result)
            {
                // 求解成功后，检查与期望位姿的误差是否过大（>5mm 或 >60deg），过大则打日志
                checkDesiredPoseErrorAndLog(pose_vec, q, "handleServiceRequest");
                res.success = true;
                q0_ = Eigen::VectorXd(q);
                //
                const int start_idx = q.size() - 2*single_arm_num_;
                for (int j = 0; j < 2*single_arm_num_; ++j)
                {
                    res.q_arm.push_back(q[start_idx+j]);
                }
                // 如果只控制单手，则将其他手臂的关节位置设置为0
                if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                    std::fill(res.q_arm.begin() + single_arm_num_, res.q_arm.begin() + single_arm_num_ * 2, 0);
                else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                    std::fill(res.q_arm.begin() + 0, res.q_arm.begin() + single_arm_num_, 0);
                if(start_idx > 0)//包含躯干
                {
                    res.with_torso = true;
                    res.q_torso = std::vector<double>(q.data(), q.data() + start_idx);
                }
                kuavo_msgs::twoArmHandPose msg = publish_ik_result_info(q);
                res.hand_poses = msg;
            }
            if(print_ik_info_)
                printIkResultInfo(ik_cmd_left_, ik_cmd_right_, q, result);
            // 返回响应
            return true;
        }

        // 处理服务请求的回调函数（使用多个参考点：伪逆解、限位中点、用户q0、上一次有效解）
        bool handleServiceRequest_multiple_refer_q(kuavo_msgs::twoArmHandPoseCmdSrv::Request &req,
                                kuavo_msgs::twoArmHandPoseCmdSrv::Response &res) 
        {
            const auto &cmd = req.twoArmHandPoseCmdRequest;
            applyTwoArmCmdCommon(cmd);

            // 保存用户传入的 q0（如果有）
            Eigen::VectorXd user_q0 = buildUserQ0FromCmd(q0_);
            
            // 构建位姿向量（包括腰部变换）
            auto start = std::chrono::high_resolution_clock::now();
            Eigen::Quaterniond left_quat = ik_cmd_left_.quat;
            Eigen::Vector3d left_pos = ik_cmd_left_.pos_xyz;
            Eigen::Quaterniond right_quat = ik_cmd_right_.quat;
            Eigen::Vector3d right_pos = ik_cmd_right_.pos_xyz;
            Eigen::Vector3d left_elbow_pos = ik_cmd_left_.elbow_pos_xyz;
            Eigen::Vector3d right_elbow_pos = ik_cmd_right_.elbow_pos_xyz;
            
            if (has_waist_joint_)
            {
                std::tie(left_pos, left_quat) = transformPoseByWaistYaw(left_pos, left_quat);
                std::tie(right_pos, right_quat) = transformPoseByWaistYaw(right_pos, right_quat);
                std::tie(left_elbow_pos, std::ignore) = transformPoseByWaistYaw(left_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
                std::tie(right_elbow_pos, std::ignore) = transformPoseByWaistYaw(right_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
            }
            
            std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                {left_quat, left_pos}, {right_quat, right_pos},
                {Eigen::Quaterniond(1, 0, 0, 0), left_elbow_pos},
                {Eigen::Quaterniond(1, 0, 0, 0), right_elbow_pos}
            };
            
            checkInWorkspace(pose_vec[1].second, pose_vec[2].second);
            printLink7ToEefFixedOffsetsOnce();
            
            // 准备多个参考点
            const int nq = plant_ptr_->num_positions();
            std::vector<Eigen::VectorXd> reference_q_list;
            std::vector<std::string> reference_names;

            // 1. 用户传入的 q0：优先尝试
            if (use_ik_cmd_q0_ && user_q0.size() == q0_.size() && !user_q0.hasNaN())
            {
                reference_q_list.push_back(user_q0);
                reference_names.push_back("user_q0");
            }

            // 2. 上一次有效解：作为 fallback（q0_ 本身就是“上一次求解成功后的结果”容器）
            if (q0_.size() > 0 && !q0_.hasNaN())
            {
                reference_q_list.push_back(q0_);
                reference_names.push_back("last_valid(q0_)");
            }

            // 3. 限位中点
            Eigen::VectorXd q_midpoint(nq);
            Eigen::VectorXd q_lower = plant_ptr_->GetPositionLowerLimits();
            Eigen::VectorXd q_upper = plant_ptr_->GetPositionUpperLimits();
            q_midpoint = (q_lower + q_upper) / 2.0;
            reference_q_list.push_back(q_midpoint);
            reference_names.push_back("midpoint");

            // Seed 统一过滤阈值：位置误差超过该阈值的 seed 直接丢弃（避免把 drake 引到很差的局部解）
            constexpr double kMaxSeedPosError = 0.2;  // m（20cm）

            // 统一处理：打印 seed-check + 按阈值过滤 + 写入 reference 列表
            auto maybeAddSeed = [&](const std::string& name,
                                    const Eigen::VectorXd& q_seed,
                                    double left_pos_error_m,
                                    double right_pos_error_m,
                                    double left_ori_error_deg,
                                    double right_ori_error_deg) -> void
            {
                if (print_ik_info_)
                {
                    const double pos_l_mm = left_pos_error_m * 1000.0;
                    const double pos_r_mm = right_pos_error_m * 1000.0;
                    const double pos_max_mm = std::max(pos_l_mm, pos_r_mm);
                    const double ori_max_deg = std::max(left_ori_error_deg, right_ori_error_deg);

                    std::cout << "[seed-check][" << name << "]"
                              << " pos_mm(L,R,max)=("
                              << std::fixed << std::setprecision(3) << pos_l_mm << ", "
                              << std::fixed << std::setprecision(3) << pos_r_mm << ", "
                              << std::fixed << std::setprecision(3) << pos_max_mm
                              << ")"
                              << " ori_deg(L,R,max)=("
                              << std::fixed << std::setprecision(3) << left_ori_error_deg << ", "
                              << std::fixed << std::setprecision(3) << right_ori_error_deg << ", "
                              << std::fixed << std::setprecision(3) << ori_max_deg
                              << ")"
                              << std::endl;
                }

                if (left_pos_error_m <= kMaxSeedPosError && right_pos_error_m <= kMaxSeedPosError)
                {
                    reference_q_list.push_back(q_seed);
                    reference_names.push_back(name);
                }
            };
            
            // 4. 伪逆解
            Eigen::VectorXd q0_pinv = solvePseudoInverseIK(pose_vec, user_q0);
            if (!q0_pinv.hasNaN() && q0_pinv.size() == user_q0.size())
            {
                // 检查伪逆解的位置误差（20cm阈值）
                auto [computed_left_pos, computed_left_quat] = ik_.FK(q0_pinv, HighlyDynamic::HandSide::LEFT);
                auto [computed_right_pos, computed_right_quat] = ik_.FK(q0_pinv, HighlyDynamic::HandSide::RIGHT);
                const double left_pos_error = (computed_left_pos - pose_vec[1].second).norm();
                const double right_pos_error = (computed_right_pos - pose_vec[2].second).norm();
                // 同时计算姿态误差（四元数角度，deg）
                const double left_ori_error_deg = quatAngleDeg(pose_vec[1].first, computed_left_quat);
                const double right_ori_error_deg = quatAngleDeg(pose_vec[2].first, computed_right_quat);
                maybeAddSeed("pinv", q0_pinv, left_pos_error, right_pos_error, left_ori_error_deg, right_ori_error_deg);
            }
            
            // 5 Analytic seed
            // 解析求解器只返回手臂关节角，这里将其注入到 full-body q 容器中作为 seed
            if (single_arm_num_ == 7 && user_q0.size() == nq && user_q0.allFinite())
            {
                if (auto seed = solveAnalyticArmsOnly(pose_vec))
                {
                    Eigen::VectorXd q_seed = user_q0;
                    const int start_idx = q_seed.size() - 2 * single_arm_num_;
                    q_seed.segment(start_idx, single_arm_num_) = seed->first;
                    q_seed.segment(start_idx + single_arm_num_, single_arm_num_) = seed->second;
                    // 对 analytic seed 做一次 FK 对比（与 pinv 相同），并按阈值过滤
                    auto [a_left_pos, a_left_quat] = ik_.FK(q_seed, HighlyDynamic::HandSide::LEFT);
                    auto [a_right_pos, a_right_quat] = ik_.FK(q_seed, HighlyDynamic::HandSide::RIGHT);
                    const double a_left_pos_error = (a_left_pos - pose_vec[1].second).norm();
                    const double a_right_pos_error = (a_right_pos - pose_vec[2].second).norm();
                    const double a_left_ori_error_deg = quatAngleDeg(pose_vec[1].first, a_left_quat);
                    const double a_right_ori_error_deg = quatAngleDeg(pose_vec[2].first, a_right_quat);
                    maybeAddSeed("analytic", q_seed, a_left_pos_error, a_right_pos_error, a_left_ori_error_deg, a_right_ori_error_deg);
                }
            }
            
            // 尝试每个参考点进行IK求解
            Eigen::VectorXd q;
            bool success = false;
            std::string success_reference = "";
            std::vector<SolutionCandidate> candidates;
            
            for (size_t i = 0; i < reference_q_list.size(); ++i)
            {
                bool result = ik_.solve(pose_vec, reference_q_list[i], q, ik_solve_params_);
                if (result)
                {
                    SolutionCandidate cand;
                    cand.q = q;
                    cand.reference_name = reference_names[i];
                    cand.metrics = evaluateSolutionMetrics(ik_, pose_vec, q);
                    candidates.push_back(cand);

                    if (print_ik_info_)
                    {
                        std::cout << "[IK candidates] " << formatSolutionMetrics(cand) << std::endl;
                    }

                    // 如果已经满足“位置<1.5mm 且 姿态<3deg”，直接返回
                    if (shouldEarlyExit(cand.metrics))
                    {
                        if (print_ik_info_)
                        {
                            std::cout
                                << "[IK early-exit] 满足阈值(1.5mm, 3deg)，直接返回"
                                << " | pos_err=" << std::fixed << std::setprecision(3) << cand.metrics.pos_err_mm << "mm"
                                << ", ori_err=" << std::fixed << std::setprecision(3) << cand.metrics.ori_angle_deg << "deg"
                                << std::endl;
                            // 额外打印完整的打分细节（含 rpy/ori_w/score 等）
                            std::cout << "[IK early-exit][detail] " << formatSolutionMetrics(cand) << std::endl;
                        }
                        // 满足阈值就立刻返回（不再继续后续筛选/评分）
                        success = true;
                        success_reference = cand.reference_name + " (early-exit)";
                        break;
                    }
                }
            }

            // 若没有 early-exit，但存在可行解，则按模式挑选最优
            if (!success && !candidates.empty())
            {
                const size_t best_idx = pickBestSolutionIndex(candidates, ik_solve_params_);
                q = candidates[best_idx].q;
                success = true;
                const auto& m = candidates[best_idx].metrics;
                std::ostringstream oss;
                oss << candidates[best_idx].reference_name
                    << " (best: pos=" << std::fixed << std::setprecision(3) << m.pos_err_mm << "mm"
                    << ", ori=" << std::fixed << std::setprecision(3) << m.ori_angle_deg << "deg"
                    << ", score=" << std::fixed << std::setprecision(3) << m.total_score << ")";
                success_reference = oss.str();

                if (print_ik_info_)
                {
                    std::cout << "[IK best] " << formatSolutionMetrics(candidates[best_idx]) << std::endl;
                }
            }
            
            // 填充响应
            std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
            res.success = success;
            res.time_cost = elapsed.count();
            res.with_torso = false;
            
            if(success)
            {
                res.error_reason = "ik is success (reference: " + success_reference + ")";
                q0_ = q;
                
                const int start_idx = q.size() - 2*single_arm_num_;
                res.q_arm.resize(2*single_arm_num_);
                for (int j = 0; j < 2*single_arm_num_; ++j)
                    res.q_arm[j] = q[start_idx+j];
                
                if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                    std::fill(res.q_arm.begin() + single_arm_num_, res.q_arm.end(), 0);
                else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                    std::fill(res.q_arm.begin(), res.q_arm.begin() + single_arm_num_, 0);
                
                if(start_idx > 0)
                {
                    res.with_torso = true;
                    res.q_torso.assign(q.data(), q.data() + start_idx);
                }
                res.hand_poses = publish_ik_result_info(q);
            }
            else
            {
                std::ostringstream error_msg;
                error_msg << "drake ik failed with all references (tried: ";
                for (size_t i = 0; i < reference_names.size(); ++i)
                {
                    if (i > 0) error_msg << ", ";
                    error_msg << reference_names[i];
                }
                error_msg << ")";
                res.error_reason = error_msg.str();
            }
            
            if(print_ik_info_)
                printIkResultInfo(ik_cmd_left_, ik_cmd_right_, q, success);
            return true;
        }

        // 处理服务请求的回调函数
        bool free_handleServiceRequest(kuavo_msgs::twoArmHandPoseCmdFreeSrv::Request &req,
                    kuavo_msgs::twoArmHandPoseCmdFreeSrv::Response &res) 
        {
            const auto &cmd = req.twoArmHandPoseCmdFreeRequest;
            applyTwoArmCmdCommon(cmd);
            // q0 seed：默认用“上一帧 q0_”；如果用户显式提供 joint_angles_as_q0，则仅覆盖手臂段
            const Eigen::VectorXd q0_seed = buildUserQ0FromCmd(q0_);
            auto start = std::chrono::high_resolution_clock::now();
            
            // 如果有腰部关节，对位姿进行变换
            Eigen::Quaterniond left_quat = ik_cmd_left_.quat;
            Eigen::Vector3d left_pos = ik_cmd_left_.pos_xyz;
            Eigen::Quaterniond right_quat = ik_cmd_right_.quat;
            Eigen::Vector3d right_pos = ik_cmd_right_.pos_xyz;
            Eigen::Vector3d left_elbow_pos = ik_cmd_left_.elbow_pos_xyz;
            Eigen::Vector3d right_elbow_pos = ik_cmd_right_.elbow_pos_xyz;
            
            if (has_waist_joint_)
            {
                auto [left_pos_t, left_quat_t] = transformPoseByWaistYaw(left_pos, left_quat);
                auto [right_pos_t, right_quat_t] = transformPoseByWaistYaw(right_pos, right_quat);
                auto [left_elbow_pos_t, _] = transformPoseByWaistYaw(left_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
                auto [right_elbow_pos_t, __] = transformPoseByWaistYaw(right_elbow_pos, Eigen::Quaterniond(1, 0, 0, 0));
                
                left_pos = left_pos_t;
                left_quat = left_quat_t;
                right_pos = right_pos_t;
                right_quat = right_quat_t;
                left_elbow_pos = left_elbow_pos_t;
                right_elbow_pos = right_elbow_pos_t;
            }
            
            std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                    {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                    {left_quat, left_pos},
                    {right_quat, right_pos},
                    {Eigen::Quaterniond(1, 0, 0, 0), left_elbow_pos},
                    {Eigen::Quaterniond(1, 0, 0, 0), right_elbow_pos}
                    };
            Eigen::VectorXd q;
            checkInWorkspace(pose_vec[1].second, pose_vec[2].second);
            // bool result = ik_.solve(pose_vec, q0_, q, ik_solve_params_);
            bool result = solveWithBinarySearch(pose_vec, q0_seed, q);
            std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
            // std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;
            res.success = false;
            res.with_torso = false;
            res.time_cost = elapsed.count();
            if(result)
            {
                // 求解成功后，检查与期望位姿的误差是否过大（>5mm 或 >60deg），过大则打日志
                checkDesiredPoseErrorAndLog(pose_vec, q, "free_handleServiceRequest");
                res.success = true;
                q0_ = Eigen::VectorXd(q);
                //
                const int start_idx = q.size() - 2*single_arm_num_;
                for (int j = 0; j < 2*single_arm_num_; ++j)
                {
                    res.q_arm.push_back(q[start_idx+j]);
                }
                // 如果只控制单手，则将其他手臂的关节位置设置为0
                if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                    std::fill(res.q_arm.begin() + single_arm_num_, res.q_arm.begin() + single_arm_num_ * 2, 0);
                else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                    std::fill(res.q_arm.begin() + 0, res.q_arm.begin() + single_arm_num_, 0);
                if(start_idx > 0)//包含躯干
                {
                    res.with_torso = true;
                    res.q_torso = std::vector<double>(q.data(), q.data() + start_idx);
                }
                kuavo_msgs::twoArmHandPoseFree msg = publish_ik_result_info_free(q);
                res.hand_poses = msg;
            }
            if(print_ik_info_)
                printIkResultInfo(ik_cmd_left_, ik_cmd_right_, q, result);
            // 返回响应
            return true;
        }

        bool handleFKServiceRequest(kuavo_msgs::fkSrv::Request &req, kuavo_msgs::fkSrv::Response &res) 
        {
            const int num_dof = q0_.size(); 
            if(req.q.size() != num_dof)
            {
                ROS_ERROR_STREAM("The size of the request q ("<< req.q.size() << ") is not equal to the number of dof in ik_node(" << num_dof << ")");
                res.success = false;
                return false;
            }
            Eigen::VectorXd q = Eigen::VectorXd::Zero(num_dof);
            for(int i = 0; i < num_dof; i++)
                q(i) = req.q[i];
            auto msg = publish_ik_result_info(q);
            res.hand_poses = msg;
            res.success = true;
            return true;
        }

    // 基于雅可比的伪逆求解，用于提供初始猜测
    Eigen::VectorXd solvePseudoInverseIK(const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>& pose_vec, const Eigen::VectorXd& q0_init)
    {
        const int MAX_ITER = 50;
        const double POS_TOL = 1e-2;  // 位置容差（1cm）
        const double ROT_TOL = 0.1;   // 姿态容差（约5.7度）
        const double LAMBDA = 0.1;    // 阻尼系数（增大以提高稳定性，避免奇异点问题）
        const double STEP_SIZE = 0.3; // 步长系数（减小以提高稳定性，无量纲，0-1之间）
        
        Eigen::VectorXd q = q0_init;
        const int nq = plant_ptr_->num_positions();
        const int nv = plant_ptr_->num_velocities();
        
        // 获取左右手末端执行器frame
        const auto& left_frame = plant_ptr_->GetFrameByName(end_frames_name_[1]);
        const auto& right_frame = plant_ptr_->GetFrameByName(end_frames_name_[2]);
        const auto& world_frame = plant_ptr_->world_frame();
        
        // 目标位姿
        Eigen::Vector3d target_left_pos = pose_vec[1].second;
        Eigen::Quaterniond target_left_quat = pose_vec[1].first;
        Eigen::Vector3d target_right_pos = pose_vec[2].second;
        Eigen::Quaterniond target_right_quat = pose_vec[2].first;
        
        for (int iter = 0; iter < MAX_ITER; ++iter)
        {
            // 设置当前关节角
            plant_ptr_->SetPositions(plant_context_ptr_, q);
            
            // 计算当前位姿
            auto left_pose = left_frame.CalcPoseInWorld(*plant_context_ptr_);
            auto right_pose = right_frame.CalcPoseInWorld(*plant_context_ptr_);
            
            Eigen::Vector3d current_left_pos = left_pose.translation();
            Eigen::Quaterniond current_left_quat(left_pose.rotation().ToQuaternion());
            Eigen::Vector3d current_right_pos = right_pose.translation();
            Eigen::Quaterniond current_right_quat(right_pose.rotation().ToQuaternion());
            
            // 计算位置误差
            Eigen::Vector3d left_pos_error = target_left_pos - current_left_pos;
            Eigen::Vector3d right_pos_error = target_right_pos - current_right_pos;
            
            // 计算姿态误差（使用四元数差值的虚部，对于小误差这是准确的）
            // 计算从current到target的旋转四元数
            Eigen::Quaterniond left_rot_quat = target_left_quat * current_left_quat.inverse();
            Eigen::Quaterniond right_rot_quat = target_right_quat * current_right_quat.inverse();
            left_rot_quat.normalize();
            right_rot_quat.normalize();
            
            // 对于小的旋转误差，旋转误差向量近似为 2 * quat.vec() (虚部)
            // 对于大的旋转误差，使用angle-axis表示
            Eigen::Vector3d left_rot_error_vec;
            Eigen::Vector3d right_rot_error_vec;
            
            double left_angle = 2.0 * std::acos(std::abs(left_rot_quat.w()));
            double right_angle = 2.0 * std::acos(std::abs(right_rot_quat.w()));
            
            if (left_angle < 1e-3) {
                // 小角度近似
                left_rot_error_vec = 2.0 * left_rot_quat.vec();
            } else {
                // 大角度，使用angle-axis
                Eigen::Vector3d axis = left_rot_quat.vec().normalized();
                left_rot_error_vec = axis * left_angle;
            }
            
            if (right_angle < 1e-3) {
                // 小角度近似
                right_rot_error_vec = 2.0 * right_rot_quat.vec();
            } else {
                // 大角度，使用angle-axis
                Eigen::Vector3d axis = right_rot_quat.vec().normalized();
                right_rot_error_vec = axis * right_angle;
            }
            
            // 检查收敛
            double left_pos_norm = left_pos_error.norm();
            double right_pos_norm = right_pos_error.norm();
            double left_rot_norm = left_rot_error_vec.norm();
            double right_rot_norm = right_rot_error_vec.norm();
            
            if (left_pos_norm < POS_TOL && right_pos_norm < POS_TOL &&
                left_rot_norm < ROT_TOL && right_rot_norm < ROT_TOL)
            {
                break;
            }
            
            // 计算雅可比矩阵
            Eigen::MatrixXd J_left_pos(3, nv);
            Eigen::MatrixXd J_left_rot(3, nv);
            Eigen::MatrixXd J_right_pos(3, nv);
            Eigen::MatrixXd J_right_rot(3, nv);
            
            // 位置雅可比
            plant_ptr_->CalcJacobianTranslationalVelocity(
                *plant_context_ptr_,
                drake::multibody::JacobianWrtVariable::kV,
                left_frame,
                Eigen::Vector3d::Zero(),
                world_frame,
                world_frame,
                &J_left_pos);
            
            plant_ptr_->CalcJacobianTranslationalVelocity(
                *plant_context_ptr_,
                drake::multibody::JacobianWrtVariable::kV,
                right_frame,
                Eigen::Vector3d::Zero(),
                world_frame,
                world_frame,
                &J_right_pos);
            
            // 姿态雅可比（角速度雅可比）
            Eigen::MatrixXd J_left_spatial(6, nv);
            Eigen::MatrixXd J_right_spatial(6, nv);
            
            plant_ptr_->CalcJacobianSpatialVelocity(
                *plant_context_ptr_,
                drake::multibody::JacobianWrtVariable::kV,
                left_frame,
                Eigen::Vector3d::Zero(),
                world_frame,
                world_frame,
                &J_left_spatial);
            
            plant_ptr_->CalcJacobianSpatialVelocity(
                *plant_context_ptr_,
                drake::multibody::JacobianWrtVariable::kV,
                right_frame,
                Eigen::Vector3d::Zero(),
                world_frame,
                world_frame,
                &J_right_spatial);
            
            J_left_rot = J_left_spatial.bottomRows<3>();
            J_right_rot = J_right_spatial.bottomRows<3>();
            
            // 组合雅可比矩阵 [6xN] = [位置(3xN); 姿态(3xN)]
            Eigen::MatrixXd J_left(6, nv);
            J_left << J_left_pos, J_left_rot;
            
            Eigen::MatrixXd J_right(6, nv);
            J_right << J_right_pos, J_right_rot;
            
            // 组合误差向量（位置误差单位是m，姿态误差单位是rad，需要加权平衡）
            // 位置误差权重：1.0，姿态误差权重：0.1（姿态误差通常比位置误差大一个数量级）
            const double POS_WEIGHT = 1.0;
            const double ROT_WEIGHT = 0.1;
            
            Eigen::VectorXd error_left(6);
            error_left << POS_WEIGHT * left_pos_error, ROT_WEIGHT * left_rot_error_vec;
            
            Eigen::VectorXd error_right(6);
            error_right << POS_WEIGHT * right_pos_error, ROT_WEIGHT * right_rot_error_vec;
            
            // 组合成完整的加权雅可比矩阵
            Eigen::MatrixXd J_weighted(12, nv);
            // 对雅可比矩阵的对应行进行加权（使用动态block，因为nv是运行时变量）
            J_weighted.block(0, 0, 3, nv) = POS_WEIGHT * J_left_pos;
            J_weighted.block(3, 0, 3, nv) = ROT_WEIGHT * J_left_rot;
            J_weighted.block(6, 0, 3, nv) = POS_WEIGHT * J_right_pos;
            J_weighted.block(9, 0, 3, nv) = ROT_WEIGHT * J_right_rot;
            
            Eigen::VectorXd error(12);
            error << error_left, error_right;
            
            // 阻尼最小二乘求解: delta_q = (J^T * J + lambda * I)^(-1) * J^T * error
            Eigen::MatrixXd JtJ = J_weighted.transpose() * J_weighted;
            JtJ.diagonal().array() += LAMBDA;
            Eigen::VectorXd delta_q = JtJ.ldlt().solve(J_weighted.transpose() * error);
            
            // 更新关节角（使用步长控制）
            Eigen::VectorXd new_q = q + STEP_SIZE * delta_q;
            
            // 处理关节限位
            Eigen::VectorXd q_lower = plant_ptr_->GetPositionLowerLimits();
            Eigen::VectorXd q_upper = plant_ptr_->GetPositionUpperLimits();
            for (int i = 0; i < nq; ++i)
            {
                new_q[i] = std::clamp(new_q[i], q_lower[i], q_upper[i]);
            }
            
            q = new_q;
        }
        
        return q;
    }

    // 解析/半解析的手臂 IK（从 python 移植）
    //
    // - 输入：pose_vec（与 drake IK 相同，pose_vec[1]=左手目标，pose_vec[2]=右手目标）
    // - 输出：仅包含左右臂 7DoF 关节角（解析求解本身不依赖 q0/参考关节）
    //
    // 注意：解析求解器脚本的 FK 终点是 link7（zarm_*7_link），而本节点的目标帧通常是 eef_*（包含工具偏置）。
    // 因此这里会把 “eef 目标位姿” 转换为 “link7 目标位姿” 再喂给解析求解器（若无法解析固定变换则回退为直接使用 eef 目标）。
    using ArmJoints7 = Eigen::Matrix<double, 7, 1>;
    using AnalyticSeedPair = std::pair<ArmJoints7, ArmJoints7>;
    std::optional<AnalyticSeedPair> solveAnalyticArmsOnly(
        const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>& pose_vec)
    {
        if (pose_vec.size() <= 2)
            return std::nullopt;
        if (single_arm_num_ != 7)
            return std::nullopt;

        // 肩部/基座在世界坐标系下的位置
        const Eigen::Vector3d p_bSl =
            plant_ptr_->GetFrameByName(shoulder_frame_names_[0]).CalcPoseInWorld(*plant_context_ptr_).translation();
        const Eigen::Vector3d p_bSr =
            plant_ptr_->GetFrameByName(shoulder_frame_names_[1]).CalcPoseInWorld(*plant_context_ptr_).translation();

        // 期望目标位姿（world 下），先按 eef 目标读取；若可计算 link7->eef 固定变换，则转换成 link7 目标
        Eigen::Matrix3d R_left_link7 = pose_vec[1].first.toRotationMatrix();
        Eigen::Vector3d p_left_link7_world = pose_vec[1].second;
        Eigen::Matrix3d R_right_link7 = pose_vec[2].first.toRotationMatrix();
        Eigen::Vector3d p_right_link7_world = pose_vec[2].second;

        // 不使用 try/catch：显式枚举 frame 的 model instance，再用 GetFrameByName(name, instance)
        auto findFrameInstance =
            [&](const std::string& frame_name,
                drake::multibody::ModelInstanceIndex& out_instance) -> bool
        {
            const int n = plant_ptr_->num_model_instances();
            for (int i = 0; i < n; ++i)
            {
                const drake::multibody::ModelInstanceIndex mi(i);
                if (plant_ptr_->HasFrameNamed(frame_name, mi))
                {
                    out_instance = mi;
                    return true;
                }
            }
            return false;
        };

        drake::multibody::ModelInstanceIndex mi_l7(0), mi_r7(0), mi_eef_l(0), mi_eef_r(0);
        const bool has_l7 = findFrameInstance("zarm_l7_link", mi_l7);
        const bool has_r7 = findFrameInstance("zarm_r7_link", mi_r7);
        const bool has_eef_l = findFrameInstance(end_frames_name_[1], mi_eef_l);
        const bool has_eef_r = findFrameInstance(end_frames_name_[2], mi_eef_r);

        if (has_l7 && has_r7 && has_eef_l && has_eef_r)
        {
            const auto& l7 = plant_ptr_->GetFrameByName("zarm_l7_link", mi_l7);
            const auto& r7 = plant_ptr_->GetFrameByName("zarm_r7_link", mi_r7);
            const auto& eef_l = plant_ptr_->GetFrameByName(end_frames_name_[1], mi_eef_l);  // 通常为 "eef_left"
            const auto& eef_r = plant_ptr_->GetFrameByName(end_frames_name_[2], mi_eef_r);  // 通常为 "eef_right"

            const auto X_W_l7 = l7.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_r7 = r7.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_eef_l = eef_l.CalcPoseInWorld(*plant_context_ptr_);
            const auto X_W_eef_r = eef_r.CalcPoseInWorld(*plant_context_ptr_);

            const auto X_l7_eef = X_W_l7.inverse() * X_W_eef_l;  // link7->eef 固定变换
            const auto X_r7_eef = X_W_r7.inverse() * X_W_eef_r;  // link7->eef 固定变换

            const drake::math::RigidTransform<double> X_W_eef_l_des(
                drake::math::RotationMatrix<double>(R_left_link7), p_left_link7_world);
            const drake::math::RigidTransform<double> X_W_eef_r_des(
                drake::math::RotationMatrix<double>(R_right_link7), p_right_link7_world);

            const auto X_W_l7_des = X_W_eef_l_des * X_l7_eef.inverse();
            const auto X_W_r7_des = X_W_eef_r_des * X_r7_eef.inverse();

            R_left_link7 = X_W_l7_des.rotation().matrix();
            p_left_link7_world = X_W_l7_des.translation();
            R_right_link7 = X_W_r7_des.rotation().matrix();
            p_right_link7_world = X_W_r7_des.translation();
        }
        else
        {
            if (print_ik_info_)
            {
                std::cout << "[analytic][warn] cannot resolve frames for eef->link7 conversion:"
                          << " has_l7=" << has_l7
                          << " has_r7=" << has_r7
                          << " has_eef_l=" << has_eef_l << "(" << end_frames_name_[1] << ")"
                          << " has_eef_r=" << has_eef_r << "(" << end_frames_name_[2] << ")"
                          << " -> fallback to using eef pose directly"
                          << std::endl;
            }
        }

        const Eigen::Matrix3d R_left = R_left_link7;
        const Eigen::Vector3d p_left = p_left_link7_world - p_bSl;
        const Eigen::Matrix3d R_right = R_right_link7;
        const Eigen::Vector3d p_right = p_right_link7_world - p_bSr;

        motion_capture_ik::AnalyticArmIk::Options opts;
        opts.verbose = false;
        ArmJoints7 q_left;
        ArmJoints7 q_right;
        const bool ok_l = motion_capture_ik::AnalyticArmIk::SolveLeftArm(R_left, p_left, opts, q_left);
        const bool ok_r = motion_capture_ik::AnalyticArmIk::SolveRightArm(R_right, p_right, opts, q_right);
        if (!(ok_l && ok_r && q_left.allFinite() && q_right.allFinite()))
            return std::nullopt;

        return AnalyticSeedPair{q_left, q_right};
    }

    bool solveWithBinarySearch(const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>& pose_vec, const Eigen::VectorXd& q0_, Eigen::VectorXd& q) {
        // Step 1: 使用最粗略的精度进行求解
        auto param_tmp = ik_solve_params_;
        const double initial_tol = 20e-3;  // 初始最高精度
        param_tmp.pos_constraint_tol = initial_tol;
        param_tmp.oritation_constraint_tol = initial_tol;
        bool result = ik_.solve(pose_vec, q0_, q, param_tmp);
        auto q0_tmp = q;
        // std::cout << "q0_tmp success?: " << result << std::endl;
        if (!result) {
            // 如果最粗略精度求解失败，直接退出
            return false;
        }
        // Step 2: 使用最高精度进行求解
        result = ik_.solve(pose_vec, q0_tmp, q, ik_solve_params_);
        if (result) {
            // 如果最高精度求解成功，直接返回
            std::cout << "Highest precision solve success" << std::endl;
            return true;
        }
        // Step 3: 二分法求解
        double low_tol = param_tmp.pos_constraint_tol;  // 初始最低精度
        auto high_tol = ik_solve_params_.pos_constraint_tol;  // 初始最高精度, 失败的情况下，使用最高精度
        // std::cout << "pos_constraint_tol: " << ik_solve_params_.pos_constraint_tol << std::endl;
        // std::cout << "Initial Binary search: [" << std::fixed << std::setprecision(3) << low_tol <<", " << high_tol << "]" << std::endl;

        const double min_tol_diff = 2e-3;  // 最小精度区间，2mm

        int compute_count = 0;
        while (std::abs(high_tol - low_tol) > min_tol_diff) {
            double mid_tol = (low_tol + high_tol) / 2.0;
            param_tmp.pos_constraint_tol = mid_tol;
            param_tmp.oritation_constraint_tol = mid_tol;
            result = ik_.solve(pose_vec, q0_tmp, q, param_tmp);

            if (result) {
                // 如果求解成功，缩小下界
                low_tol = mid_tol;
            } else {
                // 如果求解失败，缩小上界
                high_tol = mid_tol;
            }
            // ROS_INFO_STREAM("Binary search: [" << compute_count << "]->[" << low_tol <<", " << high_tol << "]");
            ++compute_count;
        }
    
        return true;
    }

    private:
        HighlyDynamic::CoMIK ik_;
        drake::multibody::MultibodyPlant<double>* plant_ptr_;
        std::unique_ptr<drake::systems::Diagram<double>> diagram_ptr_;
        std::unique_ptr<drake::systems::Context<double>> diagram_context_ptr_;
        drake::systems::Context<double> *plant_context_ptr_;
        Eigen::VectorXd q0_;
        IkCmd ik_cmd_left_;
        IkCmd ik_cmd_right_;
        const int single_arm_num_{7}; 
        HighlyDynamic::HandSide hand_side_;
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber sub_ik_cmd_;
        ros::Subscriber sensor_data_raw_sub_;
        ros::Publisher joint_pub_;
        ros::Publisher time_cost_pub_;
        ros::Publisher ik_result_pub_;
        ros::Publisher ik_result_free_pub_;
        ros::Publisher head_body_pose_pub_;
        ros::ServiceServer ik_server_;
        ros::ServiceServer ik_server_muli_refer_;
        ros::ServiceServer ik_free_server_;
        ros::ServiceServer fk_server_;
        bool recived_cmd_ = false;
        bool recived__new_cmd_ = false;
        HighlyDynamic::IKParams ik_solve_params_;
        bool use_ik_cmd_q0_{false};
        long long int ik_count_{0};
        long long int ik_success_count_{0};
        std::vector<std::string> shoulder_frame_names_;
        std::vector<std::string> end_frames_name_;  // 保存end_frames_name用于伪逆求解
        bool print_ik_info_{true};
        // waist joint compensation
        bool has_waist_joint_{false};
        int waist_joint_index_{-1};
        double current_waist_yaw_{0.0};
        std::mutex waist_yaw_mutex_;
};
}

void loadJson(nlohmann::json &json_data, const std::string &filename)
{
    std::ifstream file(filename);
    if (file.is_open())
        file >> json_data;
    else
        std::cerr << "Failed to open config file: " << filename << std::endl;
}

int main(int argc, char* argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "ik_publisher");
    ros::NodeHandle nh;
    std::string model_path = "models/biped_gen4.0/urdf/biped_v3_arm.urdf";
    int control_hand_side = 2; // 0: left, 1: right, 2: both
    if(ros::param::has("model_path"))
    {
        ros::param::get("model_path", model_path);
        
        std::cout << "model_path: " << model_path << std::endl;
    }else
    {
        std::string package_path = ros::package::getPath("kuavo_assets");
        model_path = package_path + "/models/biped_s13/urdf/drake/biped_v3_arm.urdf";
    }
    if(ros::param::has("control_hand_side"))
    {
        ros::param::get("control_hand_side", control_hand_side);
        std::cout << "control_hand_side: " << control_hand_side << std::endl;
    }
    // json
    int robot_version_int=13;
    if (nh.hasParam("/robot_version"))
        nh.getParam("/robot_version", robot_version_int);

    // Handle version 15 special case: use version 14 config
    if (robot_version_int == 15) {
        robot_version_int = 14;
    }

    auto kuavo_assests_path = HighlyDynamic::getPackagePath("kuavo_assets");
    std::string model_config_file = kuavo_assests_path + "/config/kuavo_v"+std::to_string(robot_version_int)+"/kuavo.json";
    std::cout << "model_config_file: " << model_config_file << std::endl;
    nlohmann::json json_data;
    loadJson(json_data, model_config_file);
    auto end_frames_name_ik = json_data["end_frames_name_ik"].get<std::vector<std::string>>();
    auto shoulder_frame_names = json_data["shoulder_frame_names"].get<std::vector<std::string>>();
    // auto lower_arm_length = json_data["lower_arm_length"].get<double>();
    auto num_arm_dof = json_data["NUM_ARM_JOINT"].get<int>();
    auto eef_z_offset = json_data["eef_z_offset"].get<double>()/100.0;
    int single_arm_num = num_arm_dof/2;
    std::cout << "single_arm_num: " << single_arm_num << std::endl;
    Eigen::Vector3d custom_eef_frame_pos = Eigen::Vector3d(0, 0, eef_z_offset);
    HighlyDynamic::ArmsIKNode arm_ik_node(nh, model_path, end_frames_name_ik, shoulder_frame_names, custom_eef_frame_pos, HighlyDynamic::intToHandSide(control_hand_side), single_arm_num);
    double ws_r = arm_ik_node.getWorkSpaceRadius();
    std::cout << "Work space radius: " << ws_r << std::endl;
    arm_ik_node.run();

    return 0;
}
