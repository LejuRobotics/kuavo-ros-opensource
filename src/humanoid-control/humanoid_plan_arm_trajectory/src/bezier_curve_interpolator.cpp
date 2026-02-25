#include "bezier_curve_interpolator.h"
#include <algorithm>
#include <urdf/model.h>

namespace ocs2 {
namespace humanoid {

BezierCurve::BezierCurve(double start_time, double end_time, 
            drake::trajectories::BezierCurve<double> pos_traj,
            std::unique_ptr<drake::trajectories::Trajectory<double>> vel_traj,
            std::unique_ptr<drake::trajectories::Trajectory<double>> acc_traj)
    : start_time(start_time), end_time(end_time), pos_traj(std::move(pos_traj)),
      vel_traj(std::move(vel_traj)), acc_traj(std::move(acc_traj)) {}

BezierCurveInterpolator::BezierCurveInterpolator(const std::string& name, int joint_num, const std::string& interpolate_type)
    : HumanoidPlanArmTrajectory(name, joint_num, interpolate_type) {
}

void BezierCurveInterpolator::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
    nh_ = &nh;
    private_nh_ = &private_nh;

    initializeCommon();
    initializeSpecific();
    initializeLimitations();
}

void BezierCurveInterpolator::reset() {
    step_ = 0.0;
    is_stopped_ = false;
    is_finished_ = false;
    stop_step_ = 0.0;
    total_time_ = 0.0;
    interpolate_finished_ = false;
    bezier_curves_.clear();
    traj_.points.clear();
    joint_names_.clear();
}

void BezierCurveInterpolator::initializeLimitations() {
    std::string urdf_file_path;

    // Wait for urdfFile parameter
    ros::Rate wait_rate(1);
    while (ros::ok()) {
        if (nh_->hasParam("urdfFile")) {
            break;
        }
        ROS_WARN("Waiting for 'urdfFile' parameter to be set...");
        wait_rate.sleep();
    }
    
    if (!nh_->getParam("urdfFile", urdf_file_path)) {
        ROS_ERROR("Failed to get param 'urdfFile'");
        return;
    }

    urdf::Model model;
    if (!model.initFile(urdf_file_path)) {
        ROS_ERROR("Failed to parse URDF file");
        return;
    }

    int end_effector_joints_num;
    ros::Rate rate(1);
    while (ros::ok()) {
        if (nh_->getParam("end_effector_joints_num", end_effector_joints_num)) {
            printf("end_effector_joints_num: %d\n", end_effector_joints_num);
            break;
        } else {
            ROS_WARN("Waiting for 'end_effector_joints_num' parameter to be set...");
        }
        rate.sleep();
    }

    int arm_joint_num = arm_joint_names_.size();
    int head_joint_num = head_joint_names_.size();
    // 关节总数 * 3, [0] 为下限, [1] 为上限, [2] 为是否存在(0: 不存在, 1: 存在)
    Eigen::MatrixXd joint_limits(arm_joint_num + HAND_JOINT_TOTAL_NUM + head_joint_num, 3);
    joint_limits.setZero(); 

    for (size_t i = 0; i < arm_joint_num; ++i) {
        const std::string& joint_name = arm_joint_names_[i];
        std::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_name);

        if (joint && joint->limits) {
            joint_limits(i, JOINT_LIMIT_LOWER) = joint->limits->lower;
            joint_limits(i, JOINT_LIMIT_UPPER) = joint->limits->upper;
            joint_limits(i, JOINT_LIMIT_STATUS) = JOINT_LIMIT_EXISTS;
        } else {
            ROS_WARN("[BezierCurveInterpolator] Joint %s not found or has no limits.", joint_name.c_str());
        }
    }

    int one_hand_exist_joint_num = end_effector_joints_num / 2;
    int one_hand_total_joint_num = HAND_JOINT_TOTAL_NUM / 2;
    for (size_t i = 0; i < HAND_JOINT_TOTAL_NUM; ++i) {
        if (i < one_hand_exist_joint_num) {
            joint_limits(i + arm_joint_num, JOINT_LIMIT_LOWER) = hand_joint_lower_limit_;
            joint_limits(i + arm_joint_num, JOINT_LIMIT_UPPER) = hand_joint_upper_limit_;
            joint_limits(i + arm_joint_num, JOINT_LIMIT_STATUS) = JOINT_LIMIT_EXISTS;
        }
        if (i < (one_hand_exist_joint_num + one_hand_total_joint_num) && i >= one_hand_total_joint_num) {
            joint_limits(i + arm_joint_num, JOINT_LIMIT_LOWER) = hand_joint_lower_limit_;
            joint_limits(i + arm_joint_num, JOINT_LIMIT_UPPER) = hand_joint_upper_limit_;
            joint_limits(i + arm_joint_num, JOINT_LIMIT_STATUS) = JOINT_LIMIT_EXISTS;
        }
        if (joint_limits(i + arm_joint_num, JOINT_LIMIT_STATUS) == JOINT_LIMIT_NOT_EXISTS) {
            ROS_WARN("[BezierCurveInterpolator] Joint %ld not found or has no limits.", i + arm_joint_num);
        }
    }

    for (size_t i = 0; i < head_joint_num; ++i) {
        const std::string& joint_name = head_joint_names_[i];
        std::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_name);

        if (joint && joint->limits) {
            joint_limits(i + arm_joint_num + HAND_JOINT_TOTAL_NUM, JOINT_LIMIT_LOWER) = joint->limits->lower;
            joint_limits(i + arm_joint_num + HAND_JOINT_TOTAL_NUM, JOINT_LIMIT_UPPER) = joint->limits->upper;
            joint_limits(i + arm_joint_num + HAND_JOINT_TOTAL_NUM, JOINT_LIMIT_STATUS) = JOINT_LIMIT_EXISTS;
        } else {
            ROS_WARN("[BezierCurveInterpolator] Joint %s not found or has no limits.", joint_name.c_str());
        }
    }

    joint_limits_ = joint_limits;
}

void BezierCurveInterpolator::initializeSpecific() {
    std::cout << "interpolate_type_: " << interpolate_type_ << std::endl;
    
    plan_arm_traj_srv_ = nh_->advertiseService(interpolate_type_ + "/plan_arm_trajectory", 
                                              &BezierCurveInterpolator::planArmTrajectoryBezierCurveCallback, this);
}

void BezierCurveInterpolator::update() {
    if (!interpolate_finished_ || bezier_curves_.empty() || joint_num_ == 0) {
        return;
    }
    step_ += dt_;
    double current_step = is_stopped_ ? stop_step_ : step_;
    ROS_DEBUG("[BezierCurveInterpolator] Trajectory progress: %f / %f", current_step, total_time_);

    std::vector<double> positions(joint_num_, 0.0);
    std::vector<double> velocities(joint_num_, 0.0);
    std::vector<double> accelerations(joint_num_, 0.0);

    if (!bezier_curves_.empty()) {
        for (size_t i = 0; i < bezier_curves_.size(); ++i) {
            evaluate(i, bezier_curves_[i], current_step, positions, velocities, accelerations);
        }
    }

    is_finished_ = current_step >= total_time_;
    int progress = static_cast<int>(current_step * 1000);  // ms
    updateTrajectoryPoint(positions, velocities, accelerations);
    // updateJointState(positions, velocities);
    updateTrajectoryState(progress, is_finished_);
}

void BezierCurveInterpolator::evaluate(const int index, const std::list<BezierCurve>& curve_list, double current_step, 
                                       std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations) {
    auto it = std::find_if(curve_list.begin(), curve_list.end(),
                           [current_step](const BezierCurve& curve) { return current_step <= curve.end_time; });

    if (it == curve_list.end()) {
        it = std::prev(curve_list.end());
    }

    const auto& curve = *it;
    double t = std::clamp((std::min(current_step, total_time_) - curve.start_time) / (curve.end_time - curve.start_time), 0.0, 1.0);

    Eigen::Vector2d pos = curve.pos_traj.value(t);
    Eigen::Vector2d vel = curve.vel_traj->value(t);
    Eigen::Vector2d acc = curve.acc_traj->value(t);

    if (static_cast<int>(joint_limits_(index, JOINT_LIMIT_STATUS)) == JOINT_LIMIT_EXISTS) {
        positions[index] = std::clamp(pos.y(), joint_limits_(index, JOINT_LIMIT_LOWER), joint_limits_(index, JOINT_LIMIT_UPPER));
    } else {
        positions[index] = pos.y();
    }
    velocities[index] = vel.y();
    accelerations[index] = acc.y();
}

void BezierCurveInterpolator::interpolate() {
    bezier_curves_.clear();
    bezier_curves_.reserve(control_points_.size());
    joint_num_ = control_points_.size();
    for (size_t i = 0; i < control_points_.size(); i++) {
        auto curve_list = std::list<BezierCurve>();
        for (size_t j = 1; j < control_points_[i].size(); j++) {
            createSingleBezierCurve(i, j, curve_list);
        }
        bezier_curves_.emplace_back(std::move(curve_list));
    }
    interpolate_finished_ = true;
}

void BezierCurveInterpolator::createSingleBezierCurve(size_t i, size_t j, std::list<BezierCurve>& curve_list) {
    double start_time = control_points_[i][j-1][0][0];
    double end_time = control_points_[i][j][0][0];

    Eigen::Matrix<double, 2, 4> control_points_matrix;
    control_points_matrix.col(0) = control_points_[i][j-1][0];
    control_points_matrix.col(1) = control_points_[i][j-1][2];
    control_points_matrix.col(2) = control_points_[i][j][1];
    control_points_matrix.col(3) = control_points_[i][j][0];
    drake::trajectories::BezierCurve<double> pos_traj(0, 1, control_points_matrix);
    auto vel_traj = pos_traj.MakeDerivative();
    auto acc_traj = pos_traj.MakeDerivative(2);

    curve_list.emplace_back(start_time, end_time, std::move(pos_traj), std::move(vel_traj), std::move(acc_traj));
}

bool BezierCurveInterpolator::planArmTrajectoryBezierCurveCallback(kuavo_msgs::planArmTrajectoryBezierCurve::Request& req, 
                                                                    kuavo_msgs::planArmTrajectoryBezierCurve::Response& res) {
    
    std::cout << "BezierCurveInterpolator::planArmTrajectoryBezierCurveCallback" << std::endl;
    current_interpolate_type_ = interpolate_type_;
    std::cout << "current_interpolate_type_: " << current_interpolate_type_ << std::endl;
    reset();

    total_time_ = req.end_frame_time - req.start_frame_time;
    ControlPointsType control_points;
    for (const auto& frames : req.multi_joint_bezier_trajectory) {
      auto curve_points = frames.bezier_curve_points;
      std::vector<std::vector<Eigen::VectorXd>> joint_control_points;

      for (size_t i = 0; i < curve_points.size(); ++i) {
          const auto& curve_point = curve_points[i];
          double curve_start_time = curve_point.end_point[0];
          double curve_end_time = (i + 1 < curve_points.size()) ? curve_points[i+1].end_point[0] : curve_start_time;
          
          std::vector<Eigen::VectorXd> frame_points;
          Eigen::VectorXd end_point(2);
          Eigen::VectorXd left_control_point(2);
          Eigen::VectorXd right_control_point(2);
          end_point << curve_point.end_point[0], curve_point.end_point[1];
          left_control_point << curve_point.left_control_point[0], curve_point.left_control_point[1];
          right_control_point << curve_point.right_control_point[0], curve_point.right_control_point[1];
          frame_points.push_back(end_point);
          frame_points.push_back(left_control_point);
          frame_points.push_back(right_control_point);
          joint_control_points.push_back(frame_points);
      }
      control_points.push_back(joint_control_points);
    }
    control_points_ = control_points;
    joint_names_ = req.joint_names;
    interpolate();

    res.success = true;
    return true;
}

} // namespace humanoid
} // namespace ocs2
