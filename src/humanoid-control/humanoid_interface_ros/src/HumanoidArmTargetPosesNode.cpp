#include <ros/ros.h>
#include <ros/init.h>
#include <ros/package.h>
#include <cmath>
#include <vector>
#include <algorithm>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

#include "kuavo_msgs/armTargetPoses.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "humanoid_interface/command/HumanoidHandTarget.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

using namespace ocs2;

class ArmTrajectoryCommandNode {
public:
    ArmTrajectoryCommandNode(ros::NodeHandle& nodeHandle, const std::string& robotName = "humanoid")
        : nh_(nodeHandle), robotName_(robotName) {
        initializeParameters();

        targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nh_, robotName_));
        trajectoryPublisher_ = nh_.advertise<ocs2_msgs::mpc_target_trajectories>(robotName_ + "_mpc_target_arm", 1);
        kuavo_arm_traj_pub_ = nh_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 1, true);
        commandSub_ = nh_.subscribe("/kuavo_arm_target_poses", 10, &ArmTrajectoryCommandNode::commandCallback, this);
        observationSub_ = nh_.subscribe(robotName_ + "_wbc_observation", 10, &ArmTrajectoryCommandNode::observationCallback, this);
        arm_traj_mode_service_server_ = nh_.advertiseService("/arm_traj_change_mode",  &ArmTrajectoryCommandNode::changeArmCtlModeCallback, this);
        get_arm_mode_service_client_ = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");
        is_rl_controller_sub_ = nh_.subscribe("/humanoid_controller/is_rl_controller_", 10, &ArmTrajectoryCommandNode::isRlControllerCallback, this);
        arm_control_mode_sub_ = nh_.subscribe("/humanoid/mpc/arm_control_mode", 10, &ArmTrajectoryCommandNode::armControlModeCallback, this);
    }

    void run() {
        ROS_INFO("[ArmTrajNode]: Waiting for first observation...");
        while (!receivedObservation_ && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[ArmTrajNode]: First observation received.");
        ros::spin();
    }

private:
    static constexpr double kDegToRad = 3.141592653589793 / 180.0;
    static constexpr double kRadToDeg = 180.0 / 3.141592653589793;

    void initializeParameters() {
        while (!nh_.hasParam("/mpc/mpcArmsDof") || !nh_.hasParam("/armRealDof")) {
            sleep(1);
        }
        ros::param::get("/mpc/mpcArmsDof", num_mpc_arm_joints_);
        ros::param::get("/armRealDof", num_arm_joints_);
        half_num_arm_joints_ = num_arm_joints_ / 2;
        half_num_mpc_arm_joints_ = num_mpc_arm_joints_ / 2;

        nh_.param("/comHeight", comHeight_, 0.8);
        nh_.param("/targetArmDisplacementVelocity", targetVelocity_, 0.5);
        nh_.param("/defaultJointState", defaultJointState_, std::vector<double>(12, 0.0));

        control_mode_ = 0;
        is_rl_controller_ = 0.0;
    }

    bool isRlController() const { return is_rl_controller_ > 0.5; }

    vector_t getTrajectoryStartState() const {
        vector_t targetState = observation_.state.segment(armJointStartIndex_, num_arm_joints_);
        if (isFistTrajAfterChangeMode) {
            if (initstate_.size() == num_arm_joints_) {
                return initstate_;
            }
            ROS_WARN("[ArmTrajNode]: initstate_ is not initialized, use current arm state as trajectory start.");
            return targetState;
        }
        if (hasLastTargetState_) {
            return lastTargetState_;
        }
        return targetState;
    }

    TargetTrajectories buildArmTargetTrajectoriesFromMsg(const kuavo_msgs::armTargetPoses::ConstPtr& msg,
                                                         bool clear_first_traj_flag) {
        scalar_array_t timeTrajectory;
        vector_array_t stateTrajectory;
        vector_t targetState = getTrajectoryStartState();
        const scalar_t currentTime = observation_.time;

        timeTrajectory.push_back(currentTime);
        stateTrajectory.push_back(targetState);

        for (size_t i = 0; i < msg->times.size(); ++i) {
            for (size_t j = 0; j < static_cast<size_t>(num_arm_joints_); ++j) {
                targetState(j) = msg->values[i * num_arm_joints_ + j] * kDegToRad;
            }
            timeTrajectory.push_back(currentTime + msg->times[i]);
            stateTrajectory.push_back(targetState);
        }

        if (clear_first_traj_flag) {
            isFistTrajAfterChangeMode = false;
        }

        return generateTargetTrajectories(timeTrajectory, stateTrajectory, observation_);
    }

    void publishKuavoArmTraj(const vector_t& q_rad, const vector_t& v_rad) {
        if (q_rad.size() != num_arm_joints_) {
            return;
        }
        sensor_msgs::JointState js_msg;
        js_msg.header.stamp = ros::Time::now();
        js_msg.name.resize(num_arm_joints_);
        js_msg.position.resize(num_arm_joints_);
        js_msg.velocity.resize(num_arm_joints_);
        for (int i = 0; i < num_arm_joints_; ++i) {
            js_msg.name[i] = "arm_joint_" + std::to_string(i);
            js_msg.position[i] = q_rad(i) * kRadToDeg;
            js_msg.velocity[i] = (v_rad.size() == num_arm_joints_) ? (v_rad(i) * kRadToDeg) : 0.0;
        }
        kuavo_arm_traj_pub_.publish(js_msg);
    }

    vector_t computeTrajectoryVelocity(scalar_t time, const vector_t& q) const {
        const auto& times = rl_arm_target_trajectories_.timeTrajectory;
        if (times.size() < 2) {
            return vector_t::Zero(num_arm_joints_);
        }
        if (time <= times.front()) {
            const vector_t v = (rl_arm_target_trajectories_.getDesiredState(times[1]) - q) /
                               std::max(times[1] - times.front(), 1e-6);
            return v;
        }
        if (time >= times.back()) {
            const size_t n = times.size();
            const vector_t v = (q - rl_arm_target_trajectories_.getDesiredState(times[n - 2])) /
                               std::max(times.back() - times[n - 2], 1e-6);
            return v;
        }
        const auto upper_it = std::upper_bound(times.begin(), times.end(), time);
        const size_t idx1 = static_cast<size_t>(std::distance(times.begin(), upper_it));
        const size_t idx0 = idx1 - 1;
        return (rl_arm_target_trajectories_.stateTrajectory[idx1] -
                rl_arm_target_trajectories_.stateTrajectory[idx0]) /
               std::max(times[idx1] - times[idx0], 1e-6);
    }

    void sampleAndPublishRlArmTraj(scalar_t time) {
        if (!rl_traj_active_ || rl_arm_target_trajectories_.timeTrajectory.empty()) {
            return;
        }
        const vector_t q = rl_arm_target_trajectories_.getDesiredState(time);
        const vector_t v = computeTrajectoryVelocity(time, q);
        publishKuavoArmTraj(q, v);
    }

    void activateRlArmTrajectory(const TargetTrajectories& trajectories) {
        rl_arm_target_trajectories_ = trajectories;
        rl_traj_active_ = true;
        sampleAndPublishRlArmTraj(observation_.time);
    }

    void stopRlTrajectory() {
        rl_traj_active_ = false;
        rl_arm_target_trajectories_ = TargetTrajectories();
    }

    void commandCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg) {
        ROS_INFO("[ArmTrajNode]: Received arm target poses");

        if (msg->values.empty() || msg->times.empty() ||
            msg->values.size() != msg->times.size() * static_cast<size_t>(num_arm_joints_)) {
            ROS_WARN("[ArmTrajNode]: Invalid armTargetPoses data. Empty values or mismatched sizes.");
            return;
        }

        if (!receivedObservation_) {
            ROS_WARN("[ArmTrajNode]: Haven't received observation yet. Ignoring command.");
            return;
        }

        if (control_mode_ != 2) {
            ROS_WARN_THROTTLE(1.0,
                "[ArmTrajNode]: Arm control mode is %d (need 2: external_control). Ignoring command.", control_mode_);
            return;
        }

        if (isRlController() && rl_traj_active_) {
            hasLastTargetState_ = true;
            lastTargetState_ = rl_arm_target_trajectories_.getDesiredState(observation_.time);
            isFistTrajAfterChangeMode = false;
        }

        const bool clear_first_flag = isFistTrajAfterChangeMode;
        TargetTrajectories targetTrajectories = buildArmTargetTrajectoriesFromMsg(msg, clear_first_flag);

        if (!targetTrajectories.stateTrajectory.empty()) {
            lastTargetState_ = targetTrajectories.stateTrajectory.back();
            hasLastTargetState_ = true;
        }

        if (isRlController()) {
            // 与 MPC 相同的时间轴（observation_.time），在 observation 回调里采样并发布 kuavo_arm_traj
            activateRlArmTrajectory(targetTrajectories);
            return;
        }

        auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
        trajectoryPublisher_.publish(mpcTargetTrajectoriesMsg);
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        observation_ = ros_msg_conversions::readObservationMsg(*msg);

        int waistNums = 1;
        if (!nh_.getParam("/mpc/mpcWaistDof", waistNums)) {
            // use default
        }
        armJointStartIndex_ = 12 + 12 + waistNums;
        if (!receivedObservation_) {
            initstate_ = observation_.state.segment(armJointStartIndex_, num_arm_joints_);
        }
        receivedObservation_ = true;

        if (isRlController() && rl_traj_active_) {
            sampleAndPublishRlArmTraj(observation_.time);
        }
    }

    TargetTrajectories generateTargetTrajectories(const scalar_array_t& timeTrajectory,
                                                  const vector_array_t& stateTrajectory,
                                                  const SystemObservation& observation) {
        if (timeTrajectory.empty() || stateTrajectory.empty() ||
            timeTrajectory.size() != stateTrajectory.size()) {
            throw std::runtime_error("[ArmTrajNode]: Invalid time or state trajectory.");
        }

        size_t inputDim = observation.input.size();
        vector_array_t inputTrajectory(timeTrajectory.size(), vector_t::Zero(inputDim));
        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    void isRlControllerCallback(const std_msgs::Float64::ConstPtr& msg) {
        const bool was_rl = isRlController();
        is_rl_controller_ = msg->data;
        if (was_rl && !isRlController()) {
            stopRlTrajectory();
        }
    }

    void armControlModeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.empty()) {
            ROS_ERROR("[ArmTrajNode]: The dimension of arm control mode is 0!!");
            return;
        }

        if (isRlController()) {
            if (msg->data.size() > 1) {
                control_mode_ = static_cast<int>(msg->data[1]);
            } else {
                ROS_WARN("[ArmTrajNode]: RL mode but arm_control_mode data size < 2, using data[0]");
                control_mode_ = static_cast<int>(msg->data[0]);
            }
        } else {
            control_mode_ = static_cast<int>(msg->data[0]);
        }

        if (control_mode_ != 2 && rl_traj_active_) {
            stopRlTrajectory();
        }
    }

    bool changeArmCtlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
    {
        if (!receivedObservation_) {
            ROS_WARN("[ArmTrajNode]: Haven't received observation yet. Ignoring mode change.");
            res.result = false;
            return true;
        }

        const int control_mode = req.control_mode;
        enable_ctrl_ = control_mode;
        res.result = true;
        ROS_INFO("[ArmTrajNode]: Arm control mode change requested: %d", control_mode);

        if (control_mode == control_mode_) {
            return true;
        }

        callSetArmModeSrv(control_mode);

        if (control_mode == 2) {
            vector_t zeroState = observation_.state.segment(armJointStartIndex_, num_arm_joints_);

            bool is_mode_change_success = false;
            const ros::Time change_mode_start_time = ros::Time::now();

            while (!is_mode_change_success) {
                if ((ros::Time::now() - change_mode_start_time).toSec() > 2.0) {
                    ROS_ERROR("[ArmTrajNode]: Change mode timeout exceeded 2 seconds");
                    break;
                }

                ros::spinOnce();

                if (isRlController()) {
                    sampleAndPublishRlArmTraj(observation_.time);
                } else {
                    // 每次迭代用最新 observation_.time 重新生成零轨迹，避免时间戳过期导致 MPC 拒绝
                    scalar_array_t zeroTimeTrajectory{observation_.time};
                    vector_array_t zeroStateTrajectory{observation_.state.segment(armJointStartIndex_, num_arm_joints_)};
                    auto zeroTrajectories = generateTargetTrajectories(zeroTimeTrajectory, zeroStateTrajectory, observation_);
                    trajectoryPublisher_.publish(
                        ros_msg_conversions::createTargetTrajectoriesMsg(zeroTrajectories));
                }

                if (control_mode_ == control_mode) {
                    is_mode_change_success = true;
                }
                ros::Duration(0.01).sleep();
            }

            isFistTrajAfterChangeMode = true;
            initstate_ = zeroState;
            hasLastTargetState_ = false;
            stopRlTrajectory();
            ROS_INFO("[ArmTrajNode]: External control mode change done (RL=%s)", isRlController() ? "true" : "false");
        } else {
            stopRlTrajectory();
        }

        return true;
    }

    void callSetArmModeSrv(int32_t mode) {
        kuavo_msgs::changeArmCtrlMode srv;
        srv.request.control_mode = mode;
        auto change_arm_mode_service_client_ =
            nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");

        if (change_arm_mode_service_client_.call(srv)) {
            ROS_INFO("[ArmTrajNode]: SetArmModeSrv call successful");
        } else {
            ROS_ERROR("[ArmTrajNode]: Failed to call SetArmModeSrv");
        }
    }

    ros::NodeHandle nh_;
    std::string robotName_;
    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    ros::Subscriber commandSub_;
    ros::Subscriber observationSub_;
    ros::Subscriber is_rl_controller_sub_;
    ros::Subscriber arm_control_mode_sub_;
    ros::Publisher trajectoryPublisher_;
    ros::Publisher kuavo_arm_traj_pub_;

    SystemObservation observation_;
    bool receivedObservation_ = false;

    double comHeight_;
    double targetVelocity_;
    std::vector<double> defaultJointState_;
    int num_arm_joints_;
    int num_mpc_arm_joints_;
    int half_num_arm_joints_;
    int half_num_mpc_arm_joints_;
    int control_mode_;
    bool enable_ctrl_{false};
    double is_rl_controller_;

    bool isFistTrajAfterChangeMode = true;
    vector_t initstate_;
    vector_t lastTargetState_;
    bool hasLastTargetState_ = false;

    size_t armJointStartIndex_;
    ros::ServiceServer arm_traj_mode_service_server_;
    ros::ServiceClient get_arm_mode_service_client_;

    // AMP/RL: 与 MPC 相同的 TargetTrajectories，按 observation_.time 采样
    bool rl_traj_active_{false};
    TargetTrajectories rl_arm_target_trajectories_;
};

int main(int argc, char* argv[]) {
    const std::string robotName = "humanoid";

    ros::init(argc, argv, robotName + "_arm_trajectory_command_node");
    ros::NodeHandle nodeHandle;

    ArmTrajectoryCommandNode node(nodeHandle, robotName);
    node.run();

    return 0;
}
