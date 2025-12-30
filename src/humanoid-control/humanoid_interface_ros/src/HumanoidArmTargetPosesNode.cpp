#include <ros/ros.h>
#include <ros/init.h>
#include <ros/package.h>
#include <cmath>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

#include "kuavo_msgs/armTargetPoses.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "humanoid_interface/command/HumanoidHandTarget.h"

using namespace ocs2;

class ArmTrajectoryCommandNode {
public:
    ArmTrajectoryCommandNode(ros::NodeHandle& nodeHandle, const std::string& robotName = "humanoid")
        : nh_(nodeHandle), robotName_(robotName) {
        // Initialize parameters
        initializeParameters();

        // Setup publishers and subscribers
        targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nh_, robotName_));
        trajectoryPublisher_ = nh_.advertise<ocs2_msgs::mpc_target_trajectories>(robotName_ + "_mpc_target_arm", 1);
        commandSub_ = nh_.subscribe("/kuavo_arm_target_poses", 10, &ArmTrajectoryCommandNode::commandCallback, this);
        observationSub_ = nh_.subscribe(robotName_ + "_wbc_observation", 10, &ArmTrajectoryCommandNode::observationCallback, this);
        arm_traj_mode_service_server_ = nh_.advertiseService("/arm_traj_change_mode",  &ArmTrajectoryCommandNode::changeArmCtlModeCallback, this);
        get_arm_mode_service_client_ = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");

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
    void initializeParameters() {
        // Load parameters from the ROS parameter server or set default values
        
        {
          // wait for parameters
          while (!nh_.hasParam("/mpc/mpcArmsDof") || !nh_.hasParam("/armRealDof"))
          {
            sleep(1);
          }
          ros::param::get("/mpc/mpcArmsDof", num_mpc_arm_joints_);
          ros::param::get("/armRealDof", num_arm_joints_);
          half_num_arm_joints_ = num_arm_joints_/2;
          half_num_mpc_arm_joints_ = num_mpc_arm_joints_/2;
        }
        nh_.param("/comHeight", comHeight_, 0.8);
        nh_.param("/targetArmDisplacementVelocity", targetVelocity_, 0.5);
        nh_.param("/defaultJointState", defaultJointState_, std::vector<double>(12, 0.0));

        control_mode_ = 0;   // Set default control mode
    }

    void commandCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg) {
        std::cout << "[ArmTrajNode]: Received arm target poses" << std::endl;

        if (msg->values.empty() || msg->times.empty() || msg->values.size() != msg->times.size() * num_arm_joints_) {
            ROS_WARN("[ArmTrajNode]: Invalid armTargetPoses data. Empty values or mismatched sizes.");
            return;
        }

        if (!receivedObservation_) {
            ROS_WARN("[ArmTrajNode]: Haven't received observation yet. Ignoring command.");
            return;
        }


        // Extract times and target poses
        scalar_array_t timeTrajectory;
        vector_array_t stateTrajectory;
        vector_t targetState = observation_.state.segment(12+12,num_arm_joints_);

        scalar_t currentTime = observation_.time;

        if (isFistTrajAfterChangeMode) 
        {
            stateTrajectory.push_back(initstate_);
            isFistTrajAfterChangeMode = false;
        }
        else
        {
            stateTrajectory.push_back(targetState);
        }
        timeTrajectory.push_back(currentTime);


        for (size_t i = 0; i < msg->times.size(); ++i) {
            // Create a copy of the current state

            // Extract the target arm joint positions
            for (size_t j = 0; j < num_arm_joints_; ++j) {
                scalar_t jointPosition = msg->values[i * num_arm_joints_ + j] * 3.1415926535 /180.0;
                targetState(j) = jointPosition;
            }

            // Adjust the time relative to the current observation time
            scalar_t adjustedTime = currentTime + msg->times[i];

            timeTrajectory.push_back(adjustedTime);
            stateTrajectory.push_back(targetState);
        }
        
        // Generate target trajectories
        auto targetTrajectories = generateTargetTrajectories(timeTrajectory, stateTrajectory, observation_);

        // Publish the target trajectories to MPC
        auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
        trajectoryPublisher_.publish(mpcTargetTrajectoriesMsg);
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        observation_ = ros_msg_conversions::readObservationMsg(*msg);
        receivedObservation_ = true;

        // Determine the starting index of the arm joints in the state vector
        // This should be adjusted based on actual state vector structure
        armJointStartIndex_ = 24;  // Update this index according to  state vector structure
    }

    TargetTrajectories generateTargetTrajectories(const scalar_array_t& timeTrajectory,
                                                  const vector_array_t& stateTrajectory,
                                                  const SystemObservation& observation) {
        // Check if time and state trajectories are valid
        if (timeTrajectory.empty() || stateTrajectory.empty() || timeTrajectory.size() != stateTrajectory.size()) {
            throw std::runtime_error("[ArmTrajNode]: Invalid time or state trajectory.");
        }

        // Create input trajectory (dummy inputs)
        size_t inputDim = observation.input.size();
        vector_array_t inputTrajectory(timeTrajectory.size(), vector_t::Zero(inputDim));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    bool getArmCtlModeSrv(){
        // 创建获取control_mode的服务请求
        kuavo_msgs::changeArmCtrlMode get_mode_srv;
        get_mode_srv.request.control_mode = 0;
        if (get_arm_mode_service_client_.call(get_mode_srv))
        {       
            control_mode_ = get_mode_srv.response.mode;
            return true;
        }
        else 
        {   
            std::cout << "[ArmTrajNode]: ArmCtlMode Get Fail !" << std::endl;
            return false;
        }
    }

    bool changeArmCtlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
    {
        int control_mode = req.control_mode;
        enable_ctrl_ = control_mode;
        res.result = true;
        std::cout << "[ArmTrajNode]: Arm control mode changed to " << control_mode << "\n";

        // 当前control mode与目标相同时
        if(getArmCtlModeSrv())
        {
            if (control_mode == control_mode_) return true;
        }
        else return false;

        callSetArmModeSrv(control_mode);
        // 等待OCS2 轨迹初始点同步
        if (control_mode == 2)
        {
        scalar_array_t zeroTimeTrajectory;
        vector_array_t zeroStateTrajectory;
        scalar_t zeroTime = observation_.time;
        vector_t zeroState = observation_.state.segment(12 + 12, num_arm_joints_);
        zeroTimeTrajectory.push_back(zeroTime);
        zeroStateTrajectory.push_back(zeroState);
        auto zeroTrajectories = generateTargetTrajectories(zeroTimeTrajectory, zeroStateTrajectory, observation_);
        auto mpczeroTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(zeroTrajectories);
        
        bool is_mode_change_success = false;

        // 记录mode改变后轨迹同步的开始时间
        ros::Time change_mode_start_time = ros::Time::now();


        while (! is_mode_change_success)
        {
            // 检查同步是否超时
            ros::Duration change_mode_elapsed_time = ros::Time::now() - change_mode_start_time;
            if (change_mode_elapsed_time.toSec() > 2.0)
            {
                ROS_ERROR("[ArmTrajNode]: Change mode timeout exceeded 2 seconds");
                break;
            }
            trajectoryPublisher_.publish(mpczeroTrajectoriesMsg);

            if (getArmCtlModeSrv())
            {
                // std::cout << "mode_srv control mode : " << control_mode_ << std::endl;
                if (control_mode_ == control_mode) is_mode_change_success = true;
            }

            ros::Duration(0.01).sleep();
        }
        
        ros::Duration change_mode_total_time = ros::Time::now() - change_mode_start_time;
        std::cout << "[ArmTrajNode]: Control mode change loop total time  " << change_mode_total_time.toSec() << " seconds\n";

        isFistTrajAfterChangeMode = true;
        initstate_ = zeroState;

        std::cout << "[ArmTrajNode]: External Control Mode Change Done  \n";
        }

        return true;
    }

    void callSetArmModeSrv(int32_t mode) {
        // Modify arm control mode
        // control_mode has three modes:
        // 0: keep pose
        // 1: auto_swing_arm
        // 2: external_control
        kuavo_msgs::changeArmCtrlMode srv;
        srv.request.control_mode = mode;
        auto change_arm_mode_service_client_ = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");

        // Call the service
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
    ros::Publisher trajectoryPublisher_;
    ros::Publisher ArmTargetTrajectoriesPublisher_;


    SystemObservation observation_;
    bool receivedObservation_ = false;

    // Parameters
    double comHeight_;
    double targetVelocity_;
    std::vector<double> defaultJointState_;
    int num_arm_joints_;
    int num_mpc_arm_joints_;
    int half_num_arm_joints_;
    int half_num_mpc_arm_joints_;
    int control_mode_;
    bool enable_ctrl_{false};

    bool isFistTrajAfterChangeMode = true;
    vector_t initstate_;

    // Index where the arm joints start in the state vector
    size_t armJointStartIndex_;
    ros::ServiceServer arm_traj_mode_service_server_;
    ros::ServiceClient get_arm_mode_service_client_;

};
    
int main(int argc, char* argv[]) {
    const std::string robotName = "humanoid";

    // Initialize ROS node
    ros::init(argc, argv, robotName + "_arm_trajectory_command_node");
    ros::NodeHandle nodeHandle;

    // Create and run the ArmTrajectoryCommandNode
    ArmTrajectoryCommandNode node(nodeHandle, robotName);
    node.run();

    return 0;
}
