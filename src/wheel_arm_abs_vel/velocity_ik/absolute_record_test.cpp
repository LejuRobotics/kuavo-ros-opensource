// Run only on an isolated ROS master. Exercises production recording and model FK, no control loop.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <iostream>
#include <atomic>
#include <ros/master.h>

struct VelocityIkOfflineTest {
  static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  }
  static void run(PinocchioVelocityIk& n) {
    initializeAbsoluteRecording(n.nh_, n.legacy_lower_record_pub_,
                                n.legacy_eef_record_pub_, n.legacy_input_record_pub_);
    n.legacy_arm_record_pub_ = n.nh_.advertise<sensor_msgs::JointState>("/whole_body_velocity_ik/record/kuavo_arm_traj", 2);
    kuavo_msgs::twoArmHandPose eef;
    kuavo_msgs::Float32MultiArrayStamped input;
    int eef_count = 0;
    std::atomic<int> input_count{0};
    std::vector<ros::Subscriber> subscriptions;
    subscriptions.push_back(n.nh_.subscribe<kuavo_msgs::twoArmHandPose>("/whole_body_velocity_ik/record/drake_ik/eef_pose", 10,
        [&](const kuavo_msgs::twoArmHandPose::ConstPtr& m) { eef = *m; ++eef_count; }));
    subscriptions.push_back(n.nh_.subscribe<kuavo_msgs::Float32MultiArrayStamped>("/whole_body_velocity_ik/record/drake_ik/input_pos", 10,
        [&](const kuavo_msgs::Float32MultiArrayStamped::ConstPtr& m) { input = *m; ++input_count; }));
    auto measured = Eigen::VectorXd::Zero(n.decisionDofCount()).eval();
    measured(n.baseDecisionIndex(0)) = 1.2;
    measured(n.baseDecisionIndex(1)) = -.3;
    measured(n.baseDecisionIndex(2)) = .5;
    measured(n.lowerBodyDecisionIndex(3)) = .15;
    measured(n.armDecisionIndex(0, 1)) = .12;
    measured(n.armDecisionIndex(1, 1)) = -.13;
    auto reference = measured;
    reference(n.armDecisionIndex(0, 2)) = .25;
    reference(n.armDecisionIndex(1, 2)) = -.2;
    PinocchioVelocityIk::TargetSnapshot target;
    target.valid = true;
    n.setPlantDecisionPositionsInContext(n.plant_context_.get(), &n.q_full_, measured);
    std::array<velocity_ik::Pose, 2> expected_measured;
    const auto& base = n.plant_->GetFrameByName(n.base_frame_name_);
    for (int arm = 0; arm < 2; ++arm) {
      const auto tcp = n.plant_->GetFrameByName(n.hand_frame_names_[arm]).CalcPoseInWorld(*n.plant_context_);
      target.arm[arm].hand_position = tcp.translation();
      target.arm[arm].hand_orientation = Eigen::Quaterniond(tcp.rotation().matrix());
      expected_measured[arm] = n.plant_->GetFrameByName(n.absolute_record_frame_names_[arm])
          .CalcPose(*n.plant_context_, base);
    }
    std_msgs::Header header;
    header.seq = 42; header.stamp = ros::Time(123, 456); header.frame_id = "world";
    wheel_arm_abs_vel::WholeBodyIkReference formal;
    formal.header = header; formal.valid = true; formal.takeover_requested = true;
    for (const auto& name : {"knee_joint", "leg_joint", "waist_pitch_joint", "waist_yaw_joint"})
      formal.dof_names.push_back(name);
    for (int arm = 0; arm < 2; ++arm)
      for (int j = 0; j < 7; ++j)
        formal.dof_names.push_back(std::string("zarm_") + (arm ? "r" : "l") + std::to_string(j + 1) + "_joint");
    formal.q.resize(18, .1); formal.dq.resize(18, .2);
    sensor_msgs::JointState arm_message, lower_message;
    require(wheel_arm_abs_vel::arm_record::convert(formal, arm_message), "arm conversion failed");
    require(wheel_arm_abs_vel::arm_record::convertLowerBody(formal, lower_message), "lower conversion failed");
    // Give independent rosbag record discovery time to subscribe too.
    for (int i = 0; i < 100; ++i) { ros::spinOnce(); ros::WallDuration(.02).sleep(); }
    for (int i = 0; i < 20; ++i) {
      n.setPlantDecisionPositionsInContext(n.diagnostic_context_.get(), &n.diagnostic_q_full_, measured);
      n.publishAbsoluteRecordInput(header, target);
      n.setPlantDecisionPositionsInContext(n.diagnostic_context_.get(), &n.diagnostic_q_full_, reference);
      n.publishAbsoluteRecordEef(header, reference);
      n.legacy_arm_record_pub_.publish(arm_message);
      n.legacy_lower_record_pub_.publish(lower_message);
      ros::spinOnce(); ros::WallDuration(.02).sleep();
    }
    ros::spinOnce();
    require(eef_count > 0 && input_count.load() > 0, "missing combined pose publication");
    require(input.data.data.size() == 14, "input pose size");
    require(eef.header.frame_id == n.base_frame_name_ && input.header.frame_id == n.base_frame_name_, "frame mismatch");
    require(eef.header.stamp == header.stamp && input.header.stamp == header.stamp, "source stamp mismatch");
    n.setPlantDecisionPositionsInContext(n.plant_context_.get(), &n.q_full_, reference);
    for (int arm = 0; arm < 2; ++arm) {
      const auto expected_ref = n.plant_->GetFrameByName(n.absolute_record_frame_names_[arm])
          .CalcPose(*n.plant_context_, base);
      const auto& hand = arm ? eef.right_pose : eef.left_pose;
      for (int j = 0; j < 3; ++j) {
        require(std::abs(input.data.data[arm * 7 + j] - expected_measured[arm].translation()(j)) < 1e-6,
                "input world/base or TCP-offset conversion mismatch");
        require(std::abs(hand.pos_xyz[j] - expected_ref.translation()(j)) < 1e-6, "reference FK mismatch");
      }
      const Eigen::Quaterniond expected_q(expected_measured[arm].rotation().matrix());
      for (int j = 0; j < 4; ++j)
        require(std::abs(input.data.data[arm * 7 + 3 + j] - expected_q.coeffs()(j)) < 1e-6,
                "input quaternion xyzw mismatch");
      for (int j = 0; j < 7; ++j)
        require(std::abs(hand.joint_angles[j] - reference(n.armDecisionIndex(arm, j))) < 1e-6,
                "arm joint order or radians mismatch");

    }
    ros::master::V_TopicInfo topics;
    require(ros::master::getTopics(topics), "master topic query failed");
    for (const auto& topic : topics)
      require(topic.name.find("/ik_fk_result/") != 0 && topic.name != "/kuavo_arm_traj_sensor_data" &&
              topic.name != "/kuavo_arm_traj" && topic.name != "/lb_leg_traj" &&
              topic.name != "/drake_ik/eef_pose" && topic.name != "/drake_ik/input_pos",
              "unexpected incremental compatibility publisher");
    const int previous_input_count = input_count.load();
    target.valid = false;
    n.publishAbsoluteRecordInput(header, target);
    for (int i = 0; i < 10; ++i) { ros::spinOnce(); ros::WallDuration(.02).sleep(); }
    require(input_count.load() == previous_input_count, "invalid target produced input pose");
    std::cout << "PASS: 4 absolute-compatible topics, model FK, world/base transform, nonzero TCP offsets, units/order, invalid target suppression\n";
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "absolute_record_test", ros::init_options::NoSigintHandler);
  ros::NodeHandle p("~");
  p.setParam("robot_version", argc > 1 ? std::stoi(argv[1]) : 62);
  p.setParam("enable_natural_hang_bootstrap", false);
  p.setParam("left_tcp_translation", std::vector<double>{.03, .02, -.04});
  p.setParam("right_tcp_translation", std::vector<double>{-.02, .01, .05});
  p.setParam("left_tcp_rpy", std::vector<double>{.1, -.2, .3});
  p.setParam("right_tcp_rpy", std::vector<double>{-.2, .1, -.3});
  try { PinocchioVelocityIk node; VelocityIkOfflineTest::run(node); }
  catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
