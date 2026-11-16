#include <iostream>
#include <Eigen/Core>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

#include "plantIK.h"
#include "motion_capture_ik/drake_parser_compat.hpp"

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

const double TO_RADIAN = M_PI / 180.0;

int main(int argc, char* argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "ik_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 10);

    std::string model_path;
    if(ros::param::has("model_path"))
    {
        ros::param::get("model_path", model_path);        
        std::cout << "model_path: " << model_path << std::endl;
    }else
    {
        std::string package_path = ros::package::getPath("kuavo_assets");
        model_path = package_path + "/models/biped_s40/urdf/drake/biped_v3_arm.urdf";
    }
    std::vector<std::string> end_frames_name = {"torso", "l_hand_roll", "r_hand_roll"};
    // std::vector<std::string> end_frames_name = {"torso", "l_hand_roll"};
    double dt = 0.001;
    
    drake::systems::DiagramBuilder<double> builder;
    drake::multibody::MultibodyPlant<double>* plant = builder.AddSystem<drake::multibody::MultibodyPlant>(dt);
    drake::multibody::Parser parser(plant);
    (void)motion_capture_ik::drake_parser_compat::AddUrdfModel(parser, model_path);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("torso"));
    plant->Finalize();
    std::cout << "plant nq: " << plant->num_positions() << ", nv: " << plant->num_velocities() << std::endl;

    std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
    std::unique_ptr<drake::systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    drake::systems::Context<double> *plant_context = &diagram->GetMutableSubsystemContext(*plant, diagram_context.get());

    // Eigen::Vector3d p0_hand_left(-0.074, 0.2547, -0.0216);
    Eigen::Vector3d p0_hand_left(-0.0175, 0.2547, -0.0302);
    Eigen::Vector3d p0_hand_right(-0.0175, -0.2547, -0.0302);
    HighlyDynamic::CoMIK ik(plant, end_frames_name);
    double torsoY = 0 * TO_RADIAN;
    HighlyDynamic::FramePoseVec pose_vec{
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
        {Eigen::Quaterniond(1, 0, 0, 0), p0_hand_left},
        {Eigen::Quaterniond(1, 0, 0, 0), p0_hand_right}};
    // std::vector<std::vector<Eigen::Vector3d>> pose_vec{
        // {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(0, 0, 0.0)},
        // {Eigen::Vector3d(0, 0, torsoY), p0_hand_left}};
    Eigen::VectorXd q;
    Eigen::VectorXd q0 = plant->GetPositions(*plant_context);
    std::cout << "q0: " << q0.transpose() << std::endl;

    std::vector<double> times;
    while(ros::ok()) {
        auto start = std::chrono::high_resolution_clock::now();
        bool result = ik.solve(pose_vec, q0, q);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        times.push_back(elapsed.count());
        std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;

        if (result == false) {
            std::cout << "torso: " << pose_vec[0].first.coeffs() << ", " << pose_vec[0].second.transpose() << "\n";
            std::cout << "lhand: " << pose_vec[1].first.coeffs() << ", " << pose_vec[1].second.transpose() << "\n";
            std::cout << "rhand: " << pose_vec[2].first.coeffs() << ", " << pose_vec[2].second.transpose() << "\n";
            throw std::runtime_error("Failed to IK0!");
        } else {
            // 发布JointState消息
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            for (int j = 0; j < 14; ++j) {
                joint_state.name.push_back("joint_"+std::to_string(j+1));
            }
            int start_idx = 0;
            for (int j = start_idx; j < start_idx+7; ++j) {
                joint_state.position.push_back(q[j]);
            }
            for (int j = start_idx+7; j < start_idx+14; ++j) {
                joint_state.position.push_back(q[j]);
            }
            joint_pub.publish(joint_state);
        }
    }

    double total_time = 0;
    for (double t : times) {
        total_time += t;
    }
    double average_time = total_time / times.size();
    std::cout << "Average time for N times executions: " << average_time << " ms" << std::endl;
    
    {
        std::cout << "res q left : " << q.segment(0, 7).transpose() << std::endl;
        std::cout << "res q right: " << q.tail(7).transpose() << std::endl;
    }
    return 0;
}