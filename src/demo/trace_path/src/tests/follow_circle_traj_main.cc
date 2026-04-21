#include <ros/ros.h>
#include <iostream>

#include "path_generator.h"
#include "circle_path_generator.h"

#include "path_tracer.h"
#include "mpc_path_tracer.h"

using namespace trace_path;
int main (int argc, char **argv)
{
    ros::init (argc, argv, "trace_path_circle_demo_node");
    ros::NodeHandle nh;
    
    std::cout <<" \n***********************************\n\n* Follow Circle Traj\n";
    std::cout <<" \n***********************************\n";

    // MPC 
    
    // 参考路径中, 点的间隙 = speed * dt  (单位：m)

    double kRadius;
    nh.param("kRadius", kRadius, 1.0);  // 从参数服务器获取 radius 参数，默认值为 1.0
    std::cout << "The value of kRadius is: " << kRadius << std::endl;

    double max_linear_vel;
    nh.param("max_linear_vel", max_linear_vel, 0.20);   // 从参数服务器获取 max_linear_vel 参数，默认值为 0.20
    std::cout << "The value of max_linear_vel is: " << max_linear_vel << std::endl;

    double max_angular_vel;
    nh.param("max_angular_vel", max_angular_vel, M_PI/8);    // 从参数服务器获取 max_angular_vel 参数，默认值为 M_PI/8(0.4rad/s)
    std::cout << "The value of max_angular_vel is: " << max_angular_vel << std::endl;

    CircleParams circle_params = {.radius = kRadius, .speed = 0.25,.dt = 0.1};

    // 创建路径生成器
    auto circle_gen = PathGeneratorFactory::Create<CirclePathGenerator>(circle_params);

    auto mpc_path_tracer = PathTracerFactory::Create<MpcPathTracer>(nh);
    mpc_path_tracer->set_max_linear_velocity(max_linear_vel);     // x 速度 0.2 m/s
    mpc_path_tracer->set_max_angular_velocity(max_angular_vel);  // yaw 速度 0.4 rad/s

    /* 圆形 */
    mpc_path_tracer->Follow(circle_gen);

    return 0;
}
