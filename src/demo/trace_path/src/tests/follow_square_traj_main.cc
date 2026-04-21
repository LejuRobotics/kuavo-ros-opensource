#include <ros/ros.h>
#include <iostream>

#include "path_generator.h"
#include "square_path_generator.h"

#include "path_tracer.h"
#include "mpc_path_tracer.h"

using namespace trace_path;
int main (int argc, char **argv)
{
    ros::init (argc, argv, "trace_path_square_demo_node");
    ros::NodeHandle nh;
    
    std::cout <<" \n***********************************\n\n* Follow Square Traj\n";
    std::cout <<" \n***********************************\n";

    // MPC 
    
    // 参考路径中, 点的间隙 = speed * dt  (单位：m)
    double side_length;
    nh.param("side_length", side_length, 2.0);  // 从参数服务器获取 side_length 参数，默认值为 2.0
    std::cout << "The value of side_length is: " << side_length << std::endl;
    SquareParams square_params = {.side_length = side_length, .speed = 0.25, .dt = 0.1}; 
    double max_linear_vel;
    nh.param("max_linear_vel", max_linear_vel, 0.20);   // 从参数服务器获取 max_linear_vel 参数，默认值为 0.20
    std::cout << "The value of max_linear_vel is: " << max_linear_vel << std::endl;

    double max_angular_vel;
    nh.param("max_angular_vel", max_angular_vel, M_PI/8);    // 从参数服务器获取 max_angular_vel 参数，默认值为 M_PI/8(0.4rad/s)
    std::cout << "The value of max_angular_vel is: " << max_angular_vel << std::endl;

    // 创建路径生成器
    auto square_gen = PathGeneratorFactory::Create<SquarePathGenerator>(square_params);

    auto mpc_path_tracer = PathTracerFactory::Create<MpcPathTracer>(nh);
    mpc_path_tracer->set_max_linear_velocity(max_linear_vel); // 降低速度 0.2 m/s
    mpc_path_tracer->set_max_angular_velocity(max_angular_vel);
    
    /* 正方形 */
    mpc_path_tracer->Follow(square_gen);


    return 0;
}