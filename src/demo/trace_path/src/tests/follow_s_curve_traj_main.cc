#include <ros/ros.h>
#include <iostream>

#include "path_generator.h"
#include "s_curve_path_generator.h"

#include "path_tracer.h"
#include "mpc_path_tracer.h"

using namespace trace_path;
int main (int argc, char **argv)
{
    ros::init (argc, argv, "trace_path_s_curve_demo_node");
    ros::NodeHandle nh;
    
    std::cout <<" \n***********************************\n\n* Follow S Curve Traj\n";
    std::cout <<" \n***********************************\n";

    // MPC 
    
    // 参考路径中, 点的间隙 = speed * dt  (单位：m)
    double amplitude;
    nh.param("amplitude", amplitude, 1.25);  // 从参数服务器获取 amplitude 参数，默认值为 1.25
    std::cout << "The value of amplitude is: " << amplitude << std::endl;
    double length;
    nh.param("length", length, 2.5);  // 从参数服务器获取 length 参数，默认值为 2.5
    std::cout << "The value of length is: " << length << std::endl;
    SCurveParams s_curve_params = {.length = length, .amplitude = amplitude, .speed = 0.25, .dt = 0.1};
    // SCurveParams s_curve_params = {.length = 3.5, .amplitude = 1.25, .speed = 0.25, .dt = 0.1};
    double max_linear_vel;
    nh.param("max_linear_vel", max_linear_vel, 0.20);   // 从参数服务器获取 max_linear_vel 参数，默认值为 0.20
    std::cout << "The value of max_linear_vel is: " << max_linear_vel << std::endl;

    double max_angular_vel;
    nh.param("max_angular_vel", max_angular_vel, M_PI/8);    // 从参数服务器获取 max_angular_vel 参数，默认值为 M_PI/8(0.4rad/s)
    std::cout << "The value of max_angular_vel is: " << max_angular_vel << std::endl;

    // 创建路径生成器
    auto s_curve_gen = PathGeneratorFactory::Create<SCurvePathGenerator>(s_curve_params);

    auto mpc_path_tracer = PathTracerFactory::Create<MpcPathTracer>(nh);
    mpc_path_tracer->set_max_linear_velocity(max_linear_vel); // 降低速度 0.2 m/s
    mpc_path_tracer->set_max_angular_velocity(max_angular_vel);
    
    /* "S" 曲线  */
    mpc_path_tracer->Follow(s_curve_gen); // "S" 曲线 



    return 0;
}