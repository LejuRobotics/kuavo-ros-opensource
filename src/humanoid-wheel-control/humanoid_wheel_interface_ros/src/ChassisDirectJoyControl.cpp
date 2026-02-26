/**
 * @file ChassisDirectJoyControl.cpp
 * @brief 底盘直接控制节点 - 手柄模式
 * 
 * 直接通过 /move_base/base_cmd_vel 控制底盘运动，不依赖上层控制器
 * 
 * 操作: 左摇杆-移动, 右摇杆(水平)-旋转, BACK-急停
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <map>
#include <kuavo_common/common/json.hpp>

namespace chassis_direct_control
{

class ChassisDirectJoyControl
{
public:
    ChassisDirectJoyControl(ros::NodeHandle& nh) : nh_(nh), joy_received_(false)
    {
        // 默认轴/按钮映射
        axis_linear_x_ = 1;   // AXIS_LEFT_STICK_X
        axis_linear_y_ = 0;   // AXIS_LEFT_STICK_Y
        axis_angular_z_ = 3;  // AXIS_RIGHT_STICK_YAW
        button_stop_ = 6;     // BUTTON_BACK
        
        // 从参数服务器获取配置
        nh_.param<double>("linear_scale_x", linear_scale_x_, 0.3);
        nh_.param<double>("linear_scale_y", linear_scale_y_, 0.3);
        nh_.param<double>("angular_scale_z", angular_scale_z_, 0.3);
        nh_.param<double>("deadzone", deadzone_, 0.1);
        
        // 加载手柄映射配置
        std::string channel_map_path;
        if (nh_.getParam("channel_map_path", channel_map_path))
        {
            loadJoyConfig(channel_map_path);
        }
        
        // 创建发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/move_base/base_cmd_vel", 10);
        joy_sub_ = nh_.subscribe("/joy", 10, &ChassisDirectJoyControl::joyCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.02), &ChassisDirectJoyControl::timerCallback, this);
        
        ROS_INFO("============================================");
        ROS_INFO("    底盘直接控制 - 手柄模式");
        ROS_INFO("============================================");
        ROS_INFO("左摇杆: 移动 | 右摇杆: 旋转 | BACK: 急停");
        ROS_INFO("轴映射: x=%d, y=%d, yaw=%d", axis_linear_x_, axis_linear_y_, axis_angular_z_);
        ROS_INFO("============================================");
    }

    void run() { ros::spin(); }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;
    
    double linear_scale_x_, linear_scale_y_, angular_scale_z_, deadzone_;
    int axis_linear_x_, axis_linear_y_, axis_angular_z_, button_stop_;
    geometry_msgs::Twist current_cmd_;
    ros::Time last_joy_time_;
    bool joy_received_;
    
    void loadJoyConfig(const std::string& path)
    {
        try {
            std::ifstream ifs(path);
            if (!ifs.is_open()) {
                ROS_WARN("无法打开配置: %s", path.c_str());
                return;
            }
            
            nlohmann::json data;
            ifs >> data;
            
            if (data.contains("JoyAxis")) {
                auto& axis = data["JoyAxis"];
                if (axis.contains("AXIS_LEFT_STICK_X")) axis_linear_x_ = axis["AXIS_LEFT_STICK_X"];
                if (axis.contains("AXIS_LEFT_STICK_Y")) axis_linear_y_ = axis["AXIS_LEFT_STICK_Y"];
                if (axis.contains("AXIS_RIGHT_STICK_YAW")) axis_angular_z_ = axis["AXIS_RIGHT_STICK_YAW"];
            }
            if (data.contains("JoyButton") && data["JoyButton"].contains("BUTTON_BACK")) {
                button_stop_ = data["JoyButton"]["BUTTON_BACK"];
            }
            ROS_INFO("加载配置: %s", path.c_str());
        } catch (const std::exception& e) {
            ROS_WARN("配置加载失败: %s", e.what());
        }
    }
    
    double applyDeadzone(double val) { return std::abs(val) < deadzone_ ? 0.0 : val; }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        last_joy_time_ = ros::Time::now();
        joy_received_ = true;
        
        // 检查数据有效性
        if (msg->axes.size() <= static_cast<size_t>(std::max({axis_linear_x_, axis_linear_y_, axis_angular_z_}))) {
            ROS_WARN_THROTTLE(1.0, "手柄轴数量不足");
            return;
        }
        
        // 急停
        if (msg->buttons.size() > static_cast<size_t>(button_stop_) && msg->buttons[button_stop_]) {
            current_cmd_ = geometry_msgs::Twist();
            ROS_INFO_THROTTLE(1.0, "[急停]");
            return;
        }
        
        // 计算速度
        current_cmd_.linear.x = applyDeadzone(msg->axes[axis_linear_x_]) * linear_scale_x_;
        current_cmd_.linear.y = applyDeadzone(msg->axes[axis_linear_y_]) * linear_scale_y_;
        current_cmd_.angular.z = applyDeadzone(msg->axes[axis_angular_z_]) * angular_scale_z_;
        
        if (std::abs(current_cmd_.linear.x) > 0.01 || std::abs(current_cmd_.linear.y) > 0.01 || 
            std::abs(current_cmd_.angular.z) > 0.01) {
            ROS_INFO_THROTTLE(0.2, "vx=%.2f vy=%.2f wz=%.2f", 
                current_cmd_.linear.x, current_cmd_.linear.y, current_cmd_.angular.z);
        }
    }
    
    void timerCallback(const ros::TimerEvent&)
    {
        if (!joy_received_) return;
        
        // 超时保护：1秒无消息则停止
        if ((ros::Time::now() - last_joy_time_).toSec() > 1.0) {
            current_cmd_ = geometry_msgs::Twist();
            std::cout << "No joy message received for 1 second, stopping chassis" << std::endl;
        }
        
        cmd_vel_pub_.publish(current_cmd_);
    }
};

} // namespace chassis_direct_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_direct_joy_control");
    ros::NodeHandle nh("~");
    chassis_direct_control::ChassisDirectJoyControl(nh).run();
    return 0;
}
