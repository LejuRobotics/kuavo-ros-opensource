#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <vector>
#include <signal.h>  // for signal handling

#include "drivers_sbus.h"

#include <h12pro_controller_node/h12proRemoteControllerChannel.h>

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Shutting down...", signum);
    // Perform any necessary cleanup here
    ros::shutdown();
}

int main(int argc, char **argv) {
    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ros::init(argc, argv, "h12pro_channel_publisher_node");
    ros::NodeHandle nh;

    while (initSbus() == -1) {
        ROS_ERROR("Sbus init failed, retrying...");
        ros::Duration(1.0).sleep();  // 暂停 1 秒
    }

    ros::Publisher pub_channel = nh.advertise<h12pro_controller_node::h12proRemoteControllerChannel>("h12pro_channel", 1);
    h12pro_controller_node::h12proRemoteControllerChannel channel_msg;
    /* 按遥控器类型决定发布通道数（REMOTE_CONTROLLER_TYPE 由
     * deploy_autostart.sh 询问后写入 service/环境变量，经 roslaunch 继承）：
     *   g11      -> 16 通道（驱动已解出 16，含虚拟 CH13~16）
     *   h12/g12  -> 12 通道（旧行为，H12/G12 遥控器只发 12 个，保持原样）
     *   未配置/未知 -> 12 通道（兼容旧部署，避免收到多余默认通道）
     */
    const char *rc_type = getenv("REMOTE_CONTROLLER_TYPE");
    bool is_g11 = (rc_type != NULL) && (strcmp(rc_type, "g11") == 0);
    int total_channels = is_g11 ? 16 : 12;
    channel_msg.channels.resize(total_channels);
    ROS_INFO("Remote controller type: %s, publish %d channels",
             rc_type ? rc_type : "(unset)", total_channels);
    // 发布频率 50Hz：SBUS 手柄输入硬件更新率本就在 50Hz 量级，250Hz 属过度采样
    // （同一帧重复发布 5 次），高频空发持续占用订阅方（joy_node / nodelet_manager）
    // 的 CPU 与网络带宽。50Hz（20ms 一帧）对手柄控制无感知。
    ros::Rate rate(50);

    /* ===== 断链后的"停发"策略 =====
     * 遥控器断链后驱动会把通道复位为默认值并置 sbus_state=0，但本节点会**一直**
     * 以 50Hz 继续发这批复位帧。若此时话题上还存在外部虚拟手柄(测试/联调)的
     * sbus_state=1 帧，订阅方就会在"复位帧/正常帧"之间交替 -> 机器人走停走停。
     *
     * 这里改成：断链后先继续发布 disconnect_publish_frames 帧(默认 50 帧 = 1s)，
     * 让订阅方有机会收到 sbus_state=0 并触发各自的断连保护
     * （ocs2_h12pro_node 收到 state=0 会立即补一帧中性通道让摇杆回中），
     * 之后停止发布，把 /h12pro_channel 让给虚拟手柄独占。
     *
     * 关键：只"停发"、**不退出进程**。monitor_ocs2_h12pro.py 的 PING_NODES 含
     * /h12pro_channel_publisher，进程一旦退出会被判定异常 -> stop_tree 后重新拉起，
     * 又回到"持续发布"的状态（这正是手动杀本节点清不干净的原因）。
     *
     * 遥控器重新连上(sbus_state 回到 1)后自动恢复发布。
     */
    int disconnect_publish_frames = 50;
    nh.param<int>("disconnect_publish_frames", disconnect_publish_frames, 50);
    if (disconnect_publish_frames < 1) {
        disconnect_publish_frames = 1;
    }
    int disconnected_frames = 0;   // 连续收到的断链帧数(仅在发布态累计)
    bool publishing = true;        // 当前是否在往话题上发帧

    while (ros::ok()) {
        recSbusData();
        // 遥控器断连/关机检测: 驱动内 setitimer 50ms 累计, checkSbusTimeOut
        // 在 ≥2 个周期(约100ms)未收到新帧时把通道复位为默认值且 sbus_state=0,
        // 避免"遥控器已关但还在发布最后姿态"导致机器人持续按旧值动作。
        checkSbusTimeOut();
        channel_msg.channels[0] = SbusRxData.channel_1;
        channel_msg.channels[1] = SbusRxData.channel_2;
        channel_msg.channels[2] = SbusRxData.channel_3;
        channel_msg.channels[3] = SbusRxData.channel_4;
        channel_msg.channels[4] = SbusRxData.channel_5;
        channel_msg.channels[5] = SbusRxData.channel_6;
        channel_msg.channels[6] = SbusRxData.channel_7;
        channel_msg.channels[7] = SbusRxData.channel_8;
        channel_msg.channels[8] = SbusRxData.channel_9;
        channel_msg.channels[9] = SbusRxData.channel_10;
        channel_msg.channels[10] = SbusRxData.channel_11;
        channel_msg.channels[11] = SbusRxData.channel_12;
        if (is_g11) {
            channel_msg.channels[12] = SbusRxData.channel_13;
            channel_msg.channels[13] = SbusRxData.channel_14;
            channel_msg.channels[14] = SbusRxData.channel_15;
            channel_msg.channels[15] = SbusRxData.channel_16;
        }
        channel_msg.sbus_state = SbusRxData.sbus_state;

        // 断链: 先补发若干帧(让订阅方收到 sbus_state=0 并回中), 之后停发让出话题
        if (SbusRxData.sbus_state == 0) {
            if (publishing) {
                ++disconnected_frames;
                if (disconnected_frames > disconnect_publish_frames) {
                    publishing = false;
                    ROS_WARN("Remote controller disconnected: sent %d reset frames, "
                             "stop publishing /h12pro_channel to leave the topic to "
                             "an external/virtual joystick. (node stays alive)",
                             disconnected_frames - 1);
                }
            }
        } else {
            // 重新连上: 恢复发布
            if (!publishing) {
                publishing = true;
                ROS_INFO("Remote controller reconnected: resume publishing /h12pro_channel");
            }
            disconnected_frames = 0;
        }

        if (publishing) {
            pub_channel.publish(channel_msg);
        }
        rate.sleep();
    }

    return 0;
}
