#include <thread>  // ★ 新增：使用 std::thread 需要此头文件

#ifndef _LINUX
// Windows
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

#pragma warning(disable : 4996)

#else
// Linux
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifdef __APPLE__
#include <unistd.h>
#else
#include <linux/unistd.h>
#endif
#include <dirent.h>
#include <iostream>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <vector>
#define MAX_PATH 256
#endif

#include <map>      // ★ 存储 RigidBody ID -> Name 映射需要
#include <string>

#include "SeekerSDKTypes.h"
#include "SeekerSDKClient.h"
#include "Utility.h"

// ROS头文件
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "tf2_ros/transform_broadcaster.h"  // 新增：TF2 广播器
#include "geometry_msgs/TransformStamped.h" // 新增：TF变换消息

#include "motioncapture/ForcePlateData.h" // 自定义消息

// 根据平台决定是否使用 __cdecl
#ifndef _LINUX
void __cdecl DataHandler(sFrameOfMocapData *data, void *pUserData);
void __cdecl MessageHandler(int msgType, char *msg);
void __cdecl ForcePlateHandler(sForcePlates *pForcePlate, void *pUserData);
#else
void DataHandler(sFrameOfMocapData *data, void *pUserData);
void MessageHandler(int msgType, char *msg);
void ForcePlateHandler(sForcePlates *pForcePlate, void *pUserData);
#endif

int CreateClient(char *serverIp);

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;

SeekerSDKClient *theClient;
char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

// ★ 用于存储 RigidBody 的 ID->Name 映射
std::map<int, std::string> g_rigidBodyNames;

// ROS发布者结构体
struct ROSPublishers {
    ros::Publisher marker_pub;
    // 方案 2：给 car 和 robot 分别准备不同的 Publisher
    ros::Publisher rigid_body_car_pub;
    ros::Publisher rigid_body_robot_pub;
    ros::Publisher rigid_body_base_pub;
    ros::Publisher force_plate_pub;
    // 如有更多刚体，可在此添加更多 publisher
    
    // 新增：TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster;
};

// 回调数据结构体
struct CallbackData {
    ROSPublishers* ros_pubs;
    SeekerSDKClient* client;
};

// Linux 辅助函数
#ifdef _LINUX
int get_localip(const char *eth_name, char *local_ip_addr)
{
    int ret = -1;
    register int fd;
    struct ifreq ifr;
    if (local_ip_addr == NULL || eth_name == NULL)
    {
        return ret;
    }
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0)
    {
        strcpy(ifr.ifr_name, eth_name);
        if (!(ioctl(fd, SIOCGIFADDR, &ifr)))
        {
            ret = 0;
            strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
            printf("Found %s:%s\n", eth_name, local_ip_addr);
        }
    }
    if (fd > 0)
    {
        close(fd);
    }
    return ret;
}
#endif

int CreateClient(char *szServerIP)
{
    // 释放之前的客户端
    if (theClient)
    {
        theClient->Uninitialize();
        delete theClient;
        theClient = nullptr;
    }

    // 创建 SeekerSDK 客户端
    theClient = new SeekerSDKClient();

    // 打印版本信息
    unsigned char ver[4];
    theClient->SeekerSDKVersion(ver);
    printf("SeekerSDK Sample Client 2.4.0.4177 (SeekerSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // 初始化客户端并连接到 SeekerSDK 服务器
    int retCode = -1;
    retCode = theClient->Initialize(szServerIP); // szMyIPAddress 是客户端IP，szServerIP 是服务器IP
    strncpy(szServerIPAddress, szServerIP, sizeof(szServerIPAddress)-1);
    szServerIPAddress[sizeof(szServerIPAddress)-1] = '\0';

    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server. Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // 打印服务器信息
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        theClient->GetServerDescription(&ServerDescription);
        if (!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("Successfully connected to server\n");
    }

    // 获取数据描述
    {
        sDataDescriptions *ps = nullptr;
        theClient->GetDataDescriptions(&ps);
        if (ps)
        {
            for (int dsIndex = 0; dsIndex < ps->nDataDescriptions; ++dsIndex)
            {
                const auto &dataDescription = ps->arrDataDescriptions[dsIndex];

                switch (dataDescription.type)
                {
                case Descriptor_MarkerSetEx:
                    printf("MarkerSetExName : %s\n", dataDescription.Data.MarkerSetData->szName);
                    // ... 此处省略对 Marker 的打印，如需可自行恢复 ...
                    break;
                case Descriptor_MarkerSet:
                    printf("MarkerSetName : %s\n", dataDescription.Data.MarkerSetDescription->szName);
                    // ... 同上 ...
                    break;
                case Descriptor_RigidBody:
                {
                    // ★ 记录 RigidBody 的 ID->Name 映射
                    int rbID = dataDescription.Data.RigidBodyDescription->ID;
                    std::string rbName = dataDescription.Data.RigidBodyDescription->szName;
                    g_rigidBodyNames[rbID] = rbName;

                    printf("RigidBody: %s (ID=%d)\n", rbName.c_str(), rbID);
                    break;
                }
                case Descriptor_Skeleton:
                    // ... 如有骨骼可自行处理 ...
                    break;
                case Descriptor_ForcePlate:
                    // ... 如有力板可自行处理 ...
                    break;
                default:
                    break;
                }
            }

            theClient->FreeDataDescriptions(ps);
            ps = nullptr;
        }
    }

    theClient->WaitForForcePlateInit();
    // 数据回调和力板回调在 main 中设置

    return ErrorCode_OK;
}

#ifndef _LINUX
int _tmain(int argc, _TCHAR *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    int iResult = -1;

#ifndef _LINUX
    // Windows 特定初始化
    WSADATA wsaData;
    int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (ret != 0)
    {
        return false;
    }
    // 获取主机名和 IP 地址
    char hostname[256];
    ret = gethostname(hostname, sizeof(hostname));
    if (ret == SOCKET_ERROR)
    {
        return false;
    }
    // 获取主机 IP
    HOSTENT *host = gethostbyname(hostname);
    if (host == NULL)
    {
        return false;
    }

    printf("Found all IP addresses. [Please ensure there is an IP address in the same network with the server]\n");
    for (int iCount = 0; host->h_addr_list[iCount] != NULL; ++iCount)
    {
        strcpy(szMyIPAddress, inet_ntoa(*(in_addr *)host->h_addr_list[iCount]));
        printf("%d) [%s]\n", iCount + 1, szMyIPAddress);
    }
#else
    // Linux 特定初始化
    // get_localip("eth0", szMyIPAddress);
#endif

    // 初始化 ROS
    ros::init(argc, argv, "motioncapture_client");
    ros::NodeHandle nh;

    // 设置发布者
    ROSPublishers ros_pubs;
    // 这里仍然保留对 Marker 的发布（如果不需要可注释）
    ros_pubs.marker_pub = nh.advertise<visualization_msgs::Marker>("seeker_markers", 1000);

    // ★ 分别为 car 与 robot 创建不同的话题
    ros_pubs.rigid_body_car_pub = nh.advertise<geometry_msgs::Pose>("car_pose", 1000);
    ros_pubs.rigid_body_robot_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1000);
    ros_pubs.rigid_body_base_pub = nh.advertise<geometry_msgs::Pose>("base_pose", 1000);

    // 力板数据
    ros_pubs.force_plate_pub = nh.advertise<motioncapture::ForcePlateData>("seeker_force_plates", 1000);

    // 准备回调数据
    CallbackData callback_data;
    callback_data.ros_pubs = &ros_pubs;

    // 初始化 SeekerSDK 客户端并连接
    char ipBuf[100] = {0};
    printf("Please input the server IP\n");
    scanf("%s", ipBuf);

    iResult = CreateClient(ipBuf);
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. Exiting\n");
#ifndef _LINUX
        getch();
#else
        getchar();
#endif
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }

    // 将客户端指针赋值给回调数据
    callback_data.client = theClient;

    // 设置回调函数，并传递回调数据
    theClient->SetDataCallback(DataHandler, &callback_data);
    theClient->SetForcePlateCallback(ForcePlateHandler, &callback_data);

    // 启动 ROS 的 spin 线程
    std::thread ros_thread([](){
        ros::spin();
    });

#ifndef _LINUX
    getch();
#else
    getchar();
    getchar();
#endif

    // 清理工作
    theClient->Uninitialize();
    ros_thread.join();

    return ErrorCode_OK;
}

// DataHandler 实现
#ifndef _LINUX
void __cdecl DataHandler(sFrameOfMocapData *data, void *pUserData)
#else
void DataHandler(sFrameOfMocapData *data, void *pUserData)
#endif
{
    CallbackData* cb_data = static_cast<CallbackData*>(pUserData);
    ROSPublishers* publishers = cb_data->ros_pubs;
    // SeekerSDKClient* pClient = cb_data->client; // 如果需要使用，可保留

    // 当前时间戳，用于所有消息
    ros::Time current_time = ros::Time::now();

    // -------------------
    // 1) 发布 markers（若需要可保留，不需要可注释）
    // -------------------
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = current_time;
    marker_msg.header.frame_id = "mocap_frame";
    marker_msg.ns = "seeker_markers";
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.scale.x = 0.05; // 根据需要调整
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;

    for (int i = 0; i < data->nMarkerSets; i++)
    {
        sMarkerSetData *markerset = &data->MocapData[i];
        for (int i_Marker = 0; i_Marker < markerset->nMarkers; i_Marker++)
        {
            geometry_msgs::Pose pose;
            pose.position.x = markerset->Markers[i_Marker][0];
            pose.position.y = markerset->Markers[i_Marker][1];
            pose.position.z = markerset->Markers[i_Marker][2];
            pose.orientation.w = 1.0; // 无方向

            marker_msg.id = i * 1000 + i_Marker;
            marker_msg.pose = pose;
            publishers->marker_pub.publish(marker_msg);
        }
    }

    // -------------------
    // 2) 分别发布 car 和 robot 刚体
    // -------------------
    for (int i = 0; i < data->nRigidBodies; i++)
    {
        // 取出刚体的位姿
        geometry_msgs::Pose rb_pose;
        rb_pose.position.x = data->RigidBodies[i].x;
        rb_pose.position.y = data->RigidBodies[i].y;
        rb_pose.position.z = data->RigidBodies[i].z;
        rb_pose.orientation.x = data->RigidBodies[i].qx;
        rb_pose.orientation.y = data->RigidBodies[i].qy;
        rb_pose.orientation.z = data->RigidBodies[i].qz;
        rb_pose.orientation.w = data->RigidBodies[i].qw;

        // 根据 RigidBody ID 查找名称
        int rbID = data->RigidBodies[i].ID;
        if (g_rigidBodyNames.find(rbID) != g_rigidBodyNames.end())
        {
            std::string rbName = g_rigidBodyNames[rbID];

            // 判断名称来发布到不同话题
            if (rbName == "car")
            {
                publishers->rigid_body_car_pub.publish(rb_pose);
            }
            else if (rbName == "robot")
            {
                publishers->rigid_body_robot_pub.publish(rb_pose);
                
                // 发布 TF: mocap_frame -> mocap_robot
                geometry_msgs::TransformStamped transform_robot;
                transform_robot.header.stamp = current_time;
                transform_robot.header.frame_id = "mocap_frame";
                transform_robot.child_frame_id = "mocap_robot";
                
                // 设置变换的平移部分
                transform_robot.transform.translation.x = rb_pose.position.x;
                transform_robot.transform.translation.y = rb_pose.position.y;
                transform_robot.transform.translation.z = rb_pose.position.z;
                
                // 设置变换的旋转部分
                transform_robot.transform.rotation = rb_pose.orientation;
                
                // 广播变换
                publishers->tf_broadcaster.sendTransform(transform_robot);
            }
            else if (rbName == "base")
            {
                publishers->rigid_body_base_pub.publish(rb_pose);
                
                // 发布 TF: mocap_frame -> mocap_base
                geometry_msgs::TransformStamped transform_base;
                transform_base.header.stamp = current_time;
                transform_base.header.frame_id = "mocap_frame";
                transform_base.child_frame_id = "mocap_base";
                
                // 设置变换的平移部分
                transform_base.transform.translation.x = rb_pose.position.x;
                transform_base.transform.translation.y = rb_pose.position.y;
                transform_base.transform.translation.z = rb_pose.position.z;
                
                // 设置变换的旋转部分
                transform_base.transform.rotation = rb_pose.orientation;
                
                // 广播变换
                publishers->tf_broadcaster.sendTransform(transform_base);
            }
            else
            {
                // 如果有其他名字，可以继续 else if，或忽略
            }
        }
        else
        {
            // 如果查不到 ID->Name 的映射，可忽略或打印日志
        }
    }
}

// ForcePlateHandler 实现
#ifndef _LINUX
void __cdecl ForcePlateHandler(sForcePlates *pForcePlate, void *pUserData)
#else
void ForcePlateHandler(sForcePlates *pForcePlate, void *pUserData)
#endif
{
    CallbackData* cb_data = static_cast<CallbackData*>(pUserData);
    ROSPublishers* publishers = cb_data->ros_pubs;

    if (pForcePlate != nullptr)
    {
        for (int plateIdx = 0; plateIdx < pForcePlate->nForcePlates; ++plateIdx)
        {
            motioncapture::ForcePlateData force_plate_msg;
            force_plate_msg.frame = pForcePlate->iFrame;
            force_plate_msg.plate_id = plateIdx;

            // 复制 Fxyz
            force_plate_msg.Fxyz.clear();
            for(int j = 0; j < 3; j++)
                force_plate_msg.Fxyz.push_back(pForcePlate->ForcePlates[plateIdx].Fxyz[j]);

            // 复制 xyz
            force_plate_msg.xyz.clear();
            for(int j = 0; j < 3; j++)
                force_plate_msg.xyz.push_back(pForcePlate->ForcePlates[plateIdx].xyz[j]);

            force_plate_msg.Mfree = pForcePlate->ForcePlates[plateIdx].Mfree;

            publishers->force_plate_pub.publish(force_plate_msg);
        }
    }
}

// 可选：MessageHandler 实现（如需打印额外信息）
#ifndef _LINUX
void __cdecl MessageHandler(int msgType, char *msg)
#else
void MessageHandler(int msgType, char *msg)
#endif
{
    printf("\n%s\n", msg);
}
