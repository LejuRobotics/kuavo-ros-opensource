/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '产品介绍',
      items: [
        'basic_usage/kuavo-ros-control/docs/1产品介绍/产品介绍',
        {
          type: 'category',
          label: '四代产品',
          items: [
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO 标准版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO 进阶版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO MaxA版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO MaxB版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO 展厅版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO 展厅算力版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4PRO产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/KUAVO_4_1产品介绍',
          ],
        },
        {
          type: 'category',
          label: '五代产品',
          items: [
            'basic_usage/kuavo-ros-control/docs/1产品介绍/五代产品/KUAVO_5 进阶版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/五代产品/KUAVO_5 MaxA版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/五代产品/KUAVO_5 MaxB版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/五代产品/KUAVO_5 展厅算力版产品介绍',
          ],
        },
        {
          type: 'category',
          label: '轮臂产品',
          items: [
            'basic_usage/kuavo-ros-control/docs/1产品介绍/轮臂产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/轮臂产品/Kuavo 5-W 轮臂产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/轮臂产品/Kuavo 5-W MAX A版产品介绍',
            'basic_usage/kuavo-ros-control/docs/1产品介绍/轮臂产品/Kuavo 5-W MAX B版产品介绍',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '快速开始',
      items: [
        {
          type: 'category',
          label: '人形产品快速开始',
          items: [
            'basic_usage/kuavo-ros-control/docs/2快速开始/快速开始',
          ],
        },
        {
          type: 'category',
          label: 'Kuavo 5-W 操作说明',
          items: [
            'basic_usage/kuavo-ros-control/docs/2快速开始/轮臂手柄控制说明文档/Kuavo 5-W 手柄控制说明',
            'basic_usage/kuavo-ros-control/docs/2快速开始/VR操作/Kuavo 5-W VR操作',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '调试教程',
      items: [
        {
          type: 'category',
          label: '人形产品调试教程',
          items: [
            'basic_usage/kuavo-ros-control/docs/3调试教程/快速调试',
            'basic_usage/kuavo-ros-control/docs/3调试教程/配置文件说明',
            'basic_usage/kuavo-ros-control/docs/3调试教程/启动前准备',
            'basic_usage/kuavo-ros-control/docs/3调试教程/机器人关节标定',
            'basic_usage/kuavo-ros-control/docs/3调试教程/上肢控制模式',
          ],
        },
        {
          type: 'category',
          label: 'Kuavo 5-W 调试教程',
          items: [
            'basic_usage/kuavo-ros-control/docs/3调试教程/Kuavo 5-W 快速调试',
            'basic_usage/kuavo-ros-control/docs/3调试教程/Kuavo 5-W 启动前准备',
            'basic_usage/kuavo-ros-control/docs/3调试教程/Kuavo 5-W 辨识手臂和头部电机方向',
            'basic_usage/kuavo-ros-control/docs/3调试教程/Kuavo 5-W 全身零点标定',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '开发接口',
      items: [
        {
          type: 'category',
          label: '人形产品开发接口',
          items: [
            'basic_usage/kuavo-ros-control/docs/4开发接口/仿真环境使用',
            'basic_usage/kuavo-ros-control/docs/4开发接口/SDK介绍',
            'basic_usage/kuavo-ros-control/docs/4开发接口/接口使用文档',
            'basic_usage/kuavo-ros-control/docs/4开发接口/ROS2接口使用文档',
            'basic_usage/kuavo-ros-control/docs/4开发接口/手臂防碰撞功能使用文档',
            {
              type: 'category',
              label: 'PythonSDK使用文档',
              items: [
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/kuavo-humanoid-websocket-sdk介绍',
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/Websockts通信',
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/头部以及手部控制接口',
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/机器人信息获取接口',
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/机器人移动以及步态接口',
                'basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/音频播放接口',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Kuavo 5-W 开发接口',
          items: [
            'basic_usage/kuavo-ros-control/docs/4开发接口/Kuavo 5-W 接口使用文档',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '功能案例',
      items: [
      {
          type: 'category',
          label: '四代案例',
          items: [
            {
              type: 'category',
              label: '通用案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/H12遥控器使用开发案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/VR使用开发案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/落足点规划案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/二维码检测使用案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/灵巧手手势使用案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/路径轨迹规划案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/数据采集案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/yolov8目标检测案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/手臂正逆运动学案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/机器人手臂示教案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/手臂轨迹规划案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/键盘控制案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/强化学习案例gym版',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/接入deepseek大模型语音交互案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/大模型联网搜索与视觉推理案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/二维码抓取水瓶案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/基于人脸识别的简易语音交互案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/全身打太极动作案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/强化学习案例lab版',
                'basic_usage/kuavo-ros-control/docs/5功能案例/通用案例/接入豆包实时语音大模型案例',
              ],
            },
            {
              type: 'category',
              label: '扩展案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/拓展案例/手腕相机抓取放置案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/拓展案例/百度EdgeBoard接入机器人案例',
              ],
            },
            {
              type: 'category',
              label: '综合案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/综合案例/策略模块搬箱子案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/综合案例/机器人开源导航案例',
              ],
            },
            {
              type: 'category',
              label: '轮臂案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/轮臂案例/基础使用',
              ],
            },
            'basic_usage/kuavo-ros-control/docs/5功能案例/案例目录',
          ],
        },
        {
          type: 'category',
          label: '五代案例',
          items: [
            'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/全身舞蹈动作案例',
            'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/YOLOV8识别及抓取案例',
            'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/扭腰搬箱子案例',
            'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/豆包大模型交互案例',
            'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/基于il的物料分拣案例',
          ],
        },
        {
          type: 'category',
          label: 'Kuavo 5-W 案例',
          items: [
            {  
              type: 'category',
              label: 'Kuavo 5-W 轮臂案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/轮臂案例/基础使用',
              ],
            },
            {
              type: 'category',
              label: 'pytree案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/轮臂组合案例',
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/case_wheel_test_arm',
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/case_wheel_test_head',
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/case_wheel_test_move',
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/case_wheel_test_torso',
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/pytree案例/case_wheel_test_torso_joint',
              ],
            },
            {
              type: 'category',
              label: 'Kuavo 5-W 轮臂数据采集案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/五代案例/Kuavo 5-W 轮臂数据采集案例/拆垛案例',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: '科研框架',
          items: [
            'basic_usage/kuavo-ros-control/docs/5功能案例/科研框架/强化学习运动控制框架合集',
            {
              type: 'category',
              label: '模仿学习使用案例',
              items: [
                'basic_usage/kuavo-ros-control/docs/5功能案例/科研框架/模仿学习使用案例/案例概述',
                'basic_usage/kuavo-ros-control/docs/5功能案例/科研框架/模仿学习使用案例/具身智能数据处理与模型训练框架',
                'basic_usage/kuavo-ros-control/docs/5功能案例/科研框架/模仿学习使用案例/仿真环境使用',
              ],
            },
          ],
        }
      ],
    },
    {
      type: 'category',
      label: '常用工具',
      items: [
        'basic_usage/kuavo-ros-control/docs/6常用工具/日志工具',
        'basic_usage/kuavo-ros-control/docs/6常用工具/硬件测试工具',
        'basic_usage/kuavo-ros-control/docs/6常用工具/热点工具',
        'basic_usage/kuavo-ros-control/docs/6常用工具/有线VR方案使用指南',
        'basic_usage/kuavo-ros-control/docs/6常用工具/Kuavo机器人桌面软件使用手册',
        'basic_usage/kuavo-ros-control/docs/6常用工具/Kuavo机器人启动脚本使用指南',
      ],
    },
    {
      type: 'category',
      label: '镜像恢复',
      items: [
        'basic_usage/kuavo-ros-control/docs/8上下位机镜像烧录/下位机烧录镜像',
        'basic_usage/kuavo-ros-control/docs/8上下位机镜像烧录/上位机NX烧录镜像',
        'basic_usage/kuavo-ros-control/docs/8上下位机镜像烧录/上位机AGX烧录镜像',
      ],
    },
    {
      type: 'category',
      label: 'Q&A',
      items: [
        'basic_usage/kuavo-ros-control/docs/7常见问题与故障排查/故障排查',
      ],
    },
    {
      type: 'category',
      label: 'Changelog',
      items: [
        'basic_usage/kuavo-ros-control/docs/kuavo更新日志',
        'basic_usage/kuavo-ros-control/docs/更新说明',
      ],
    },
  ],
};

module.exports = sidebars;
