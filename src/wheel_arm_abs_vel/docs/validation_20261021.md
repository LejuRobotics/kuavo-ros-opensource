# 2026-10-21 绝对式仿真退出与文档复验

## 修改边界

本MR仅修改src/wheel_arm_abs_vel/。公共MuJoCo源码、公共nodelet库、humanoid-control和test分支均未修改。本MR未合并。

## 退出问题

公共MujocoNodelet销毁joinable工作线程会触发std::terminate；PhysicsLoop的LCM接收线程没有停止条件，局部对象析构后仍可运行。另将模型释放从物理线程末尾移到渲染和全部工作线程结束后，消除渲染访问已释放模型的风险。

绝对式改用私有进程absolute_mujoco_sim。构建时只在build目录生成生命周期适配单元，保留原物理/控制/ROS实现；修改锚点不匹配即失败，要求复核。独立进程不创建无用途LCM线程，不使用公共MujocoNodelet，信号处理器只请求退出。默认普通X11图像传输兼容private IPC；容器SYS_NICE和rtprio权限修复Mapper优先级报错。

## 干净环境结果

- 全新Ubuntu20.04 x86_64容器、空home/build/devel，使用最小基础镜像，完整首装依赖；pip check通过。
- 通过yyb现有SSH权限重新拉取同一仓库的MR分支。文档仍以未来合并后的dev为用户入口。
- 按文档一次性配置仿真ROBOT_VERSION=62，后续从交互Shell继承，sudo保留；无手工指定Drake路径的额外启动步骤。
- 完整构建62个目标包成功，14包有警告，0失败，耗时19分10.8秒。
- 文档预检通过：Python导入、四个launch解析、节点存在、动态库无缺失。
- 初次CTest中两项暴露既有S63仿真门槛不一致；修复后重跑49项，49/49通过。
- 全量编译源码提交6b1ef9ac3451d1e17a2c004f0d06653876bb8f0d；随后debf39d63a781fb015301d9f67e115b280dd81ec只修正launch条件，已由回归和S63实际仿真验证，不涉及二进制变化。最终文档提交不改变程序。
- S62与S63均确认前台MuJoCo窗口与反馈，/real=false，未启动实机。
- Quest指定127.0.0.1协议模拟：握手成功，1185条姿态与1185条按键，0个自动发现广播。
- Pico协议模拟：从自动广播取得网卡地址并发送数据，1180条姿态与1180条按键。两组关节反馈均为有限数，连接标志为true。
- S62 Ctrl+C/SIGTERM/关闭窗口、S63 Ctrl+C：均出现仿真clean exit和控制线程join；未出现terminate、realloc、LCM通知管道错误、SIGABRT、SIGSEGV或SIGKILL升级。

## 限制与证据

真实头显、FOLLOW动作轨迹和真机稳定性未在本轮验收。编译与运行仍有非致命模型/TF等待提示；短时测试订阅者退出附近有XMLRPC连接拒绝，未影响反馈或退出，未据此声称日志全无警告。

原始证据保存在yyb电脑/home/yyb/research/absolute-exit-validation-20261021/：build.log、preflight-regression.log、regression-final.log、final-results.json、sim-*.log、quest/pico协议结果及前台截图。

复验期间dev前进到19c7fd19f98fa34737e84281816ad333cd1ab13d，仅公共增量控制两文件变化。只读合并预览无冲突；未将dev合入候选，也未合并MR。结论限定于上述实测候选。
