# 绝对式仿真退出
绝对式仿真以独立的 absolute_mujoco_sim 进程运行，公共 mujoco_cpp 源文件、库、nodelet 及其他启动入口保持不变。

## 原因与修复
- 公共 MujocoNodelet 的 joinable 工作线程没有析构回收，会触发 std::terminate。私有入口在主线程执行渲染循环，不创建这条 nodelet 工作线程。
- 原 PhysicsLoop 启动的 LCM 接收线程无停止条件，当前 ROS 控制链未使用它。私有编译单元不创建这条无用途线程，避免它在局部对象销毁后访问 LCM。
- 原物理线程结束即释放模型/数据，渲染循环可能仍在访问。私有入口等待渲染、物理和手部发布线程结束，再销毁渲染器及模型数据。
- SIGINT/SIGTERM 只请求退出，不在信号处理器中 join 或输出日志。
- KUAVO_SIM_X11_NO_SHM 默认1，仅本仿真进程使用普通X11图像传输，解决private IPC容器的MIT-SHM兼容问题；设0可保留系统默认路径。

## 源码边界
cmake/absolute_simulation.cmake 从公共 MuJoCo 源码构建独立可执行文件。scripts/prepare_absolute_sim.py 只在 build 目录生成生命周期调整后的编译单元，校验每个修改锚点唯一；上游生命周期发生变化时编译明确失败，要求重新审查。其余物理、模型、ROS话题和末端逻辑复用原源码，不链接或替换公共nodelet库。

## 验证边界
退出需同时确认仿真输出clean exit、控制线程join、roslaunch日志没有SIGABRT/SIGSEGV/SIGKILL或terminate；roslaunch进程返回0本身不足以证明子进程正常退出。
