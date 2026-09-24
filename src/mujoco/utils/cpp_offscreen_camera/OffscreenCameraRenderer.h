#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_cpp {

// 单线程的离屏光栅化渲染器，一次 GL 上下文服务多路相机。
//
// 替换 RayCasterCamera 的"打射线 + 按 geom 颜色上色"路径：用 mjr_render 真正渲染，
// RGB 来自 GL 颜色缓冲（带光照、材质、阴影），深度来自深度缓冲。视锥由 mjModel 里
// 该相机的 fovy 决定，和射线路径共用同一份 XML。
//
// 为什么是 GLX 而不是 EGL（实测结论，别再试 EGL）：
//   本进程里 mj::Simulate 已经用 GLFW 建了 GLX 上下文。此时 **任何** EGL 路径都会在
//   eglMakeCurrent 上失败（返回 EGL_FALSE，且 eglGetError() 还是 0x3000=EGL_SUCCESS，
//   所以日志里会显示 "EGL error 0x3000" 这种假象）：
//     - EGL_DEFAULT_DISPLAY（X11 平台）：失败
//     - EGL_PLATFORM_DEVICE_EXT（device 平台）：失败
//     - 无 DISPLAY / 有 DISPLAY、主线程 / 子线程：全部失败
//   只有在完全没有 GLFW/GLX 上下文的进程里 EGL 才正常（之前的探针就是这样，所以
//   "探针里能跑"不代表"节点里能跑"）。而 mj::Simulate 的窗口是这个节点的既有行为，
//   不能为了相机把它去掉。
//   改用 GLX pbuffer 后两条路都通，且不依赖 EGL 的任何平台扩展。
//
// 线程约束：Initialize / Render / Shutdown 必须在同一个线程调用。GL 上下文是线程
// 私有的，且 GLX 的 Display* 也不跨线程共享 —— 所以本类在自己的线程里独立
// XOpenDisplay，不使用 GLFW 那一个。
//
// 依赖 DISPLAY（GLX 要有 X 连接）。无头场景需要 Xvfb，而不是切回 EGL。
//
// 生命周期：mjModel 换了（sim 重新 load 模型）必须重新 Initialize —— GL 侧的显示
// 列表和纹理都绑在旧模型上。调用方负责检测模型指针变化并重建。
class OffscreenCameraRenderer {
public:
  OffscreenCameraRenderer();
  ~OffscreenCameraRenderer();

  OffscreenCameraRenderer(const OffscreenCameraRenderer &) = delete;
  OffscreenCameraRenderer &operator=(const OffscreenCameraRenderer &) = delete;

  // 建 GLX pbuffer + mjrContext + mjvScene，离屏缓冲设为 width×height。
  // 可重复调用（内部先 Shutdown）。失败返回 false 并写 reason。
  // m 非 const：离屏缓冲尺寸要写回 mjModel 的 vis.global，mjr_makeContext 据此分配 FBO。
  bool Initialize(mjModel *m, int width, int height, std::string *reason);
  void Shutdown();
  bool initialized() const { return initialized_; }

  // 渲染指定相机。rgb 是 width*height*3 字节 RGB8；depth 是 width*height 的米制 z 深度，
  // 0 表示该像素没打到任何 geom。两个缓冲都按 ROS 行序（第 0 行 = 图像顶部），调用方分配。
  // d 会传给 mjv_updateScene，后者签名要非 const（它只读，但不改 API 就没法传 const）。
  bool Render(const mjModel *m, mjData *d, int camera_id,
              std::uint8_t *rgb, float *depth);

  int width() const { return width_; }
  int height() const { return height_; }

private:
  void ReleaseGl();

  // GLX 句柄的不透明存储。刻意不在头文件里 include <GL/glx.h> 和 <X11/Xlib.h>：
  // X11 头会 `#define Status int`，打爆 OpenCV 的 enum Status（stitching.hpp），
  // 而 mujoco_node.cc 两者都 include。GLX 类型只在 .cpp 里出现。
  void *display_ = nullptr;     // Display*
  void *gl_context_ = nullptr;  // GLXContext
  unsigned long pbuffer_ = 0;   // GLXPbuffer

  mjrContext render_context_;
  mjvScene scene_;
  mjvOption option_;
  mjvCamera camera_;
  bool context_made_ = false;

  std::vector<std::uint8_t> color_buffer_;
  std::vector<float> depth_buffer_;

  int width_ = 0;
  int height_ = 0;
  bool initialized_ = false;
};

}  // namespace mujoco_cpp
