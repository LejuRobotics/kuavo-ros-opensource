#include "OffscreenCameraRenderer.h"

#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace mujoco_cpp {
namespace {

// 场景几何体上限。三个任务的场景都很小（实测 ngeom≈120），留足余量。
constexpr int kMaxSceneGeom = 10000;

// 句柄在 .h 里以 void* / unsigned long 存（见头文件里的说明），这里统一转回真实类型。
inline Display *AsDisplay(void *handle) {
  return static_cast<Display *>(handle);
}
inline GLXContext AsContext(void *handle) {
  return static_cast<GLXContext>(handle);
}
inline GLXPbuffer AsPbuffer(unsigned long handle) {
  return static_cast<GLXPbuffer>(handle);
}

std::string XErrorString(const char *stage) {
  std::string message(stage);
  message += " failed";
  return message;
}

}  // namespace

OffscreenCameraRenderer::OffscreenCameraRenderer() {
  mjr_defaultContext(&render_context_);
  mjv_defaultScene(&scene_);
  mjv_defaultOption(&option_);
  mjv_defaultCamera(&camera_);
}

OffscreenCameraRenderer::~OffscreenCameraRenderer() { Shutdown(); }

bool OffscreenCameraRenderer::Initialize(mjModel *m, int width, int height,
                                         std::string *reason) {
  Shutdown();
  if (m == nullptr || width <= 0 || height <= 0) {
    if (reason) *reason = "invalid model or size";
    return false;
  }
  width_ = width;
  height_ = height;

  // 1) 本线程自己的 X 连接。GLX 的 Display* 不能跨线程共享，所以不能复用 GLFW 那个。
  display_ = XOpenDisplay(nullptr);
  if (AsDisplay(display_) == nullptr) {
    const char *display_env = std::getenv("DISPLAY");
    if (reason) {
      *reason = std::string("XOpenDisplay failed (DISPLAY=") +
                (display_env ? display_env : "unset") +
                "); GLX needs an X server, use Xvfb when headless";
    }
    ReleaseGl();
    return false;
  }

  const int screen = DefaultScreen(AsDisplay(display_));
  // pbuffer 不能要求 GLX_DOUBLEBUFFER；FBConfig 与后续 visual 必须同源。
  const int fb_attribs[] = {GLX_DRAWABLE_TYPE, GLX_PBUFFER_BIT,
                            GLX_RENDER_TYPE, GLX_RGBA_BIT,
                            GLX_RED_SIZE, 8,
                            GLX_GREEN_SIZE, 8,
                            GLX_BLUE_SIZE, 8,
                            GLX_DEPTH_SIZE, 24,
                            None};
  int fb_count = 0;
  GLXFBConfig *fb_configs =
      glXChooseFBConfig(AsDisplay(display_), screen, fb_attribs, &fb_count);
  if (fb_configs == nullptr || fb_count < 1) {
    if (reason) *reason = XErrorString("glXChooseFBConfig");
    if (fb_configs) XFree(fb_configs);
    ReleaseGl();
    return false;
  }
  const GLXFBConfig fb_config = fb_configs[0];

  gl_context_ =
      glXCreateNewContext(AsDisplay(display_), fb_config, GLX_RGBA_TYPE,
                          nullptr, True);
  if (AsContext(gl_context_) == nullptr) {
    if (reason) *reason = XErrorString("glXCreateNewContext");
    XFree(fb_configs);
    ReleaseGl();
    return false;
  }

  const int pbuffer_attribs[] = {GLX_PBUFFER_WIDTH, width,
                                 GLX_PBUFFER_HEIGHT, height, None};
  pbuffer_ = static_cast<unsigned long>(
      glXCreatePbuffer(AsDisplay(display_), fb_config, pbuffer_attribs));
  if (AsPbuffer(pbuffer_) == 0) {
    if (reason) *reason = XErrorString("glXCreatePbuffer");
    XFree(fb_configs);
    ReleaseGl();
    return false;
  }
  XFree(fb_configs);

  if (!glXMakeCurrent(AsDisplay(display_), AsPbuffer(pbuffer_),
                      AsContext(gl_context_))) {
    if (reason) *reason = XErrorString("glXMakeCurrent");
    ReleaseGl();
    return false;
  }

  // 2) 离屏缓冲尺寸必须写在模型上，mjr_makeContext 会据此分配 FBO。
  m->vis.global.offwidth = width;
  m->vis.global.offheight = height;

  mjr_makeContext(m, &render_context_, mjFONTSCALE_100);
  context_made_ = true;
  // mjr_makeContext 在部分驱动上会留下 0x502 (GL_INVALID_OPERATION)，不影响后续渲染。
  // 这里主动清一次，避免污染我们自己后面的错误判断。
  while (glGetError() != GL_NO_ERROR) {
  }
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);

  mjv_makeScene(m, &scene_, kMaxSceneGeom);
  // 只要几何体：不画 site/joint/actuator/contact 等调试图元，也不要 label。
  for (int group = 0; group < mjNGROUP; ++group) {
    option_.geomgroup[group] = 1;
    option_.sitegroup[group] = 0;
    option_.jointgroup[group] = 0;
    option_.tendongroup[group] = 0;
    option_.actuatorgroup[group] = 0;
  }
  option_.flags[mjVIS_CAMERA] = 0;
  option_.flags[mjVIS_LIGHT] = 0;
  option_.flags[mjVIS_CONTACTPOINT] = 0;
  option_.flags[mjVIS_CONTACTFORCE] = 0;
  option_.flags[mjVIS_JOINT] = 0;
  option_.flags[mjVIS_ACTUATOR] = 0;
  option_.flags[mjVIS_COM] = 0;

  camera_.type = mjCAMERA_FIXED;

  color_buffer_.assign(static_cast<std::size_t>(width) * height * 3, 0);
  depth_buffer_.assign(static_cast<std::size_t>(width) * height, 0.0f);

  initialized_ = true;
  return true;
}

void OffscreenCameraRenderer::Shutdown() {
  if (context_made_) {
    // 先切回这个上下文才能安全释放它持有的 GL 资源。
    if (AsDisplay(display_) != nullptr && AsContext(gl_context_) != nullptr) {
      glXMakeCurrent(AsDisplay(display_), AsPbuffer(pbuffer_),
                     AsContext(gl_context_));
    }
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    context_made_ = false;
  }
  ReleaseGl();
  color_buffer_.clear();
  depth_buffer_.clear();
  width_ = 0;
  height_ = 0;
  initialized_ = false;
}

void OffscreenCameraRenderer::ReleaseGl() {
  if (AsDisplay(display_) != nullptr) {
    glXMakeCurrent(AsDisplay(display_), None, nullptr);
    if (AsContext(gl_context_) != nullptr) {
      glXDestroyContext(AsDisplay(display_), AsContext(gl_context_));
    }
    if (AsPbuffer(pbuffer_) != 0) {
      glXDestroyPbuffer(AsDisplay(display_), AsPbuffer(pbuffer_));
    }
    XCloseDisplay(AsDisplay(display_));
  }
  gl_context_ = nullptr;
  pbuffer_ = 0;
  display_ = nullptr;
}

bool OffscreenCameraRenderer::Render(const mjModel *m, mjData *d, int camera_id,
                                     std::uint8_t *rgb, float *depth) {
  if (!initialized_ || m == nullptr || d == nullptr || rgb == nullptr ||
      depth == nullptr) {
    return false;
  }
  if (camera_id < 0 || camera_id >= m->ncam) {
    return false;
  }
  // 当前线程必须持有本对象的上下文。跨线程调用时 glXGetCurrentContext 返回别的
  // 指针（或 nullptr），这里直接拒绝而不是画到错误的缓冲上。
  if (glXGetCurrentContext() != AsContext(gl_context_)) {
    return false;
  }

  camera_.fixedcamid = camera_id;
  mjv_updateScene(m, d, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

  const mjrRect viewport{0, 0, width_, height_};
  mjr_render(viewport, &scene_, &render_context_);
  // mjr_readPixels 的客户端缓冲从 (0,0) 开始，且第 0 行是图像**底部**（GL 约定）。
  mjr_readPixels(color_buffer_.data(), depth_buffer_.data(), viewport,
                 &render_context_);

  // 转成 ROS 行序（第 0 行 = 顶部）：整行翻转。
  const std::size_t row_bytes = static_cast<std::size_t>(width_) * 3;
  for (int row = 0; row < height_; ++row) {
    std::memcpy(rgb + static_cast<std::size_t>(row) * row_bytes,
                color_buffer_.data() +
                    static_cast<std::size_t>(height_ - 1 - row) * row_bytes,
                row_bytes);
  }
  for (int row = 0; row < height_; ++row) {
    std::memcpy(depth + static_cast<std::size_t>(row) * width_,
                depth_buffer_.data() +
                    static_cast<std::size_t>(height_ - 1 - row) * width_,
                sizeof(float) * width_);
  }

  // 深度缓冲的 raw 值转米制 z 深度。mjr_readPixels 给的是标准 GL 深度，
  // 0 → znear，1 → zfar；视锥用 mjvScene 里渲染时实际用的那一对（和 mjModel 的
  // vis.map 不一定相同，所以直接读 scn.camera）。
  const float znear = scene_.camera[0].frustum_near;
  const float zfar = scene_.camera[0].frustum_far;
  const float depth_scale = zfar - znear;
  for (std::size_t i = 0; i < static_cast<std::size_t>(width_) * height_; ++i) {
    const float raw = depth[i];
    if (raw <= 0.0f || raw >= 1.0f) {
      depth[i] = 0.0f;
      continue;
    }
    depth[i] = znear * zfar / (zfar - raw * depth_scale);
  }
  return true;
}

}  // namespace mujoco_cpp
