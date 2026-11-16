// Mesa's X11 shared-memory images cannot cross a private container IPC namespace.
// Apply only to this simulator; ordinary socket image transfer remains available.
#include <X11/Xlib.h>
#include <dlfcn.h>
#include <cstdlib>
#include <cstring>
namespace {
bool disabled() {
  const char* value = std::getenv("KUAVO_SIM_X11_NO_SHM");
  return value && std::strcmp(value, "1") == 0;
}
}
extern "C" Bool XQueryExtension(Display* d, const char* name, int* opcode, int* event, int* error) {
  if (disabled() && std::strcmp(name, "MIT-SHM") == 0) return False;
  using Query = Bool (*)(Display*, const char*, int*, int*, int*);
  const auto query = reinterpret_cast<Query>(dlsym(RTLD_NEXT, "XQueryExtension"));
  return query ? query(d, name, opcode, event, error) : False;
}
extern "C" Bool XShmQueryVersion(Display* d, int* major, int* minor, Bool* pixmaps) {
  if (disabled()) return False;
  using Query = Bool (*)(Display*, int*, int*, Bool*);
  const auto query = reinterpret_cast<Query>(dlsym(RTLD_NEXT, "XShmQueryVersion"));
  return query ? query(d, major, minor, pixmaps) : False;
}
extern "C" Bool XShmQueryExtension(Display* d) {
  if (disabled()) return False;
  using Query = Bool (*)(Display*);
  const auto query = reinterpret_cast<Query>(dlsym(RTLD_NEXT, "XShmQueryExtension"));
  return query ? query(d) : False;
}
