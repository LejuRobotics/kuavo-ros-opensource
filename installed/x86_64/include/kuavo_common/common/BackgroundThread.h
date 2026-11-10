#pragma once
#include <pthread.h>
#include <sched.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>
#include <fstream>
namespace kuavo_thread {
// Launch-local fence. Never edits another thread, CPU isolation or RT limits.
inline int background(const char* name) {
  pthread_setname_np(pthread_self(), name);
  sched_param p{};
  int result=pthread_setschedparam(pthread_self(),SCHED_OTHER,&p);
  if(result) std::fprintf(stderr,"[%s] SCHED_OTHER failed: %s\n",name,std::strerror(result));
  cpu_set_t mask; CPU_ZERO(&mask);
  const char* text=std::getenv("KUAVO_BACKGROUND_CPUS");
  if(text && *text) {
    std::istringstream list(text); std::string part;
    while(std::getline(list,part,',')) {char* end=nullptr;long cpu=std::strtol(part.c_str(),&end,10);if(end!=part.c_str()&&!*end&&cpu>=0&&cpu<CPU_SETSIZE)CPU_SET(cpu,&mask);}
  } else {
    // Preserve the caller's placement if no launch profile was provided.
    if(pthread_getaffinity_np(pthread_self(),sizeof(mask),&mask))return -1;
    std::ifstream file("/sys/devices/system/cpu/isolated");std::string ranges;std::getline(file,ranges);std::istringstream list(ranges);std::string part;
    while(std::getline(list,part,',')){int lo=0,hi=0;if(std::sscanf(part.c_str(),"%d-%d",&lo,&hi)==1)hi=lo;for(int c=lo;c<=hi&&c<CPU_SETSIZE;++c)if(c>=0)CPU_CLR(c,&mask);}
  }
  if(CPU_COUNT(&mask)==0){std::fprintf(stderr,"[%s] empty background CPU mask; check launch profile\n",name);return -1;}
  int affinity=pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask);
  if(affinity)std::fprintf(stderr,"[%s] affinity failed: %s\n",name,std::strerror(affinity));
  return result?result:affinity;
}
// SDK-created children inherit the fence; restores the initializing caller.
class BackgroundScope {
 cpu_set_t mask_{};sched_param param_{};int policy_=SCHED_OTHER;char name_[16]{};
 public:
 BackgroundScope(){pthread_getaffinity_np(pthread_self(),sizeof(mask_),&mask_);pthread_getschedparam(pthread_self(),&policy_,&param_);pthread_getname_np(pthread_self(),name_,sizeof(name_));background("hand_init");}
 ~BackgroundScope(){pthread_setaffinity_np(pthread_self(),sizeof(mask_),&mask_);pthread_setschedparam(pthread_self(),policy_,&param_);pthread_setname_np(pthread_self(),name_);}
};
}
