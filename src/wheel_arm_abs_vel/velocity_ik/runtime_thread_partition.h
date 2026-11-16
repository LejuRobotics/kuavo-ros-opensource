#pragma once
#include <pthread.h>
#include <sched.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <iostream>

namespace velocity_ik {
inline void configureIkThread() {
  const char* cpus=std::getenv("KUAVO_IK_CPUS");
  if (!cpus || !*cpus) return;
  cpu_set_t selected; CPU_ZERO(&selected);
  std::stringstream input(cpus); std::string value;
  while (std::getline(input,value,',')) {
    std::size_t used=0; const int cpu=std::stoi(value,&used);
    if (used!=value.size() || cpu<0 || cpu>=CPU_SETSIZE)
      throw std::runtime_error("Invalid IK CPU configuration");
    CPU_SET(cpu,&selected);
  }
  int error=pthread_setaffinity_np(pthread_self(),sizeof(selected),&selected);
  if (error) throw std::runtime_error(std::string("IK affinity: ")+std::strerror(error));
  const char* priority=std::getenv("KUAVO_IK_PRIORITY");
  sched_param param{}; param.sched_priority=priority?std::stoi(priority):40;
  if (param.sched_priority<1 || param.sched_priority>=90)
    throw std::runtime_error("IK priority must remain below WBC");
  error=pthread_setschedparam(pthread_self(),SCHED_FIFO,&param);
  if (error) throw std::runtime_error(std::string("IK thread FIFO: ")+std::strerror(error));
  pthread_setname_np(pthread_self(),"kuavo_ik_tick");
  std::cerr<<"[runtime-partition] IK thread CPUs="<<cpus<<" FIFO="<<param.sched_priority<<std::endl;
}
}
