#include "async_diagnostics.h"
#include <cassert>
#include <iostream>
#include <stdexcept>
struct Sample { uint64_t sequence; double q[21],dq[21],ddq[21]; };
int main() {
  velocity_ik::AsyncPodDiagnostics<Sample,2> worker;
  std::atomic<bool> entered{false},release{false};std::atomic<int> received{0};
  worker.start([&](const Sample& sample){
    entered=true;while(!release.load())std::this_thread::yield();
    for(int i=0;i<21;++i){assert(sample.q[i]==double(sample.sequence));assert(sample.dq[i]==2.*sample.sequence);assert(sample.ddq[i]==3.*sample.sequence);}
    ++received;
  });
  auto make=[](uint64_t sequence){Sample s{};s.sequence=sequence;for(int i=0;i<21;++i){s.q[i]=sequence;s.dq[i]=2.*sequence;s.ddq[i]=3.*sequence;}return s;};
  assert(worker.submit(make(1)));
  while(!entered.load())std::this_thread::yield();
  auto s=make(2);assert(worker.submit(s));s=make(3);assert(worker.submit(s));
  s=make(999); // mutating the producer object cannot alter queued transactions
  assert(!worker.submit(s));assert(worker.dropped()==1);
  release=true;worker.stop();assert(received==3);assert(!worker.submit(s));
  for(uint64_t tick=0;tick<10000;++tick){
    int due=0;for(unsigned phase=0;phase<5;++phase)due+=velocity_ik::diagnosticPhaseDue(tick,5,phase);
    assert(due==1);
  }
  std::cout<<"PASS blocked consumer: immutable q/dq/ddq, bounded full-queue drop, drain/stop and phase uniqueness\n";
}
