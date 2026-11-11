#include <hardware_node/LatestClawCommand.h>
#include <hardware_node/LejuClawModeCache.h>
#include <kuavo_common/common/BackgroundThread.h>
#include <memory>
#include <array>
#include <thread>
#include <vector>
#include <atomic>
#include <algorithm>
#include <cassert>
#include <iostream>
struct Command {int seq;std::array<double,2> target;};
int main(){
 using Ptr=std::shared_ptr<const Command>;LatestClawCommand<Ptr> box;
 const auto received=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i)box.put(std::make_shared<Command>(Command{i,{double(i),-double(i)}}),received,true);
 auto item=box.take();assert(item.msg->seq==99 && item.received==received && item.high_freq);assert(!box.take().msg);
 box.put(item.msg,received,true);box.accepting(false);assert(!box.take().msg);assert(!box.put(item.msg,received,true));box.accepting(true);assert(!box.take().msg);
 std::atomic<bool> running{true},blocked{false};std::atomic<int> consumed{0};
 std::thread worker([&]{kuavo_thread::background("claw_test");while(running){auto m=box.take();if(m.msg){++consumed;blocked=true;std::this_thread::sleep_for(std::chrono::milliseconds(100));blocked=false;}else std::this_thread::sleep_for(std::chrono::milliseconds(2));}});
 box.put(item.msg,received,true);while(!blocked)std::this_thread::yield();
 HighlyDynamic::LejuClawModeCache cache;std::atomic<int64_t> joystick{1};std::vector<double> us;us.reserve(100000);
 // Same ingress operations as the real callback, with driver stalled 100 ms.
 for(int i=0;i<100000;++i){auto start=std::chrono::steady_clock::now();const auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(start.time_since_epoch()).count();const bool high=cache.highFrequency(joystick.load(),stamp);box.put(item.msg,start,high);auto end=std::chrono::steady_clock::now();us.push_back(std::chrono::duration<double,std::micro>(end-start).count());}
 running=false;worker.join();std::sort(us.begin(),us.end());
 assert(us[us.size()/2]<1.0); // Distribution, not an impossible scheduler max bound.
 std::cout<<"100000 callback-equivalent writes while I/O sleeps 100ms: p50_us="<<us[50000]<<" p99_us="<<us[99000]<<" max_us="<<us.back()<<" consumed="<<consumed<<"\n";
}
