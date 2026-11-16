#include "runtime_timing_probe.h"
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <iostream>
int main() {
 using namespace velocity_ik::runtime_probe;
 auto c=std::make_unique<Collector>();WindowStats totals;
 std::mutex mutex;std::condition_variable cv;bool entered=false,release=false;
 c->start([&](const Record& r){
   std::unique_lock<std::mutex> lock(mutex);entered=true;cv.notify_all();
   cv.wait(lock,[&]{return release;});totals.merge(r.window);
 });
 Record r;r.size=2;r.points[0]={0,0,0,0,0,0,0};r.points[1]={99,1,1,0,0,0,0};c->submit(r);
 {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return entered;});}
 for(int k=0;k<1000;++k){r.points[1].wall=k==999?999999:10;c->submit(r);}
 {std::lock_guard<std::mutex> lock(mutex);release=true;}cv.notify_all();
 c->stop();
 assert(totals.cycles==1001);assert(totals.wall_max==999999);
 c->stop();assert(totals.cycles==1001);
 std::cout<<"PASS all-cycle maxima and counts survive full detail queue, tail flush, repeated stop\n";
}
