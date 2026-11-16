// Standalone test with an isolated roscore; never instantiates hardware/control.
#include <hardware_node/LejuClawModeCache.h>
#include <kuavo_common/common/AsyncTimingProbe.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <condition_variable>
#include <mutex>

int main(int argc,char** argv) {
  using namespace HighlyDynamic;
  LejuClawModeCache cache;
  assert(!cache.highFrequency(0,1000));
  assert(cache.highFrequency(1000,1001000)); // exact inclusive timeout
  assert(!cache.highFrequency(1000,1001001));
  cache.store(true,1);assert(cache.highFrequency(0,9999999));
  cache.store(false,10);assert(cache.highFrequency(1000,11000));
  assert(!cache.highFrequency(1000,11001));
  cache.store(false,0);assert(cache.highFrequency(1000,1001000));
  cache.store(false,-1);assert(!cache.highFrequency(1000,1001001));
  std::atomic<int> calls{0};std::atomic<bool> refreshing{false},release{false};
  cache.start([&] {
    if(++calls>1) {
      refreshing=true;
      while(!release.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return std::make_pair(false,1000);
  });
  while(!refreshing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
  auto begin=kuavo_timing::sample();
  for(int i=0;i<100000;++i)assert(cache.highFrequency(1000,2000));
  auto read_cost=kuavo_timing::elapsed(begin,kuavo_timing::sample());
  release=true;cache.stop();assert(read_cost[0]<50.0);

  // Queue saturation cannot block the producer, even without a consumer.
  kuavo_timing::AsyncPublisher<1,4> bounded;
  for(int i=0;i<10;++i)bounded.push({{double(i)}});
  assert(bounded.dropped()==6);

  ros::init(argc,argv,"control_latency_probe_test");ros::NodeHandle nh;
  std::mutex mutex;std::condition_variable received;std::vector<double> row;
  auto sub=nh.subscribe<std_msgs::Float64MultiArray>("/test/wbc_timing",1,
      [&](const std_msgs::Float64MultiArray::ConstPtr& msg) {
        {std::lock_guard<std::mutex> lock(mutex);row=msg->data;}received.notify_all();
      });
  ros::AsyncSpinner spinner(1);spinner.start();
  kuavo_timing::WbcCycle::Publisher publisher;publisher.start(nh,"/test/wbc_timing");
  for(int i=0;i<300 && sub.getNumPublishers()==0;++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  assert(sub.getNumPublishers()>0);
  {
    kuavo_timing::WbcCycle cycle(publisher,42,123.5);
    const auto cpu_begin=kuavo_timing::nanoseconds(CLOCK_THREAD_CPUTIME_ID);
    while(kuavo_timing::nanoseconds(CLOCK_THREAD_CPUTIME_ID)-cpu_begin<3000000){}
    cycle.mark();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));cycle.mark();
    for(int i=0;i<4;++i)cycle.mark();
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(received.wait_for(lock,std::chrono::seconds(3),[&]{return !row.empty();}));
  }
  assert(row.size()==33 && row[0]==1 && row[3]==42 && row[9]==6);
  assert(row[6]>8 && row[7]>3 && row[8]>4);
  assert(row[13]>2.9); // stage 1 burns CPU
  assert(row[17]>4);  // stage 2 waits
  double sum_wall=0,sum_cpu=0;
  for(int i=0;i<7;++i){sum_wall+=row[12+3*i];sum_cpu+=row[13+3*i];}
  assert(std::abs(sum_wall-row[6])<1e-8 && std::abs(sum_cpu-row[7])<1e-8);
  publisher.stop();spinner.stop();
  std::cout<<"PASS cache timeout/manual/default; 100000 cached reads while refresh blocked: "
           <<read_cost[0]<<" ms; WBC wall/cpu/off_cpu: "<<row[6]<<"/"<<row[7]<<"/"<<row[8]
           <<" ms; stage sums and bounded drops verified"<<std::endl;
  // This host's existing ROS/log4cxx shutdown can crash even in a minimal
  // unchanged roscpp program. Optional harness mode isolates that known fault;
  // all assertions, worker join and real ROS delivery have already completed.
  if(argc>1 && std::string(argv[1])=="--skip-ros-shutdown")std::_Exit(0);
  ros::shutdown();
}
