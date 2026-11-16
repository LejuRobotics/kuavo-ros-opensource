// Private ROS graph only. Mirrors the IK target subscription/AsyncSpinner(2),
// with no solver, controller or hardware. Records CLOCK_MONOTONIC at callback entry.
#include <ros/ros.h>
#include <wheel_arm_abs_vel/TeleopTcpTarget.h>
#include <chrono>
#include <fstream>
#include <mutex>
#include <vector>
#include <cstdlib>
#include <cassert>
#include <unistd.h>
#include <thread>
struct Row {uint64_t stamp,arrival;unsigned state;bool valid;};
int main(int argc,char** argv) {
 const char* master=std::getenv("ROS_MASTER_URI");
 assert(master && std::string(master)=="http://127.0.0.1:11439");assert(argc==3);
 const std::string output=argv[1];const double seconds=std::stod(argv[2]);
 ros::init(argc,argv,"mapper_bench_cpp_sink");ros::NodeHandle nh;
 std::mutex mutex;std::vector<Row> rows;rows.reserve(6000);
 auto callback=[&](const wheel_arm_abs_vel::TeleopTcpTarget::ConstPtr& msg){
  timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);
  Row row{(msg->header.stamp.toNSec()+500000)/1000000,uint64_t(t.tv_sec)*1000000000+t.tv_nsec,unsigned(msg->control_state),bool(msg->control_valid)};
  std::lock_guard<std::mutex> lock(mutex);rows.push_back(row);
 };
 auto sub=nh.subscribe<wheel_arm_abs_vel::TeleopTcpTarget>("/mm/teleop_tcp_target",1,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
 ros::AsyncSpinner spinner(2);spinner.start();std::ofstream(output+".cpp.ready")<<getpid();
 const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
 while(ros::ok() && std::chrono::steady_clock::now()<until && !std::ifstream(output+".stop").good())std::this_thread::sleep_for(std::chrono::milliseconds(10));
 spinner.stop();std::ofstream file(output);file<<'[';
 for(size_t i=0;i<rows.size();++i){auto&r=rows[i];if(i)file<<',';file<<'['<<r.stamp<<','<<r.arrival<<','<<r.state<<','<<(r.valid?"true":"false")<<']';}
 file<<']';file.close();
 // Same pre-existing ROS/log4cxx shutdown issue as topic_logger_latency_test.
 // Callback workers are stopped and the output is closed before exiting.
 std::_Exit(0);
}
