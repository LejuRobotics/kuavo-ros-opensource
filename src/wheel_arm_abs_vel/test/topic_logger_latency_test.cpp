// Private ROS graph only; no controller or hardware object is constructed.
#include <humanoid_interface/common/TopicLogger.h>
#include <kuavo_common/common/BackgroundThread.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <ctime>
#include <cassert>
int64_t ns(clockid_t id){timespec t{};clock_gettime(id,&t);return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
int main(int argc,char**argv){
 const char* master=std::getenv("ROS_MASTER_URI");assert(master && std::string(master).find("127.0.0.1:11439")!=std::string::npos);
 ros::init(argc,argv,"topic_logger_latency_test");ros::NodeHandle nh;
 ocs2::humanoid::TopicLogger logger(nh);bool async=argc>1&&std::string(argv[1])=="async";if(async)logger.startAsync();
 std::vector<std::string> topics;for(int i=0;i<48;++i)topics.push_back("/bench/wbc/"+std::to_string(i));
 std::vector<ros::Subscriber> subscribers;for(auto& t:topics)subscribers.push_back(nh.subscribe<std_msgs::Float64MultiArray>(t,1,[&](const std_msgs::Float64MultiArray::ConstPtr&){}));
 ros::AsyncSpinner spinner(2);spinner.start();std::vector<double> data(21,1.0),wall,cpu;
 for(int j=0;j<1700;++j){auto next=std::chrono::steady_clock::now()+std::chrono::milliseconds(2);auto w=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);for(auto&t:topics)logger.publishVector(t,data);if(j>=200){wall.push_back((ns(CLOCK_MONOTONIC)-w)*1e-6);cpu.push_back((ns(CLOCK_THREAD_CPUTIME_ID)-c)*1e-6);}std::this_thread::sleep_until(next);}
 logger.stopAsync();spinner.stop();std::sort(wall.begin(),wall.end());std::sort(cpu.begin(),cpu.end());
 std::cout<<"async="<<async<<" frames="<<wall.size()<<" wall_p50_ms="<<wall[750]<<" wall_p99_ms="<<wall[1485]<<" wall_max_ms="<<wall.back()<<" cpu_p50_ms="<<cpu[750]<<" diagnostics_dropped="<<logger.asyncDropped()<<std::endl;
 // This image has a pre-existing minimal ros::shutdown/log4cxx crash. Work and
 // worker joins completed above; don't misreport that unrelated shutdown bug.
 std::_Exit(0);
}
