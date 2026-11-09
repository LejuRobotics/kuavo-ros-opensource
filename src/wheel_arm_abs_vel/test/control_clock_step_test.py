#!/usr/bin/env python3
"""Execute production timing fragments with stepped realtime and steady ticks.

The test process interposes clock_gettime; the machine clock is never changed.
Control, observation and cache fragments are extracted from the source under test.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
root = parser.parse_args().source_root
nodelet = (root / 'src/absolute/controllerNodelet.cpp').read_text()
controller = (root / 'src/absolute/humanoidController_wheel_absolute.cpp').read_text()
manager = (root / 'include/wheel_arm_abs_vel/humanoid_controllers/absolute/ControlDataManager.h').read_text()
clock = re.search(r'using Clock = [^;]+;', nodelet).group()
update = controller[controller.index('  void humanoidControllerWheelWbc::update('):]
begin = update.find('    static const double firstTime')
if begin < 0:
    begin = update.index('    // The nodelet supplies')
timing = update[begin:update.index('    const auto updateWallStart')]
observation = controller[controller.index('    current_time_ = sensorData.timeStamp_;'):]
observation = observation[:observation.index('    // 使用angles包')]
cache = manager[manager.index('template<typename T>\nstruct TimestampedData'):manager.index('\nclass ControlDataManager')]
sample_start = controller.index('const auto sample = wheel_absolute_control::sampleStartupArmAction(')
sample_end = controller.index(');', sample_start)
startup = controller[sample_start:sample_end]
startup_anchor = 'mono' if 'startup_action_last_time_' not in controller else 'wall'
elapsed = startup.split('preTargetActions_-startActions_,', 1)[1].rsplit(', robotPreActionDesiredTime_', 1)[0]

code = r'''
#include <cassert>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <vector>
static double wall = 1000., mono = 100.;
extern "C" int clock_gettime(clockid_t id, timespec* out) noexcept {
  const double value = id == CLOCK_REALTIME ? wall : mono;
  out->tv_sec = static_cast<time_t>(value);
  out->tv_nsec = static_cast<long>((value - out->tv_sec) * 1e9);
  return 0;
}
namespace ros {
struct Duration { double value; explicit Duration(double x): value(x) {} double toSec() const {return value;} };
struct Time {
  double value{0}; Time()=default; explicit Time(double x):value(x){}
  static Time now(){return Time(wall);} double toSec()const{return value;}
  Duration operator-(const Time& other)const{return Duration(value-other.value);}
};
}
struct Logger {void publishValue(const char*,double){}};
''' + clock + '\n' + cache + r'''
struct ControlProbe {
  double dt_{.002}, control_elapsed_time_{0}, observed_dt{0}, observed_time{0};
  double pending_control_period_{0};
  std::vector<double> accepted_periods;
  Logger* ros_logger_{nullptr};
  void update(const ros::Time& time, const ros::Duration& dfd) {
''' + timing + r'''
    observed_dt=dt; observed_time=curTime;
    accepted_periods.push_back(dt);
  }
};
struct Sensor {ros::Time timeStamp_; std::chrono::steady_clock::time_point receiveStamp_;};
struct ObservationProbe {
  bool is_real_{true}, observation_initialized_{false};
  ros::Time current_time_,last_time_;
  std::chrono::steady_clock::time_point last_sensor_receive_{};
  double last_yaw_{0},accumulated_yaw_{0};
  double step(const Sensor& sensorData) {
    double odomData[3]{};
''' + observation + r'''
    return diff_time;
  }
};
double startupElapsed(double startup_action_start_time_, const ros::Time& time) {
  return ''' + elapsed + r''';
}
int main() {
  ControlProbe control;
  ObservationProbe observation;
  TimestampedData<int> cache;
  cache.update(1);
  auto last=Clock::now();
  observation.step({ros::Time(wall),std::chrono::steady_clock::now()});
  const double startup_start=''' + startup_anchor + r''';
  for (int i=0;i<1000;++i) {
    mono+=.002; wall+=.002;
    if(i==100) wall+=1794212331.257496;
    if(i==300) wall-=3600.;
    if(i==500) wall=1.;
    auto now=Clock::now();
    const double dt=std::chrono::duration<double>(now-last).count();last=now;
    control.update(ros::Time(wall),ros::Duration(dt));
    assert(std::abs(control.observed_dt-.002)<1e-7);
    assert(std::abs(observation.step({ros::Time(wall),std::chrono::steady_clock::now()})-.002)<1e-7);
    assert(std::abs(startupElapsed(startup_start,ros::Time(wall))-(i+1)*.002)<1e-7);
    assert(cache.isValid(3.));
    assert(cache.timestamp.toSec()==1000.); // External stamp remains a ROS stamp.
  }
  assert(std::abs(control.observed_time-2.)<1e-6);
  assert(!cache.isValid(1.));
  // A real scheduling delay remains visible; no clamping or timeout stop added.
  mono+=.2;wall+=.2;
  control.update(ros::Time(wall),ros::Duration(.2));
  assert(control.observed_dt==.2);
  // Early calls do not consume elapsed time. The next accepted dynamics
  // period must span the entire interval since the last accepted update.
  ControlProbe irregular;
  for(double period : {.002, .0008, .0012, .002}) {
    mono+=period; wall+=period;
    irregular.update(ros::Time(wall),ros::Duration(period));
  }
  assert(irregular.accepted_periods.size()==3);
  for(double period:irregular.accepted_periods) assert(std::abs(period-.002)<1e-12);
  assert(std::abs(irregular.observed_time-.006)<1e-12);
  const auto accepted=irregular.accepted_periods.size();
  irregular.update(ros::Time(wall),ros::Duration(0));
  for(int i=0;i<3;++i) irregular.update(ros::Time(wall),ros::Duration(.0003));
  assert(irregular.accepted_periods.size()==accepted);
  irregular.update(ros::Time(wall),ros::Duration(.0003));
  assert(std::abs(irregular.observed_dt-.0012)<1e-12);
  irregular.update(ros::Time(wall),ros::Duration(.0008));
  irregular.update(ros::Time(wall),ros::Duration(.2));
  assert(std::abs(irregular.observed_dt-.2008)<1e-12);
  assert(std::abs(irregular.observed_time-.208)<1e-12);
  // Recorded sensor time remains meaningful for playback/simulation.
  ObservationProbe replay;replay.is_real_=false;
  replay.step({ros::Time(20),std::chrono::steady_clock::now()});
  assert(std::abs(replay.step({ros::Time(20.01),std::chrono::steady_clock::now()})-.01)<1e-9);
  std::puts("PASS: 1000 cycles; realtime steps; short/duplicate calls conserve elapsed and dynamics periods; real delay and playback preserved");
}
'''
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    (path / 'clock.cpp').write_text(code)
    subprocess.run(['c++', '-std=c++17', '-O0', str(path / 'clock.cpp'), '-o', str(path / 'clock')], check=True)
    subprocess.run([str(path / 'clock')], check=True)
