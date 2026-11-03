#include "continuous_reference.h"
#include "frame_prediction.h"
#include <cassert>
#include <iostream>
#include <limits>
using namespace velocity_ik;
constexpr double dt=.002,epoch=1e12;
int main() {
  FramePredictionSettings cfg;assert(cfg.valid());
  auto bad=cfg;bad.decay=0.;assert(!bad.valid());bad=cfg;bad.min_hold=.010;assert(!bad.valid());
  FramePrediction clock;
  const double received=epoch+.020e9;
  clock.update(epoch,received,1.,received,cfg);
  assert(clock.valid && std::abs(clock.age-.020)<1e-12 && clock.rate==1.);
  // Repeated packets with a later receipt do not renew the admitted observation.
  clock.update(epoch,received+.080e9,1.,received+.080e9,cfg);
  assert(std::abs(clock.silence-.080)<1e-12 && clock.rate<.3);
  const double old_receive=clock.received_ns;
  clock.update(epoch-1e6,received+.090e9,.999,received+.090e9,cfg);
  assert(clock.received_ns==old_receive);
  clock.update(epoch,received,1.,received+10e9,cfg);
  assert(clock.age<=cfg.max_alignment_age+cfg.max_hold+cfg.decay && clock.rate<1e-100);
  clock.reset();assert(!clock.valid && clock.rate==0.);
  // Snapshot can be received just after tick_start; do not emit a spurious zero.
  clock.update(epoch,received,1.,received-1000.,cfg);assert(clock.valid && clock.rate==1.);

  // Noise-free 72Hz input with 16/30ms existing latency, one skipped frame,
  // no new-epoch dip. Real production predictor and shaper, not a Python copy.
  for(double latency:{.016,.030}) {
    clock.reset();AdaptiveVectorReference ref;AdaptiveRotationReference rot;
    AdaptiveReferenceSettings settings;double sample=0.,receipt=0.;
    double low=1e9,high=-1e9,old_lo=1e9,old_hi=-1e9;
    ContinuousVector legacy;
    for(int i=0;i<3000;++i) {
      const double t=i*dt;
      if(i%7==0 && i!=2107) {receipt=t;sample=t-latency;}
      clock.update(epoch+sample*1e9,epoch+receipt*1e9,10.+sample,epoch+t*1e9,cfg);
      const double x=.2*(sample+clock.age);
      ref.stepAdaptive(Eigen::Vector3d(x,0,0),Eigen::Vector3d(.2*clock.rate,0,0),dt,12.,false,settings);
      rot.stepAdaptive(Eigen::Quaterniond(Eigen::AngleAxisd(.4*(sample+clock.age),Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d(0,0,.4*clock.rate),dt,12.,false,settings);
      const double raw_age=t-sample,rate=raw_age<=.015?1.:std::exp(-(raw_age-.015)/.005);
      const double age=raw_age<=.015?raw_age:.015+.005*(1.-rate);
      legacy.step(Eigen::Vector3d(.2*(sample+age),0,0),Eigen::Vector3d(.02*rate,0,0),dt,12.);
      if(i>1000) {
        assert(clock.rate==1.);low=std::min(low,ref.v.x());high=std::max(high,ref.v.x());
        old_lo=std::min(old_lo,legacy.v.x());old_hi=std::max(old_hi,legacy.v.x());
        assert(std::abs(rot.v.z()-.4)<.001);
      }
    }
    std::cout<<"constant speed latency_ms="<<latency*1000<<" old_pp="<<old_hi-old_lo<<" new_pp="<<high-low<<'\n';
    assert(high-low<1e-8 && std::abs(high-.2)<.001 && old_hi-old_lo>.01);
  }

  // Parameter-induced acceleration changes are bounded in vector norm,
  // including simultaneous frequency/guide changes and rotated frames.
  ReferenceParameterTransition tr;
  const Eigen::Vector3d error(.02,-.01,.03),input(.3,-.2,.1),velocity(.1,.2,-.1);
  tr.update(1.,0.,error,input,velocity,dt,300.,true);
  for(int i=0;i<1000;++i) {
    tr.update((i % 2) ?12.:1.,(i % 2) ?.1:0.,error,input,velocity,dt,300.,true);
    assert(tr.acceleration_change<=300.*dt+1e-9);
    assert(tr.frequency>=1. && tr.frequency<=12. && tr.guide>=0. && tr.guide<=.1);
  }
  // Capture a small stationary target after quiet, compare release peak.
  AdaptiveReferenceSettings smooth,old;smooth.position_parameter_jerk=300.;old.bumpless_enabled=false;
  AdaptiveVectorReference a,b;double pa=0.,pb=0.;
  for(int i=0;i<2000;++i) {
    a.stepAdaptive(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),dt,12.,true,smooth);
    b.stepAdaptive(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),dt,12.,true,old);
  }
  for(int i=0;i<1000;++i) {
    const Eigen::Vector3d target(.012,0,0);
    a.stepAdaptive(target,Eigen::Vector3d::Zero(),dt,12.,true,smooth);
    b.stepAdaptive(target,Eigen::Vector3d::Zero(),dt,12.,true,old);
    pa=std::max(pa,a.v.norm());pb=std::max(pb,b.v.norm());
    assert(a.p.x()>=-1e-10 && a.p.x()<=.01201);
  }
  std::cout<<"quiet release peak old="<<pb<<" new="<<pa<<'\n';
  assert(pa<pb*.95 && std::abs(a.p.x()-.012)<1e-5);

  // Rapid large intent must retain response rather than turning into a slow
  // acquisition. Start after quiet; input 0.6m/s for 0.2s then reverse.
  a=AdaptiveVectorReference{};b=AdaptiveVectorReference{};
  double x=0.,response=-1.,reverse=-1.,old_response=-1.,old_reverse=-1.;
  for(int i=0;i<2500;++i) {
    const double speed=i<1500?0.:(i<1600?.6:-.6);x+=speed*dt;
    a.stepAdaptive(Eigen::Vector3d(x,0,0),Eigen::Vector3d(speed,0,0),dt,12.,true,smooth);
    b.stepAdaptive(Eigen::Vector3d(x,0,0),Eigen::Vector3d(speed,0,0),dt,12.,true,old);
    if(i>=1500 && response<0 && a.v.x()>.54)response=(i-1500)*dt;
    if(i>=1600 && reverse<0 && a.v.x()<-.54)reverse=(i-1600)*dt;
    if(i>=1500 && old_response<0 && b.v.x()>.54)old_response=(i-1500)*dt;
    if(i>=1600 && old_reverse<0 && b.v.x()<-.54)old_reverse=(i-1600)*dt;
  }
  std::cout<<"90% response_ms="<<response*1000<<" reversal_ms="<<reverse*1000
      <<" old_response_ms="<<old_response*1000<<" old_reversal_ms="<<old_reverse*1000<<'\n';
  assert(response>=0. && response<.090 && reverse>=0. && reverse<.080);
  assert(response-old_response<=.0200001 && reverse-old_reverse<=.0100001);
  AdaptiveRotationReference ra,rb;double peak_a=0.,peak_b=0.;
  for(int i=0;i<3000;++i) {
    const Eigen::Quaterniond target(Eigen::AngleAxisd(i<2000?0.:.020,Eigen::Vector3d::UnitZ()));
    ra.stepAdaptive(target,Eigen::Vector3d::Zero(),dt,12.,true,smooth);
    rb.stepAdaptive(target,Eigen::Vector3d::Zero(),dt,12.,true,old);
    if(i>=2000) {peak_a=std::max(peak_a,ra.v.norm());peak_b=std::max(peak_b,rb.v.norm());}
    assert(ra.transition.acceleration_change<=smooth.orientation_parameter_jerk*dt+1e-8);
  }
  std::cout<<"quiet angular release peak old="<<peak_b<<" new="<<peak_a<<'\n';
  assert(peak_a<peak_b && std::abs(ra.q.norm()-1.)<1e-12);
  ra=AdaptiveRotationReference{};rb=AdaptiveRotationReference{};
  double angle=0.;response=reverse=old_response=old_reverse=-1.;
  for(int i=0;i<2400;++i) {
    const double speed=i<1500?0.:(i<1600?3.:-3.);angle+=speed*dt;
    const Eigen::Quaterniond target(Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()));
    ra.stepAdaptive(target,Eigen::Vector3d(0,0,speed),dt,12.,true,smooth);
    rb.stepAdaptive(target,Eigen::Vector3d(0,0,speed),dt,12.,true,old);
    if(i>=1500 && response<0 && ra.v.z()>2.7)response=(i-1500)*dt;
    if(i>=1500 && old_response<0 && rb.v.z()>2.7)old_response=(i-1500)*dt;
    if(i>=1600 && reverse<0 && ra.v.z()<-2.7)reverse=(i-1600)*dt;
    if(i>=1600 && old_reverse<0 && rb.v.z()<-2.7)old_reverse=(i-1600)*dt;
  }
  std::cout<<"angular 90% response_ms="<<response*1000<<" old_ms="<<old_response*1000
      <<" reverse_ms="<<reverse*1000<<" old_reverse_ms="<<old_reverse*1000<<'\n';
  assert(response>=0. && reverse>=0. && response-old_response<=.0200001 && reverse-old_reverse<=.0100001);
  std::cout<<"reference dynamics contracts passed\n";
}
