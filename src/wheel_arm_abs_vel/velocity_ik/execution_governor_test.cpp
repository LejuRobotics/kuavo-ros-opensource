#include "execution_governor.h"
#include "weighted_qp.h"
#include <chrono>
#include <iostream>
#include <random>

using namespace velocity_ik;
using Eigen::MatrixXd;
using Eigen::VectorXd;
static int checks=0;
static void require(bool ok,const char* message) {
  ++checks; if(!ok) throw std::runtime_error(message);
}
static VectorXd solve(WeightedQp& qp,const std::vector<ExecutionGovernorRow>& rows,
    double wanted,double lower=-4.,double upper=4.) {
  MatrixXd h=MatrixXd::Identity(2,2), c(0,2);
  VectorXd g(2),lo(2),hi(2),cl(0),cu(0),x;
  g<<-wanted,-2.5; lo<<lower,-4.;hi<<upper,4.;
  augmentExecutionGovernorQp(rows,&h,&g,&lo,&hi,&c,&cl,&cu);
  int code=0;
  require(qp.solve(h,g,lo,hi,c,cl,cu,&x,&code) && code==0,"governor QP failed");
  require(x[0]>=lower-2e-8 && x[0]<=upper+2e-8,"hard bounds changed");
  require(std::abs(x[1]-2.5)<1e-7,"unrelated joint was slowed");
  return x;
}
int main() {
  try {
    ExecutionGovernorConfig p;
    require(validExecutionGovernorConfig(p),"default config invalid");
    for(int field=0;field<7;++field) {
      auto bad=p;
      double* values[]={&bad.error_deadband,&bad.error_full,&bad.prediction_time,
          &bad.penalty_ratio,&bad.smoothing_ratio,&bad.attack_time,&bad.release_time};
      *values[field]=std::numeric_limits<double>::quiet_NaN();
      require(!validExecutionGovernorConfig(bad),"NaN config accepted");
    }
    WeightedQp qp; qp.prepareWorkspaces({{2,0},{4,2},{6,4}});
    // Fast healthy motion is exactly transparent, including residual risk memory.
    for(double speed:{-4.,-3.,0.,3.,4.}) for(double error:{-.05,0.,.05}) {
      const auto row=executionGovernorRow(p,0,error,speed,0.,0.,speed,.002,1.,1.);
      require(row.smoothing==0.,"healthy motion retained smoothing latch");
      const auto base=solve(qp,{},speed),out=solve(qp,{row},speed);
      require(std::abs(base[0]-out[0])<1e-7,"healthy motion lost speed");
      require(out.tail(2).cwiseAbs().maxCoeff()<1e-7,"healthy motion penalized");
    }
    for(double sign:{-1.,1.}) {
      auto row=executionGovernorRow(p,0,sign*.4,sign*3.,0.,0.,0.,.002,1.,1.);
      const auto outward=solve(qp,{row},sign*3.);
      require(sign*outward[0]>.1 && sign*outward[0]<1.,"lag not softly regulated");
      // Inward recovery must remain fast, including a still-opposite measured velocity.
      row=executionGovernorRow(p,0,sign*.4,-sign*3.,0.,0.,sign*2.,.002,1.,1.);
      require(row.smoothing==0.,"reversal braking smoothed unnecessarily");
      require(std::abs(solve(qp,{row},-sign*3.)[0]+sign*3.)<1e-7,"recovery slowed");
      // Severe overshoot with stationary target cannot make the reference chase feedback.
      row=executionGovernorRow(p,0,sign*1.5,0.,0.,0.,-sign*8.,.002,1.,1.);
      require(row.lower<=0. && row.upper>=0.,"zero excluded");
      require(std::abs(solve(qp,{row},0.)[0])<1e-7,"feedback excited stationary reference");
    }
    // Hard braking / nonzero acceleration box always wins over a soft tube.
    const auto severe=executionGovernorRow(p,0,1.8,3.,0.,0.,-8.,.002,1.,1.);
    require(solve(qp,{severe},3.,2.9,3.1)[0]>=2.9-2e-8,"braking infeasible");
    // Release is bounded and cannot create a multi-second slow mode.
    double risk=1.;
    for(int i=0;i<100;++i) {
      auto r=executionGovernorRow(p,0,0.,4.,0.,0.,4.,.002,risk,1.);
      risk=r.risk;require(r.smoothing==0.,"healthy release still slows motion");
    }
    require(risk<.007,"release too slow");
    // Randomized feasibility and both-shoulder augmentation under narrow hard boxes.
    std::mt19937 gen(20260922);std::uniform_real_distribution<double> uni(-1.,1.);
    const auto start=std::chrono::steady_clock::now();
    for(int k=0;k<3000;++k) {
      const double v=3.8*uni(gen),q=2.*uni(gen),mv=8.*uni(gen);
      auto r=executionGovernorRow(p,0,q,v,50.*uni(gen),0.,mv,.002,.5,1.);
      require(std::isfinite(r.lower) && r.lower<=0. && r.upper>=0.,"bad soft interval");
      auto other=executionGovernorRow(p,1,0.,2.5,0.,0.,2.5,.002,0.,1.);
      const double lo=std::max(-4.,v-.1),hi=std::min(4.,v+.1);
      r.smooth_target=std::clamp(r.smooth_target,lo,hi);
      solve(qp,{r,other},4.*uni(gen),lo,hi);
    }
    std::cout<<"PASS execution governor "<<checks<<" checks; 3000 paired QPs in "
      <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()
      <<" ms; healthy +/-4 rad/s unchanged, recovery free, hard braking feasible\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
