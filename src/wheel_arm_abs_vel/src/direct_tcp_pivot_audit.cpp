#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <iostream>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
struct Frame {double t;std::array<Sample,2> hand;};
std::vector<Frame> read(const std::string& path){
  std::ifstream in(path);if(!in)throw std::runtime_error("Missing calibration input");Json j;in>>j;std::vector<Frame> out;
  for(const auto& row:j.at("samples")){Frame f;f.t=row.at("t");for(int a=0;a<2;++a){auto x=row.at("controller").at(a);f.hand[a]=sample({x.at(0),x.at(1),x.at(2),x.at(3),x.at(4),x.at(5),x.at(6)},Vec::Zero(),Vec::Zero());}out.push_back(f);}return out;
}
struct Normal {Rot H=Rot::Zero();Vec g=Vec::Zero();double p2=0.;int n=0;};
Normal equations(const std::vector<Frame>& rows,double begin,double end,double width,int degree,int side){
  Normal out;
  for(double start=begin;start+width<=end+1.e-9;start+=width){
    std::vector<const Frame*> frames;for(const auto& r:rows)if(r.t>=start && r.t<start+width)frames.push_back(&r);
    if(frames.size()<std::size_t(degree+4))throw std::runtime_error("Insufficient independent calibration samples");
    Eigen::MatrixXd B(frames.size(),degree+1);
    for(std::size_t i=0;i<frames.size();++i){double t=(frames[i]->t-start)/width-.5;for(int k=0;k<=degree;++k)B(i,k)=std::pow(t,k);}
    const auto qr=B.colPivHouseholderQr();
    for(int arm=0;arm<2;++arm){
      if(side>=0 && side!=arm)continue;
      Eigen::MatrixXd p(frames.size(),3),R(frames.size(),9);
      Rot mirror=Rot::Identity();if(side<0 && arm==1)mirror(0,0)=-1.;
      for(std::size_t i=0;i<frames.size();++i){p.row(i)=frames[i]->hand[arm].pose.translation().transpose();Rot r=frames[i]->hand[arm].pose.linear()*mirror;for(int a=0;a<3;++a)for(int b=0;b<3;++b)R(i,a*3+b)=r(a,b);}
      p-=B*qr.solve(p).eval();R-=B*qr.solve(R).eval();
      for(std::size_t i=0;i<frames.size();++i){Rot a;for(int k=0;k<3;++k)for(int l=0;l<3;++l)a(k,l)=R(i,k*3+l);const Vec v=p.row(i).transpose();out.H+=a.transpose()*a;out.g-=a.transpose()*v;out.p2+=v.squaredNorm();++out.n;}
    }
  }return out;
}
Json vec(const Vec& v){return {v.x(),v.y(),v.z()};}
Sample at(const std::vector<Frame>& frames,double t,int side){
  auto p=std::lower_bound(frames.begin(),frames.end(),t,[](const Frame& f,double x){return f.t<x;});
  if(p==frames.begin() || p==frames.end())throw std::runtime_error("Audit window outside input");
  auto a=std::prev(p);double u=(t-a->t)/(p->t-a->t);Sample s;s.pose=Pose::Identity();s.pose.translation()=(1-u)*a->hand[side].pose.translation()+u*p->hand[side].pose.translation();
  s.pose.linear()=Eigen::Quaterniond(a->hand[side].pose.linear()).slerp(u,Eigen::Quaterniond(p->hand[side].pose.linear())).toRotationMatrix();s.linear=s.angular=Vec::Zero();return s;
}
}
int main(int argc,char**argv){
  if(argc!=4)throw std::runtime_error("pivot_audit CALIBRATION.json EPISODES.json OUTPUT.json");
  auto train=read(argv[1]),episodes=read(argv[2]);Json output;
  for(int degree:{1,2})for(double width:{.5,1.,2.,5.})for(int side:{-1,0,1}){
    const auto n=equations(train,50.,85.,width,degree,side),val=equations(train,85.,120.,width,degree,side);
    Eigen::SelfAdjointEigenSolver<Rot> eigen(n.H);const Vec r=n.H.ldlt().solve(n.g);
    const double rms=std::sqrt((val.p2-2*r.dot(val.g)+r.dot(val.H*r))/val.n);
    Json item{{"degree",degree},{"window_s",width},{"side",side},{"offset_m",vec(r)},{"eigenvalues",vec(eigen.eigenvalues())},{"validation_residual_rms_m",rms},{"norm_m",r.norm()}};
    for(int arm=0;arm<2;++arm){Vec offset=r;if(side<0 && arm==1)offset.x()*=-1.;const auto a=at(episodes,130.808,arm),b=at(episodes,131.308,arm);
      item["issue2_virtual_endpoint_dz_mm"][arm]=1000.*((b.pose.translation()+b.pose.linear()*offset)-(a.pose.translation()+a.pose.linear()*offset)).z();}
    output["fits"].push_back(item);
  }
  for(int arm=0;arm<2;++arm){const auto a=at(episodes,130.808,arm),b=at(episodes,131.308,arm);output["issue2_controller_delta_m"][arm]=vec(b.pose.translation()-a.pose.translation());output["issue2_delta_R_z"][arm]=vec((b.pose.linear()-a.pose.linear()).row(2).transpose());}
  for(const auto& win:std::vector<std::pair<double,double>>{{130.808,131.308},{172.1078,172.4975},{172.,173.}}){
    Json item{{"start",win.first},{"end",win.second}};
    for(int arm=0;arm<2;++arm){
      const auto a=at(episodes,win.first,arm),b=at(episodes,win.second,arm);
      Rot H=Rot::Zero();Vec rhs=Vec::Zero();double energy=0.;int n=0;
      for(const auto& frame:episodes)if(frame.t>=win.first && frame.t<=win.second){const Rot M=frame.hand[arm].pose.linear()-a.pose.linear();const Vec dp=frame.hand[arm].pose.translation()-a.pose.translation();H+=M.transpose()*M;rhs-=M.transpose()*dp;energy+=dp.squaredNorm();++n;}
      const Vec r=H.ldlt().solve(rhs);Eigen::SelfAdjointEigenSolver<Rot> e(H);
      item["arms"][arm]={{"controller_delta_m",vec(b.pose.translation()-a.pose.translation())},{"Rstart",{{a.pose.linear()(0,0),a.pose.linear()(0,1),a.pose.linear()(0,2)},{a.pose.linear()(1,0),a.pose.linear()(1,1),a.pose.linear()(1,2)},{a.pose.linear()(2,0),a.pose.linear()(2,1),a.pose.linear()(2,2)}}},{"deltaR",{{(b.pose.linear()-a.pose.linear())(0,0),(b.pose.linear()-a.pose.linear())(0,1),(b.pose.linear()-a.pose.linear())(0,2)},{(b.pose.linear()-a.pose.linear())(1,0),(b.pose.linear()-a.pose.linear())(1,1),(b.pose.linear()-a.pose.linear())(1,2)},{(b.pose.linear()-a.pose.linear())(2,0),(b.pose.linear()-a.pose.linear())(2,1),(b.pose.linear()-a.pose.linear())(2,2)}}},{"diagnostic_only_stationary_pivot_m",vec(r)},{"pivot_norm_m",r.norm()},{"pivot_residual_m",std::sqrt((energy-2*r.dot(rhs)+r.dot(H*r))/n)},{"eigenvalues",vec(e.eigenvalues())}};
    }output["diagnostic_only_windows_not_calibration"].push_back(item);
  }
  std::ofstream out(argv[3]);out<<output.dump(2);std::cout<<output.dump(2)<<std::endl;
}
