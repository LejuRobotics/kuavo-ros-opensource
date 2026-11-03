// Offline parameter sensitivity only. No evaluation-window branch enters Mapper.
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <fstream>
#include <iostream>
#include <algorithm>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& a){return sample({a.at(0),a.at(1),a.at(2),a.at(3),a.at(4),a.at(5),a.at(6)},{a.at(7),a.at(8),a.at(9)},{a.at(10),a.at(11),a.at(12)});}
struct Row {double t,z,j7;Eigen::Vector4d q;};
Eigen::Vector4d joints(const Output& o,const WristGeometry& g,const Rot& parent,const Vec& shoulder){
  const Vec u=o.elbow.pose.translation()-shoulder,v=o.elbow.pose.linear()*g.forearm_vector;
  const double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-std::atan2(g.upper_vector.x(),-g.upper_vector.z())-std::atan2(u.cross(v).norm(),u.dot(v));
  const Rot R=parent.transpose()*o.elbow.pose.linear()*rotation(1,-q4);
  return Eigen::Vector4d(std::atan2(R(0,2),R(2,2)),std::atan2(-R(1,2),std::hypot(R(1,0),R(1,1))),std::atan2(R(1,0),R(1,1)),q4);
}
Row at(const std::vector<Row>& rows,double t){auto b=std::lower_bound(rows.begin(),rows.end(),t,[](const Row& r,double v){return r.t<v;});auto a=std::prev(b);const double u=(t-a->t)/(b->t-a->t);return {t,(1-u)*a->z+u*b->z,(1-u)*a->j7+u*b->j7,(1-u)*a->q+u*b->q};}
Json window(const std::vector<Row>& rows,double start,double end){
  const auto a=at(rows,start),b=at(rows,end);Eigen::Vector4d max=(b.q-a.q).cwiseAbs();double rise=0.,reverse=0.,minimum=a.z;
  for(const auto& x:rows)if(x.t>=start && x.t<=end){max=max.cwiseMax((x.q-a.q).cwiseAbs());rise=std::max(rise,x.j7-a.j7);minimum=std::min(minimum,x.z);reverse=std::max(reverse,x.z-minimum);}
  const double degrees=180/M_PI;
  return {{"j3_endpoint_deg",degrees*(b.q-a.q)[2]},{"j1_j4_max_deg",degrees*max.maxCoeff()},{"joints_max_deg",{degrees*max[0],degrees*max[1],degrees*max[2],degrees*max[3]}},{"j7_rise_mm",rise*1000},{"tcp_reverse_mm",reverse*1000},{"tcp_endpoint_mm",(b.z-a.z)*1000}};
}
}
int main(int argc,char**argv){
  if(argc!=4)throw std::runtime_error("parameter_audit CONFIG EPISODES OUTPUT");Config c(argv[1]);RobotModel model(c);
  std::vector<std::string> names;std::vector<double> q;for(const auto& x:c.document.at("seed_joints").items()){names.push_back(x.key());q.push_back(x.value());}
  const auto robot=model.evaluate(names,q,std::vector<double>(q.size(),0.));const auto g=model.geometry()[0];
  const double q1=c.document.at("seed_joints").at("zarm_l1_joint");const Rot parent=robot[0].shoulder.pose.linear()*rotation(1,-q1);
  std::ifstream in(argv[2]);Json fixture;in>>fixture;const auto& input=fixture.at("samples");const auto first=decode(input[0]["controller"][0]);Json result;
  for(double cutoff:{3.})for(double x:{-.005,0.,.005,.01})for(double y:{.025,.03,.035,.04,.045,.05})for(double z:{.02,.025,.03,.035,.04}){
    ArmConfig ac=c.arms[0];ac.reference_cutoff=cutoff;ac.controller_to_virtual.translation()=Vec(x,y,z);EngagedMapper mapper(ac,g,first,robot[0]);
    std::vector<Row> rows;double max_position=0.;
    for(const auto& r:input){const auto o=mapper.advance(decode(r["controller"][0]),robot[0],r["t"].get<double>()-input[0]["t"].get<double>()+c.engage_duration);rows.push_back({r["t"],o.command.pose.translation().z(),(o.command.pose*g.j7_to_tcp.inverse()).translation().z(),joints(o,g,parent,robot[0].shoulder.pose.translation())});max_position=std::max(max_position,o.position_projection.norm());}
    const auto shortw=window(rows,172.1078,172.4975),extended=window(rows,172.,173.),issue2=window(rows,130.808,131.308);
    const double score=std::max({std::abs(shortw["j3_endpoint_deg"].get<double>())/1.5,shortw["j1_j4_max_deg"].get<double>()/6.,extended["joints_max_deg"][1].get<double>()/4.4,extended["joints_max_deg"][3].get<double>()/5.2,extended["joints_max_deg"][2].get<double>()/2.2,issue2["j7_rise_mm"].get<double>()/9.,issue2["tcp_reverse_mm"].get<double>()*10.});
    result.push_back({{"position_cutoff_hz",cutoff},{"left_raw_offset_m",{x,y,z}},{"short",shortw},{"extended",extended},{"issue2",issue2},{"score",score},{"max_position_projection_mm",1000*max_position}});
  }
  std::sort(result.begin(),result.end(),[](const Json& a,const Json& b){return a["score"].get<double>()<b["score"].get<double>();});
  std::ofstream out(argv[3]);out<<result.dump(2);for(int k=0;k<12;++k)std::cout<<result[k].dump()<<std::endl;
}
