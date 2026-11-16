// Offline replay and independent full-model witness audit. Never publishes.
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <fstream>
#include <iostream>
#include <algorithm>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& x){return sample({x[0],x[1],x[2],x[3],x[4],x[5],x[6]},Vec(x[7],x[8],x[9]),Vec(x[10],x[11],x[12]));}
Json encode(const Sample& s){Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w(),s.linear.x(),s.linear.y(),s.linear.z(),s.angular.x(),s.angular.y(),s.angular.z()};}
}
int main(int argc,char**argv){
 if(argc!=4)throw std::invalid_argument("direct_tcp_distortion_replay CONFIG ACTUAL_TRACE OUTPUT");
 Config c(argv[1]);RobotModel model(c);std::ifstream f(argv[2]);if(!f)throw std::runtime_error("Missing actual trace");
 std::vector<Json> rows;std::string line;while(std::getline(f,line))rows.push_back(Json::parse(line));
 Json anchor_reference,first_controller;bool anchored=false;
 for(const auto& r:rows){if(r.value("kind","")=="anchor"){anchored=true;break;}if(r.value("kind","")=="reference")anchor_reference=r;
 if(first_controller.is_null()&&r.value("kind","")=="mapper"&&r.contains("controller"))first_controller=r["controller"];}
 if(!anchored||anchor_reference.is_null()||first_controller.is_null())throw std::runtime_error("Missing exact replay anchor");
 auto names=anchor_reference["joint_names"].get<std::vector<std::string>>();auto q=anchor_reference["q"].get<std::vector<double>>();auto dq=anchor_reference["dq"].get<std::vector<double>>();const auto robot=model.evaluate(names,q,dq);
 std::array<EngagedMapper,2> maps{EngagedMapper(c.arms[0],model.geometry()[0],decode(first_controller[0]),robot[0]),EngagedMapper(c.arms[1],model.geometry()[1],decode(first_controller[1]),robot[1])};
 std::ofstream out(argv[3]);if(!out)throw std::runtime_error("Cannot write replay");double raw_diff=0.,command_diff=0.,fk_diff=0.;std::size_t count=0;
 for(const auto& r:rows){if(r.value("kind","")!="mapper"||(r.value("phase","")!="motion"&&r.value("phase","")!="settle"))continue;
 Json row{{"t",r["source_time"]},{"reference_time",r["reference_time"]},{"phase",r["phase"]},{"source_sample_time",r.value("source_sample_time",0.)}};
 for(int a=0;a<2;++a){const auto o=maps[a].advance(decode(r["controller"][a]),robot[a],r["reference_time"]);const auto& g=model.geometry()[a];
  row["raw"][a]=encode(o.raw);row["projected"][a]=encode(o.projected);row["nominal"][a]=encode(o.nominal_reference);row["command"][a]=encode(o.command);row["elbow"][a]=encode(o.elbow);
  row["upstream"][a]=o.upstream;row["upstream_rate"][a]=o.upstream_rate;row["projected_upstream"][a]=o.projected_upstream;row["position_cutoff_hz"][a]=o.position_cutoff_hz;
  row["requested_v6"][a]={o.requested_v6.x(),o.requested_v6.y(),o.requested_v6.z()};
  row["j7"][a]=encode({o.command.pose*g.j7_to_tcp.inverse(),Vec::Zero(),Vec::Zero()});
  auto witness_q=q;bool in_domain=true;
  for(int j=0;j<7;++j){const auto name=c.document["arms"][a]["arm_joints"][j].get<std::string>();const auto k=std::find(names.begin(),names.end(),name)-names.begin();if(k==int(names.size()))throw std::runtime_error("Missing witness joint");
   const double value=j<4?o.projected_upstream[j]:o.applied_wrist[j-4],lo=j<4?g.upstream_lower[j]:g.lower[j-4],hi=j<4?g.upstream_upper[j]:g.upper[j-4];
   witness_q[k]=value;in_domain=in_domain&&value>=lo-1.e-10&&value<=hi+1.e-10;row["witness_q"][a][j]=value;
  }
  const auto witness=model.evaluate(names,witness_q,std::vector<double>(q.size(),0.))[a];const double pe=(witness.tcp.pose.translation()-o.raw.pose.translation()).norm(),ae=Eigen::AngleAxisd(witness.tcp.pose.linear()*o.raw.pose.linear().transpose()).angle();
  row["static_witness"][a]={{"within_original_domain_and_mapper_margin",in_domain},{"position_error_m",pe},{"orientation_error_rad",ae},{"exact_pose_feasible",in_domain&&pe<1.e-6&&ae<1.e-5}};
  const double radius=g.upper_length+g.forearm_length+g.j6_to_j7.norm()+g.j7_to_tcp.translation().norm();row["sphere_lower_bound_m"][a]=std::max(0.,(o.raw.pose.translation()-robot[a].shoulder.pose.translation()).norm()-radius);
  raw_diff=std::max(raw_diff,(o.raw.pose.matrix()-decode(r["raw"][a]).pose.matrix()).norm());command_diff=std::max(command_diff,(o.command.pose.matrix()-decode(r["command"][a]).pose.matrix()).norm());fk_diff=std::max(fk_diff,(witness.tcp.pose.matrix()-o.projected.pose.matrix()).norm());
 }out<<row.dump()<<'\n';++count;
 }
 std::cout<<Json{{"samples",count},{"raw_replay_error",raw_diff},{"command_difference_from_recorded",command_diff},{"independent_static_fk_error",fk_diff},{"interpretation","Exact witness certifies STATIC feasibility only. Failure to find this particular witness is not an infeasibility proof. Sphere bound is a lower bound, not closest reachable error."}}.dump(2)<<'\n';
}
