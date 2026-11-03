// Targeted offline full-URDF witnesses; NEVER a runtime IK or controller.
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& a){return sample({a.at(0),a.at(1),a.at(2),a.at(3),a.at(4),a.at(5),a.at(6)},Vec::Zero(),Vec::Zero());}
Vec wrist(const Rot& r){return {std::atan2(-r(0,1),r(1,1)),std::asin(r(2,1)),std::atan2(-r(2,0),r(2,2))};}
}
int main(int argc,char**argv){
 if(argc!=5&&argc!=6)throw std::runtime_error("direct_tcp_static_feasibility CONFIG FIXTURE SOURCE_TIME OUTPUT [ACTUAL_TRACE]");Config c(argv[1]);RobotModel model(c);auto urdf=urdf::parseURDFFile(c.urdf);
 std::vector<std::string> names;std::vector<double> base;for(const auto& x:c.document["seed_joints"].items()){names.push_back(x.key());base.push_back(x.value());}std::vector<double> rate(base.size(),0.);
 if(argc==6){std::ifstream trace(argv[5]);std::string line;Json last;bool found=false;while(std::getline(trace,line)){auto r=Json::parse(line);if(r.value("kind","")=="anchor"){found=true;break;}if(r.value("kind","")=="reference")last=r;}if(!found||last.is_null())throw std::runtime_error("Missing actual engage reference");names=last.at("joint_names").get<std::vector<std::string>>();base=last.at("q").get<std::vector<double>>();rate=last.at("dq").get<std::vector<double>>();}
 auto anchor=model.evaluate(names,base,rate);std::ifstream in(argv[2]);Json fixture;in>>fixture;const auto& rows=fixture["samples"];double time=std::stod(argv[3]);auto row=std::lower_bound(rows.begin(),rows.end(),time,[](const Json& r,double t){return r["t"].get<double>()<t;});if(row==rows.end())row=std::prev(rows.end());Json report{{"source_time",(*row)["t"]},{"interpretation","Finite full-model static witness search about the chosen upstream geometry, each J1-J4 varied within +/-3 deg. A found witness proves static feasibility only; no witness DOES NOT prove global infeasibility or feasible trajectory."}};
 report["anchor_source"]=argc==6?argv[5]:"CONFIG_SEED_OFFLINE_ONLY";
 for(int a=0;a<2;++a){auto g=model.geometry()[a];EngagedMapper mapper(c.arms[a],g,decode(rows.front()["controller"][a]),anchor[a]);auto o=mapper.map(decode((*row)["controller"][a]),anchor[a],1.);std::array<int,7> ids;for(int j=0;j<7;++j)ids[j]=std::find(names.begin(),names.end(),c.document["arms"][a]["arm_joints"][j].get<std::string>())-names.begin();
  const Vec u=o.elbow.pose.translation()-anchor[a].shoulder.pose.translation(),v=o.elbow.pose.linear()*g.forearm_vector;double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-std::atan2(g.upper_vector.x(),-g.upper_vector.z())-std::atan2(u.cross(v).norm(),u.dot(v));const Rot parent=anchor[a].shoulder.pose.linear()*rotation(1,-base[ids[0]]),R=parent.transpose()*o.elbow.pose.linear()*rotation(1,-q4);std::vector<double> center=base;center[ids[0]]=std::atan2(R(0,2),R(2,2));center[ids[1]]=std::atan2(-R(1,2),std::hypot(R(1,0),R(1,1)));center[ids[2]]=std::atan2(R(1,0),R(1,1));center[ids[3]]=q4;
  for(double weight:{.5,1.,1.25,1.5,2.,3.}){auto ac=c.arms[a];ac.shoulder_rotation_weights[1]=weight;EngagedMapper m(ac,g,decode(rows.front()["controller"][a]),anchor[a]);auto x=m.map(decode((*row)["controller"][a]),anchor[a],1.);report["metric_candidates"][a].push_back({{"weight_j2",weight},{"static_position_mm",1000*x.position_projection.norm()},{"requested_j6_deg",x.requested_wrist[1]*180/M_PI}});}
  const Rot targetR=o.raw.pose.linear()*g.j7_to_tcp.linear().transpose();double score=1.e99;Json best;int accepted=0,total=0;
  for(int i=-3;i<=3;++i)for(int j=-3;j<=3;++j)for(int k=-3;k<=3;++k)for(int l=-3;l<=3;++l){auto q=center;const int steps[4]={i,j,k,l};for(int z=0;z<4;++z)q[ids[z]]+=steps[z]*M_PI/180.;auto state=model.evaluate(names,q,std::vector<double>(q.size(),0.))[a];Vec requested=wrist(state.forearm.transpose()*targetR);for(int z=0;z<3;++z)q[ids[z+4]]=softProject(requested[z],g.lower[z],g.upper[z],c.arms[a].wrist_width).value;
   bool valid=true;for(int z=0;z<7;++z){auto limits=urdf->joints_.at(names[ids[z]])->limits;valid=valid&&q[ids[z]]>=limits->lower&&q[ids[z]]<=limits->upper;}if(!valid)continue;++total;
   auto fk=model.evaluate(names,q,std::vector<double>(q.size(),0.))[a];const double p=(fk.tcp.pose.translation()-o.raw.pose.translation()).norm(),angle=Eigen::AngleAxisd(fk.tcp.pose.linear()*o.raw.pose.linear().transpose()).angle(),cost=std::max(p/c.position_budget,angle/c.orientation_budget);accepted+=cost<=1.;if(cost<score){score=cost;Json arm=Json::array();for(int z=0;z<7;++z)arm.push_back(q[ids[z]]);best={{"position_error_mm",1000*p},{"orientation_error_deg",angle*180/M_PI},{"joints_rad",arm},{"upstream_delta_deg",{double(i),double(j),double(k),double(l)}},{"normalized_budget_score",score}};}}
  report["arms"][a]={{"selected_geometry_static_error_mm",1000*o.position_projection.norm()},{"valid_candidates",total},{"budget_feasible_static_witnesses",accepted},{"best",best}};
 }std::ofstream out(argv[4]);out<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<std::endl;
}
