// Finite offline bandwidth selection and consistency audit; no runtime windows.
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <fstream>
#include <iostream>
#include <algorithm>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& a){return sample({a.at(0),a.at(1),a.at(2),a.at(3),a.at(4),a.at(5),a.at(6)},{a.at(7),a.at(8),a.at(9)},{a.at(10),a.at(11),a.at(12)});}
Json vec(const Vec& v){return {v.x(),v.y(),v.z()};}
Json encode(const Sample& s){Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w(),s.linear.x(),s.linear.y(),s.linear.z(),s.angular.x(),s.angular.y(),s.angular.z()};}
struct Row {double t;Eigen::Vector4d q;double z,j7;};
Eigen::Vector4d joints(const Output& o,const WristGeometry& g,const Rot& parent,const Vec& shoulder){
 const Vec u=o.elbow.pose.translation()-shoulder,v=o.elbow.pose.linear()*g.forearm_vector;
 const double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-std::atan2(g.upper_vector.x(),-g.upper_vector.z())-std::atan2(u.cross(v).norm(),u.dot(v));
 const Rot R=parent.transpose()*o.elbow.pose.linear()*rotation(1,-q4);
 return {std::atan2(R(0,2),R(2,2)),std::atan2(-R(1,2),std::hypot(R(1,0),R(1,1))),std::atan2(R(1,0),R(1,1)),q4};
}
Row at(const std::vector<Row>& rows,double t){auto b=std::lower_bound(rows.begin(),rows.end(),t,[](const Row& r,double v){return r.t<v;});if(b==rows.end()||b==rows.begin())throw std::runtime_error("Missing offline window");auto a=std::prev(b);double u=(t-a->t)/(b->t-a->t);return {t,(1-u)*a->q+u*b->q,(1-u)*a->z+u*b->z,(1-u)*a->j7+u*b->j7};}
Json window(const std::vector<Row>& rows,double s,double e){auto a=at(rows,s),b=at(rows,e);double max=(b.q-a.q).cwiseAbs().maxCoeff(),rise=0.,reverse=0.,low=a.z;for(const auto& r:rows)if(r.t>=s&&r.t<=e){max=std::max(max,(r.q-a.q).cwiseAbs().maxCoeff());rise=std::max(rise,r.j7-a.j7);low=std::min(low,r.z);reverse=std::max(reverse,r.z-low);}return {{"j3_endpoint_deg",(b.q-a.q)[2]*180/M_PI},{"j1_j4_max_deg",max*180/M_PI},{"j7_rise_mm",1000*rise},{"tcp_reverse_mm",1000*reverse}};}
Json stats(std::vector<double> x){std::sort(x.begin(),x.end());return {{"p95",x.at(std::size_t(.95*(x.size()-1)))},{"max",x.back()}};}
}
int main(int argc,char**argv){
 if(argc!=5)throw std::invalid_argument("direct_tcp_convergence CONFIG FIXTURE TYPE OUTPUT_PREFIX");
 Config c(argv[1]);RobotModel model(c);std::vector<std::string> names;std::vector<double> q;
 for(const auto& x:c.document.at("seed_joints").items()){names.push_back(x.key());q.push_back(x.value());}
 const auto robot=model.evaluate(names,q,std::vector<double>(q.size(),0.));std::ifstream in(argv[2]);Json fixture;in>>fixture;const auto& samples=fixture.at("samples");
 const double begin=samples.front().at("t"),end=samples.back().at("t");Json results;
 for(double hz:{3.,6.,9.,12.,c.arms[0].reference_cutoff}){
  Json report{{"position_hz",hz},{"wrist_hz",hz==c.arms[0].reference_cutoff?c.arms[0].wrist_reference_cutoff:hz},{"theoretical_free_group_delay_ms",1000*3*std::sqrt(std::pow(2.,1./3.)-1.)/(2*M_PI*hz)}};
  std::ofstream trace(std::string(argv[4])+"_"+std::to_string(hz)+"hz.jsonl");
  for(int a=0;a<2;++a){auto ac=c.arms[a];ac.reference_cutoff=hz;ac.wrist_reference_cutoff=hz==c.arms[0].reference_cutoff?c.arms[0].wrist_reference_cutoff:hz;auto g=model.geometry()[a];
   const auto first=decode(samples.front()["controller"][a]);EngagedMapper mapper(ac,g,first,robot[a]);
   const Rot parent=robot[a].shoulder.pose.linear()*rotation(1,-c.document["seed_joints"][a==0?"zarm_l1_joint":"zarm_r1_joint"].get<double>());
   std::vector<Row> rows;std::vector<double> ep,eo;double domain=0.,geometry=0.,maxderivative=0.,maxwderivative=0.;int domain_bad=0;Output last;
   const auto accept=[&](const Sample& input,double t,const std::string& phase){
    const auto before=mapper;const double elapsed=t-begin+c.engage_duration;auto o=mapper.advance(input,robot[a],elapsed);
    const double pos=1000*(o.command.pose.translation()-o.raw.pose.translation()).norm(),ang=180/M_PI*Eigen::AngleAxisd(o.command.pose.linear()*o.raw.pose.linear().transpose()).angle();ep.push_back(pos);eo.push_back(ang);
    const double violation=std::max((o.reference_wrist-o.final_domain_upper).maxCoeff(),(o.final_domain_lower-o.reference_wrist).maxCoeff());domain=std::max(domain,violation);domain_bad+=violation>1.e-9;
    const Rot r6=o.elbow.pose.linear()*rotation(2,o.reference_wrist[0])*rotation(0,o.reference_wrist[1]);
    const Vec p6=(o.command.pose*g.j7_to_tcp.inverse()).translation()-r6*g.j6_to_j7;
    geometry=std::max({geometry,std::abs((o.elbow.pose.translation()-robot[a].shoulder.pose.translation()).norm()-g.upper_length),std::abs((p6-o.elbow.pose.translation()).norm()-g.forearm_length)});
    if(rows.size()>0){const double eps=1.e-6;auto m=before,p=before;auto minus=m.advance(input,robot[a],elapsed-eps),plus=p.advance(input,robot[a],elapsed+eps);
      maxderivative=std::max(maxderivative,((plus.command.pose.translation()-minus.command.pose.translation())/(2*eps)-o.command.linear).norm());Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());maxwderivative=std::max(maxwderivative,(aa.axis()*aa.angle()/(2*eps)-o.command.angular).norm());}
    rows.push_back({t,joints(o,g,parent,robot[a].shoulder.pose.translation()),o.command.pose.translation().z(),(o.command.pose*g.j7_to_tcp.inverse()).translation().z()});
    trace<<Json{{"t",t},{"reference_time",elapsed},{"arm",a},{"phase",phase},{"raw",encode(o.raw)},{"projected",encode(o.projected)},{"command",encode(o.command)},{"elbow",encode(o.elbow)},{"reference_wrist",vec(o.reference_wrist)},{"wrist_rate",vec(o.reference_wrist_rate)},{"domain_lower",vec(o.final_domain_lower)},{"domain_upper",vec(o.final_domain_upper)}}.dump()<<'\n';last=o;
   };
   for(const auto& row:samples)accept(decode(row["controller"][a]),row["t"],"motion");
   auto rest=decode(samples.back()["controller"][a]);rest.linear.setZero();rest.angular.setZero();
   for(int k=1;k<=360;++k)accept(rest,end+k/120.,"settle");
   auto& arm=report["arms"][a];arm={{"position_mm",stats(ep)},{"orientation_deg",stats(eo)},{"domain_bad_samples",domain_bad},{"max_domain_violation_rad",domain},{"max_geometry_residual_m",geometry},{"max_pose_twist_linear_error_m_s",maxderivative},{"max_pose_twist_angular_error_rad_s",maxwderivative},{"settled_raw_projected_mm",1000*last.position_projection.norm()},{"settled_raw_command_mm",1000*(last.raw.pose.translation()-last.command.pose.translation()).norm()},{"settled_projected_command_mm",1000*last.position_filter.norm()},{"settled_speed_m_s",last.command.linear.norm()}};
   if(std::string(argv[3])=="episodes"&&a==0){arm["issue2"]=window(rows,130.808,131.308);arm["short"]=window(rows,172.1078,172.4975);arm["extended"]=window(rows,172.,173.);}
  }results.push_back(report);std::cout<<report.dump()<<std::endl;
 }std::ofstream out(std::string(argv[4])+"_metrics.json");out<<results.dump(2)<<'\n';
}
