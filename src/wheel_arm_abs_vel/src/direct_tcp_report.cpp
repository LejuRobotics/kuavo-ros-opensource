#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
using J=nlohmann::json;
namespace {
double degrees(double x){return x*180./std::acos(-1.);}
std::vector<J> select(const std::vector<J>& rows,const std::string& kind){std::vector<J> r;for(const auto& x:rows)if(x.at("kind")==kind && x.at("phase")=="motion")r.push_back(x);return r;}
double val(const J& row,const std::string& key,int arm,int col){return arm<0?row.at(key).at(col).get<double>():row.at(key).at(arm).at(col).get<double>();}
double at(const std::vector<J>& rows,double t,const std::string& key,int arm,int col){
  auto b=std::lower_bound(rows.begin(),rows.end(),t,[](const J& x,double y){return x.at("source_time").get<double>()<y;});
  if(b==rows.begin())return val(*b,key,arm,col);
  if(b==rows.end())return val(rows.back(),key,arm,col);
  const auto a=std::prev(b);const double ta=a->at("source_time"),tb=b->at("source_time"),u=(t-ta)/(tb-ta);
  return (1.-u)*val(*a,key,arm,col)+u*val(*b,key,arm,col);
}
J window(const std::vector<J>& rows,double start,double end,const std::string& key,int arm,int col,double scale=1.){
  const double first=at(rows,start,key,arm,col);double lo=first,hi=first,maxreverse=0.,running_min=first;
  auto accept=[&](double x){lo=std::min(lo,x);hi=std::max(hi,x);running_min=std::min(running_min,x);maxreverse=std::max(maxreverse,x-running_min);};
  for(const auto& x:rows){const double t=x.at("source_time");if(t>=start && t<=end)accept(val(x,key,arm,col));}
  const double last=at(rows,end,key,arm,col);accept(last);
  return {{"start_value",first*scale},{"end_value",last*scale},{"endpoint",(last-first)*scale},{"max_positive_from_start",(hi-first)*scale},{"max_abs_from_start",std::max(hi-first,first-lo)*scale},{"ptp",(hi-lo)*scale},{"max_reverse_from_running_min",maxreverse*scale}};
}
double quantile(std::vector<double> x,double p){if(x.empty())throw std::runtime_error("Missing metric samples");std::sort(x.begin(),x.end());const double at=p*(x.size()-1);const auto i=std::size_t(at);return x[i]+(at-i)*(x[std::min(i+1,x.size()-1)]-x[i]);}
J summary(const std::vector<double>& x){return {{"p50",quantile(x,.5)},{"p95",quantile(x,.95)},{"max",quantile(x,1.)}};}
}
int main(int argc,char** argv){
  if(argc!=5)throw std::runtime_error("direct_tcp_report TRACE.jsonl OUTPUT.json CONFIG.json TYPE");
  const wheel_arm_abs_vel::direct_tcp::Config config(argv[3]);
  std::ifstream in(argv[1]);if(!in.good())throw std::runtime_error("Missing trace");
  std::vector<J> rows;std::string line;while(std::getline(in,line))rows.push_back(J::parse(line));
  const std::string trial_type=argv[4];
  J errors=J::array();
  const auto error=[&](const std::string& s){errors.push_back(s);};
  if(trial_type!="episodes"&&trial_type!="regression"&&trial_type!="synthetic")error("Unknown trial type");
  const auto meta=std::find_if(rows.begin(),rows.end(),[](const J& r){return r.value("kind","")=="metadata";});
  const auto anchor=std::find_if(rows.begin(),rows.end(),[](const J& r){return r.value("kind","")=="anchor";});
  if(meta==rows.end())error("Missing frame/model/config metadata");
  else if(meta->value("trial_type","")!=trial_type || meta->at("config")!=config.document || meta->at("frame")!=config.output_frame || meta->at("tcp_frames")!=J(config.controlled_frames))error("Frame/TCP/model/config/type mismatch");
  if(anchor==rows.end())error("Missing engage geometry");
  std::vector<std::string> dofs{"base_x","base_y","base_yaw","knee_joint","leg_joint","waist_pitch_joint","waist_yaw_joint"};
  for(const auto& arm:config.document.at("arms"))for(const auto& n:arm.at("arm_joints"))dofs.push_back(n);
  for(const auto& row:rows){
    const auto kind=row.value("kind","");if(kind!="mapper"&&kind!="reference")continue;
    const auto phase=row.value("phase","");if(phase!="motion"&&phase!="settle"&&phase!="acquire")continue;
    const std::vector<std::string> keys=kind=="mapper"?std::vector<std::string>{"source_time","phase_time","reference_time","command","elbow"}:std::vector<std::string>{"source_time","phase_time","reference_time","tracking","solver_valid","fault","dynamic_fault","joint_names","q","dq","ddq","measured_q","fk_feedback","fk_reference","j7_feedback","j7_reference","shaped_tcp"};
    for(const auto& key:keys)if(!row.contains(key))error("Missing "+kind+" field: "+key);
    if(kind=="mapper"&&(phase=="motion"||phase=="settle"))for(const auto& key:{"raw","projected","reference_wrist","reference_wrist_rate","final_domain_lower","final_domain_upper","position_filter","orientation_filter","position_projection","orientation_projection","controller","virtual","j7_target"})if(!row.contains(key))error(std::string("Missing mapper field: ")+key);
    if(kind=="reference"&&row.contains("joint_names")&&row["joint_names"]!=J(dofs))error("Joint names/order mismatch");
    if(kind=="reference")for(const auto& key:{"q","dq","ddq","measured_q"})if(row.contains(key)&&row[key].size()!=21)error(std::string("Bad joint dimension: ")+key);
    const auto poses=kind=="mapper"?std::vector<std::string>{"command","raw","projected"}:std::vector<std::string>{"fk_feedback","fk_reference","j7_feedback","j7_reference","shaped_tcp"};
    for(const auto& key:poses)if(row.contains(key)){
      if(row[key].size()!=2){error("Bad arm dimension");continue;}
      for(const auto& pose:row[key]){
        if(pose.size()!=13){error("Bad pose/twist dimension");continue;}
        bool finite=true;for(const auto& x:pose)finite=finite&&x.is_number()&&std::isfinite(x.get<double>());
        if(!finite){error("Nonfinite pose/twist");continue;}
        double norm=0.;for(int k=3;k<7;++k)norm+=std::pow(pose[k].get<double>(),2);
        if(std::abs(norm-1.)>1.e-6)error("Invalid quaternion");
      }
    }
  }
  const double begin=trial_type=="episodes"?125.:trial_type=="regression"?175.:0.;
  const double end=trial_type=="episodes"?180.:trial_type=="regression"?230.:40.;
  for(const auto& kind:{"mapper","reference"})for(const auto& phase:{"motion","settle","acquire"}){
    std::vector<double> times;for(const auto& row:rows)if(row.value("kind","")==kind&&row.value("phase","")==phase&&row.contains("source_time")&&row.contains("phase_time"))times.push_back(row.at(std::string(phase)=="motion"?"source_time":"phase_time"));
    if(times.size()<2){error(std::string("Missing phase: ")+kind+"/"+phase);continue;}
    double gap=0.;for(std::size_t k=1;k<times.size();++k){if(times[k]<times[k-1] || (times[k]==times[k-1] && std::string(kind)=="mapper"))error(std::string("Nonmonotonic clock: ")+kind+"/"+phase);gap=std::max(gap,times[k]-times[k-1]);}
    if(std::string(kind)=="reference"){
      double previous=-1.;gap=0.;for(const auto& r:rows)if(r.value("kind","")==kind&&r.value("phase","")==phase){const double t=r.at("phase_time");if(previous>=0.){if(t<=previous)error("Nonmonotonic diagnostic receive clock");gap=std::max(gap,t-previous);}previous=t;}
    }
    if(gap>.05)error(std::string("Severe sample gap: ")+kind+"/"+phase);
    const double lo=std::string(phase)=="motion"?begin:0.,hi=std::string(phase)=="motion"?end:std::string(phase)=="settle"?3.:10.;
    if(times.front()>lo+.10 || times.back()<hi-.10)error(std::string("Incomplete window: ")+kind+"/"+phase);
  }
  if(!errors.empty()){
    J incomplete{{"trial_type",trial_type},{"status","INCOMPLETE"},{"errors",errors},{"acceptance",{{"all_required_gates_pass",false}}}};
    std::ofstream output(argv[2]);output<<incomplete.dump(2)<<'\n';std::cerr<<incomplete.dump(2)<<std::endl;return 2;
  }
  const auto reference=select(rows,"reference"),mapper=select(rows,"mapper");
  if(reference.empty() || mapper.empty())throw std::runtime_error("No motion samples; failed trial, no acceptance metrics");
  J result{{"trace",argv[1]},{"metric_version","v5_cpp_no_hold_reference_1; exact endpoints + all interior samples; source time from replay phase clock"}};
  for(const auto& x:rows)if(x.at("kind")=="reference"){
    const auto phase=x.at("phase").get<std::string>();auto& p=result["phases"][phase];
    if(p.is_null())p={{"samples",0},{"tracking_samples",0},{"solver_valid_samples",0},{"fault_samples",0}};
    p["samples"]=p["samples"].get<int>()+1;p["tracking_samples"]=p["tracking_samples"].get<int>()+int(x.at("tracking").get<bool>());
    p["solver_valid_samples"]=p["solver_valid_samples"].get<int>()+int(x.at("solver_valid").get<int>()==1);p["fault_samples"]=p["fault_samples"].get<int>()+int(x.at("fault").get<int>()!=0 || x.at("dynamic_fault").get<int>()!=0);
  }
  for(auto& p:result["phases"].items()){
    const double n=p.value()["samples"];p.value()["tracking_fraction"]=p.value()["tracking_samples"].get<double>()/n;
    p.value()["solver_valid_fraction"]=p.value()["solver_valid_samples"].get<double>()/n;
  }
  for(int a=0;a<2;++a){
    std::vector<double> e,ep,eo,rawz;
    for(const auto& r:reference){
      double norm=0.;for(int k=0;k<3;++k){const double d=val(r,"fk_feedback",a,k)-at(mapper,r.at("source_time"),"command",a,k);norm+=d*d;}
      e.push_back(std::sqrt(norm)*1000.);
    }
    result["tcp_error_mm"][a]=summary(e);
    if(mapper.front().contains("position_projection")){
      int exceeded=0;for(const auto& r:mapper){double p=0.,o=0.;for(int k=0;k<3;++k){p+=std::pow(val(r,"position_projection",a,k),2);o+=std::pow(val(r,"orientation_projection",a,k),2);}ep.push_back(std::sqrt(p)*1000.);eo.push_back(degrees(std::sqrt(o)));double totalp=0.;for(int k=0;k<3;++k)totalp+=std::pow(val(r,"command",a,k)-val(r,"raw",a,k),2);
      const Eigen::Quaterniond qc(val(r,"command",a,6),val(r,"command",a,3),val(r,"command",a,4),val(r,"command",a,5)),qr(val(r,"raw",a,6),val(r,"raw",a,3),val(r,"raw",a,4),val(r,"raw",a,5));
      exceeded+=std::sqrt(totalp)>config.position_budget || Eigen::AngleAxisd(qc*qr.conjugate()).angle()>config.orientation_budget;}
      result["projection_position_mm"][a]=summary(ep);result["projection_orientation_deg"][a]=summary(eo);result["budget_exceeded_fraction"][a]=double(exceeded)/mapper.size();
    }
  }
  if(mapper.front().contains("position_filter"))for(int a=0;a<2;++a){
    std::vector<double> fp,fo,tp,to;
    for(const auto& row:mapper){
      double p=0.,o=0.,totalp=0.;for(int k=0;k<3;++k){p+=std::pow(val(row,"position_filter",a,k),2);o+=std::pow(val(row,"orientation_filter",a,k),2);totalp+=std::pow(val(row,"command",a,k)-val(row,"raw",a,k),2);}
      const Eigen::Quaterniond qc(val(row,"command",a,6),val(row,"command",a,3),val(row,"command",a,4),val(row,"command",a,5)),qr(val(row,"raw",a,6),val(row,"raw",a,3),val(row,"raw",a,4),val(row,"raw",a,5));
      fp.push_back(1000*std::sqrt(p));fo.push_back(degrees(std::sqrt(o)));tp.push_back(1000*std::sqrt(totalp));to.push_back(degrees(Eigen::AngleAxisd(qc*qr.conjugate()).angle()));
    }
    result["reference_filter_position_mm"][a]=summary(fp);result["reference_filter_orientation_deg"][a]=summary(fo);
    result["total_command_deviation_from_raw_mm"][a]=summary(tp);result["total_command_deviation_from_raw_deg"][a]=summary(to);
  }
  const double first=reference.front().at("source_time"),last=reference.back().at("source_time");
  for(int k=0;k<7;++k)result["base_lower_reference"][k]=window(reference,first,last,"q",-1,k,degrees(1.));
  for(const auto& x:reference)if(!x.at("solver_valid").get<int>())result["invalid_solver_source_times"].push_back(x.at("source_time"));
  if(trial_type=="episodes"){
    for(const auto& key:{"j7_reference","j7_feedback","fk_reference","fk_feedback","shaped_tcp"})result["issue2"][key]=window(reference,130.808,131.308,key,0,2,1000.);
    for(const auto& key:{"command","j7_target"})result["issue2"][key]=window(mapper,130.808,131.308,key,0,2,1000.);
    if(mapper.front().contains("raw"))for(const auto& key:{"controller","virtual","raw","projected"})result["issue2"][key]=window(mapper,130.808,131.308,key,0,2,1000.);
    result["issue2"]["pass_j7_height"]=result["issue2"]["j7_feedback"]["max_positive_from_start"].get<double>()<=9.;
    result["issue2"]["tcp_nonincreasing"]=result["issue2"]["fk_feedback"]["max_reverse_from_running_min"].get<double>()<=0.;
    for(const auto& w:std::vector<std::pair<std::string,std::pair<double,double>>>{{"short",{172.1078,172.4975}},{"extended",{172.,173.}}}){
      double maxexcursion=0.;for(int k=0;k<7;++k){auto m=window(reference,w.second.first,w.second.second,"measured_q",-1,k+7,degrees(1.));result["issue3"][w.first]["joints"][k]=m;if(k<4)maxexcursion=std::max(maxexcursion,m["max_abs_from_start"].get<double>());}
      result["issue3"][w.first]["j1_j4_max_excursion_deg"]=maxexcursion;result["issue3"][w.first]["pass_j1_j4"]=maxexcursion<=6.;
    }
    result["issue3"]["pass_j3_short"]=std::abs(result["issue3"]["short"]["joints"][2]["endpoint"].get<double>())<=1.5;
  }
  std::vector<double> speed;
  for(const auto& x:reference)for(const auto& v:x.at("dq"))speed.push_back(std::abs(v.get<double>()));
  result["joint_speed_rad_s"]=summary(speed);
  if(reference.front().contains("ddq")){
    std::vector<double> acceleration;for(const auto& x:reference)for(const auto& v:x.at("ddq"))acceleration.push_back(std::abs(v.get<double>()));
    result["joint_acceleration_rad_s2"]=summary(acceleration);
  }
  if(trial_type=="synthetic"){
    const std::array<std::string,5> names{"J7","J6","J5","J4","J6_boundary_return"};
    for(int s=0;s<5;++s){double maxptp=0.;for(int a=0;a<2;++a)for(int k=0;k<(s==3?3:4);++k){auto m=window(reference,s*8.+.25,(s+1)*8.-.05,"measured_q",-1,7+a*7+k,degrees(1.));result["synthetic"][names[s]][a][k]=m;maxptp=std::max(maxptp,m["ptp"].get<double>());}result["synthetic_max_upstream_ptp_deg"][names[s]]=maxptp;}
    result["synthetic_boundary_pass"]=result["synthetic_max_upstream_ptp_deg"]["J6_boundary_return"].get<double>()<=3.;
    for(int a=0;a<2;++a)for(int k=0;k<4;++k)result["return_j1_j4_deg"][a][k]=degrees(at(reference,last,"measured_q",-1,7+a*7+k)-at(reference,first,"measured_q",-1,7+a*7+k));
  }
  if(trial_type=="synthetic"){
    for(int a=0;a<2;++a)for(int k=0;k<4;++k){
      double before=0.,after=0.;int nb=0,na=0;
      for(const auto& x:rows)if(x.at("kind")=="reference"){
        if(x.at("phase")=="acquire" && x.at("phase_time").get<double>()>9.){before+=val(x,"measured_q",-1,7+a*7+k);++nb;}
        if(x.at("phase")=="settle" && x.at("phase_time").get<double>()>2.){after+=val(x,"measured_q",-1,7+a*7+k);++na;}
      }
      if(nb==0 || na==0)throw std::runtime_error("Missing acquire/settle return residual evidence");
      result["return_after_settle_j1_j4_deg"][a][k]=degrees(after/na-before/nb);
    }
  }
  bool healthy=true;for(const auto& phase:{"motion","settle"}){const auto& p=result["phases"][phase];healthy=healthy && p["samples"]==p["tracking_samples"] && p["samples"]==p["solver_valid_samples"] && p["fault_samples"].get<int>()==0;}
  result["acceptance"]["all_sampled_motion_and_settle_healthy"]=healthy;
  if(result.contains("issue2"))result["acceptance"]["issue2_and_issue3_pass"]=result["issue2"]["pass_j7_height"].get<bool>() && result["issue2"]["tcp_nonincreasing"].get<bool>() && result["issue3"]["pass_j3_short"].get<bool>() && result["issue3"]["short"]["pass_j1_j4"].get<bool>() && result["issue3"]["extended"]["pass_j1_j4"].get<bool>();
  if(result.contains("synthetic")){
    result["acceptance"]["synthetic_boundary_pass"]=result["synthetic_boundary_pass"];
    for(int segment=0;segment<5;++segment)for(int a=0;a<2;++a)for(int k=0;k<7;++k)result["synthetic_all_joint_motion_deg"][segment][a][k]=window(reference,segment*8.,(segment+1)*8.-.01,"measured_q",-1,7+a*7+k,degrees(1.));
  }
  // A separate hard gate recomputed from poses, including filtering and settle.
  bool budget_pass=true;
  result["budget_acceptance"]["position_limit_mm"]=1000.*config.position_budget;
  result["budget_acceptance"]["orientation_limit_deg"]=degrees(config.orientation_budget);
  result["budget_acceptance"]["reference"]="same-frame command minus original anchored raw TCP; all motion and settle samples; no time alignment or exclusions";
  for(int a=0;a<2;++a) {
    int samples=0,violations=0,missing=0;double maxp=0.,maxo=0.;J first_bad,last_bad;
    for(const auto& row:rows)if(row.at("kind")=="mapper" &&
        (row.at("phase")=="motion" || row.at("phase")=="settle")) {
      if(!row.contains("raw")){++missing;continue;}
      ++samples;double p2=0.;for(int k=0;k<3;++k)p2+=std::pow(val(row,"command",a,k)-val(row,"raw",a,k),2);
      const Eigen::Quaterniond qc(val(row,"command",a,6),val(row,"command",a,3),val(row,"command",a,4),val(row,"command",a,5));
      const Eigen::Quaterniond qr(val(row,"raw",a,6),val(row,"raw",a,3),val(row,"raw",a,4),val(row,"raw",a,5));
      const double p=std::sqrt(p2),o=Eigen::AngleAxisd(qc*qr.conjugate()).angle();
      maxp=std::max(maxp,p);maxo=std::max(maxo,o);
      if(p>config.position_budget || o>config.orientation_budget) {
        ++violations;J bad{{"source_time",row.at("source_time")},{"phase",row.at("phase")},{"position_mm",1000.*p},{"orientation_deg",degrees(o)}};
        if(first_bad.is_null())first_bad=bad;
        last_bad=bad;
      }
    }
    const bool pass=samples>0 && violations==0 && missing==0;budget_pass=budget_pass && pass;
    result["budget_acceptance"]["arms"][a]={{"samples",samples},{"missing_raw_samples",missing},{"violating_samples",violations},{"max_position_mm",1000.*maxp},
      {"max_orientation_deg",degrees(maxo)},{"first_violation",first_bad},{"last_violation",last_bad},{"pass",pass}};
  }
  result["acceptance"]["distortion_budget_pass"]=budget_pass;
  bool all=healthy && budget_pass;
  if(result.contains("issue2"))all=all && result["acceptance"]["issue2_and_issue3_pass"].get<bool>();
  if(result.contains("synthetic"))all=all && result["synthetic_boundary_pass"].get<bool>();
  // Final-velocity domain and geometric compatibility of emitted targets.
  wheel_arm_abs_vel::direct_tcp::RobotModel robot_model(config);
  bool coherent=true;
  for(int a=0;a<2;++a){
    const auto& g=robot_model.geometry()[a];double maxdomain=0.,maxgeometry=0.;int bad=0,zero_twist_moving=0;
    std::vector<J> samples;for(const auto& r:rows)if(r.at("kind")=="mapper"&&(r.at("phase")=="motion"||r.at("phase")=="settle"))samples.push_back(r);
    for(const auto& r:samples){
      wheel_arm_abs_vel::direct_tcp::Vec q,qd,shoulder;
      for(int k=0;k<3;++k){q[k]=val(r,"reference_wrist",a,k);qd[k]=val(r,"reference_wrist_rate",a,k);shoulder[k]=anchor->at("shoulders").at(a).at(k);}
      for(int k=0;k<3;++k){const double up=std::max(0.,qd[k]),down=std::max(0.,-qd[k]);
        const double lo=g.lower[k]+g.stopping_linear[k]*down+g.stopping_quadratic[k]*down*down,hi=g.upper[k]-g.stopping_linear[k]*up-g.stopping_quadratic[k]*up*up;
        const double violation=std::max(lo-q[k],q[k]-hi);maxdomain=std::max(maxdomain,violation);bad+=violation>1.e-9;
        if(std::abs(lo-val(r,"final_domain_lower",a,k))>1.e-10 || std::abs(hi-val(r,"final_domain_upper",a,k))>1.e-10)coherent=false;
      }
      const auto decode=[&](const std::string& key){return wheel_arm_abs_vel::direct_tcp::sample({val(r,key,a,0),val(r,key,a,1),val(r,key,a,2),val(r,key,a,3),val(r,key,a,4),val(r,key,a,5),val(r,key,a,6)},wheel_arm_abs_vel::direct_tcp::Vec::Zero(),wheel_arm_abs_vel::direct_tcp::Vec::Zero());};
      const auto tcp=decode("command"),elbow=decode("elbow");
      using wheel_arm_abs_vel::direct_tcp::rotation;
      const auto r6=elbow.pose.linear()*rotation(2,q[0])*rotation(0,q[1]);
      const wheel_arm_abs_vel::direct_tcp::Vec p6=(tcp.pose*g.j7_to_tcp.inverse()).translation()-r6*g.j6_to_j7;
      maxgeometry=std::max({maxgeometry,std::abs((elbow.pose.translation()-shoulder).norm()-g.upper_length),std::abs((p6-elbow.pose.translation()).norm()-g.forearm_length),
        (tcp.pose.linear()-r6*rotation(1,q[2])*g.j7_to_tcp.linear()).norm()});
    }
    std::vector<double> lin,ang,settlelin,settleang;
    for(std::size_t i=1;i+1<samples.size();++i){const auto& r=samples[i];if(samples[i-1]["phase"]!=r["phase"]||samples[i+1]["phase"]!=r["phase"])continue;
      const double dt=samples[i+1]["reference_time"].get<double>()-samples[i-1]["reference_time"].get<double>();if(dt<=0.)throw std::runtime_error("Nonmonotonic reference advance clock");
      wheel_arm_abs_vel::direct_tcp::Vec v,omega;double emitted=0.;for(int k=0;k<3;++k){v[k]=(val(samples[i+1],"command",a,k)-val(samples[i-1],"command",a,k))/dt;omega[k]=val(r,"command",a,10+k);emitted+=std::pow(val(r,"command",a,7+k),2);}
      const double movement=v.norm();for(int k=0;k<3;++k)v[k]-=val(r,"command",a,7+k);
      const auto quat=[&](const J& x){return Eigen::Quaterniond(val(x,"command",a,6),val(x,"command",a,3),val(x,"command",a,4),val(x,"command",a,5));};
      Eigen::AngleAxisd d(quat(samples[i+1])*quat(samples[i-1]).conjugate());const double ae=(d.axis()*d.angle()/dt-omega).norm();
      lin.push_back(v.norm());ang.push_back(ae);
      if(r["phase"]=="settle"){settlelin.push_back(v.norm());settleang.push_back(ae);zero_twist_moving+=movement>1.e-5 && emitted<1.e-20;}
    }
    const auto& last=samples.back();double sp=0.,total=0.,extra=0.;for(int k=0;k<3;++k){sp+=std::pow(val(last,"projected",a,k)-val(last,"raw",a,k),2);total+=std::pow(val(last,"command",a,k)-val(last,"raw",a,k),2);extra+=std::pow(val(last,"command",a,k)-val(last,"projected",a,k),2);}
    result["consistency"][a]={{"domain_violation_samples",bad},{"max_domain_violation_rad",maxdomain},{"max_geometry_residual",maxgeometry},{"zero_twist_while_settle_moving_samples",zero_twist_moving},
      {"finite_difference_linear_error_m_s",summary(lin)},{"finite_difference_angular_error_rad_s",summary(ang)},{"settle_finite_difference_linear_error_m_s",summary(settlelin)},{"settle_finite_difference_angular_error_rad_s",summary(settleang)},
      {"settled_raw_projected_mm",1000*std::sqrt(sp)},{"settled_raw_command_mm",1000*std::sqrt(total)},{"settled_projected_command_mm",1000*std::sqrt(extra)}};
    coherent=coherent && bad==0 && maxgeometry<1.e-8 && zero_twist_moving==0;
    for(const auto& layer:std::vector<std::pair<std::string,std::string>>{{"command","shaped_tcp"},{"shaped_tcp","fk_reference"},{"fk_reference","fk_feedback"}}){
      std::vector<double> position,orientation;for(const auto& r:reference){double d2=0.;
        const auto x=[&](const std::string& key,int k){return key=="command"?at(mapper,r["source_time"],key,a,k):val(r,key,a,k);};
        for(int k=0;k<3;++k)d2+=std::pow(x(layer.first,k)-x(layer.second,k),2);position.push_back(1000*std::sqrt(d2));
        Eigen::Quaterniond qa(x(layer.first,6),x(layer.first,3),x(layer.first,4),x(layer.first,5)),qb(x(layer.second,6),x(layer.second,3),x(layer.second,4),x(layer.second,5));
        // Quaternion interpolation above is used only for diagnostic layer
        // comparison, normalized explicitly as interpolation, never raw repair.
        orientation.push_back(degrees(Eigen::AngleAxisd(qa.normalized()*qb.normalized().conjugate()).angle()));
      }
      result["layer_errors_motion"][layer.first+"_to_"+layer.second][a]={{"position_mm",summary(position)},{"orientation_deg",summary(orientation)}};
    }
  }
  // Replay the emitted continuous state on the recorded reference clock.
  // This catches wrong nonzero twists as well as the old premature zero twist.
  // Tiny central differences stay inside each interval's fixed forcing; coarse
  // 120 Hz differences above are diagnostics and mix different source updates.
  namespace d=wheel_arm_abs_vel::direct_tcp;
  const auto decode_sample=[](const J& x){return d::sample({x.at(0),x.at(1),x.at(2),x.at(3),x.at(4),x.at(5),x.at(6)},d::Vec(x.at(7),x.at(8),x.at(9)),d::Vec(x.at(10),x.at(11),x.at(12)));};
  J anchor_reference,first_controller;
  for(const auto& r:rows){
    if(r.value("kind","")=="anchor")break;
    if(r.value("kind","")=="reference")anchor_reference=r;
    if(first_controller.is_null()&&r.value("kind","")=="mapper"&&r.contains("controller"))first_controller=r.at("controller");
  }
  if(anchor_reference.is_null()||first_controller.is_null())throw std::runtime_error("Missing reference-state replay anchor");
  const auto anchored=robot_model.evaluate(anchor_reference.at("joint_names").get<std::vector<std::string>>(),anchor_reference.at("q").get<std::vector<double>>(),anchor_reference.at("dq").get<std::vector<double>>());
  for(int a=0;a<2;++a){
    d::EngagedMapper replay(config.arms[a],robot_model.geometry()[a],decode_sample(first_controller[a]),anchored[a]);
    double pose_error=0.,twist_error=0.,raw_error=0.,linear_error=0.,angular_error=0.,previous_time=0.;int count=0,settle_count=0;
    double max_linear_source_time=0.,max_angular_source_time=0.;
    double upstream_violation=0.,upstream_speed_violation=0.,independent_fk_error=0.,independent_twist_error=0.;int upstream_bad=0,upstream_samples=0;
    const auto joint_names=anchor_reference.at("joint_names").get<std::vector<std::string>>();
    const auto anchor_q=anchor_reference.at("q").get<std::vector<double>>();
    std::array<std::size_t,7> arm_indices;
    for(int j=0;j<7;++j){const std::string name=config.document.at("arms").at(a).at("arm_joints").at(j);
      const auto found=std::find(joint_names.begin(),joint_names.end(),name);
      if(found==joint_names.end())throw std::runtime_error("Missing joint for independent full-chain audit");
      arm_indices[j]=found-joint_names.begin();}
    for(const auto& r:rows)if(r.value("kind","")=="mapper"&&(r.value("phase","")=="motion"||r.value("phase","")=="settle")){
      const double time=r.at("reference_time");auto before=replay;
      const auto input=decode_sample(r.at("controller")[a]),logged=decode_sample(r.at("command")[a]),raw=decode_sample(r.at("raw")[a]);
      const auto output=replay.advance(input,anchored[a],time);
      const auto& g=robot_model.geometry()[a];bool outside=false;
      for(int j=0;j<4;++j){const double rate=output.upstream_rate[j],up=std::max(0.,rate),down=std::max(0.,-rate);
        const double lo=g.upstream_lower[j]+g.upstream_stopping_linear[j]*down+g.upstream_stopping_quadratic[j]*down*down;
        const double hi=g.upstream_upper[j]-g.upstream_stopping_linear[j]*up-g.upstream_stopping_quadratic[j]*up*up;
        const double violation=std::max(lo-output.upstream[j],output.upstream[j]-hi),speed=std::abs(rate)-g.upstream_vmax[j];
        upstream_violation=std::max(upstream_violation,violation);upstream_speed_violation=std::max(upstream_speed_violation,speed);
        outside=outside||violation>1.e-9||speed>1.e-9;
      }upstream_bad+=outside;++upstream_samples;
      auto audit_q=anchor_q;std::vector<double> audit_dq(audit_q.size(),0.);
      for(int j=0;j<7;++j){audit_q[arm_indices[j]]=j<4?output.upstream[j]:output.reference_wrist[j-4];audit_dq[arm_indices[j]]=j<4?output.upstream_rate[j]:output.reference_wrist_rate[j-4];}
      const auto independent=robot_model.evaluate(joint_names,audit_q,audit_dq)[a];
      const auto logged_elbow=decode_sample(r.at("elbow")[a]);
      independent_fk_error=std::max({independent_fk_error,(independent.tcp.pose.matrix()-logged.pose.matrix()).norm(),(independent.elbow.pose.translation()-logged_elbow.pose.translation()).norm()});
      independent_twist_error=std::max({independent_twist_error,(independent.tcp.linear-logged.linear).norm(),(independent.tcp.angular-logged.angular).norm(),(independent.elbow.linear-logged_elbow.linear).norm()});
      pose_error=std::max(pose_error,(output.command.pose.matrix()-logged.pose.matrix()).norm());
      twist_error=std::max({twist_error,(output.command.linear-logged.linear).norm(),(output.command.angular-logged.angular).norm()});
      raw_error=std::max(raw_error,(output.raw.pose.matrix()-raw.pose.matrix()).norm());
      const double eps=1.e-6;
      if(time-previous_time>2*eps){auto m=before,p=before;const auto minus=m.advance(input,anchored[a],time-eps),plus=p.advance(input,anchored[a],time+eps);
        const double le=((plus.command.pose.translation()-minus.command.pose.translation())/(2*eps)-logged.linear).norm();
        if(le>linear_error){linear_error=le;max_linear_source_time=r.value("source_time",0.);}
        Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());const double ae=(aa.axis()*aa.angle()/(2*eps)-logged.angular).norm();
        if(ae>angular_error){angular_error=ae;max_angular_source_time=r.value("source_time",0.);}
        ++count;if(r["phase"]=="settle")++settle_count;
      }previous_time=time;
    }
    const bool passed=pose_error<1.e-9&&raw_error<1.e-9&&twist_error<1.e-9&&linear_error<1.e-5&&angular_error<1.e-4&&settle_count>300;
    result["reference_replay_consistency"][a]={{"pass",passed},{"checked_samples",count},{"settle_samples",settle_count},{"max_pose_matrix_error",pose_error},{"max_raw_matrix_error",raw_error},{"max_logged_twist_error",twist_error},{"max_derivative_linear_source_time",max_linear_source_time},{"max_derivative_angular_source_time",max_angular_source_time},{"max_derivative_linear_error_m_s",linear_error},{"max_derivative_angular_error_rad_s",angular_error}};
    coherent=coherent&&passed;
    const bool upstream_pass=upstream_bad==0&&upstream_samples>0&&independent_fk_error<1.e-8&&independent_twist_error<1.e-8;
    result["upstream_full_chain_audit"][a]={{"pass",upstream_pass},{"samples",upstream_samples},{"violating_samples",upstream_bad},
      {"max_domain_violation_rad",upstream_violation},{"max_speed_violation_rad_s",upstream_speed_violation},
      {"max_independent_fk_error",independent_fk_error},{"max_independent_twist_error",independent_twist_error}};
    coherent=coherent&&upstream_pass;
  }
  result["acceptance"]["final_geometry_and_domain_consistent"]=coherent;all=all&&coherent;
  result["config"]=config.document;
  result["trial_type"]=trial_type;
  result["coverage_complete"]=true;
  result["status"]=all?"PASS":"FAIL";
  result["acceptance"]["all_required_gates_pass"]=all;
  std::ofstream output(argv[2]);output<<result.dump(2);std::cout<<result.dump(2)<<std::endl;return 0;
}
