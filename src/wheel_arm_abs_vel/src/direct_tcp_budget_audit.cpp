#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <Eigen/SVD>
#include <fstream>
#include <iostream>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& a){return sample({a.at(0),a.at(1),a.at(2),a.at(3),a.at(4),a.at(5),a.at(6)},a.size()==13?Vec(a[7],a[8],a[9]):Vec::Zero(),a.size()==13?Vec(a[10],a[11],a[12]):Vec::Zero());}
Json encode(const Sample& s){Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w()};}
Json read(const char* p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing event input");Json j;f>>j;return j;}
}
int main(int argc,char**argv){
 if(argc!=5)throw std::runtime_error("direct_tcp_budget_audit CONFIG FIXTURE SOURCE_ROWS OUTPUT");
 Config c(argv[1]);RobotModel model(c);std::vector<std::string> names;std::vector<double> q;for(const auto& x:c.document["seed_joints"].items()){names.push_back(x.key());q.push_back(x.value());}auto robot=model.evaluate(names,q,std::vector<double>(q.size(),0.));
 const auto fixture=read(argv[2]),source=read(argv[3]);const auto& rows=fixture.at("samples");Json report;
 report["scope"]="Observed coordinate event, no correction applied. No reference-space SDK event is recorded. All original raw data retained.";
 for(std::size_t i=1;i<source.size();++i){const auto& before=source[i-1]["selected_poses"];const auto& after=source[i]["selected_poses"];Vec ca=Vec::Zero(),cb=Vec::Zero();for(int j=0;j<7;++j){ca+=decode(before[j]).pose.translation()/7.;cb+=decode(after[j]).pose.translation()/7.;}Rot H=Rot::Zero();for(int j=0;j<7;++j)H+=(decode(before[j]).pose.translation()-ca)*(decode(after[j]).pose.translation()-cb).transpose();Eigen::JacobiSVD<Rot> svd(H,Eigen::ComputeFullU|Eigen::ComputeFullV);Rot sign=Rot::Identity();sign(2,2)=(svd.matrixV()*svd.matrixU().transpose()).determinant();Rot R=svd.matrixV()*sign*svd.matrixU().transpose();Vec t=cb-R*ca;double sum=0.,max=0.,maxrot=0.;for(int j=0;j<7;++j){auto a=decode(before[j]),b=decode(after[j]);double e=(R*a.pose.translation()+t-b.pose.translation()).norm();sum+=e*e;max=std::max(max,e);maxrot=std::max(maxrot,Eigen::AngleAxisd(R*a.pose.linear()*b.pose.linear().transpose()).angle());}Pose T=Pose::Identity();T.linear()=R;T.translation()=t;report["common_fit"].push_back({{"before_receive_time",source[i-1]["bag_time_relative_s"]},{"after_receive_time",source[i]["bag_time_relative_s"]},{"transform",encode({T,Vec::Zero(),Vec::Zero()})},{"rotation_deg",Eigen::AngleAxisd(R).angle()*180/M_PI},{"rms_position_mm",1000*std::sqrt(sum/7.)},{"max_position_mm",1000*max},{"max_orientation_residual_deg",maxrot*180/M_PI}});}
 const double start=rows.front()["t"];
 for(int a=0;a<2;++a){auto previous=decode(rows.front()["controller"][a]);EngagedMapper mapper(c.arms[a],model.geometry()[a],previous,robot[a]);auto last=mapper.advance(previous,robot[a],c.engage_duration);
  for(std::size_t i=1;i<rows.size();++i){const double time=rows[i]["t"],beforetime=rows[i-1]["t"],elapsed=time-start+c.engage_duration;const auto now=decode(rows[i]["controller"][a]);auto old_at_same_time=previous;old_at_same_time.pose.translation()+=(time-beforetime)*previous.linear;const double w=previous.angular.norm();if(w>0.)old_at_same_time.pose.linear()=Eigen::AngleAxisd((time-beforetime)*w,previous.angular/w).toRotationMatrix()*previous.pose.linear();auto copy=mapper;const auto before=copy.advance(old_at_same_time,robot[a],elapsed),after=mapper.advance(now,robot[a],elapsed);
   const double jump=(after.raw.pose.translation()-before.raw.pose.translation()).norm();
   if(jump>2*c.position_budget)report["same_update_events"].push_back({{"arm",a},{"source_sample_time_before",beforetime},{"source_sample_time_after",time},{"replay_receive_clock",time-start},{"reference_advance_clock",elapsed},{"clock_note","Replay reception only; original network receive clock unavailable. Both branches advanced to IDENTICAL reference time before applying new observation."},{"raw_before",encode(before.raw)},{"raw_after",encode(after.raw)},{"command_before",encode(before.command)},{"command_after",encode(after.command)},{"command_same_update_change_m",(before.command.pose.translation()-after.command.pose.translation()).norm()},{"command_motion_since_previous_sample_m",(after.command.pose.translation()-last.command.pose.translation()).norm()},{"same_update_raw_change_mm",1000*jump},{"conditional_both_sides_minimax_lower_bound_mm",500*jump},{"sampled_after_actual_error_mm",1000*(after.command.pose.translation()-after.raw.pose.translation()).norm()}});
   previous=now;last=after;
  }
 }
 report["conclusion"]="The two-ball lower bound applies ONLY if both left/right targets at the same receive update must meet the budget with a continuous command. Different source sample timestamps alone do not prove this premise. Sampled acceptance remains independently measured; no universal robot infeasibility is asserted.";
 std::ofstream out(argv[4]);out<<report.dump(2)<<'\n';
}
