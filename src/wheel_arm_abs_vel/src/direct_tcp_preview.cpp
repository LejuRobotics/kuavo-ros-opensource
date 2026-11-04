#include "wheel_arm_abs_vel/direct_tcp/sample_json.h"
#include <fstream>
#include <iostream>
using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample decode(const Json& a){return sampleFromJson(a);}
Json encode(const Sample& s){Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w(),s.linear.x(),s.linear.y(),s.linear.z(),s.angular.x(),s.angular.y(),s.angular.z()};}
Json vec(const Vec& x){return {x.x(),x.y(),x.z()};}
}
int main(int argc,char** argv){
  if(argc!=4)throw std::runtime_error("direct_tcp_preview CONFIG FIXTURE OUTPUT.jsonl");
  Config c(argv[1]);RobotModel model(c);std::vector<std::string> names;std::vector<double> q;
  for(const auto& x:c.document.at("seed_joints").items()){names.push_back(x.key());q.push_back(x.value());}
  const auto robot=model.evaluate(names,q,std::vector<double>(q.size(),0.));
  std::ifstream input(argv[2]);Json fixture;input>>fixture;const auto& rows=fixture.at("samples");
  const double begin=rows.at(0).at("t");std::array<EngagedMapper,2> maps{
    EngagedMapper(c.arms[0],model.geometry()[0],decode(rows.at(0).at("controller").at(0)),robot[0]),
    EngagedMapper(c.arms[1],model.geometry()[1],decode(rows.at(0).at("controller").at(1)),robot[1])};
  std::ofstream output(argv[3]);std::array<double,2> maxp{0.,0.},maxr{0.,0.};
  for(const auto& row:rows){const double t=row.at("t");Json trace{{"t",t}};
    for(int a=0;a<2;++a){
      try{
        const auto out=maps[a].advance(decode(row.at("controller").at(a)),robot[a],t-begin+c.engage_duration);
        trace["upstream"][a]=out.upstream;trace["upstream_rate"][a]=out.upstream_rate;
        trace["requested_p6"][a]=vec(out.requested_p6);trace["position_cutoff_hz"][a]=out.position_cutoff_hz;
        trace["reference_wrist"][a]=vec(out.reference_wrist);trace["reference_wrist_rate"][a]=vec(out.reference_wrist_rate);trace["final_domain_lower"][a]=vec(out.final_domain_lower);trace["final_domain_upper"][a]=vec(out.final_domain_upper);trace["projected"][a]=encode(out.projected);trace["raw"][a]=encode(out.raw);trace["command"][a]=encode(out.command);trace["elbow"][a]=encode(out.elbow);
        trace["requested_wrist"][a]=vec(out.requested_wrist);trace["applied_wrist"][a]=vec(out.applied_wrist);
        trace["raw_j7"][a]=vec((out.raw.pose*model.geometry()[a].j7_to_tcp.inverse()).translation());
        trace["j7"][a]=vec((out.command.pose*model.geometry()[a].j7_to_tcp.inverse()).translation());
        maxp[a]=std::max(maxp[a],out.position_projection.norm());maxr[a]=std::max(maxr[a],out.orientation_projection.norm());
      }catch(const std::exception& e){std::cerr<<"preview failed at source t="<<t<<", arm="<<a<<": "<<e.what()<<std::endl;throw;}
    }output<<trace.dump()<<'\n';
  }
  std::cout<<Json{{"samples",rows.size()},{"max_position_projection_m",maxp},{"max_orientation_projection_rad",maxr}}.dump(2)<<std::endl;
}
