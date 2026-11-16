// Offline bounded nonlinear witness search; not a replacement for the live QP.
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <Eigen/Cholesky>
#include <fstream>
#include <iostream>
#include <algorithm>
using namespace wheel_arm_abs_vel::direct_tcp;
using Joint=Eigen::Matrix<double,7,1>;using Error=Eigen::Matrix<double,6,1>;
namespace { Sample decode(const Json& x){return sample({x[0],x[1],x[2],x[3],x[4],x[5],x[6]},Vec::Zero(),Vec::Zero());} }
int main(int argc,char**argv){
 if(argc!=5)throw std::invalid_argument("reachability_witness CONFIG ORIGINAL_TRACE DECOMPOSITION_TRACE OUTPUT");
 Config cfg(argv[1]);RobotModel model(cfg);std::ifstream input(argv[2]);Json anchor;std::string line;
 while(std::getline(input,line)){const auto r=Json::parse(line);if(r.value("kind","")=="anchor")break;if(r.value("kind","")=="reference")anchor=r;}
 auto names=anchor.at("joint_names").get<std::vector<std::string>>();auto base=anchor.at("q").get<std::vector<double>>();
 std::array<Json,2> worst;double maxerr[2]={-1,-1};std::ifstream replay(argv[3]);
 while(std::getline(replay,line)){auto r=Json::parse(line);for(int a=0;a<2;++a){const double e=(decode(r["projected"][a]).pose.translation()-decode(r["raw"][a]).pose.translation()).norm();if(e>maxerr[a]){maxerr[a]=e;worst[a]=r;}}}
 Json result{{"scope","Finite independent static witnesses using full URDF, original stopping reserve and retained Mapper margins. Failed search is NOT proof of infeasibility; found poses are NOT a trajectory certificate."}};
 for(int a=0;a<2;++a){std::array<int,7> ids;const auto& g=model.geometry()[a];Joint lower,upper,center;
  for(int j=0;j<7;++j){const std::string name=cfg.document["arms"][a]["arm_joints"][j];ids[j]=std::find(names.begin(),names.end(),name)-names.begin();center[j]=worst[a]["witness_q"][a][j];lower[j]=j<4?g.upstream_lower[j]:g.lower[j-4];upper[j]=j<4?g.upstream_upper[j]:g.upper[j-4];}
  const auto target=decode(worst[a]["raw"][a]);
  const auto fk=[&](const Joint& q){auto full=base;for(int j=0;j<7;++j)full[ids[j]]=q[j];return model.evaluate(names,full,std::vector<double>(full.size(),0.))[a].tcp;};
  const double rotation_scale=cfg.position_budget/cfg.orientation_budget;
  const auto error=[&](const Sample& x)->Error{const Eigen::AngleAxisd aa(target.pose.linear()*x.pose.linear().transpose());Error e;e.head<3>()=target.pose.translation()-x.pose.translation();e.tail<3>()=rotation_scale*aa.angle()*aa.axis();return e;};
  for(double radius:{0.,1.5}){Joint lo=lower,hi=upper;if(radius>0.)for(int j=0;j<4;++j){lo[j]=std::max(lo[j],center[j]-radius*M_PI/180.);hi[j]=std::min(hi[j],center[j]+radius*M_PI/180.);}
   double best=1.e99;Joint bestq=center;int iterations=0;
   for(int seed=0;seed<12;++seed){Joint q=center;if(seed==1)for(int j=0;j<7;++j)q[j]=std::clamp(base[ids[j]],lo[j],hi[j]);if(seed>1)for(int j=0;j<7;++j)q[j]=lo[j]+(.5+.49*std::sin(seed*12.9898+j*78.233))*(hi[j]-lo[j]);
    double damping=1.e-5;
    for(int it=0;it<100;++it){++iterations;const auto x=fk(q);const Error e=error(x);const double score=std::max(e.head<3>().norm()/cfg.position_budget,e.tail<3>().norm()/cfg.position_budget);
     if(score<best){best=score;bestq=q;}if(best<1.e-5)break;
     Eigen::Matrix<double,6,7> J;const double eps=1.e-5;
     for(int j=0;j<7;++j){Joint next=q;next[j]+=eps;const auto y=fk(next);const Eigen::AngleAxisd aa(y.pose.linear()*x.pose.linear().transpose());J.block<3,1>(0,j)=(y.pose.translation()-x.pose.translation())/eps;J.block<3,1>(3,j)=rotation_scale*aa.angle()*aa.axis()/eps;}
     Joint step=(J.transpose()*J+damping*Eigen::Matrix<double,7,7>::Identity()).ldlt().solve(J.transpose()*e);if(step.cwiseAbs().maxCoeff()>.2)step*=.2/step.cwiseAbs().maxCoeff();bool accepted=false;
     for(double scale:{1.,.5,.25,.125,.0625}){Joint next=(q+scale*step).cwiseMax(lo).cwiseMin(hi);if(error(fk(next)).squaredNorm()<e.squaredNorm()-1.e-15){q=next;accepted=true;damping=std::max(1.e-8,damping*.5);break;}}
     if(!accepted){damping*=10.;if(damping>1.e3)break;}
    }if(best<1.e-5)break;
   }
   const auto check=fk(bestq);const Error e=error(check);Json qjson=Json::array();for(int j=0;j<7;++j)qjson.push_back(bestq[j]);
   result["arms"][a].push_back({{"t",worst[a]["t"]},{"static_mapper_error_mm",1000*maxerr[a]},{"upstream_box_about_mapper_deg",radius},{"box_note",radius==0.?"No extra upstream box":"Each upstream joint confined around Mapper static witness; not a time-window guarantee"},{"iterations",iterations},{"position_error_mm",1000*e.head<3>().norm()},{"orientation_error_deg",e.tail<3>().norm()/rotation_scale*180/M_PI},{"within_budget",best<=1.},{"normalized_budget_score",best},{"max_upstream_change_deg",(bestq-center).head<4>().cwiseAbs().maxCoeff()*180/M_PI},{"q",qjson},{"within_domain",(bestq.array()>=lo.array()-1.e-10).all()&&(bestq.array()<=hi.array()+1.e-10).all()}});
  }
 }
 std::ofstream out(argv[4]);out<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';
}
