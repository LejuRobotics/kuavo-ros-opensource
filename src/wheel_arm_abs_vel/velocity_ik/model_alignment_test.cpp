// Offline canonical-model regression. Never publishes or drives a controller.
#include "model_pinocchio.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
using json=nlohmann::json;
void check(bool ok,const std::string& reason){if(!ok)throw std::runtime_error(reason);}
int main(int argc,char** argv){
 try{
  check(argc==3,"usage: model_alignment_test assets_directory private_contract");
  const std::string assets=argv[1];
  json cfg;std::ifstream(assets+"/config/kuavo_v62/kuavo.json")>>cfg;
  json overlay;std::ifstream(argv[2])>>overlay;cfg.update(overlay);
  const std::string path=assets+"/models/"+cfg.at("velocity_ik_urdf").get<std::string>();
  auto fixed=cfg.at("velocity_ik_fixed_joints").get<std::vector<std::string>>();
  velocity_ik::ModelPinocchio model(path,"base_link",fixed);
  const Eigen::Vector3d offset(.017,-.023,.031);
  const auto rotation=Eigen::AngleAxisd(.23,Eigen::Vector3d::UnitY()).toRotationMatrix();
  for(auto side:{"l","r"})model.addTcp(std::string("test_")+side,
      std::string("zarm_")+side+"7_end_effector",offset,rotation);
  model.Finalize();check(model.num_positions()==21&&model.num_velocities()==21,"21D reduction");
  auto context=model.CreateDefaultContext();
  velocity_ik::ModelPinocchio legacy(assets+"/models/"+cfg.at("arm_urdf").get<std::string>(),"base_link");
  legacy.Finalize();auto legacy_context=legacy.CreateDefaultContext();
  check(legacy.num_positions()==21&&legacy.num_velocities()==21,"legacy construction compatibility");
  // Independent full URDF parse: continuous wheel and head coordinates remain
  // present and neutral here. Map every controlled joint by name, not index.
  pinocchio::Model full;pinocchio::urdf::buildModel(path,full);pinocchio::Data data(full);
  auto neutral=pinocchio::neutral(full);
  std::vector<std::string> names{"whole_body_base_x_joint","whole_body_base_y_joint","whole_body_base_yaw_joint",
    "knee_joint","leg_joint","waist_pitch_joint","waist_yaw_joint"};
  for(auto side:{"l","r"})for(int j=1;j<=7;++j)names.push_back(std::string("zarm_")+side+std::to_string(j)+"_joint");
  double max_pose=0,max_rot=0,max_jac=0,max_alignment_residual=0;
  for(int sample=0;sample<128;++sample){
   auto q=model.GetPositions(*context);q.setZero();auto qf=neutral;
   for(size_t i=0;i<names.size();++i){
    auto joint=model.GetJointByName(names[i]);double value=.25*std::sin(.17*sample+.37*i);
    if(names[i]=="knee_joint")value=.4+.3*std::sin(.17*sample);
    if(names[i]=="leg_joint")value=-.6+.4*std::cos(.17*sample);
    q[joint.position_start()]=value;
    if(full.existJointName(names[i]))qf[full.joints[full.getJointId(names[i])].idx_q()]=value;
   }
   model.SetPositions(context.get(),q);pinocchio::framesForwardKinematics(full,data,qf);
   auto qold=legacy.GetPositions(*legacy_context);
   for(const auto& name:names)qold[legacy.GetJointByName(name).position_start()]=q[model.GetJointByName(name).position_start()];
   legacy.SetPositions(legacy_context.get(),qold);
   Eigen::Isometry3d world=Eigen::Isometry3d::Identity();
   world.translation()<<q[model.GetJointByName(names[0]).position_start()],q[model.GetJointByName(names[1]).position_start()],0;
   world.linear()=Eigen::AngleAxisd(q[model.GetJointByName(names[2]).position_start()],Eigen::Vector3d::UnitZ()).toRotationMatrix();
   for(auto frame:{"base_link","waist_yaw_link","zarm_l1_link","zarm_r1_link","zarm_l4_link","zarm_r4_link","zarm_l7_end_effector","zarm_r7_end_effector"}){
    check(full.existFrame(frame),std::string("missing full frame ")+frame);
    const auto expected=data.oMf[full.getFrameId(frame)];auto actual=model.GetFrameByName(frame).CalcPoseInWorld(*context);
    max_pose=std::max(max_pose,(actual.p-world*expected.translation()).norm());
    max_rot=std::max(max_rot,(actual.R.matrix()-world.linear()*expected.rotation()).norm());
    if(std::string(frame)!="base_link"){
      // Current dev already aligns both S62 base heights at 0.31 m.
      auto before=legacy.GetFrameByName(frame).CalcPoseInWorld(*legacy_context);
      max_alignment_residual=std::max(max_alignment_residual,
          (actual.p-before.p).norm());
    }
   }
   if(sample%8==0)for(auto side:{"l","r"})for(auto relative:{"universe","waist_yaw_link"}){
    auto frame=model.GetFrameByName(std::string("test_")+side),ref=model.GetFrameByName(relative);
    Eigen::MatrixXd J;model.spatialJacobian(*context,frame,Eigen::Vector3d::Zero(),ref,ref,&J);
    const auto p0=frame.CalcPose(*context,ref);const double eps=1e-7;
    for(int k=0;k<21;++k){
     auto plus=q;plus[k]+=eps;model.SetPositions(context.get(),plus);auto p1=frame.CalcPose(*context,ref);
     auto minus=q;minus[k]-=eps;model.SetPositions(context.get(),minus);auto pm=frame.CalcPose(*context,ref);
     Eigen::Matrix3d skew=(p1.R.matrix()-pm.R.matrix())/(2*eps)*p0.R.matrix().transpose();
     Eigen::Matrix<double,6,1> fd;fd<<skew(2,1),skew(0,2),skew(1,0),(p1.p-pm.p)/(2*eps);
     max_jac=std::max(max_jac,(fd-J.col(k)).cwiseAbs().maxCoeff());
    }
    model.SetPositions(context.get(),q);
   }
  }
  bool missing=false,duplicate=false;
  try{velocity_ik::ModelPinocchio bad(path,"base_link",{"missing_joint"});}catch(const std::runtime_error&){missing=true;}
  try{auto twice=fixed;twice.push_back(fixed[0]);velocity_ik::ModelPinocchio bad(path,"base_link",twice);}catch(const std::runtime_error&){duplicate=true;}
  check(missing&&duplicate,"invalid reduction must fail");
  check(max_pose<1e-12&&max_rot<1e-12&&max_jac<2e-8&&max_alignment_residual<1e-12,"FK/Jacobian mismatch");
  std::cout<<json({{"legacy_to_main_height_m",0.0},{"max_height_alignment_residual_m",max_alignment_residual},{"samples",128},{"nq",21},{"nv",21},{"max_position_m",max_pose},{"max_rotation_matrix_error",max_rot},{"max_jacobian_error",max_jac},{"invalid_reduction_rejected",true},{"status","PASS"}}).dump(2)<<"\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
