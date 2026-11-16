#include "model_pinocchio.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <set>
#include <fstream>
#include <sstream>
#include <map>
#include <stdexcept>
namespace velocity_ik {
struct ModelContext::Impl {
  pinocchio::Data data;
  Eigen::VectorXd q;
  bool positions_valid=false;
  explicit Impl(const pinocchio::Model& m):data(m),q(pinocchio::neutral(m)){}
};
ModelContext::ModelContext()=default;
ModelContext::~ModelContext()=default;
struct ModelPinocchio::Impl {
  pinocchio::Model model;
  std::map<std::string,Frame> frames;
};
ModelPinocchio::ModelPinocchio(const std::string& urdf,const std::string& root,
    const std::vector<std::string>& fixed_joints):impl_(new Impl) {
  std::ifstream input(urdf);
  if(!input) throw std::runtime_error("Cannot read URDF: "+urdf);
  std::string xml((std::istreambuf_iterator<char>(input)),{});
  const auto pos=xml.rfind("</robot>");
  if(pos==std::string::npos) throw std::runtime_error("Invalid URDF robot");
  // Same virtual x/y/yaw chain as D; fixed by the 18-axis layout. It keeps
  // full-state/wire identities explicit without using a quaternion root.
  std::string chain=R"(<link name="whole_body_fixed_world"/>
<link name="whole_body_base_x_link"/><link name="whole_body_base_y_link"/>
<joint name="whole_body_base_x_joint" type="prismatic"><parent link="whole_body_fixed_world"/><child link="whole_body_base_x_link"/><axis xyz="1 0 0"/><limit lower="-1000" upper="1000" effort="1" velocity="1"/></joint>
<joint name="whole_body_base_y_joint" type="prismatic"><parent link="whole_body_base_x_link"/><child link="whole_body_base_y_link"/><axis xyz="0 1 0"/><limit lower="-1000" upper="1000" effort="1" velocity="1"/></joint>
<joint name="whole_body_base_yaw_joint" type="revolute"><parent link="whole_body_base_y_link"/><child link=")"+root+R"("/><axis xyz="0 0 1"/><limit lower="-1000000" upper="1000000" effort="1" velocity="1"/></joint>)";
  xml.insert(pos,chain);
  auto tree = ::urdf::parseURDF(xml);
  if (!tree) throw std::runtime_error("Cannot parse URDF: " + urdf);
  // Match WBC: fix the named wheel/head joints in the parsed URDF before
  // building Pinocchio. This also retains URDF joint/link frame identities.
  std::set<std::string> locked;
  for (const auto& name : fixed_joints) {
    const auto it = tree->joints_.find(name);
    if (it == tree->joints_.end())
      throw std::runtime_error("Missing fixed model joint: " + name);
    if (!locked.insert(name).second)
      throw std::runtime_error("Duplicate fixed model joint: " + name);
    it->second->type = ::urdf::Joint::FIXED;
  }
  pinocchio::urdf::buildModel(tree, impl_->model);
  if(impl_->model.nq!=impl_->model.nv) throw std::runtime_error("Expected scalar joint coordinates");
}
ModelPinocchio::~ModelPinocchio()=default;
void ModelPinocchio::addTcp(const std::string& name,const std::string& parent,const Eigen::Vector3d& t,const Eigen::Matrix3d& R){
  auto& m=impl_->model;
  if(!m.existFrame(parent)) throw std::runtime_error("Missing TCP parent "+parent);
  const auto id=m.getFrameId(parent); const auto f=m.frames[id];
  m.addFrame(pinocchio::Frame(name,f.parent,id,f.placement*pinocchio::SE3(R,t),pinocchio::OP_FRAME));
}
void ModelPinocchio::Finalize(){
  for(unsigned long i=0;i<impl_->model.frames.size();++i)
    impl_->frames.emplace(impl_->model.frames[i].name,Frame{this,i});
}
int ModelPinocchio::num_positions()const{return impl_->model.nq;}
int ModelPinocchio::num_velocities()const{return impl_->model.nv;}
std::unique_ptr<ModelContext> ModelPinocchio::CreateDefaultContext()const{
  auto c=std::make_unique<ModelContext>();c->impl=std::make_unique<ModelContext::Impl>(impl_->model);
  SetPositions(c.get(),c->impl->q);return c;
}
Eigen::VectorXd ModelPinocchio::GetPositions(const ModelContext& c)const{return c.impl->q;}
void ModelPinocchio::SetPositions(ModelContext* c,const Eigen::VectorXd& q)const{
  if(q.size()!=impl_->model.nq||!q.allFinite())throw std::invalid_argument("Invalid Pinocchio q");
  if(c->impl->positions_valid && (c->impl->q.array()==q.array()).all())return;
  c->impl->q=q;
  c->impl->positions_valid=true;
  pinocchio::computeJointJacobians(impl_->model,c->impl->data,q);
  pinocchio::updateFramePlacements(impl_->model,c->impl->data);
}
Joint ModelPinocchio::GetJointByName(const std::string& name)const{
  const auto& m=impl_->model;
  if(!m.existJointName(name))throw std::runtime_error("Missing joint "+name);
  const auto& j=m.joints[m.getJointId(name)];
  return {j.idx_q(),j.idx_v(),j.nq(),j.nv(),m.lowerPositionLimit.segment(j.idx_q(),j.nq()),m.upperPositionLimit.segment(j.idx_q(),j.nq())};
}
const Frame& ModelPinocchio::GetFrameByName(const std::string& name)const{return impl_->frames.at(name);}
const Frame& ModelPinocchio::world_frame()const{return impl_->frames.at("universe");}
Pose ModelPinocchio::pose(const ModelContext& c,const Frame& f,const Frame& a)const{
  const auto p=c.impl->data.oMf[a.index].inverse()*c.impl->data.oMf[f.index];
  return {p.translation(),{p.rotation()}};
}
Pose Frame::CalcPose(const ModelContext& c,const Frame& a)const{return model->pose(c,*this,a);}
Pose Frame::CalcPoseInWorld(const ModelContext& c)const{return CalcPose(c,model->world_frame());}
void ModelPinocchio::spatialJacobian(const ModelContext& c,const Frame& f,const Eigen::Vector3d& point,const Frame& a,const Frame& e,Eigen::MatrixXd* out)const{
  const auto& m=impl_->model;auto& d=c.impl->data;
  Eigen::Matrix<double,6,Eigen::Dynamic> jf(6,m.nv),ja(6,m.nv);jf.setZero();ja.setZero();
  pinocchio::getFrameJacobian(m,d,f.index,pinocchio::LOCAL_WORLD_ALIGNED,jf);
  pinocchio::getFrameJacobian(m,d,a.index,pinocchio::LOCAL_WORLD_ALIGNED,ja);
  const Eigen::Vector3d off=d.oMf[f.index].rotation()*point;
  const Eigen::Vector3d delta=d.oMf[f.index].translation()+off-d.oMf[a.index].translation();
  const Eigen::Matrix3d re=d.oMf[e.index].rotation().transpose();
  out->resize(6,m.nv);
  out->topRows(3)=re*(jf.bottomRows(3)-ja.bottomRows(3));
  out->bottomRows(3)=re*(jf.topRows(3)-pinocchio::skew(off)*jf.bottomRows(3)-ja.topRows(3)+pinocchio::skew(delta)*ja.bottomRows(3));
}
void ModelPinocchio::translationJacobian(const ModelContext& c,const Frame& f,const Eigen::Vector3d& p,const Frame& a,const Frame& e,Eigen::MatrixXd* out)const{
 Eigen::MatrixXd j;spatialJacobian(c,f,p,a,e,&j);*out=j.bottomRows(3);
}
}
