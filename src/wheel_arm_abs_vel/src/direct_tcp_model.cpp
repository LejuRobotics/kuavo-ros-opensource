#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include "../velocity_ik/operating_envelope.h"
#include <pinocchio/parsers/urdf.hpp>
#include <filesystem>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace wheel_arm_abs_vel::direct_tcp {
namespace {
void require(bool condition,const std::string& reason) {
  if(!condition) throw std::invalid_argument(reason);
}
Json read(const std::string& file) {
  std::ifstream in(file); require(in.good(),"Missing configuration: "+file);
  Json j; in>>j; return j;
}
std::string sha256(const std::string& file) {
  std::ifstream in(file,std::ios::binary);require(in.good(),"Missing URDF: "+file);
  std::string bytes((std::istreambuf_iterator<char>(in)),{});
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size(),digest);
  std::ostringstream out;for(auto c:digest)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c);
  return out.str();
}
Pose pose(const pinocchio::SE3& x) {
  Pose t=Pose::Identity();t.linear()=x.rotation();t.translation()=x.translation();return t;
}
Json xyz(const Vec& x) { return {x.x(),x.y(),x.z()}; }
}
Config::Config(const std::string& path):document(read(path)) {
  const auto& j=document;
  require(j.at("mapping_contract")=="V5_WRIST_PIVOT_TCP","Explicit V5 mapping contract required");
  require(j.at("mode")=="DIRECT_CONTROLLER_TCP","Explicit DIRECT_CONTROLLER_TCP required");
  urdf=j.at("urdf");model_sha256=j.at("model_sha256");
  if(urdf.rfind("package://",0)==0) {
    const auto slash=urdf.find('/',10);require(slash!=std::string::npos,"Invalid URDF package URI");
    const auto package=ros::package::getPath(urdf.substr(10,slash-10));
    require(!package.empty(),"URDF package not found");urdf=package+urdf.substr(slash);
  } else if(std::filesystem::path(urdf).is_relative()) {
    urdf=(std::filesystem::path(path).parent_path()/urdf).lexically_normal().string();
  }
  require(sha256(urdf)==model_sha256,"Model SHA256 mismatch");
  calibration=j.at("calibration_status");input_provenance=j.at("input_provenance");
  input_frame=j.at("input_frame");output_frame=j.at("output_frame");
  simulation_only=j.at("simulation_only");budget_status=j.at("budget_status");
  require(j.at("body_contract")=="NONE_FIXED_BASE_CONTROLLER_ONLY" || (!simulation_only && j.at("body_contract")=="FIXED_BASE_CONTROLLER_WITH_CHEST_ADMISSION_ONLY"),"Explicit fixed-base input contract required");
  position_budget=j.at("position_budget_m");orientation_budget=j.at("orientation_budget_rad");
  engage_duration=j.at("engage_duration_s");source_max_age=j.at("source_max_age_s");source_gap_timeout=j.at("source_gap_timeout_s");
  require(std::isfinite(source_gap_timeout) && source_gap_timeout>0.,"Invalid source gap timeout");
  require(std::isfinite(source_max_age) && source_max_age>0.,"Invalid source age contract");
  require(!input_provenance.empty() && !input_frame.empty() && output_frame=="direct_tcp_base","Incomplete frame/provenance contract");
  require(budget_status=="PROVISIONAL_ENGINEERING_BUDGET", "Explicit provisional budget required");
  if (!simulation_only) {
    require(j.at("deployment_contract")=="HOST46_400062_CONTROLLER_TRANSPORT_V1", "Explicit hardware deployment contract required");
    require(input_provenance=="QUEST_CONTROLLER_MODE_TRANSPORT_SDK_ORIGIN_UNVERIFIED", "Hardware transport provenance required");
  }
  require(std::isfinite(position_budget) && std::isfinite(orientation_budget) && std::isfinite(engage_duration) && position_budget>0. && orientation_budget>0. && engage_duration>0.,"Invalid budgets/engage duration");
  for(int a=0;a<2;++a) {
    const auto& x=j.at("arms").at(a);auto& c=arms[a];
    controlled_frames[a]=x.at("controlled_frame");
    c.controller_to_virtual=transform(x.at("controller_to_virtual"));
    c.alignment=transform(x.at("tracking_to_base_axes")).linear();
    const auto weights=x.at("shoulder_rotation_weights").get<std::array<double,3>>();c.shoulder_rotation_weights=Eigen::Map<const Vec>(weights.data());
    require(c.shoulder_rotation_weights.allFinite() && (c.shoulder_rotation_weights.array()>0.).all(),"Invalid strictly positive Mapper shoulder metric");
    c.wrist_reference_cutoff=x.at("wrist_reference_cutoff_hz");
    require(std::isfinite(c.wrist_reference_cutoff) && c.wrist_reference_cutoff>0.,"Invalid wrist reference cutoff");
    c.reference_cutoff=x.at("reference_cutoff_hz");require(std::isfinite(c.reference_cutoff) && c.reference_cutoff>0.,"Invalid explicit reference cutoff");
    c.reference_fast_cutoff=x.value("reference_fast_cutoff_hz",0.);
    c.reference_fast_start=x.value("reference_fast_error_start_m",.03);
    c.reference_fast_full=x.value("reference_fast_error_full_m",.08);
    c.feasible_reference_fast_cutoff=x.value("feasible_reference_fast_cutoff_hz",0.);
    require(std::isfinite(c.feasible_reference_fast_cutoff) && (c.feasible_reference_fast_cutoff==0. || c.feasible_reference_fast_cutoff>=std::max(c.reference_cutoff,c.reference_fast_cutoff)),"Invalid feasible-reference bandwidth");
    require(std::isfinite(c.reference_fast_cutoff) && (c.reference_fast_cutoff==0. || c.reference_fast_cutoff>=c.reference_cutoff) &&
        std::isfinite(c.reference_fast_start) && std::isfinite(c.reference_fast_full) &&
        c.reference_fast_start>0. && c.reference_fast_full>c.reference_fast_start,"Invalid reference bandwidth schedule");
    c.reach_margin=x.at("reach_margin_m");c.reach_width=x.at("reach_width_m");c.wrist_width=x.at("wrist_width_rad");
    require(std::isfinite(c.reach_margin) && std::isfinite(c.reach_width) && std::isfinite(c.wrist_width) && c.reach_margin>c.reach_width && c.reach_width>0. && c.wrist_width>0.,"Invalid projection widths");
  }
}
struct RobotModel::Impl {
  pinocchio::Model model;
  mutable pinocchio::Data data;
  Eigen::VectorXd q,dq;
  std::array<WristGeometry,2> geometry;
  std::array<pinocchio::FrameIndex,2> tcp,forearm,j7,shoulder;
  std::array<std::array<pinocchio::JointIndex,3>,2> shoulder_joints;
  Json audit;
  pinocchio::FrameIndex torso;
  explicit Impl(const Config& c):model(load(c)),data(model),q(pinocchio::neutral(model)),dq(Eigen::VectorXd::Zero(model.nv)) {
    require(model.nq==model.nv,"Scalar-joint model required");
    for(const auto& item:c.document.at("seed_joints").items()) {
      require(model.existJointName(item.key()),"Unknown seed joint: "+item.key());
      q[model.joints[model.getJointId(item.key())].idx_q()]=item.value().get<double>();
    }
    if (c.document.contains("position_limits")) {
      for (const auto& item:c.document.at("position_limits").items()) {
        require(model.existJointName(item.key()), "Unknown operating joint");
        const int k=model.joints[model.getJointId(item.key())].idx_q();
        model.lowerPositionLimit[k]=std::max(model.lowerPositionLimit[k],item.value().at(0).get<double>());
        model.upperPositionLimit[k]=std::min(model.upperPositionLimit[k],item.value().at(1).get<double>());
        require(model.lowerPositionLimit[k]<model.upperPositionLimit[k], "Empty operating joint interval");
      }
    }
    update();
    require(model.existFrame(c.document.at("torso_frame")),"Missing torso frame");torso=model.getFrameId(c.document.at("torso_frame"));
    for(int a=0;a<2;++a) {
      const auto& x=c.document.at("arms").at(a);auto& g=geometry[a];
      auto frame=[&](const std::string& n) {require(model.existFrame(n),"Missing model frame: "+n);return model.getFrameId(n);};
      shoulder[a]=frame(x.at("shoulder_frame"));forearm[a]=frame(x.at("forearm_frame"));
      tcp[a]=frame(x.at("tcp_frame"));j7[a]=frame(x.at("j7_frame"));
      const auto joint_names=x.at("arm_joints").get<std::vector<std::string>>();
      require(joint_names.size()==7,"Seven explicit arm joints required");
      std::array<pinocchio::JointIndex,7> ids;
      for(int k=0;k<7;++k){require(model.existJointName(joint_names[k]),"Missing arm joint");ids[k]=model.getJointId(joint_names[k]);}
      for(int k=0;k<3;++k)shoulder_joints[a][k]=ids[k];
      // Raw URDF vectors; no historical robot dimensions occur in this code.
      g.upper_vector=model.jointPlacements[ids[3]].translation();g.upper_length=g.upper_vector.norm();
      g.elbow_lower=model.lowerPositionLimit[model.joints[ids[3]].idx_q()];g.elbow_upper=model.upperPositionLimit[model.joints[ids[3]].idx_q()];
      const auto f5=model.jointPlacements[ids[4]],f6=model.jointPlacements[ids[5]],f7=model.jointPlacements[ids[6]];
      require(f5.rotation().isApprox(Rot::Identity(),1.e-12) && f6.rotation().isApprox(Rot::Identity(),1.e-12) && f7.rotation().isApprox(Rot::Identity(),1.e-12),"Unsupported wrist placement rotations");
      require(f6.translation().cross(Vec::UnitZ()).norm()<1.e-12,"Unsupported J5→J6 lever axis");
      g.forearm_vector=f5.translation()+f6.translation();g.forearm_length=g.forearm_vector.norm();
      g.j6_to_j7=f7.translation();
      g.shoulder=data.oMf[shoulder[a]].translation();
      g.j7_to_tcp=pose(data.oMf[j7[a]].inverse()*data.oMf[tcp[a]]);
      require(g.j7_to_tcp.matrix().isApprox(transform(x.at("j7_to_tcp")).matrix(),1.e-12),"Tool transform disagrees with full URDF");
      // Confirm supported Z-X-Y wrist axes from the actual Pinocchio Jacobian.
      Eigen::Matrix<double,6,Eigen::Dynamic> J(6,model.nv);J.setZero();
      pinocchio::getFrameJacobian(model,data,j7[a],pinocchio::LOCAL_WORLD_ALIGNED,J);
      const std::array<int,4> axes{{1,0,2,1}};
      for(int k=0;k<4;++k)require(J.bottomRows<3>().col(model.joints[ids[k]].idx_v()).isApprox(data.oMi[ids[k]].rotation()*Vec::Unit(axes[k]),1.e-10),"Unsupported upstream axes: require Y-X-Z-Y");
      for(int k=1;k<4;++k)require(model.jointPlacements[ids[k]].rotation().isApprox(Rot::Identity(),1.e-12),"Unsupported upstream placement rotation");
      for(int k=1;k<3;++k)require(model.jointPlacements[ids[k]].translation().norm()<1.e-12,"Unsupported separated shoulder axes");
      const Rot r4=data.oMf[forearm[a]].rotation();
      const Rot r5=data.oMi[ids[4]].rotation(),r6=data.oMi[ids[5]].rotation();
      require(J.bottomRows<3>().col(model.joints[ids[4]].idx_v()).isApprox(r4*Vec::UnitZ(),1.e-10) &&
          J.bottomRows<3>().col(model.joints[ids[5]].idx_v()).isApprox(r5*Vec::UnitX(),1.e-10) &&
          J.bottomRows<3>().col(model.joints[ids[6]].idx_v()).isApprox(r6*Vec::UnitY(),1.e-10),"Unsupported wrist axes: require Z-X-Y");
      // Read-only mirror of the existing proximal stopping contract. These
      // values constrain Mapper geometry; they never write solver parameters.
      for(int k=0;k<4;++k) {
        const auto& p=x.at("upstream_operating").at(k);velocity_ik::OperatingJoint op;
        const int qi=model.joints[ids[k]].idx_q();
        op.q=q[qi];op.v=0.;op.current_acceleration=0.;
        op.lower=model.lowerPositionLimit[qi]+p.at("position_margin").get<double>();
        op.upper=model.upperPositionLimit[qi]-p.at("position_margin").get<double>();
        op.dt=p.at("dt");op.vmax=p.at("vmax");op.acceleration=p.at("acceleration");
        op.braking=p.at("braking");op.outward=p.at("outward");op.delay=p.at("delay");
        op.lower_guard=p.at("guard");op.upper_guard=op.lower_guard;op.jerk=p.at("jerk");
        op.extra_position=p.at("extra_position");op.extra_time=p.at("extra_time");op.separate_stop=p.at("separate_stop");
        require(velocity_ik::validOperatingJoint(op),"Invalid upstream operating contract");
        g.upstream_lower[k]=op.lower+velocity_ik::operatingStopReserve(op,0.,false,true);
        g.upstream_upper[k]=op.upper-velocity_ik::operatingStopReserve(op,0.,true,true);
        require(op.delay>0. && op.jerk>0. && !op.separate_stop && op.acceleration==op.braking && op.outward==op.braking,"Unsupported upstream stopping profile");
        const double ramp=(op.outward+op.braking)/op.jerk;
        g.upstream_stopping_linear[k]=op.delay+ramp+(op.outward*op.delay+.5*(op.outward-op.braking)*ramp)/op.braking+op.extra_time;
        g.upstream_stopping_quadratic[k]=.5/op.braking;g.upstream_vmax[k]=op.vmax;
        for(double v:{.1,.5,1.,2.,4.})require(std::abs(velocity_ik::operatingStopReserve(op,v,true,true)-velocity_ik::operatingStopReserve(op,0.,true,true)-g.upstream_stopping_linear[k]*v-g.upstream_stopping_quadratic[k]*v*v)<1.e-12,"Upstream stopping branch mismatch");
        require(g.upstream_upper[k]-g.upstream_lower[k]>.07,"Empty shoulder/elbow domain");
        audit["arms"][a]["upstream"][k]={{"lower",g.upstream_lower[k]},{"upper",g.upstream_upper[k]},
          {"limit_source","configured model limits + unchanged zero-speed operating reserve"}};
      }
      for(int k=0;k<3;++k) {
        const auto& p=x.at("operating").at(k);velocity_ik::OperatingJoint op;
        const int qi=model.joints[ids[k+4]].idx_q();
        op.q=q[qi];op.v=0.;op.current_acceleration=0.;
        op.lower=model.lowerPositionLimit[qi]+p.at("position_margin").get<double>();
        op.upper=model.upperPositionLimit[qi]-p.at("position_margin").get<double>();
        op.dt=p.at("dt");op.vmax=p.at("vmax");op.acceleration=p.at("acceleration");
        op.braking=p.at("braking");op.outward=p.at("outward");op.delay=p.at("delay");
        op.lower_guard=p.at("guard");op.upper_guard=op.lower_guard;op.jerk=p.at("jerk");
        op.extra_position=p.at("extra_position");op.extra_time=p.at("extra_time");op.separate_stop=p.at("separate_stop");
        require(velocity_ik::validOperatingJoint(op),"Invalid explicit operating contract");
        const double lower_reserve=velocity_ik::operatingStopReserve(op,0.,false,true);
        const double upper_reserve=velocity_ik::operatingStopReserve(op,0.,true,true);
        const double margin=p.at("mapper_margin");require(std::isfinite(margin) && margin>=0.,"Invalid Mapper margin");
        g.lower[k]=op.lower+lower_reserve+margin;g.upper[k]=op.upper-upper_reserve-margin;
        require(g.upper[k]-g.lower[k]>2*c.arms[a].wrist_width,"Empty wrist projection free range");
        require(op.delay>0. && op.jerk>0. && !op.separate_stop && op.acceleration==op.braking && op.outward==op.braking,"Unsupported analytic stopping-preview profile");
        const double ramp=(op.outward+op.braking)/op.jerk;
        g.stopping_linear[k]=op.delay+ramp+(op.outward*op.delay+.5*(op.outward-op.braking)*ramp)/op.braking+op.extra_time;
        g.stopping_quadratic[k]=.5/op.braking;
        // This is the analytic polynomial of the ORIGINAL stopping function,
        // not a changed controller reserve. Validate its active branch once.
        for(double v:{.1,.5,1.,2.,4.})require(std::abs(velocity_ik::operatingStopReserve(op,v,true,true)-upper_reserve-g.stopping_linear[k]*v-g.stopping_quadratic[k]*v*v)<1.e-12,"Stopping preview branch mismatch");
        audit["arms"][a]["wrist"][k]={{"lower",g.lower[k]},{"upper",g.upper[k]},
          {"zero_speed_upper_before_mapper_margin",op.upper-upper_reserve},
          {"static_reserve",upper_reserve},{"speed_linear",g.stopping_linear[k]},{"speed_quadratic",g.stopping_quadratic[k]},{"limit_source","URDF + original operatingStopReserve/jerkLimitedStopDistance"}};
      }
      audit["arms"][a]["upper_length_m"]=g.upper_length;
      audit["arms"][a]["forearm_length_m"]=g.forearm_length;
      audit["arms"][a]["j6_to_j7_m"]=xyz(g.j6_to_j7);
      audit["arms"][a]["shoulder_m"]=xyz(g.shoulder);
    }
    audit["model_sha256"]=c.model_sha256;
    audit["projection_domain"]="Static zero-speed comfort domain; not a dynamic feasibility certificate";
  }
  static pinocchio::Model load(const Config& c) {
    auto tree=::urdf::parseURDFFile(c.urdf);require(bool(tree),"Cannot parse pinned URDF");
    for(const auto& n:c.document.at("fixed_joints")) {
      const auto name=n.get<std::string>();require(tree->joints_.count(name)==1,"Missing configured fixed joint");
      tree->joints_.at(name)->type=::urdf::Joint::FIXED;
    }
    pinocchio::Model m;pinocchio::urdf::buildModel(tree,m);return m;
  }
  void update(){pinocchio::computeJointJacobians(model,data,q);pinocchio::updateFramePlacements(model,data);}
  Sample state(pinocchio::FrameIndex f) const {
    Eigen::Matrix<double,6,Eigen::Dynamic> j(6,model.nv);j.setZero();
    pinocchio::getFrameJacobian(model,data,f,pinocchio::LOCAL_WORLD_ALIGNED,j);
    const Eigen::Matrix<double,6,1> v=j*dq;return {pose(data.oMf[f]),v.head<3>(),v.tail<3>()};
  }
};
RobotModel::RobotModel(const Config& c):impl_(std::make_unique<Impl>(c)){}
RobotModel::~RobotModel()=default;
const std::array<WristGeometry,2>& RobotModel::geometry()const{return impl_->geometry;}
const Json& RobotModel::audit()const{return impl_->audit;}
std::array<RobotArm,2> RobotModel::evaluate(const std::vector<std::string>& names,const std::vector<double>& q,const std::vector<double>& dq) {
  require(names.size()==q.size() && q.size()==dq.size(),"Joint state size mismatch");
  for(std::size_t k=0;k<names.size();++k) {
    if(names[k]=="base_x" || names[k]=="base_y" || names[k]=="base_yaw") {
      require(std::abs(q[k])<1.e-6 && std::abs(dq[k])<1.e-6,"Fixed-base trial requires zero base coordinates");continue;
    }
    require(impl_->model.existJointName(names[k]) && std::isfinite(q[k]) && std::isfinite(dq[k]),"Invalid external robot state");
    const auto& j=impl_->model.joints[impl_->model.getJointId(names[k])];
    impl_->q[j.idx_q()]=q[k];impl_->dq[j.idx_v()]=dq[k];
  }
  impl_->update();std::array<RobotArm,2> out;
  for(int a=0;a<2;++a){const auto f=impl_->state(impl_->forearm[a]);
    Eigen::Matrix<double,6,Eigen::Dynamic> J(6,impl_->model.nv);J.setZero();pinocchio::getFrameJacobian(impl_->model,impl_->data,impl_->forearm[a],pinocchio::LOCAL_WORLD_ALIGNED,J);Rot axes;
    for(int k=0;k<3;++k)axes.col(k)=J.bottomRows<3>().col(impl_->model.joints[impl_->shoulder_joints[a][k]].idx_v());
    out[a]={impl_->state(impl_->tcp[a]),f.pose.linear(),f.angular,f,impl_->state(impl_->shoulder[a]),axes};
    const auto j1=impl_->shoulder_joints[a][0];
    out[a].shoulder_parent=(impl_->data.oMi[impl_->model.parents[j1]]*impl_->model.jointPlacements[j1]).rotation();
  }
  return out;
}
Sample RobotModel::torsoSample()const{return impl_->state(impl_->torso);}
Pose RobotModel::torso()const{return pose(impl_->data.oMf[impl_->torso]);}
std::array<Sample,2> RobotModel::joint7()const{return {impl_->state(impl_->j7[0]),impl_->state(impl_->j7[1])};}
}  // namespace wheel_arm_abs_vel::direct_tcp
