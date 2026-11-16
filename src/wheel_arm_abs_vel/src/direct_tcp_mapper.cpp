#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>

namespace wheel_arm_abs_vel::direct_tcp {
Control controlFromWire(std::uint8_t value) {
  switch(value) {
    case 0:return Control::Inactive;
    case 1:return Control::Armed;
    case 2:return Control::Active;
    default:std::fputs("Unsupported direct control state; only INACTIVE/ARMED/ACTIVE are accepted\n",stderr);std::abort();
  }
}
Rot rotation(int axis, double angle) {
  return Eigen::AngleAxisd(angle, Vec::Unit(axis)).toRotationMatrix();
}
Sample sample(const std::array<double,7>& x, const Vec& v, const Vec& w) {
  Eigen::Map<const Eigen::Matrix<double,7,1>> values(x.data());
  Eigen::Quaterniond q(x[6],x[3],x[4],x[5]);
  if (!values.allFinite() || !v.allFinite() || !w.allFinite() ||
      std::abs(q.squaredNorm()-1.) > 1.e-6)
    throw std::invalid_argument("INVALID_INPUT: nonfinite sample or nonunit quaternion");
  Pose p=Pose::Identity(); p.linear()=q.toRotationMatrix();
  p.translation()=values.head<3>();
  return {p,v,w};
}
Pose transform(const Json& matrix) {
  Eigen::Matrix4d m;
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) m(i,j)=matrix.at(i).at(j).get<double>();
  if (!m.allFinite() || !m.row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1.e-12) ||
      !(m.topLeftCorner<3,3>().transpose()*m.topLeftCorner<3,3>()).isApprox(Rot::Identity(),1.e-10) ||
      std::abs(m.topLeftCorner<3,3>().determinant()-1.)>1.e-10)
    throw std::invalid_argument("Invalid configured SE3");
  Pose p; p.matrix()=m; return p;
}
ScalarProjection softProject(double x,double lo,double hi,double b) {
  if (x<=lo-b) return {lo,0.};
  if (x<lo+b) { const double u=x-lo+b; return {lo+u*u/(4*b),u/(2*b)}; }
  if (x<=hi-b) return {x,1.};
  if (x<hi+b) { const double u=hi+b-x; return {hi-u*u/(4*b),u/(2*b)}; }
  return {hi,0.};
}
namespace {
Rot skew(const Vec& v) {
  Rot x;x<<0.,-v.z(),v.y(),v.z(),0.,-v.x(),-v.y(),v.x(),0.;return x;
}
Vec angular(const Rot& r,const Rot& rd) {
  const Rot w=rd*r.transpose();return Vec(w(2,1)-w(1,2),w(0,2)-w(2,0),w(1,0)-w(0,1))*.5;
}
struct Unit { Vec value,rate; };
Unit unit(const Vec& v,const Vec& vd) {
  const double n=v.norm();if(n<1.e-9)throw std::domain_error("INVALID_TARGET: undefined geometric direction");
  const Vec u=v/n;return {u,(vd-u*u.dot(vd))/n};
}
Rot leftJacobian(const Vec& x);
struct ArmGeometry { Sample elbow;Rot forearm;Vec angular;Vec pivot,pivot_rate;std::array<double,4> q,rate; };
ArmGeometry forwardArm(const WristGeometry& g,const std::array<double,4>& q,const std::array<double,4>& rate) {
  const Rot s1=g.shoulder_parent*rotation(1,q[0]),s2=s1*rotation(0,q[1]),s3=s2*rotation(2,q[2]);
  const Vec w3=g.shoulder_parent*Vec::UnitY()*rate[0]+s1*Vec::UnitX()*rate[1]+s2*Vec::UnitZ()*rate[2];
  const Rot forearm=s3*rotation(1,q[3]);const Vec w=w3+s3*Vec::UnitY()*rate[3];
  const Vec upper=s3*g.upper_vector,lower=forearm*g.forearm_vector;
  Pose pe=Pose::Identity();pe.translation()=g.shoulder+upper;pe.linear()=forearm;
  const Vec elbow_rate=w3.cross(upper);
  return {{pe,elbow_rate,w},forearm,w,pe.translation()+lower,elbow_rate+w.cross(lower),q,rate};
}
// The only redundant rotation is selected by the shortest shoulder rotation
// from the engage posture. Minimizing elbow displacement instead would make
// the nearly invisible upper-arm twist (J3) absorb lateral wrist translation.
ArmGeometry armGeometry(const WristGeometry& g,const Rot& anchor,const Rot& metric,const Vec& p,const Vec& pd,bool project_joints=true) {
  const Vec reach=p-g.shoulder;const double d=reach.norm();
  const auto n=unit(reach,pd);const double dd=n.value.dot(pd),l1=g.upper_length,l2=g.forearm_length;
  const double a=(l1*l1-l2*l2+d*d)/(2.*d),h2=l1*l1-a*a;
  if(h2<=1.e-12)throw std::domain_error("INVALID_TARGET: unreachable or straight two-link arm");
  const double cosine=(d*d-l1*l1-l2*l2)/(2.*l1*l2),sine=d*std::sqrt(h2)/(l1*l2);
  const double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-
      std::atan2(g.upper_vector.x(),-g.upper_vector.z())-std::atan2(sine,cosine);
  const double q4d=d*dd/(l1*l2*sine);const Rot r=rotation(1,q4),rd=skew(Vec::UnitY()*q4d)*r;
  const auto source=unit(anchor*(g.upper_vector+r*g.forearm_vector),anchor*rd*g.forearm_vector);
  const Vec cross=source.value.cross(n.value),crossd=source.rate.cross(n.value)+source.value.cross(n.rate);
  const double dot=source.value.dot(n.value),dotd=source.rate.dot(n.value)+source.value.dot(n.rate);
  if(1.+dot<1.e-7)throw std::domain_error("INVALID_TARGET: antipodal shoulder rotation");
  const Rot K=skew(cross),Kd=skew(crossd);
  const Rot change=Rot::Identity()+K+K*K/(1.+dot);
  const Rot changed=Kd+(Kd*K+K*Kd)/(1.+dot)-K*K*dotd/((1.+dot)*(1.+dot));
  // One closed-form scalar redundancy choice in the engage joint tangent
  // metric. Positive metric entries balance motion; no axis is fixed. Rotation
  // about the reached direction leaves the complete two-link endpoint EXACT.
  const Eigen::AngleAxisd aa(change);const Vec phi=aa.axis()*aa.angle();
  const Vec phid=leftJacobian(phi).inverse()*angular(change,changed);
  const double numerator=n.value.dot(metric*phi),denominator=n.value.dot(metric*n.value);
  const double beta=-numerator/denominator;
  const double betad=-(n.rate.dot(metric*phi)+n.value.dot(metric*phid))/denominator+
      numerator*2.*n.rate.dot(metric*n.value)/(denominator*denominator);
  const Vec tangent=n.value*beta,tangentd=n.rate*beta+n.value*betad;
  const double t=tangent.norm();const Rot swivel=t==0.?Rot::Identity():Eigen::AngleAxisd(t,tangent/t).toRotationMatrix();
  const Rot swiveld=skew(leftJacobian(tangent)*tangentd)*swivel;
  const Rot r3=swivel*change*anchor,r3d=(swiveld*change+swivel*changed)*anchor;
  // Resolve the selected redundant pose into actual Y-X-Z shoulder joints.
  // Project JOINTS, then regenerate BOTH the elbow and endpoint from the same
  // chain. Projecting the hint alone would silently break the TCP contract.
  const Rot local=g.shoulder_parent.transpose()*r3;
  const double cos2=std::hypot(local(1,0),local(1,1));
  if(cos2<1.e-7)throw std::domain_error("INVALID_TARGET: shoulder Euler singularity");
  Vec angles(std::atan2(local(0,2),local(2,2)),std::atan2(-local(1,2),cos2),
      std::atan2(local(1,0),local(1,1)));
  const Rot ry=rotation(1,angles[0]),rx=rotation(0,angles[1]);
  Rot axes;axes.col(0)=Vec::UnitY();axes.col(1)=ry*Vec::UnitX();axes.col(2)=ry*rx*Vec::UnitZ();
  Vec rates=axes.inverse()*g.shoulder_parent.transpose()*angular(r3,r3d);
  if(project_joints)for(int k=0;k<3;++k){const auto v=softProject(angles[k],g.upstream_lower[k],g.upstream_upper[k],.035);angles[k]=v.value;rates[k]*=v.derivative;}
  const auto elbow_angle=project_joints?softProject(q4,g.upstream_lower[3],g.upstream_upper[3],.035):ScalarProjection{q4,1.};
  const Rot s1=g.shoulder_parent*rotation(1,angles[0]),s2=s1*rotation(0,angles[1]),s3=s2*rotation(2,angles[2]);
  const Vec w3=g.shoulder_parent*Vec::UnitY()*rates[0]+s1*Vec::UnitX()*rates[1]+s2*Vec::UnitZ()*rates[2];
  const Rot forearm=s3*rotation(1,elbow_angle.value);
  const Vec w=w3+s3*Vec::UnitY()*(elbow_angle.derivative*q4d);
  const Vec upper=s3*g.upper_vector,lower=forearm*g.forearm_vector;
  Pose pe=Pose::Identity();pe.translation()=g.shoulder+upper;pe.linear()=forearm;
  const Vec elbow_rate=w3.cross(upper);
  return {{pe,elbow_rate,w},forearm,w,pe.translation()+lower,elbow_rate+w.cross(lower),
    {angles[0],angles[1],angles[2],elbow_angle.value},{rates[0],rates[1],rates[2],elbow_angle.derivative*q4d}};
}
struct Wrist {Vec q,rate;Rot r6;Vec w6;};
Wrist wrist(const Rot& r4,const Vec& w4,const Rot& r7,const Vec& w7,const Vec& previous) {
  const Rot rel=r4.transpose()*r7;
  const Vec relw=r4.transpose()*(w7-w4);const Rot rd=skew(relw)*rel;
  const double cosine=std::hypot(rel(0,1),rel(1,1));
  if(cosine<1.e-5)throw std::domain_error("INVALID_TARGET: wrist Euler singularity");
  Vec q(std::atan2(-rel(0,1),rel(1,1)),std::atan2(rel(2,1),cosine),std::atan2(-rel(2,0),rel(2,2)));
  for(int k:{0,2})q[k]=previous[k]+std::remainder(q[k]-previous[k],2.*std::acos(-1.));
  const Vec qd((rel(0,1)*rd(1,1)-rel(1,1)*rd(0,1))/(cosine*cosine),rd(2,1)/cosine,
      (rel(2,0)*rd(2,2)-rel(2,2)*rd(2,0))/(cosine*cosine));
  const Rot r5=r4*rotation(2,q[0]),r6=r5*rotation(0,q[1]);
  return {q,qd,r6,w4+r4*Vec::UnitZ()*qd[0]+r5*Vec::UnitX()*qd[1]};
}
struct Inverse {Vec p6,v6;ArmGeometry arm;Wrist wrist;};
Inverse inverse(const WristGeometry& g,const ArmConfig& c,const Rot& anchor,const Rot& metric,
    const Sample& tcp,const Vec& previous,bool project_reach) {
  const Rot r7=tcp.pose.linear()*g.j7_to_tcp.linear().transpose();
  const Vec lever7=r7*g.j7_to_tcp.translation();
  const Vec p7=tcp.pose.translation()-lever7,v7=tcp.linear-tcp.angular.cross(lever7);
  Rot r6=r7*rotation(1,-previous[2]);Vec w6=tcp.angular;
  for(int iteration=0;iteration<64;++iteration) {
    const Vec lever6=r6*g.j6_to_j7;
    Vec p6=p7-lever6,v6=v7-w6.cross(lever6);
    if(project_reach) {
      const Vec reach=p6-g.shoulder;const double length=reach.norm();const auto n=unit(reach,v6);
      const double alpha=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-
          std::atan2(g.upper_vector.x(),-g.upper_vector.z())-g.elbow_lower;
      const double inner=std::sqrt(g.upper_length*g.upper_length+g.forearm_length*g.forearm_length+
          2.*g.upper_length*g.forearm_length*std::cos(alpha));
      const auto radial=softProject(length,inner+c.reach_margin,
          g.upper_length+g.forearm_length-c.reach_margin,c.reach_width);
      p6=g.shoulder+n.value*radial.value;
      v6=n.rate*radial.value+n.value*radial.derivative*n.value.dot(v6);
    }
    const auto arm=armGeometry(g,anchor,metric,p6,v6,project_reach);
    const auto w=wrist(arm.forearm,arm.angular,r7,tcp.angular,previous);
    const double error=(w.r6-r6).norm(),rate_error=(w.w6-w6).norm();
    if(error<1.e-12 && rate_error<1.e-10)return {p6,v6,arm,w};
    r6=w.r6;w6=w.w6;
  }
  throw std::domain_error("INVALID_TARGET: articulated wrist inverse did not converge");
}
Rot leftJacobian(const Vec& x) {
  const Rot K=skew(x);const double t=x.norm();
  if(t<1.e-7)return Rot::Identity()+.5*K+(1./6.)*K*K;
  return Rot::Identity()+(1.-std::cos(t))/(t*t)*K+(t-std::sin(t))/(t*t*t)*K*K;
}
}
Sample EngagedMapper::virtualPoint(const Sample& c) const {
  const Vec offset=c.pose.linear()*config_.controller_to_virtual.translation();
  return {c.pose*config_.controller_to_virtual,c.linear+c.angular.cross(offset),c.angular};
}
EngagedMapper::EngagedMapper(const ArmConfig& c,const WristGeometry& g,
    const Sample& ca,const RobotArm& ra)
    :config_(c),geometry_(g),robot_anchor_(ra.tcp) {
  // Configured C→W is a wrist pivot. The robot's known tool lever is attached
  // ONCE at engage in controller-local coordinates. Its velocity is the exact
  // omega×lever term; the 170 mm tool is never confused with the short grip offset.
  const Rot r7=ra.tcp.pose.linear()*g.j7_to_tcp.linear().transpose();
  config_.controller_to_virtual.translation()+=ca.pose.linear().transpose()*c.alignment.transpose()*r7*g.j7_to_tcp.translation();
  virtual_anchor_=virtualPoint(ca);
  input_raw_rotation_=ra.tcp.pose.linear();
  geometry_.shoulder=ra.shoulder.pose.translation();geometry_.shoulder_parent=ra.shoulder_parent;
  const Vec u=ra.elbow.pose.translation()-geometry_.shoulder,v=ra.forearm*g.forearm_vector;
  const double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-std::atan2(g.upper_vector.x(),-g.upper_vector.z())-
      std::atan2(u.cross(v).norm(),u.dot(v));
  shoulder_anchor_=ra.forearm*rotation(1,-q4);
  if(std::abs(ra.shoulder_axes.determinant())<1.e-7)throw std::domain_error("INVALID_TARGET: singular shoulder engage axes");
  const Rot inverse_axes=ra.shoulder_axes.inverse();shoulder_metric_=inverse_axes.transpose()*c.shoulder_rotation_weights.asDiagonal()*inverse_axes;
  previous_wrist_=wrist(ra.forearm,ra.forearm_angular,r7,ra.tcp.angular,Vec::Zero()).q;
  const auto initial=inverse(geometry_,config_,shoulder_anchor_,shoulder_metric_,ra.tcp,previous_wrist_,false);
  // Preserve the actual engage posture, including the C1 transition region.
  // An anchor outside the original stopping domain cannot be silently clipped.
  for(int j=0;j<4;++j){const double up=std::max(0.,initial.arm.rate[j]),down=std::max(0.,-initial.arm.rate[j]);
    const double lo=geometry_.upstream_lower[j]+geometry_.upstream_stopping_linear[j]*down+geometry_.upstream_stopping_quadratic[j]*down*down;
    const double hi=geometry_.upstream_upper[j]-geometry_.upstream_stopping_linear[j]*up-geometry_.upstream_stopping_quadratic[j]*up*up;
    if(initial.arm.q[j]<lo-1.e-9||initial.arm.q[j]>hi+1.e-9||std::abs(initial.arm.rate[j])>geometry_.upstream_vmax[j]+1.e-9)
      throw std::domain_error("INVALID_ANCHOR: shoulder/elbow outside original stopping domain");
  }
  reference_p6_=input_p6_=initial.p6;reference_v6_=initial.v6;reference_a6_.setZero();
  reference_up_=Eigen::Map<const Eigen::Vector4d>(initial.arm.q.data());
  upstream_dq_=Eigen::Map<const Eigen::Vector4d>(initial.arm.rate.data());
  reference_q_=input_q_=initial.wrist.q;reference_dq_=initial.wrist.rate;
  const double w=2.*std::acos(-1.)*config_.wrist_reference_cutoff/std::sqrt(std::pow(2.,1./3.)-1.);
  const auto initial_goal=armGeometry(geometry_,shoulder_anchor_,shoulder_metric_,reference_p6_,reference_v6_);
  // Initialize the tracking-error transient from the actual pose/rate. It
  // decays with the existing pole, preventing a correction-velocity step at
  // engage inside the soft transition region. Raw and its anchor are untouched.
  for(int j=0;j<4;++j)upstream_engage_correction_[j]=reference_up_[j]-initial_goal.q[j]+(upstream_dq_[j]-initial_goal.rate[j])/w;
  wrist_stage2_=reference_q_+reference_dq_/w;wrist_stage1_=wrist_stage2_+reference_dq_/w;
}
Output EngagedMapper::map(const Sample& c,const RobotArm&,double) {
  Output out;out.position_filter.setZero();out.orientation_filter.setZero();out.virtual_tcp=virtualPoint(c);const Rot& A=config_.alignment;
  Sample raw;raw.pose=Pose::Identity();
  raw.pose.translation()=robot_anchor_.pose.translation()+A*(out.virtual_tcp.pose.translation()-virtual_anchor_.pose.translation());
  raw.pose.linear()=A*out.virtual_tcp.pose.linear()*virtual_anchor_.pose.linear().transpose()*A.transpose()*robot_anchor_.pose.linear();
  raw.linear=A*out.virtual_tcp.linear;raw.angular=A*out.virtual_tcp.angular;out.raw=raw;
  const auto requested=inverse(geometry_,config_,shoulder_anchor_,shoulder_metric_,raw,previous_wrist_,true);
  out.requested_p6=requested.p6;out.requested_v6=requested.v6;
  const Vec q=requested.wrist.q;previous_wrist_=q;Vec applied,rates;
  for(int j=0;j<3;++j) {
    const auto p=softProject(q[j],geometry_.lower[j],geometry_.upper[j],config_.wrist_width);
    applied[j]=p.value;rates[j]=p.derivative*requested.wrist.rate[j];
  }
  out.requested_wrist=q;out.applied_wrist=applied;out.reference_wrist=applied;for(int j=0;j<3;++j){out.dynamic_lower[j]=geometry_.lower[j];out.dynamic_upper[j]=geometry_.upper[j];}
  const auto& arm=requested.arm;
  out.upstream=arm.q;out.upstream_rate=arm.rate;
  out.projected_upstream=arm.q;
  const Rot r5=arm.forearm*rotation(2,applied[0]),r6=r5*rotation(0,applied[1]),r7=r6*rotation(1,applied[2]);
  const Vec w5=arm.angular+arm.forearm*Vec::UnitZ()*rates[0],w6=w5+r5*Vec::UnitX()*rates[1],w7=w6+r6*Vec::UnitY()*rates[2];
  const Vec lever6=r6*geometry_.j6_to_j7,lever7=r7*geometry_.j7_to_tcp.translation();
  Sample projected;projected.pose=Pose::Identity();projected.pose.linear()=r7*geometry_.j7_to_tcp.linear();
  projected.pose.translation()=arm.pivot+lever6+lever7;
  projected.linear=arm.pivot_rate+w6.cross(lever6)+w7.cross(lever7);projected.angular=w7;
  out.projected=projected;
  out.position_projection=projected.pose.translation()-raw.pose.translation();
  Eigen::AngleAxisd change(projected.pose.linear()*raw.pose.linear().transpose());
  out.orientation_projection=change.axis()*change.angle();
  // Stateless static geometry is diagnostic, not the input to advance().
  out.command=projected;out.elbow=arm.elbow;
  out.nominal_reference=projected;
  out.reference_wrist_rate=rates;
  out.final_domain_lower=out.dynamic_lower;out.final_domain_upper=out.dynamic_upper;
  return out;
}
Output EngagedMapper::advance(const Sample& controller,const RobotArm& robot,double elapsed) {
  if(elapsed<reference_time_)throw std::invalid_argument("INVALID_INPUT: reference clock moved backwards");
  Output out=map(controller,robot,elapsed);
  const double lag=(input_p6_-reference_p6_).norm();
  const double blend=std::clamp((lag-config_.reference_fast_start)/(config_.reference_fast_full-config_.reference_fast_start),0.,1.);
  const double ordinary_fast=std::max(config_.reference_cutoff,config_.reference_fast_cutoff);
  const double ordinary_cutoff=config_.reference_cutoff+(ordinary_fast-config_.reference_cutoff)*blend*blend*(3.-2.*blend);
  const double feasible_blend=std::clamp(lag/config_.reference_fast_start,0.,1.);
  const double feasible_cutoff=config_.reference_cutoff+(std::max(ordinary_fast,config_.feasible_reference_fast_cutoff)-config_.reference_cutoff)*feasible_blend*feasible_blend*(3.-2.*feasible_blend);
  const double position_cutoff=config_.feasible_reference_fast_cutoff>0.?ordinary_cutoff+input_feasibility_*(feasible_cutoff-ordinary_cutoff):ordinary_cutoff;
  out.position_cutoff_hz=position_cutoff;
  // Static projection above is diagnostic only. The ONE continuous state is
  // driven by the original geometric request, never a previously clipped angle.
  const double dt=elapsed-reference_time_;
  // Exact constant-input position solution of (D+w)^3 x=w^3 u.
  // Fresh observations update the forcing only after state advancement.
  // Position, velocity and acceleration therefore remain continuous.
  const auto step=[&](Vec& p,Vec& v,Vec& acceleration,const Vec& input,double cutoff) {
    const double w=2.*std::acos(-1.)*cutoff/std::sqrt(std::pow(2.,1./3.)-1.);
    const Vec error=p-input,c1=v+w*error,c2=.5*(acceleration+2.*w*v+w*w*error);
    const double decay=std::exp(-w*dt);
    const Vec nextp=input+(error+c1*dt+c2*dt*dt)*decay;
    const Vec nextv=(v+(2.*c2-w*c1)*dt-w*c2*dt*dt)*decay;
    acceleration=(acceleration+(w*w*c1-4.*w*c2)*dt+w*w*c2*dt*dt)*decay;p=nextp;v=nextv;
  };
  // Follow the exact existing Cartesian reference in the free region.
  // The continuous joint state adds no second low-pass: geometric joint
  // derivatives are its feedforward. Only original stopping/speed constraints
  // can make it depart from that path. Both TCP and elbow use its final FK.
  const Vec p0=reference_p6_,v0=reference_v6_,a0=reference_a6_;
  const auto desired=[&](double time){Vec p=p0,v=v0,acc=a0;
    const double w=2.*std::acos(-1.)*position_cutoff/std::sqrt(std::pow(2.,1./3.)-1.);
    const Vec error=p-input_p6_,c1=v+w*error,c2=.5*(acc+2.*w*v+w*w*error);const double decay=std::exp(-w*time);
    p=input_p6_+(error+c1*time+c2*time*time)*decay;v=(v+(2.*c2-w*c1)*time-w*c2*time*time)*decay;
    return armGeometry(geometry_,shoulder_anchor_,shoulder_metric_,p,v);
  };
  const double tracking_gain=2.*std::acos(-1.)*config_.wrist_reference_cutoff/std::sqrt(std::pow(2.,1./3.)-1.);
  const auto upstream_velocity=[&](double time,const Eigen::Vector4d& q)->Eigen::Vector4d{
    const auto goal=desired(time);Eigen::Vector4d dq;
    for(int j=0;j<4;++j){
      const double correction=upstream_engage_correction_[j]*std::exp(-tracking_gain*(reference_time_+time));
      const double wanted=goal.rate[j]+tracking_gain*(goal.q[j]-q[j]+correction);const double sign=wanted>=0.?1.:-1.,magnitude=sign*wanted;
      const double distance=wanted>=0.?geometry_.upstream_upper[j]-q[j]:q[j]-geometry_.upstream_lower[j];
      if(distance< -1.e-9)throw std::domain_error("INVALID_TARGET: upstream joint left invariant domain");
      const double d=std::max(0.,distance),c1=geometry_.upstream_stopping_linear[j],c2=geometry_.upstream_stopping_quadratic[j];
      const double limit=std::min(geometry_.upstream_vmax[j],2.*d/(c1+std::sqrt(c1*c1+4.*c2*d)));
      if(magnitude<=.8*limit)dq[j]=wanted;
      else if(magnitude>=1.2*limit)dq[j]=sign*limit;
      else {const double residual=1.2*limit-magnitude;dq[j]=sign*(limit-residual*residual/(.8*limit));}
    }return dq;
  };
  // Keep the original integration cost at ordinary bandwidth; resolve the
  // faster pole proportionally near C1 shoulder projection transitions.
  const double upstream_step=.00005*std::min(1.,ordinary_fast/position_cutoff);
  const int count=std::max(1,int(std::ceil(dt/upstream_step)));const double up_h=dt/count;
  for(int k=0;k<count;++k){const double t=k*up_h;const auto k1=upstream_velocity(t,reference_up_),k2=upstream_velocity(t+.5*up_h,reference_up_+.5*up_h*k1),k3=upstream_velocity(t+.5*up_h,reference_up_+.5*up_h*k2),k4=upstream_velocity(t+up_h,reference_up_+up_h*k3);reference_up_+=up_h*(k1+2.*k2+2.*k3+k4)/6.;}
  upstream_dq_=upstream_velocity(dt,reference_up_);
  step(reference_p6_,reference_v6_,reference_a6_,input_p6_,position_cutoff);
  // One nonlinear third-order cascade. In the free region all three poles
  // are -w. The final state IS the emitted angle. One static C1 position map
  // supplies its equilibrium; an analytic inverse of the ORIGINAL stopping
  // reserve bounds its emitted velocity. No second angle projection exists.
  const double w=2.*std::acos(-1.)*config_.wrist_reference_cutoff/std::sqrt(std::pow(2.,1./3.)-1.);
  const auto velocity=[&](int j,double middle,double angle){
    const double lo=geometry_.lower[j],hi=geometry_.upper[j];
    const double c1=geometry_.stopping_linear[j],c2=geometry_.stopping_quadratic[j];
    const auto speed=[&](double distance){
      if(distance< -1.e-10)throw std::domain_error("INVALID_TARGET: wrist reference left its invariant domain");
      return 2.*distance/(c1+std::sqrt(c1*c1+4.*c2*distance));
    };
    const double desire=w*(softProject(middle,lo,hi,config_.wrist_width).value-angle);
    const double sign=desire>=0.?1.:-1.,magnitude=sign*desire;
    const double limit=speed(desire>=0.?hi-angle:angle-lo);
    if(magnitude<=.8*limit)return desire;
    if(magnitude>=1.2*limit)return sign*limit;
    const double residual=1.2*limit-magnitude;
    return sign*(limit-residual*residual/(.8*limit));
  };
  // RK4 integrates the one continuous state; it does not differentiate source
  // timestamps. A bounded substep resolves the fastest configured pole.
  const int steps=std::max(1,int(std::ceil(dt/(.0005/w))));const double h=dt/steps;
  for(int j=0;j<3;++j){
    const auto derivative=[&](const Vec& state)->Vec{
      return Vec(w*(input_q_[j]-state[0]),w*(state[0]-state[1]),velocity(j,state[1],state[2]));
    };
    Vec state(wrist_stage1_[j],wrist_stage2_[j],reference_q_[j]);
    for(int k=0;k<steps;++k){
      const Vec k1=derivative(state),k2=derivative(state+.5*h*k1),k3=derivative(state+.5*h*k2),k4=derivative(state+h*k3);
      state+=h*(k1+2.*k2+2.*k3+k4)/6.;
    }
    wrist_stage1_[j]=state[0];wrist_stage2_[j]=state[1];reference_q_[j]=state[2];
    reference_dq_[j]=velocity(j,state[1],state[2]);
  }
  const auto nominal_arm=desired(dt);
  // Use only the preceding observation to choose the next interval's pole.
  // A small raw-to-static residual supplies a nearby full-chain witness; it
  // does not certify dynamic reachability. Original final-rate bounds remain.
  const auto confidence=[](double ratio){const double u=std::clamp(ratio,0.,1.);return 1.-u*u*(3.-2.*u);};
  // Do not accelerate proximal following to compensate a tool-dominated
  // wrist rotation. Compare actual pivot displacement to the rotated tool
  // lever; include the reported rates and the observed increments. When the
  // input is stationary, retain the last classification while continuing to
  // follow its target normally. No pose, raw or lifecycle state is held.
  const double translation=std::max((out.requested_p6-input_p6_).norm(),dt*out.requested_v6.norm());
  const double rotational_motion=geometry_.j7_to_tcp.translation().norm()*std::max(Eigen::AngleAxisd(out.raw.pose.linear()*input_raw_rotation_.transpose()).angle(),dt*out.raw.angular.norm());
  if(translation+rotational_motion>1.e-7)input_translation_confidence_=1.-confidence((translation-rotational_motion)/(rotational_motion+1.e-12));
  input_raw_rotation_=out.raw.pose.linear();
  input_feasibility_=input_translation_confidence_*confidence(out.position_projection.norm()/.01)*confidence(out.orientation_projection.norm()/(5.*std::acos(-1.)/180.));
  input_p6_=out.requested_p6;input_q_=out.requested_wrist;reference_time_=elapsed;
  const Vec q=reference_q_,qd=reference_dq_;
  out.reference_wrist=q;out.reference_wrist_rate=qd;
  // Independent diagnostic domain computed from FINAL emitted joint rates,
  // not from the unprojected reference velocity used for preview.
  for(int j=0;j<3;++j){
    const double up=std::max(0.,qd[j]),down=std::max(0.,-qd[j]);
    out.final_domain_lower[j]=geometry_.lower[j]+geometry_.stopping_linear[j]*down+geometry_.stopping_quadratic[j]*down*down;
    out.final_domain_upper[j]=geometry_.upper[j]-geometry_.stopping_linear[j]*up-geometry_.stopping_quadratic[j]*up*up;
    out.dynamic_lower[j]=out.final_domain_lower[j];out.dynamic_upper[j]=out.final_domain_upper[j];
  }
  const auto arm=forwardArm(geometry_,{reference_up_[0],reference_up_[1],reference_up_[2],reference_up_[3]},
      {upstream_dq_[0],upstream_dq_[1],upstream_dq_[2],upstream_dq_[3]});
  out.upstream=arm.q;out.upstream_rate=arm.rate;
  const Rot r5=arm.forearm*rotation(2,q[0]),r6=r5*rotation(0,q[1]),r7=r6*rotation(1,q[2]);
  const Vec w5=arm.angular+arm.forearm*Vec::UnitZ()*qd[0],w6=w5+r5*Vec::UnitX()*qd[1],w7=w6+r6*Vec::UnitY()*qd[2];
  const Vec lever6=r6*geometry_.j6_to_j7,lever7=r7*geometry_.j7_to_tcp.translation();
  out.command.pose.linear()=r7*geometry_.j7_to_tcp.linear();out.command.pose.translation()=arm.pivot+lever6+lever7;
  out.command.linear=arm.pivot_rate+w6.cross(lever6)+w7.cross(lever7);out.command.angular=w7;out.elbow=arm.elbow;
  const Rot nr5=nominal_arm.forearm*rotation(2,q[0]),nr6=nr5*rotation(0,q[1]),nr7=nr6*rotation(1,q[2]);
  const Vec nw5=nominal_arm.angular+nominal_arm.forearm*Vec::UnitZ()*qd[0],nw6=nw5+nr5*Vec::UnitX()*qd[1],nw7=nw6+nr6*Vec::UnitY()*qd[2];
  const Vec nl6=nr6*geometry_.j6_to_j7,nl7=nr7*geometry_.j7_to_tcp.translation();
  out.nominal_reference.pose=Pose::Identity();out.nominal_reference.pose.linear()=nr7*geometry_.j7_to_tcp.linear();
  out.nominal_reference.pose.translation()=nominal_arm.pivot+nl6+nl7;
  out.nominal_reference.linear=nominal_arm.pivot_rate+nw6.cross(nl6)+nw7.cross(nl7);out.nominal_reference.angular=nw7;
  out.position_filter=out.command.pose.translation()-out.projected.pose.translation();
  Eigen::AngleAxisd filter(out.command.pose.linear()*out.projected.pose.linear().transpose());out.orientation_filter=filter.axis()*filter.angle();
  return out;
}
}  // namespace wheel_arm_abs_vel::direct_tcp
