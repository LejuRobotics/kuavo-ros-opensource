#pragma once
// Exact batch implementation of Workspace.project. The Python path remains
// the numerical oracle; neither path solves joints or changes task weights.
namespace incremental_geometry {
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
inline V elbowCircle(const V& hand,const V& shoulder,const V& preferred,
    double upper,double lower,const V& front,const V& side,bool bounded,double offset) {
  const double length=(hand-shoulder).norm();
  if(length<1.e-9)throw std::invalid_argument("undefined shoulder-to-wrist axis");
  const V axis=(hand-shoulder)/length;
  const double along=std::clamp((upper*upper-lower*lower+length*length)/(2.*length),-upper,upper);
  const V mid=shoulder+along*axis;
  const double radius=std::sqrt(std::max(0.,upper*upper-along*along));
  V tangent=preferred-mid-axis*axis.dot(preferred-mid);
  const V lateral=side-axis*side.dot(axis);
  if(tangent.squaredNorm()<1.e-16)tangent=lateral.squaredNorm()>1.e-16?lateral:V(front-axis*front.dot(axis));
  V elbow=mid+radius*tangent/std::max(1.e-12,tangent.norm());
  if(bounded && side.dot(elbow)<offset-1.e-8) {
    const double norm=lateral.norm();
    if(norm<1.e-9)throw std::invalid_argument("no elbow on admitted side");
    const V unit=lateral/norm;
    const double height=(offset-side.dot(mid))/norm;
    if(height>radius+1.e-6)throw std::invalid_argument("no elbow on admitted circle");
    V perpendicular=axis.cross(unit);
    if(perpendicular.dot(tangent)<0.)perpendicular=-perpendicular;
    elbow=mid+height*unit+std::sqrt(std::max(0.,radius*radius-height*height))*perpendicular;
  }
  return elbow;
}
inline pybind11::tuple project(const V& p,const V& v,pybind11::object elbow_obj,
    const M& R,const V& t,const M& chest_R,const V& chest,const V& shoulder,double reach,
    pybind11::object lengths,const V& lo,const V& upper_bound,double clearance,
    double elbow_weight,double hand_weight,double sign) {
  const double yaw=std::atan2(chest_R(1,0),chest_R(0,0));
  const V front(std::cos(yaw),std::sin(yaw),0.), side=sign*V(-front.y(),front.x(),0.);
  V hand=R*p+t, velocity=R*v;const V center=R*shoulder+t;
  V radial=hand-center;
  if(radial.norm()>reach)hand=center+radial*(reach/radial.norm());
  const V request=hand;
  V hi=upper_bound;hi.z()=std::min(hi.z(),chest.z()+.25);
  const bool has_lengths=!lengths.is_none(),has_elbow=!elbow_obj.is_none();
  Eigen::Vector2d lens=Eigen::Vector2d::Zero();
  if(has_lengths)lens=lengths.cast<Eigen::Vector2d>();
  const double upper=lens[0],lower=lens[1];
  for(int i=0;i<16;++i) {
    const V before=hand;
    hand=hand.cwiseMax(lo).cwiseMin(hi);
    hand+=std::max(0.,-.03-front.dot(hand-chest))*front;
    radial=hand-center;
    if(radial.norm()>reach)hand=center+radial*(reach/radial.norm());
    if(has_lengths) {
      const double distance=(hand-center).norm();
      const V axis=distance>1.e-9?V((hand-center)/distance):front;
      if(distance<std::abs(upper-lower)+1.e-8)hand=center+(std::abs(upper-lower)+1.e-8)*axis;
      V nearest=center+upper*axis;
      if(side.dot(nearest-chest)<clearance) {
        const double height=clearance-side.dot(center-chest);
        if(height>upper)throw std::invalid_argument("infeasible elbow clearance");
        V tangent=axis-side.dot(axis)*side;
        tangent=tangent.norm()>1.e-9?V(tangent/tangent.norm()):front;
        nearest=center+height*side+std::sqrt(std::max(0.,upper*upper-height*height))*tangent;
      }
      const V away=hand-nearest;
      if(away.norm()>lower)hand=nearest+lower*away/away.norm();
    }
    if((hand-before).norm()<1.e-10)break;
  }
  if((hand.array()<lo.array()-1.e-6).any() || (hand.array()>hi.array()+1.e-6).any()
      || front.dot(hand-chest)<-.030001)throw std::invalid_argument("wrist has no admitted projection");
  for(int i=0;i<3;++i)if((hand[i]<=lo[i]+1.e-7 && velocity[i]<0.) ||
      (hand[i]>=hi[i]-1.e-7 && velocity[i]>0.))velocity[i]=0.;
  if(front.dot(hand-chest)<=-.03+1.e-7)velocity-=std::min(0.,front.dot(velocity))*front;
  radial=hand-center;
  if(radial.norm()>=reach-1.e-7) {
    const V normal=radial/std::max(1.e-12,radial.norm());
    velocity-=std::max(0.,normal.dot(velocity))*normal;
  }
  pybind11::object result_elbow=pybind11::none();
  if(has_elbow) {
    V e=R*elbow_obj.cast<V>()+t;
    if(!has_lengths)e+=std::max(0.,clearance-side.dot(e-chest))*side;
    else {
      const V preferred=elbowCircle(request,center,e,upper,lower,front,side,false,0.);
      const double offset=clearance+side.dot(chest);
      e=elbowCircle(hand,center,preferred,upper,lower,front,side,true,offset);
      if((hand-request).norm()>1.e-8 && elbow_weight>0.) {
        M tangent_space=M::Identity();bool clipped=false;
        auto admit=[&](const V& normal) {
          clipped=true;const V n=tangent_space*normal;
          if(n.squaredNorm()>1.e-12)tangent_space-=n*n.transpose()/n.squaredNorm();
        };
        for(int i=0;i<3;++i)if(hand[i]<=lo[i]+1.e-7 || hand[i]>=hi[i]-1.e-7)admit(V::Unit(i));
        if(front.dot(hand-chest)<=-.03+1.e-7)admit(front);
        if(clipped) {
          const V fixed=(M::Identity()-tangent_space)*(hand-preferred),free=tangent_space*(request-preferred);
          const double rho2=lower*lower-fixed.squaredNorm();
          if(rho2>0. && free.squaredNorm()>1.e-12) {
            const V correction=preferred+fixed+std::sqrt(rho2/free.squaredNorm())*free-hand;
            const double ec=elbow_weight*(e-preferred).squaredNorm(),hc=hand_weight*correction.squaredNorm();
            if(ec+hc>1.e-14) {
              const V blended=hand+correction*(ec/(ec+hc));
              if((blended.array()>=lo.array()-1.e-8).all() && (blended.array()<=hi.array()+1.e-8).all()
                  && front.dot(blended-chest)>=-.03000001 && (blended-center).norm()<=reach+1.e-8) {
                e=elbowCircle(blended,center,preferred,upper,lower,front,side,true,offset);hand=blended;
              }
            }
          }
        }
      }
    }
    result_elbow=pybind11::cast(V(R.transpose()*(e-t)));
  }
  return pybind11::make_tuple(V(R.transpose()*(hand-t)),V(R.transpose()*velocity),result_elbow);
}
}  // namespace incremental_geometry
