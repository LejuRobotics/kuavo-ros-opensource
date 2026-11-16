#pragma once
#include "model_pinocchio.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace velocity_ik {
struct UnlockCapsule {
  Eigen::Vector3d a=Eigen::Vector3d::Zero(), b=Eigen::Vector3d::Zero();
  double radius=0.;
};
// Exact segment distance, including parallel and zero-length segments.
inline Eigen::Vector3d unlockSegmentSeparation(const UnlockCapsule& a, const UnlockCapsule& b) {
  const Eigen::Vector3d u=a.b-a.a, v=b.b-b.a, w=a.a-b.a;
  const double aa=u.dot(u), bb=u.dot(v), cc=v.dot(v), dd=u.dot(w), ee=v.dot(w);
  double s=0.,t=0.;
  constexpr double eps=1e-14;
  if(aa<=eps && cc<=eps) return w;
  if(aa<=eps) t=std::clamp(ee/cc,0.,1.);
  else if(cc<=eps) s=std::clamp(-dd/aa,0.,1.);
  else {
    const double den=aa*cc-bb*bb;
    if(den>eps) s=std::clamp((bb*ee-cc*dd)/den,0.,1.);
    t=(bb*s+ee)/cc;
    if(t<0.) {t=0.;s=std::clamp(-dd/aa,0.,1.);}
    else if(t>1.) {t=1.;s=std::clamp((bb-dd)/aa,0.,1.);}
  }
  return w+s*u-t*v;
}
inline double unlockCapsuleDistance(const UnlockCapsule& a,const UnlockCapsule& b) {
  return unlockSegmentSeparation(a,b).norm()-a.radius-b.radius;
}

// Conservative URDF primitive envelopes. A cylinder is enclosed by a capsule
// with the SAME full axis length (not shortened by its radius). Meshes are not
// silently ignored. This guard covers arms/tools against torso and each other;
// it is not a full robot/environment collision checker.
class UnlockGeometry {
 public:
  struct Shape {
    std::string frame;
    int arm=-1;
    UnlockCapsule local;
    std::array<double,7> lever{};
  };
  std::vector<Shape> shapes;
  double body_front=0.,body_side=0.;

  void load(const std::string& path,const std::string& torso,
            const std::array<std::string,2>& tcp,
            const ModelPinocchio& model,double tool_radius) {
    using boost::property_tree::ptree;
    ptree doc; boost::property_tree::read_xml(path,doc);
    struct Parent {std::string link,joint;double length=0.;};
    std::map<std::string,Parent> parents;
    const auto& robot=doc.get_child("robot");
    for(const auto& item:robot) if(item.first=="joint") {
      const auto& j=item.second;
      parents[j.get<std::string>("child.<xmlattr>.link")]=
          {j.get<std::string>("parent.<xmlattr>.link"),j.get<std::string>("<xmlattr>.name"),
           vector(j.get<std::string>("origin.<xmlattr>.xyz","0 0 0")).norm()};
    }
    // Capsule radius is rotationally invariant. Bound displacement of the
    // centerline endpoints only, then add the unchanged radius in clearance().
    // Including radius in every joint lever needlessly inflated the stop box.
    const auto setLevers=[&](Shape& shape,std::string link,double radius) {
      while(link!=torso) {
        const auto it=parents.find(link);
        if(it==parents.end()) throw std::runtime_error("unlock geometry disconnected from torso: "+link);
        for(int j=0;j<7;++j) {
          const std::string name=std::string(shape.arm==0 ? "zarm_l" : "zarm_r")+std::to_string(j+1)+"_joint";
          if(it->second.joint==name)shape.lever[j]=radius;
        }
        radius+=it->second.length;
        link=it->second.link;
      }
    };
    shapes.clear();body_front=body_side=0.;
    std::array<int,2> counts{}; int bodies=0;
    for(const auto& item:robot) if(item.first=="link") {
      const auto& l=item.second;const std::string name=l.get<std::string>("<xmlattr>.name");
      const int arm=name.rfind("zarm_l",0)==0 ? 0 : (name.rfind("zarm_r",0)==0 ? 1 : -1);
      if(name!=torso && arm<0)continue;
      for(const auto& entry:l) if(entry.first=="collision") {
        Shape shape;shape.frame=name;shape.arm=arm;
        const auto& c=entry.second; const auto& g=c.get_child("geometry");
        const auto xyz=vector(c.get<std::string>("origin.<xmlattr>.xyz","0 0 0"));
        const auto rpy=vector(c.get<std::string>("origin.<xmlattr>.rpy","0 0 0"));
        const Eigen::Matrix3d R=(Eigen::AngleAxisd(rpy.z(),Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(rpy.y(),Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(rpy.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d axis=Eigen::Vector3d::UnitZ();double half=0.,radius=0.;
        if(auto cylinder=g.get_child_optional("cylinder")) {
          radius=cylinder->get<double>("<xmlattr>.radius");
          half=.5*cylinder->get<double>("<xmlattr>.length");
        } else if(auto sphere=g.get_child_optional("sphere")) {
          radius=sphere->get<double>("<xmlattr>.radius");
        } else if(auto box=g.get_child_optional("box")) {
          const Eigen::Vector3d size=vector(box->get<std::string>("<xmlattr>.size"));
          if(size.minCoeff()<=0.)throw std::runtime_error("invalid collision box");
          Eigen::Index i;size.maxCoeff(&i);axis=Eigen::Vector3d::Unit(i);
          half=.5*size[i];radius=.5*std::sqrt(size.squaredNorm()-size[i]*size[i]);
        } else throw std::runtime_error("unsupported unlock collision primitive: "+name);
        if(!std::isfinite(radius)||!std::isfinite(half)||radius<=0.||half<0.)
          throw std::runtime_error("invalid unlock collision primitive: "+name);
        shape.local={xyz-half*R*axis,xyz+half*R*axis,radius};
        model.GetFrameByName(name);  // reject missing frames at startup
        if(arm>=0) {
          ++counts[arm];
          setLevers(shape,name,std::max(shape.local.a.norm(),shape.local.b.norm()));
        } else {
          ++bodies;
          body_front=std::max(body_front,std::max(shape.local.a.x(),shape.local.b.x())+radius);
          body_side=std::max(body_side,std::max(std::abs(shape.local.a.y()),std::abs(shape.local.b.y()))+radius);
        }
        shapes.push_back(shape);
      }
    }
    if(bodies==0 || counts[0]<3 || counts[1]<3)
      throw std::runtime_error("unlock guard requires torso and both arm collision primitives");
    const auto context=model.CreateDefaultContext();
    for(int arm=0;arm<2;++arm) {
      const std::string flange=arm==0 ? "zarm_l7_link" : "zarm_r7_link";
      const auto offset=model.GetFrameByName(tcp[arm]).CalcPose(*context,model.GetFrameByName(flange));
      Shape tool;tool.frame=flange;tool.arm=arm;
      // A nominal URDF TCP marker does not describe the installed end effector.
      // Enclose flange-to-TCP plus configurable tool thickness explicitly.
      tool.local={Eigen::Vector3d::Zero(),offset.translation(),tool_radius};
      setLevers(tool,flange,offset.translation().norm());
      shapes.push_back(tool);
    }
    if(shapes.size()>32)throw std::runtime_error("too many unlock collision primitives");
  }

  // Use the signed trapezoidal step only for the direct return. The same
  // capsule pairs and margins apply. Inside the margin a separated pair
  // may move out, provided its gap never goes below its starting value.
  bool clearReturnStep(const ModelPinocchio& model, const ModelContext& context,
      const std::string& torso, const Eigen::VectorXd& initial_step,
      const Eigen::VectorXd& mean_step, const std::array<int,14>& velocity_indices,
      double margin) const {
    struct Motion {
      UnlockCapsule capsule;
      double travel = 0., curvature = 0.;
      std::array<Eigen::Vector3d,2> initial, mean;
      bool projected = false;
    };
    std::array<Motion,32> motion;
    const auto& body = model.GetFrameByName(torso);
    for (size_t i=0; i<shapes.size(); ++i) {
      const auto& shape = shapes[i];
      auto& m = motion[i];
      m.capsule = torsoCapsule(model, context, body, shape);
      if (shape.arm < 0) continue;
      double parent_rotation = 0.;
      for (int j=0; j<7; ++j) {
        const int k = velocity_indices[7*shape.arm+j];
        const double angle = std::max(std::abs(initial_step[k]), std::abs(mean_step[k]));
        const double travel = shape.lever[j] * angle;
        m.travel += travel;
        // For a serial revolute chain the second derivative along a joint
        // displacement ray contains a diagonal term and two ancestor terms.
        m.curvature += travel * (angle + 2.*parent_rotation);
        parent_rotation += angle;
      }
    }
    // Normalize time to [0,1]. The linearized endpoint trajectory is a
    // quadratic Bezier with offsets (0, initial/2, mean). The FK remainder
    // is bounded by curvature*t*t/2, so it affects only the last control.
    const auto project = [&](size_t i, const Eigen::Vector3d& normal) {
      auto& m = motion[i];
      const auto& shape = shapes[i];
      if (!m.projected) {
        for (int e=0; e<2; ++e) {
          if (shape.arm < 0) {
            m.initial[e].setZero();
            m.mean[e].setZero();
          } else {
            Eigen::MatrixXd jacobian;
            model.translationJacobian(context, model.GetFrameByName(shape.frame),
                e == 0 ? shape.local.a : shape.local.b, body, body, &jacobian);
            m.initial[e] = .5 * (jacobian * initial_step);
            m.mean[e] = jacobian * mean_step;
          }
        }
        m.projected = true;
      }
      const double a = normal.dot(m.capsule.a), b = normal.dot(m.capsule.b);
      return Eigen::Vector2d(
          std::min(a+normal.dot(m.initial[0]), b+normal.dot(m.initial[1])),
          std::min(a+normal.dot(m.mean[0]), b+normal.dot(m.mean[1])) - .5*m.curvature);
    };
    for (size_t i=0; i<shapes.size(); ++i) for (size_t j=i+1; j<shapes.size(); ++j) {
      if (shapes[i].arm == shapes[j].arm) continue;
      const auto& a = motion[i];
      const auto& b = motion[j];
      const Eigen::Vector3d separation = unlockSegmentSeparation(a.capsule, b.capsule);
      const double length = separation.norm();
      const double radii = a.capsule.radius + b.capsule.radius;
      const double gap = length - radii;
      if (!std::isfinite(gap) || gap <= 0.) return false;
      if (gap - a.travel - b.travel >= margin) continue;
      const Eigen::Vector3d normal = separation / length;
      const Eigen::Vector2d bounds = (project(i,normal) + project(j,-normal)).array() - radii;
      // Roundoff at the stationary first control must not prevent departure.
      const double floor = std::min(gap, margin);
      if (!bounds.allFinite() || bounds.minCoeff() < floor - 1e-12 ||
          (gap < margin && bounds[1] <= gap + 1e-12)) return false;
    }
    return true;
  }

  // Joint excursion radii describe ALL configurations in a stopping box, not
  // just one guessed stopping trajectory. Sum(link reach * angular travel)
  // bounds how far any point of each capsule can move. Measured and reference
  // states are checked independently by the caller.
  double clearance(const ModelPinocchio& model,const ModelContext& context,
      const std::string& torso,const std::array<std::array<double,7>,2>& excursion,
      double* center_clearance=nullptr,std::array<double,14>* sensitivity=nullptr) const {
    double result=std::numeric_limits<double>::infinity();
    const auto& body=model.GetFrameByName(torso);
    std::array<UnlockCapsule,32> world;
    std::array<double,32> inflations{};size_t index=0;
    if(center_clearance)*center_clearance=std::numeric_limits<double>::infinity();
    for(const auto& s:shapes) {

      double inflation=0.;
      if(s.arm>=0)for(int j=0;j<7;++j) inflation+=s.lever[j]*excursion[s.arm][j];
      inflations[index]=inflation;
      world[index] = torsoCapsule(model, context, body, s);
      world[index++].radius += inflation;
    }
    for(size_t i=0;i<shapes.size();++i)for(size_t j=i+1;j<shapes.size();++j) {
      if(shapes[i].arm==shapes[j].arm)continue;
      const double distance=unlockCapsuleDistance(world[i],world[j]);
      if(center_clearance)*center_clearance=std::min(*center_clearance,distance+inflations[i]+inflations[j]);
      if(distance<result) {
        result=distance;
        if(sensitivity) {
          sensitivity->fill(0.);
          for(size_t k:{i,j})if(shapes[k].arm>=0)
            for(int q=0;q<7;++q)(*sensitivity)[7*shapes[k].arm+q]+=shapes[k].lever[q];
        }
      }
    }
    return result;
  }
  // Certify the SAME full independent-joint box as clearance(), by partitioning
  // it into smaller boxes. A large spherical link inflation can overlap the
  // body even when every actual configuration in the box is separated.
  // Both halves of every split must pass. Budget exhaustion is rejection,
  // never permission to use point sampling. No heap-backed search queue.
  // context/full_q are scratch state and may be left at a sub-box center.
  bool certifyBox(const ModelPinocchio& model,ModelContext* context,
      Eigen::VectorXd* full_q,const std::array<int,14>& q_index,
      const std::string& torso,const std::array<std::array<double,7>,2>& excursion,
      double margin,int* evaluations=nullptr) const {
    constexpr int budget=127;
    struct Box {std::array<double,14> center{},radius{};int depth=0;};
    std::array<Box,24> pending;
    std::array<double,14> origin{};
    Box box;
    if(evaluations)*evaluations=0;
    if(!std::isfinite(margin)||margin<0.)return false;
    for(int j=0;j<14;++j) {
      origin[j]=(*full_q)[q_index[j]];box.radius[j]=excursion[j/7][j%7];
      if(!std::isfinite(origin[j])||!std::isfinite(box.radius[j])||box.radius[j]<0.)return false;
    }
    int count=1,visited=0;pending[0]=box;
    while(count>0) {
      if(visited>=budget)return false;
      box=pending[--count];++visited;
      if(evaluations)*evaluations=visited;
      std::array<std::array<double,7>,2> radii{};
      for(int j=0;j<14;++j) {
        (*full_q)[q_index[j]]=origin[j]+box.center[j];
        radii[j/7][j%7]=box.radius[j];
      }
      model.SetPositions(context,*full_q);
      double center_distance;
      std::array<double,14> sensitivity{};
      const double bound=clearance(model,*context,torso,radii,&center_distance,&sensitivity);
      if(!std::isfinite(bound)||!std::isfinite(center_distance))return false;
      if(bound>=margin)continue;
      if(center_distance<margin || box.depth>=22)return false;
      int split=0;
      for(int j=1;j<14;++j)
        if(sensitivity[j]*box.radius[j]>sensitivity[split]*box.radius[split])split=j;
      if(sensitivity[split]*box.radius[split]<1e-10 || count+2>int(pending.size()))return false;
      box.radius[split]*=.5;++box.depth;
      box.center[split]-=box.radius[split];pending[count++]=box;
      box.center[split]+=2.*box.radius[split];pending[count++]=box;
    }
    return true;
  }

 private:
  static UnlockCapsule torsoCapsule(const ModelPinocchio& model, const ModelContext& context,
      const Frame& body, const Shape& shape) {
    const auto pose = model.GetFrameByName(shape.frame).CalcPose(context, body);
    return {pose.translation()+pose.rotation().matrix()*shape.local.a,
            pose.translation()+pose.rotation().matrix()*shape.local.b, shape.local.radius};
  }
  static Eigen::Vector3d vector(const std::string& text) {
    std::istringstream in(text);Eigen::Vector3d v;
    if(!(in>>v.x()>>v.y()>>v.z())||!v.allFinite())
      throw std::runtime_error("invalid URDF collision vector");
    return v;
  }
};
} // namespace velocity_ik
