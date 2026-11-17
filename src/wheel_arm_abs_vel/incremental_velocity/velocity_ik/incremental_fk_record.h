#pragma once
#include "model_pinocchio.h"
#include <array>
#include <geometry_msgs/PoseStamped.h>
#include <kuavo_msgs/twoArmHandPose.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace velocity_ik {
// Runs on the existing diagnostic worker. All old FK topics use base_link,
// link6 and the actual link7 EE, independent of the selected physical tool.
class IncrementalFkRecord {
 public:
  void initialize(ros::NodeHandle& nh) {
    markers_ = nh.advertise<visualization_msgs::MarkerArray>("/ik_debug/whole_body_ref_markers", 2);
    for (int arm=0; arm<2; ++arm) {
      const std::string side = arm ? "right" : "left";
      for (int measured=0; measured<2; ++measured) {
        const std::string suffix = measured ? "_measured" : "";
        ee_[measured][arm] = nh.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/"+side+"_ee_pose"+suffix, 2);
        wrist_[measured][arm] = nh.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/"+side+"_link6_pose"+suffix, 2);
      }
    }
  }

  bool subscribed() const {
    if (!markers_) return false;
    if (markers_.getNumSubscribers()) return true;
    for (int m=0;m<2;++m) for (int a=0;a<2;++a)
      if (ee_[m][a].getNumSubscribers() || wrist_[m][a].getNumSubscribers()) return true;
    return false;
  }

  void publish(bool measured, const ModelPinocchio& model, const ModelContext& ctx,
               const Frame& base, std_msgs::Header header) const {
    if (!subscribed()) return;
    header.frame_id = "base_link";
    const auto read = [&](const std::string& frame) {
      const auto pose=model.GetFrameByName(frame).CalcPose(ctx,base);
      geometry_msgs::PoseStamped msg;
      msg.header=header;
      auto& p=msg.pose.position;
      p.x=pose.p.x(); p.y=pose.p.y(); p.z=pose.p.z();
      const Eigen::Quaterniond q(pose.rotation().matrix());
      auto& o=msg.pose.orientation;
      o.x=q.x(); o.y=q.y(); o.z=q.z(); o.w=q.w();
      return msg;
    };
    std::array<geometry_msgs::Point,7> points;
    const bool draw = !measured && markers_.getNumSubscribers();
    if (draw) points[0]=read("waist_yaw_link").pose.position;
    for (int arm=0;arm<2;++arm) {
      const std::string prefix = arm ? "zarm_r" : "zarm_l";
      if (ee_[measured][arm].getNumSubscribers()) ee_[measured][arm].publish(read(prefix+"7_end_effector"));
      if (wrist_[measured][arm].getNumSubscribers() || draw) {
        auto pose=read(prefix+"6_link");
        if (wrist_[measured][arm].getNumSubscribers()) wrist_[measured][arm].publish(pose);
        if (draw) {
          points[1+arm]=read(prefix+"2_link").pose.position;
          points[3+arm]=read(prefix+"4_link").pose.position;
          points[5+arm]=pose.pose.position;
        }
      }
    }
    if (!draw) return;
    visualization_msgs::MarkerArray array;
    for (int i=0;i<7;++i) {
      visualization_msgs::Marker m;
      m.header=header; m.id=i; m.type=m.SPHERE; m.action=m.ADD;
      m.ns=i==0 ? "wb_ref/chest" : i<=2 ? "wb_ref/shoulder" : i<=4 ? "wb_ref/elbow" : "wb_ref/hand";
      m.pose.orientation.w=1.; m.pose.position=points[i];
      m.scale.x=m.scale.y=m.scale.z=i==0?.10:i<=4?.08:.09;
      m.color.a=.9; m.color.r=i>=5; m.color.g=(i==0?.6:(i==3||i==4));m.color.b=i<=2;
      array.markers.push_back(m);
    }
    for (int i=0;i<2;++i) {
      visualization_msgs::Marker m;
      m.header=header;m.id=11+i;m.ns=i?"wb_ref/chain_right":"wb_ref/chain_left";
      m.type=m.LINE_LIST;m.action=m.ADD;m.pose.orientation.w=1.;m.scale.x=.012;
      m.color.a=.9;m.color.b=1.;m.color.r=i;m.color.g=1-i;
      m.points={points[0],points[1+i],points[1+i],points[3+i],points[3+i],points[5+i]};
      array.markers.push_back(m);
    }
    markers_.publish(array);
  }
 private:
  ros::Publisher markers_;
  std::array<std::array<ros::Publisher,2>,2> ee_, wrist_;
};
}  // namespace velocity_ik
