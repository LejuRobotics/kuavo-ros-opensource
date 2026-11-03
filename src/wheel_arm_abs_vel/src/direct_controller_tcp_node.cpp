#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <wheel_arm_abs_vel/DirectControllerInput.h>
#include <wheel_arm_abs_vel/TeleopTcpTarget.h>
#include <wheel_arm_abs_vel/VelocityIkShadow.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <ros/ros.h>
#include <fstream>
#include <optional>
#include <iostream>
#include <algorithm>
#include <chrono>

namespace d = wheel_arm_abs_vel::direct_tcp;
using Target=wheel_arm_abs_vel::TeleopTcpTarget;
using Shadow=wheel_arm_abs_vel::VelocityIkShadow;
using Input=wheel_arm_abs_vel::DirectControllerInput;
namespace {
void require(bool valid,const std::string& why){if(!valid)throw std::runtime_error(why);}
d::Json read(const std::string& path){std::ifstream in(path);require(in.good(),"Missing input: "+path);d::Json j;in>>j;return j;}
d::Sample from(const geometry_msgs::Pose& p,const geometry_msgs::Twist& t) {
  return d::sample({p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w},
      {t.linear.x,t.linear.y,t.linear.z},{t.angular.x,t.angular.y,t.angular.z});
}
geometry_msgs::Pose toPose(const d::Pose& p){
  geometry_msgs::Pose x;x.position.x=p.translation().x();x.position.y=p.translation().y();x.position.z=p.translation().z();
  Eigen::Quaterniond q(p.linear());x.orientation.x=q.x();x.orientation.y=q.y();x.orientation.z=q.z();x.orientation.w=q.w();return x;
}
geometry_msgs::Twist toTwist(const d::Sample& s){
  geometry_msgs::Twist t;t.linear.x=s.linear.x();t.linear.y=s.linear.y();t.linear.z=s.linear.z();t.angular.x=s.angular.x();t.angular.y=s.angular.y();t.angular.z=s.angular.z();return t;
}
d::Json encode(const d::Sample& s){
  Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w(),s.linear.x(),s.linear.y(),s.linear.z(),s.angular.x(),s.angular.y(),s.angular.z()};
}
d::Sample decode(const d::Json& a){return d::sample({a.at(0),a.at(1),a.at(2),a.at(3),a.at(4),a.at(5),a.at(6)},
      {a.at(7),a.at(8),a.at(9)},{a.at(10),a.at(11),a.at(12)});}
d::Json vec(const d::Vec& x){return {x.x(),x.y(),x.z()};}
std::array<d::RobotArm,2> seed(d::RobotModel& model,const d::Config& c) {
  std::vector<std::string> n;std::vector<double> q;
  for(const auto& x:c.document.at("seed_joints").items()){n.push_back(x.key());q.push_back(x.value());}
  return model.evaluate(n,q,std::vector<double>(q.size(),0.));
}
Target packet(const d::Config& config,const std::array<d::Sample,2>& s,const std::array<d::Sample,2>& elbow,bool geometric_hint,int control,std::uint64_t session,std::uint64_t gen,std::uint32_t seq){
  Target m;m.header.stamp=ros::Time::now();m.header.seq=seq;m.header.frame_id=config.output_frame;
  m.left_controlled_frame=config.controlled_frames[0];m.right_controlled_frame=config.controlled_frames[1];
  m.left_tcp_pose=toPose(s[0].pose);m.right_tcp_pose=toPose(s[1].pose);
  m.left_tcp_twist=toTwist(s[0]);m.right_tcp_twist=toTwist(s[1]);
  m.left_valid=m.right_valid=true;m.left_elbow_hint_valid=m.right_elbow_hint_valid=geometric_hint;
  m.left_elbow_hint.x=elbow[0].pose.translation().x();m.left_elbow_hint.y=elbow[0].pose.translation().y();m.left_elbow_hint.z=elbow[0].pose.translation().z();
  m.right_elbow_hint.x=elbow[1].pose.translation().x();m.right_elbow_hint.y=elbow[1].pose.translation().y();m.right_elbow_hint.z=elbow[1].pose.translation().z();
  m.left_elbow_velocity.x=elbow[0].linear.x();m.left_elbow_velocity.y=elbow[0].linear.y();m.left_elbow_velocity.z=elbow[0].linear.z();
  m.right_elbow_velocity.x=elbow[1].linear.x();m.right_elbow_velocity.y=elbow[1].linear.y();m.right_elbow_velocity.z=elbow[1].linear.z();
  m.source=Target::SOURCE_QUEST_CONTROLLER;m.estimated_twists_valid=true;
  // No human body channel is admitted. Optional hints are constructed from the
  // robot two-link geometry and the very same TCP target, never a human elbow.
  m.body_coordinate_epoch=1;
  m.mapper_receive_stamp=m.mapper_publish_stamp=m.header.stamp;m.session_id=session;m.generation=gen;
  m.absolute_state=2;m.control_state=control;m.control_valid=true;m.torso_control_enabled=false;return m;
}

class LiveNode {
 public:
  LiveNode(ros::NodeHandle& nh,const d::Config& c):config_(c),model_(c),pub_(nh.advertise<Target>("/mm/teleop_tcp_target",2)) {
    bool real;require(nh.getParam("/real",real) && !real,"Simulation-only direct controller candidate");
    if(config_.input_provenance=="VERIFIED_CONTROLLER_GRIP") accepted_provenance_=Input::VERIFIED_CONTROLLER_GRIP;
    else if(config_.input_provenance=="SYNTHETIC_FULL_URDF") accepted_provenance_=Input::SYNTHETIC_FULL_URDF;
    else throw std::runtime_error("Native input node requires verified grip or explicit synthetic provenance; historical labels are not accepted");
    ros::NodeHandle private_nh("~");std::string logfile;require(private_nh.getParam("trace_path",logfile),"Explicit ~trace_path required");trace_.open(logfile);require(trace_.good(),"Cannot open trace");
    states_=nh.subscribe<Shadow>("/whole_body_velocity_ik/diagnostics/reference",2,&LiveNode::reference,this);
    input_=nh.subscribe<Input>("/direct_controller_tcp/input",2,&LiveNode::input,this);
    watchdog_=nh.createWallTimer(ros::WallDuration(config_.source_gap_timeout/2.),[this](const ros::WallTimerEvent&){
      if(last_control_==Input::CONTROL_ARMED || last_control_==Input::CONTROL_ACTIVE){
        const double gap=std::chrono::duration<double>(std::chrono::steady_clock::now()-last_receive_).count();
        require(gap<=config_.source_gap_timeout,"Controller input timeout: terminating without cached-pose recovery");
      }
    });
  }
 private:
  void reference(const Shadow::ConstPtr& s){require(s->q.size()==s->dq.size(),"Invalid reference dimensions");reference_=*s;}
  void input(const Input::ConstPtr& p){
    const ros::Time received=ros::Time::now();const double age=(received-p->header.stamp).toSec();
    require(age>=0. && age<=config_.source_max_age,"INVALID_INPUT: source clock/age contract violated");
    require(p->header.frame_id==config_.input_frame && p->provenance==accepted_provenance_,"Input frame/provenance mismatch");
    require(!p->header.stamp.isZero() && p->header.stamp.toSec()>last_stamp_,"INVALID_INPUT: source timestamps must increase");last_stamp_=p->header.stamp.toSec();last_receive_=std::chrono::steady_clock::now();
    require(p->tracking_valid,"INVALID_INPUT: tracking invalid; process terminates and downstream stale-input braking retains authority");
    std::array<d::Sample,2> controller{from(p->controller_pose[0],p->controller_twist[0]),from(p->controller_pose[1],p->controller_twist[1])};
    switch(d::controlFromWire(p->control_state)){
      case d::Control::Inactive: {
        maps_.reset();last_control_=p->control_state;Target m;m.header.stamp=ros::Time::now();m.session_id=p->session_id;m.generation=p->generation;m.control_valid=false;m.control_state=p->control_state;pub_.publish(m);return;}
      case d::Control::Armed:case d::Control::Active:break;
      default:std::abort();
    }
    const auto& ref=reference_.value();auto robot=model_.evaluate(ref.dof_names,ref.q,ref.dq);
    const bool new_session=!maps_ || session_!=p->session_id || generation_!=p->generation;
    const bool activating=last_control_==Target::CONTROL_ARMED && p->control_state==Target::CONTROL_ACTIVE;
    if(new_session || activating){
      require(!new_session || p->control_state==Target::CONTROL_ARMED,"A new direct session must enter ARMED before ACTIVE");
      maps_.emplace(std::array<d::EngagedMapper,2>{
        d::EngagedMapper(config_.arms[0],model_.geometry()[0],controller[0],robot[0]),
        d::EngagedMapper(config_.arms[1],model_.geometry()[1],controller[1],robot[1])});
      session_=p->session_id;generation_=p->generation;engage_=p->header.stamp.toSec();
    }
    last_control_=p->control_state;
    std::array<d::Sample,2> command,elbow;d::Json row{{"stamp",p->header.stamp.toSec()},{"session",session_},{"generation",generation_}};
    for(int a=0;a<2;++a){const auto o=maps_.value()[a].advance(controller[a],robot[a],p->header.stamp.toSec()-engage_);command[a]=o.command;elbow[a]=o.elbow;row["elbow"][a]=encode(o.elbow);row["controller"][a]=encode(controller[a]);row["virtual"][a]=encode(o.virtual_tcp);row["raw"][a]=encode(o.raw);row["projected"][a]=encode(o.projected);row["command"][a]=encode(o.command);row["reference_wrist"][a]=vec(o.reference_wrist);row["dynamic_lower"][a]=vec(o.dynamic_lower);row["dynamic_upper"][a]=vec(o.dynamic_upper);row["position_filter"][a]=vec(o.position_filter);row["orientation_filter"][a]=vec(o.orientation_filter);row["position_projection"][a]=vec(o.position_projection);row["orientation_projection"][a]=vec(o.orientation_projection);row["requested_wrist"][a]=vec(o.requested_wrist);row["applied_wrist"][a]=vec(o.applied_wrist);}
    trace_<<row.dump()<<'\n';auto out=packet(config_,command,elbow,true,p->control_state,session_,generation_,p->header.seq);out.header.stamp=p->header.stamp;out.mapper_receive_stamp=received;out.mapper_publish_stamp=ros::Time::now();out.mapper_processing_time=(out.mapper_publish_stamp-received).toSec();out.source_relative_age=(out.mapper_publish_stamp-p->header.stamp).toSec();pub_.publish(out);
  }
  d::Config config_;d::RobotModel model_;ros::Publisher pub_;ros::Subscriber states_,input_;
  std::optional<Shadow> reference_;std::optional<std::array<d::EngagedMapper,2>> maps_;
  std::uint8_t accepted_provenance_;
  ros::WallTimer watchdog_;std::chrono::steady_clock::time_point last_receive_;
  std::uint64_t session_=0,generation_=0;double engage_=0.,last_stamp_=0.;std::uint8_t last_control_=Target::CONTROL_INACTIVE;std::ofstream trace_;
};

// This runner exercises the SAME C++ mapper and the existing ROS IK/WBC/MuJoCo.
// Historical cache conversion is a separate file-format operation; no mapping
// or metric calculation is delegated to Python.
int trial(ros::NodeHandle& nh,const d::Config& c,const d::Json& fixture,const std::string& mode,const std::string& result,const std::string& trial_type){
  require(mode=="v7" || mode=="legacy" || mode=="v2" || mode=="v3","Unknown trial variant");
  bool real;require(nh.getParam("/real",real) && !real,"Trial is simulation-only");
  require(trial_type=="episodes" || trial_type=="regression" || trial_type=="synthetic","Explicit trial type required");
  d::RobotModel model(c),feedback_model(c);
  const auto initial=seed(model,c);const auto& rows=fixture.at("samples");require(rows.size()>1,"Empty fixture");
  const double begin=rows.at(0).at("t"),end=rows.back().at("t");
  std::array<d::Sample,2> first_controller{decode(rows.at(0).at("controller").at(0)),decode(rows.at(0).at("controller").at(1))};
  std::optional<std::array<d::EngagedMapper,2>> maps;
  auto pub=nh.advertise<Target>("/mm/teleop_tcp_target",2);
  std::optional<Shadow> latest;std::string phase="startup";double phase_start=ros::WallTime::now().toSec(),source_time=begin;
  std::ofstream out(result);require(out.good(),"Cannot write trial result");
  out<<d::Json{{"kind","metadata"},{"trial_type",trial_type},{"config",c.document},{"frame",c.output_frame},{"tcp_frames",c.controlled_frames},{"model_audit",model.audit()}}.dump()<<'\n';
  auto sub=nh.subscribe<Shadow>("/whole_body_velocity_ik/diagnostics/reference",10,[&](const Shadow::ConstPtr& s){
    latest=*s;d::Json j{{"kind","reference"},{"phase",phase},{"phase_time",ros::WallTime::now().toSec()-phase_start},{"source_time",source_time},
      {"reference_time",(phase=="motion"?source_time-begin:end-begin+ros::WallTime::now().toSec()-phase_start)+c.engage_duration},{"stamp",s->header.stamp.toSec()},{"tracking",s->state==Shadow::TRACKING},{"solver_valid",s->solver_valid},{"fault",s->reference_tracking_fault_latched},
      {"state",s->state},{"primary_qp_return_code",s->primary_qp_return_code},{"dynamic_fault",int(s->dynamic_fault_latched)},{"q",s->q},{"dq",s->dq},{"ddq",s->ddq},{"measured_q",s->measured_q},{"joint_names",s->dof_names}};
    if(s->q.size()==21){
      auto fk=model.evaluate(s->dof_names,s->q,s->dq);auto fj=model.joint7();
      auto measured=feedback_model.evaluate(s->dof_names,s->measured_q,std::vector<double>(21,0.));auto mj=feedback_model.joint7();
      for(int a=0;a<2;++a){j["fk_reference"][a]=encode(fk[a].tcp);j["fk_feedback"][a]=encode(measured[a].tcp);j["j7_reference"][a]=encode(fj[a]);j["j7_feedback"][a]=encode(mj[a]);
        auto shaped=from(s->torso_relative_tcp_pose[a],geometry_msgs::Twist());shaped.pose=model.torso()*shaped.pose;j["shaped_tcp"][a]=encode(shaped);}
    }
    out<<j.dump()<<'\n';
  });
  const auto deadline=ros::WallTime::now()+ros::WallDuration(30);
  while(!latest || pub.getNumSubscribers()==0){require(ros::WallTime::now()<deadline,"IK input/diagnostics unavailable");ros::spinOnce();ros::WallDuration(.01).sleep();}
  require(ros::service::waitForService("/change_arm_ctrl_mode",ros::Duration(10)),"Arm mode service unavailable");
  auto client=nh.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");kuavo_msgs::changeArmCtrlMode request;request.request.control_mode=2;
  require(client.call(request) && request.response.result,"Arm mode handshake rejected");
  std::uint32_t seq=0;const auto session=std::uint64_t(ros::WallTime::now().toSec());
  auto run=[&](const std::string& name,double duration,int control,bool moving){
    phase=name;phase_start=ros::WallTime::now().toSec();std::cout<<phase<<std::endl;ros::WallRate rate(120);std::size_t index=0;
    while(ros::ok()){
      const double elapsed=ros::WallTime::now().toSec()-phase_start;if(elapsed>=duration)break;
      ros::spinOnce();source_time=moving?begin+elapsed:(name=="settle"?end:begin);
      while(index+1<rows.size() && rows.at(index+1).at("t").get<double>()<=source_time)++index;
      const auto& row=rows.at(index);std::array<d::Sample,2> command,elbow;d::Json trace{{"kind","mapper"},{"phase",phase},{"source_time",source_time},{"phase_time",elapsed},{"reference_time",(moving?elapsed:end-begin+elapsed)+c.engage_duration}};
      auto robot=model.evaluate(latest.value().dof_names,latest.value().q,latest.value().dq);
      for(int a=0;a<2;++a){
        auto controller=decode(row.at("controller").at(a));
        // Causal constant-velocity evaluation from the latest recorded sample.
        const double age=moving?source_time-row.at("t").get<double>():0.;
        controller.pose.translation()+=age*controller.linear;
        const double w=controller.angular.norm();if(w>0.)controller.pose.linear()=Eigen::AngleAxisd(age*w,controller.angular/w).toRotationMatrix()*controller.pose.linear();
        if(!moving){controller.linear.setZero();controller.angular.setZero();}
        trace["controller"][a]=encode(controller);trace["source_sample_time"]=row.at("t");
        trace["reference_time"]=(moving?elapsed:end-begin+elapsed)+c.engage_duration;trace["receive_time"]=ros::WallTime::now().toSec();
        elbow[a]=initial[a].elbow;
        if(mode=="v7"){
          if(moving || name=="settle"){
            const auto o=maps.value()[a].advance(controller,robot[a],(moving?elapsed:end-begin+elapsed)+c.engage_duration);
            command[a]=o.command;elbow[a]=o.elbow;
            trace["virtual"][a]=encode(o.virtual_tcp);trace["raw"][a]=encode(o.raw);trace["projected"][a]=encode(o.projected);
            trace["reference_wrist_rate"][a]=vec(o.reference_wrist_rate);trace["final_domain_lower"][a]=vec(o.final_domain_lower);trace["final_domain_upper"][a]=vec(o.final_domain_upper);trace["reference_wrist"][a]=vec(o.reference_wrist);trace["dynamic_lower"][a]=vec(o.dynamic_lower);trace["dynamic_upper"][a]=vec(o.dynamic_upper);trace["position_filter"][a]=vec(o.position_filter);trace["orientation_filter"][a]=vec(o.orientation_filter);trace["position_projection"][a]=vec(o.position_projection);trace["orientation_projection"][a]=vec(o.orientation_projection);trace["requested_wrist"][a]=vec(o.requested_wrist);trace["applied_wrist"][a]=vec(o.applied_wrist);
          }else command[a]=initial[a].tcp;
        }else{
          auto p=decode(row.at(mode).at(a));const auto p0=decode(rows.at(0).at(mode).at(a));
          // Common robot starting TCP and source instant for all variants.
          p.pose.translation()=initial[a].tcp.pose.translation()+p.pose.translation()-p0.pose.translation();
          p.pose.linear()=p.pose.linear()*p0.pose.linear().transpose()*initial[a].tcp.pose.linear();
          command[a]=moving || name=="settle" ? p:initial[a].tcp;
        }
        if(!moving && !(mode=="v7" && name=="settle")){command[a].linear.setZero();command[a].angular.setZero();}
        trace["elbow"][a]=encode(elbow[a]);trace["command"][a]=encode(command[a]);
        trace["j7_target"][a]=vec((command[a].pose*model.geometry()[a].j7_to_tcp.inverse()).translation());
      }
      out<<trace.dump()<<'\n';auto m=packet(c,command,elbow,mode=="v7",control,session,1,++seq);
      pub.publish(m);rate.sleep();
    }
    ros::spinOnce();
  };
  run("armed",1.,Target::CONTROL_ARMED,false);run("acquire",10.,Target::CONTROL_ACTIVE,false);
  require(latest.value().state==Shadow::TRACKING && latest.value().solver_valid && !latest.value().reference_tracking_fault_latched,"Acquisition failed");
  const auto anchored=model.evaluate(latest.value().dof_names,latest.value().q,latest.value().dq);
  out<<d::Json{{"kind","anchor"},{"shoulders",{vec(anchored[0].shoulder.pose.translation()),vec(anchored[1].shoulder.pose.translation())}},{"tcp",{encode(anchored[0].tcp),encode(anchored[1].tcp)}}}.dump()<<'\n';
  maps.emplace(std::array<d::EngagedMapper,2>{
    d::EngagedMapper(c.arms[0],model.geometry()[0],first_controller[0],anchored[0]),
    d::EngagedMapper(c.arms[1],model.geometry()[1],first_controller[1],anchored[1])});
  run("motion",end-begin,Target::CONTROL_ACTIVE,true);run("settle",3.,Target::CONTROL_ACTIVE,false);
  auto disabled=packet(c,{initial[0].tcp,initial[1].tcp},{initial[0].elbow,initial[1].elbow},false,Target::CONTROL_INACTIVE,session,1,++seq);disabled.control_valid=false;pub.publish(disabled);ros::WallDuration(.02).sleep();
  out.flush();return 0;
}

// Independent full-URDF FK generates the source. The mapper is not inverted.
d::Json synthetic(const d::Config& c){
  d::RobotModel model(c);std::vector<std::string> names;std::vector<double> seedq;
  for(const auto& x:c.document.at("seed_joints").items()){names.push_back(x.key());seedq.push_back(x.value());}
  const auto initial=model.evaluate(names,seedq,std::vector<double>(names.size(),0.));d::Json result;
  const auto& segments=c.document.at("synthetic_segments");double start=0.;
  for(const auto& seg:segments){
    const int joint=seg.at("joint");const double duration=seg.at("duration"),amp=seg.at("amplitude");
    result["segments"].push_back({{"name",seg.at("name")},{"start",start},{"end",start+duration},{"joint",joint}});
    for(int step=0;step<int(duration*120);++step){
      const double t=step/120.,f=.5*(1.-std::cos(2.*M_PI*t/duration)),df=M_PI/duration*std::sin(2.*M_PI*t/duration);
      auto q=seedq;std::vector<double> dq(q.size(),0.);
      for(int a=0;a<2;++a){const auto name=c.document.at("arms").at(a).at("arm_joints").at(joint-1).get<std::string>();
        const auto k=std::find(names.begin(),names.end(),name)-names.begin();require(k<std::ptrdiff_t(names.size()),"Synthetic joint missing from seed");
        const double mirror=c.document.at("synthetic_mirror").at(joint-1);const double sign=a==0?1.:mirror;q[k]+=sign*amp*f;dq[k]=sign*amp*df;
      }
      model.evaluate(names,q,dq);const auto fk=model.joint7();d::Json row{{"t",start+t}};
      for(int a=0;a<2;++a){
        require(c.arms[a].alignment.isApprox(d::Rot::Identity(),1.e-12),"Synthetic fixture uses declared identity tracking axes");
        d::Sample ctrl;ctrl.pose=fk[a].pose*c.arms[a].controller_to_virtual.inverse();ctrl.angular=fk[a].angular;
        ctrl.linear=fk[a].linear-ctrl.angular.cross(ctrl.pose.linear()*c.arms[a].controller_to_virtual.translation());row["controller"][a]=encode(ctrl);
      }
      result["samples"].push_back(row);
    }
    start+=duration;
  }
  auto last=result["samples"].at(0);last["t"]=start;result["samples"].push_back(last);
  result["provenance"]="SYNTHETIC_FULL_URDF_J7_WRIST_PIVOT; source independent of Mapper";return result;
}

enum class SmokeFault { None, InvalidQuaternion, UnsupportedControl, InputLoss };
int nativeSmoke(ros::NodeHandle& nh,const d::Config& c,SmokeFault fault){
  require(c.input_provenance=="SYNTHETIC_FULL_URDF","Native smoke requires explicitly synthetic configuration");
  auto publisher=nh.advertise<Input>("/direct_controller_tcp/input",2);
  std::optional<Shadow> reference;std::optional<Target> previous;int count=0;
  auto state_sub=nh.subscribe<Shadow>("/whole_body_velocity_ik/diagnostics/reference",2,[&](const Shadow::ConstPtr& s){reference=*s;});
  auto target_sub=nh.subscribe<Target>("/mm/teleop_tcp_target",2,[&](const Target::ConstPtr& t){
    require(t->header.frame_id=="direct_tcp_base" && t->left_elbow_hint_valid && t->right_elbow_hint_valid && !t->torso_control_enabled,"Native output contract violated");
    if(previous){const auto a=from(t->left_tcp_pose,t->left_tcp_twist),b=from(previous->left_tcp_pose,previous->left_tcp_twist);require((a.pose.translation()-b.pose.translation()).norm()<.002,"Native target jumped");}
    previous=*t;++count;
  });
  const auto deadline=ros::WallTime::now()+ros::WallDuration(10);
  while(!reference || publisher.getNumSubscribers()!=1){require(ros::WallTime::now()<deadline,"Native node/robot reference unavailable");ros::spinOnce();ros::WallDuration(.01).sleep();}
  d::RobotModel model(c);auto robot=model.evaluate(reference->dof_names,reference->q,reference->dq);
  const auto j7=model.joint7();std::array<d::Sample,2> anchors;
  for(int a=0;a<2;++a){require(c.arms[a].alignment.isApprox(d::Rot::Identity(),1.e-12),"Smoke source uses identity robot tracking axes");anchors[a]={j7[a].pose*c.arms[a].controller_to_virtual.inverse(),d::Vec::Zero(),d::Vec::Zero()};}
  const double start=ros::WallTime::now().toSec();std::uint32_t seq=0;ros::WallRate rate(120);Input last;
  while(ros::WallTime::now().toSec()-start<2.){
    const double t=ros::WallTime::now().toSec()-start;if(fault==SmokeFault::InputLoss && t>1.8)break;Input m;m.header.stamp=ros::Time::now();m.header.seq=++seq;m.header.frame_id=c.input_frame;m.provenance=Input::SYNTHETIC_FULL_URDF;
    m.session_id=std::uint64_t(start);m.generation=1;m.control_state=t<.5?Target::CONTROL_ARMED:Target::CONTROL_ACTIVE;m.tracking_valid=true;
    for(int a=0;a<2;++a){auto s=anchors[a];const double u=t<.5?0.:(t-.5)/1.5;s.pose.translation().y()+=.003*.5*(1.-std::cos(2*M_PI*u));s.linear.y()=.003*M_PI/1.5*std::sin(2*M_PI*u);m.controller_pose[a]=toPose(s.pose);m.controller_twist[a]=toTwist(s);}
    if(fault==SmokeFault::InvalidQuaternion && t>1.8)m.controller_pose[0].orientation=geometry_msgs::Quaternion();
    if(fault==SmokeFault::UnsupportedControl && t>1.8)m.control_state=3; // Deliberately rejected legacy HOLD wire value.
    last=m;publisher.publish(m);ros::spinOnce();rate.sleep();
  }
  require(count>100,"Native smoke produced too few target samples");
  target_sub.shutdown();
  if(fault==SmokeFault::None){last.header.stamp=ros::Time::now();last.header.seq=++seq;last.control_state=Input::CONTROL_INACTIVE;publisher.publish(last);ros::WallDuration(.05).sleep();}
  if(fault==SmokeFault::InputLoss)ros::WallDuration(c.source_gap_timeout*2.).sleep();
  std::cout<<"Native C++ input→mapper→TeleopTcpTarget smoke passed, samples="<<count<<" fault_sent="<<static_cast<int>(fault)<<std::endl;return 0;
}
}
int main(int argc,char** argv){
  // No catch around mapping or the callback loop: invalid input/config is a
  // visible process failure. ROS owner retains its existing stale-input brake.
  ros::init(argc,argv,"direct_controller_tcp");
  if(argc==1){ros::NodeHandle nh,private_nh("~");std::string path;require(private_nh.getParam("config",path),"Explicit ~config required");d::Config c(path);LiveNode node(nh,c);ros::spin();return 0;}
  require(argc>=3,"Usage: audit|synthetic|trial CONFIG [FIXTURE MODE RESULT]");
  const std::string action=argv[1];d::Config c(argv[2]);
  if(action=="audit"){d::RobotModel model(c);std::cout<<model.audit().dump(2)<<std::endl;return 0;}
  if(action=="synthetic"){require(argc==4,"synthetic CONFIG OUTPUT");std::ofstream out(argv[3]);require(out.good(),"Cannot write fixture");out<<synthetic(c).dump();return 0;}
  if(action=="trial"){require(argc==7,"trial CONFIG FIXTURE MODE RESULT TYPE");ros::NodeHandle nh;return trial(nh,c,read(argv[3]),argv[4],argv[5],argv[6]);}
  if(action=="native_smoke" || action=="native_invalid" || action=="native_reject_hold" || action=="native_input_loss"){require(argc==3,"native_smoke|native_invalid|native_reject_hold CONFIG");ros::NodeHandle nh;return nativeSmoke(nh,c,action=="native_smoke"?SmokeFault::None:(action=="native_invalid"?SmokeFault::InvalidQuaternion:(action=="native_reject_hold"?SmokeFault::UnsupportedControl:SmokeFault::InputLoss)));}
  std::abort();
}
