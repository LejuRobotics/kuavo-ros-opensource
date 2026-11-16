#include "wheel_arm_abs_vel/direct_tcp/transport.h"
#include <wheel_arm_abs_vel/TeleopTcpTarget.h>
#include <wheel_arm_abs_vel/VelocityIkShadow.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <ros/package.h>
#include "wheel_arm_abs_vel/direct_tcp_startup.h"
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <chrono>
#include <iostream>
#include <sched.h>
#include <sys/prctl.h>
#include <cstdlib>
namespace d=wheel_arm_abs_vel::direct_tcp;
using Target=wheel_arm_abs_vel::TeleopTcpTarget;
using Shadow=wheel_arm_abs_vel::VelocityIkShadow;
using Bones=noitom_hi5_hand_udp_python::PoseInfoList;
namespace {
void check(bool ok,const std::string& why){if(!ok)throw std::runtime_error(why);}
d::Sample from(const noitom_hi5_hand_udp_python::PoseInfo& p) {
 return d::questTransportSample({p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w});
}
geometry_msgs::Pose pose(const d::Pose& p){geometry_msgs::Pose o;const auto t=p.translation();Eigen::Quaterniond q(p.linear());o.position.x=t.x();o.position.y=t.y();o.position.z=t.z();o.orientation.x=q.x();o.orientation.y=q.y();o.orientation.z=q.z();o.orientation.w=q.w();return o;}
geometry_msgs::Twist twist(const d::Sample& s){geometry_msgs::Twist t;t.linear.x=s.linear.x();t.linear.y=s.linear.y();t.linear.z=s.linear.z();t.angular.x=s.angular.x();t.angular.y=s.angular.y();t.angular.z=s.angular.z();return t;}
geometry_msgs::Point point(const d::Vec& v){geometry_msgs::Point p;p.x=v.x();p.y=v.y();p.z=v.z();return p;}
d::Json encoded(const d::Sample& s){Eigen::Quaterniond q(s.pose.linear());return {s.pose.translation().x(),s.pose.translation().y(),s.pose.translation().z(),q.x(),q.y(),q.z(),q.w(),s.linear.x(),s.linear.y(),s.linear.z(),s.angular.x(),s.angular.y(),s.angular.z()};}
// Match the validated existing runtime partition on the actual C++ callback
// thread. The legacy "mapper" launch wrapper forcibly executes ik_ros_uni.py,
// so this node must never pass through that wrapper.
void scheduleMapperThread() {
  const char* raw=std::getenv("KUAVO_RUNTIME_PARTITION_JSON");
  if(!raw)return; // Isolated transport tests do not load a hardware partition.
  const auto p=d::Json::parse(raw);const auto cpus=p.at("mapper_cpus").get<std::vector<int>>();
  check(cpus.size()==1 && cpus[0]>=0 && cpus[0]<CPU_SETSIZE,"Invalid Mapper CPU");
  for(const char* key:{"wbc_cpus","driver_reserved_cpus","ik_cpus","background_cpus"})
    for(int cpu:p.at(key).get<std::vector<int>>())check(cpu!=cpus[0],"Mapper CPU overlaps reserved runtime role");
  const int priority=p.at("mapper_fifo_priority"),ik_priority=p.at("ik_fifo_priority");
  check(priority>0 && priority<ik_priority && ik_priority<90,"Invalid Mapper/IK FIFO ordering");
  cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpus[0],&mask);
  check(sched_setaffinity(0,sizeof(mask),&mask)==0,"Cannot bind Mapper CPU");
  sched_param sched{};sched.sched_priority=priority;check(sched_setscheduler(0,SCHED_FIFO,&sched)==0,"Cannot set Mapper FIFO");
  cpu_set_t actual;CPU_ZERO(&actual);sched_param got{};
  check(sched_getaffinity(0,sizeof(actual),&actual)==0 && CPU_EQUAL(&mask,&actual) && sched_getscheduler(0)==SCHED_FIFO && sched_getparam(0,&got)==0 && got.sched_priority==priority,"Mapper scheduling readback mismatch");
  prctl(PR_SET_NAME,"v9_mapper",0,0,0);ROS_INFO("V9 C++ callback CPU=%d FIFO=%d",cpus[0],priority);
}
class QuestNode {
 public:
  QuestNode(ros::NodeHandle& nh,const d::Config& config):nh_(nh),c_(config),model_(c_) {
    check(!c_.simulation_only,"Use the explicit 400062 deployment profile");
    while(ros::ok() && (!nh_.hasParam("/robot_version") || !nh_.hasParam("/real"))) {
      ROS_WARN_THROTTLE(5.,"V9 waiting for real controller /robot_version and /real; no target or initialization request is issued");
      ros::WallDuration(.1).sleep();
    }
    check(ros::ok(),"Shutdown while waiting for controller contract");
    validateRuntime();
    ros::NodeHandle pn("~");std::string path;check(pn.getParam("trace_path",path),"Missing trace_path");
    trace_.open(path,std::ios::app);check(trace_.good(),"Cannot open trace");
    trace_<<d::Json{{"kind","startup"},{"config",c_.document},{"model_audit",model_.audit()}}.dump()<<std::endl;
    pub_=nh_.advertise<Target>("/mm/teleop_tcp_target",2);
    hands_=nh_.advertise<geometry_msgs::Vector3Stamped>("/sg100/mapper_triggers",2);
    hand_driver_=nh_.advertise<std_msgs::Float32MultiArray>("/sg100/trigger",2);
    refs_=nh_.subscribe<Shadow>("/whole_body_velocity_ik/diagnostics/reference",2,[this](const Shadow::ConstPtr& r){reference_=*r;});
    bones_=nh_.subscribe<Bones>("/leju_quest_bone_poses",2,&QuestNode::input,this);
    joys_=nh_.subscribe<kuavo_msgs::JoySticks>("/quest_joystick_data",2,&QuestNode::joystick,this);
    initialize_=pn.advertiseService("initialize",&QuestNode::initialize,this);
    stop_=pn.advertiseService("stop",&QuestNode::stop,this);
    timer_=nh_.createWallTimer(ros::WallDuration(.02),[this](const ros::WallTimerEvent&){
      if(state_!=Target::CONTROL_INACTIVE && (ros::WallTime::now()-last_receive_).toSec()>c_.source_gap_timeout) {
        disable("Quest input timeout; no replay, no automatic resume");
      }
    });
    ROS_INFO("V9 hardware mapper ready: first follow uses both front triggers; later follow uses X+A; X+B pauses/resumes ACTIVE arms.");
  }
 private:
  template<class T> void equalParam(const std::string& name,const T& expected) {
    T value;check(nh_.getParam(name,value) && value==expected,"Runtime contract mismatch: "+name);
  }
  void validateRuntime() {
    equalParam<int>("/robot_version",400062);equalParam<int>("/absolute_velocity_ik/robot_version",400062);
    equalParam<bool>("/real",true);equalParam<bool>("/absolute_velocity_ik/enable_base_motion",false);
    equalParam<bool>("/absolute_velocity_ik/automatic_hold_enabled",false);
    equalParam<bool>("/absolute_velocity_ik/continuity/hold_on_timeout",false);
    equalParam<bool>("/absolute_velocity_ik/operating_envelope/enabled",true);
    equalParam<bool>("/absolute_velocity_ik/s6_motion_profile/enabled",true);
    equalParam<std::string>("/end_effector_type","heiman");
    equalParam<double>("/quest3/base_chest_offset_x",0.);equalParam<double>("/quest3/base_height_offset",.42);equalParam<double>("/quest3/shoulder_width",.15);
    std::string model;check(nh_.getParam("/whole_body_velocity_ik/model_contract_json",model),"Missing IK model contract");
    const auto j=d::Json::parse(model);check(j.at("robot_version")==400062 && j.at("velocity_ik_position_limits")==c_.document.at("position_limits"),"Model/operating bounds differ from mapper");
    auto a=j.at("velocity_ik_fixed_joints").get<std::vector<std::string>>(),b=c_.document.at("fixed_joints").get<std::vector<std::string>>();
    check(std::set<std::string>(a.begin(),a.end())==std::set<std::string>(b.begin(),b.end()),"Model fixed joint mismatch");
    const std::string prefix="/absolute_velocity_ik/";
    for(const auto& item:std::vector<std::pair<std::string,double>>{{"position_limit_margin",.01},{"boundary_brake_guard",.02},{"arm_boundary_execution_delay_wrist",.045},{"solver_rate_hz",500.}}) {
      double value=item.second;nh_.param(prefix+item.first,value,value);check(std::abs(value-item.second)<1e-12,"Stopping scalar mismatch: "+item.first);
    }
    bool separate=false;nh_.param(prefix+"separate_stopping_contract",separate,false);check(!separate,"Stopping profile mismatch");
    for(const std::string key:{"extra_position","extra_time"}) {
      std::vector<double> values;check(nh_.getParam(prefix+"operating_envelope/"+key,values) && values.size()>=18,"Missing operating array");
      for(int a=0;a<2;++a)for(int k=0;k<3;++k)check(std::abs(values.at(values.size()-14+a*7+4+k)-c_.document.at("arms")[a]["operating"][k][key].get<double>())<1e-12,"Stopping array mismatch");
    }
    for(const std::string key:{"left_tcp_translation","right_tcp_translation","left_tcp_rpy","right_tcp_rpy"}) {
      std::vector<double> values;if(nh_.getParam(prefix+key,values))check(values.size()==3 && std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x) && std::abs(x)<1e-12;}),"Unexpected additional TCP transform");
    }
    for(int k=0;k<3;++k){const auto& p=c_.document.at("arms")[0]["operating"][k];const std::string prefix="/absolute_velocity_ik/s6_motion_profile/j"+std::to_string(k+5)+"/";
      equalParam<double>(prefix+"velocity",p.at("vmax"));equalParam<double>(prefix+"acceleration",p.at("acceleration"));equalParam<double>(prefix+"jerk",p.at("jerk"));}
  }
  void solePublisher() {
    XmlRpc::XmlRpcValue args,result,payload;args[0]=ros::this_node::getName();
    check(ros::master::execute("getSystemState",args,result,payload,true),"Cannot check target publisher ownership");
    for(int i=0;i<payload[0].size();++i)if(std::string(payload[0][i][0])=="/mm/teleop_tcp_target") {
      check(payload[0][i][1].size()==1 && std::string(payload[0][i][1][0])==ros::this_node::getName(),"Another mapper publishes TeleopTcpTarget");
    }
  }
  bool initialize(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& out) {
    if(state_!=Target::CONTROL_INACTIVE){out.message="Exit following before initialization";return true;}
    std_srvs::Trigger call;auto client=nh_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");
    out.success=client.call(call) && call.response.success;out.message=call.response.message;
    initialized_|=out.success;return true;
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& out){disable("Explicit stop");out.success=true;return true;}
  bool mode(const std::string& service,int value) {
    auto client=nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>(service);kuavo_msgs::changeArmCtrlMode call;call.request.control_mode=value;
    return client.call(call) && call.response.result;
  }
  void disable(const char* reason) {
    state_=Target::CONTROL_INACTIVE;pending_start_=false;maps_.reset();
    Target t;t.header.stamp=ros::Time::now();t.source=Target::SOURCE_QUEST_CONTROLLER;t.header.frame_id="ik_session_base";t.left_controlled_frame=c_.controlled_frames[0];t.right_controlled_frame=c_.controlled_frames[1];t.body_coordinate_epoch=1;t.session_id=session_;t.generation=1;t.control_state=Target::CONTROL_INACTIVE;t.control_valid=false;pub_.publish(t);
    geometry_msgs::Vector3Stamped h;h.header.stamp=t.header.stamp;hands_.publish(h);
    // Leave stopping/hardware ownership with the existing controller. Never
    // request a cached pose, generate HOLD, or re-anchor automatically.
    ROS_WARN_STREAM("V9 inactive: "<<reason<<". New X+A required.");
    trace_<<d::Json{{"kind","inactive"},{"reason",reason},{"stamp",t.header.stamp.toSec()}}.dump()<<std::endl;
  }
  void joystick(const kuavo_msgs::JoySticks::ConstPtr& j) {
    const bool xa=j->left_first_button_pressed && j->right_first_button_pressed;
    const bool xb=j->left_first_button_pressed && j->right_second_button_pressed;
    const bool triggers=std::isfinite(j->left_trigger) && std::isfinite(j->right_trigger) && j->left_trigger>.5 && j->right_trigger>.5;
    const bool healthy=reference_ && (ros::Time::now()-reference_->header.stamp).toSec()>=0.
        && (ros::Time::now()-reference_->header.stamp).toSec()<c_.source_max_age
        && reference_->takeover_active && reference_->session_id==session_
        && !reference_->dynamic_fault_latched && !reference_->reference_tracking_fault_latched;
    const auto action=startup_.update(triggers,xa,xb,state_==Target::CONTROL_INACTIVE,
                                     state_==Target::CONTROL_ACTIVE,healthy);
    if(action==wheel_arm_abs_vel::DirectTcpStartup::ToggleArmHold){arm_hold_=!arm_hold_;++arm_operation_seq_;}
    if(action==wheel_arm_abs_vel::DirectTcpStartup::Start){pending_start_=true;request_time_=ros::WallTime::now();}
    geometry_msgs::Vector3Stamped h;h.header.stamp=ros::Time::now();
    if(state_==Target::CONTROL_ACTIVE && (ros::WallTime::now()-last_receive_).toSec()<c_.source_max_age && std::isfinite(j->left_trigger) && std::isfinite(j->right_trigger)){
      h.vector.x=std::clamp(double(j->left_trigger),0.,1.);h.vector.y=std::clamp(double(j->right_trigger),0.,1.);h.vector.z=1.;}
    hands_.publish(h);
    std_msgs::Float32MultiArray values;if(h.vector.z==1.)values.data={float(h.vector.x),float(h.vector.y)};hand_driver_.publish(values);
  }
  void rejectInput(const std::string& reason) {
    previous_.reset();pending_start_=false;
    if(state_!=Target::CONTROL_INACTIVE)disable(reason.c_str());
    ROS_WARN_STREAM_THROTTLE(1.,"V9 rejected input: "<<reason<<"; no target reused; explicit X+A required after exit");
  }
  void input(const Bones::ConstPtr& b) {
    const ros::Time receive=ros::Time::now();const double stamp=double(b->timestamp_ms)*.001;
    const double age=receive.toSec()-stamp;
    if(!b->is_high_confidence || b->is_hand_tracking || b->poses.size()<24 || !std::isfinite(stamp) || age<-.002 || age>c_.source_max_age){
      rejectInput("Invalid controller transport/mode/age");return;
    }
    if(stamp<=last_stamp_){rejectInput("Non-increasing receiver timestamp");return;}
    std::array<d::Sample,2> controller;d::Sample chest;
    try {controller={from(b->poses[4]),from(b->poses[5])};chest=from(b->poses[23]);}
    catch(const std::invalid_argument& error) {rejectInput(error.what());return;}
    const double dt=stamp-last_stamp_;
    if(previous_ && dt<=c_.source_gap_timeout)for(int a=0;a<2;++a)controller[a]=d::poseDerivative(controller[a],previous_.value()[a],dt);
    previous_=controller;last_stamp_=stamp;last_receive_=ros::WallTime::now();
    if(pending_start_ && (last_receive_-request_time_).toSec()>1.){pending_start_=false;ROS_WARN("X+A expired before fresh input/state; press again");}
    if(!reference_ || (receive-reference_->header.stamp).toSec()<0. || (receive-reference_->header.stamp).toSec()>c_.source_max_age){
      if(state_!=Target::CONTROL_INACTIVE)disable("IK reference missing or stale");return;
    }
    const auto& r=*reference_;
    if(state_==Target::CONTROL_ACTIVE && r.takeover_active && r.control_state==Target::CONTROL_ACTIVE
        && r.session_id==session_ && r.generation==1 && !r.dynamic_fault_latched && !r.reference_tracking_fault_latched)
      startup_.completed=true;
    if(r.dynamic_fault_latched || r.reference_tracking_fault_latched){if(state_!=Target::CONTROL_INACTIVE)disable("IK execution fault");pending_start_=false;return;}
    check(r.q.size()==r.dq.size() && r.q.size()==r.dof_names.size() && r.q.size()>=18,"Invalid IK joint layout");
    // Fixed-base IK publishes base coordinates zero. All moving robot joints
    // must be present exactly once; never substitute seed values for feedback.
    std::set<std::string> names(r.dof_names.begin(),r.dof_names.end());check(names.size()==r.dof_names.size(),"Duplicate IK joint names");
    for(const auto& x:c_.document.at("seed_joints").items())check(names.count(x.key())==1,"Missing robot feedback joint "+x.key());
    auto robot=model_.evaluate(r.dof_names,r.q,r.dq);const d::Sample torso=model_.torsoSample();
    if(pending_start_){
      solePublisher();validateRuntime();pending_start_=false;
      check(mode("/wheel_arm_change_arm_ctrl_mode",2),"RM mode-2 handshake rejected");
      if(!mode("/change_arm_ctrl_mode",2)) {mode("/wheel_arm_change_arm_ctrl_mode",1);throw std::runtime_error("WBC mode-2 handshake rejected");}
      state_=Target::CONTROL_ARMED;session_=ros::WallTime::now().toNSec();armed_at_=ros::WallTime::now();
      // Next fresh input captures the session, never the pre-RPC frame.
      return;
    }
    if(state_==Target::CONTROL_INACTIVE)return;
    if(state_==Target::CONTROL_ARMED && (ros::WallTime::now()-armed_at_).toSec()>.3 && r.session_id==session_ && r.generation==1 && r.session_anchor_valid){
      maps_.emplace(std::array<d::EngagedMapper,2>{d::EngagedMapper(c_.arms[0],model_.geometry()[0],controller[0],robot[0]),d::EngagedMapper(c_.arms[1],model_.geometry()[1],controller[1],robot[1])});
      engage_=stamp;state_=Target::CONTROL_ACTIVE;arm_hold_=false;++arm_operation_seq_;ROS_INFO("V9 ACTIVE; controller anchors captured from current fresh packet");
    }
    if(state_==Target::CONTROL_ARMED && (ros::WallTime::now()-armed_at_).toSec()>5.){disable("IK did not acknowledge ARMED session");return;}
    std::array<d::Sample,2> command,elbow;d::Json trace{{"kind","mapper"},{"stamp",stamp},{"session",session_},{"control_state",state_}};
    for(int a=0;a<2;++a){
      command[a]=robot[a].tcp;elbow[a]=robot[a].elbow;command[a].linear.setZero();command[a].angular.setZero();elbow[a].linear.setZero();
      if(state_==Target::CONTROL_ACTIVE){const auto o=maps_.value()[a].advance(controller[a],robot[a],stamp-engage_);command[a]=o.command;elbow[a]=o.elbow;trace["raw"][a]=encoded(o.raw);}
      trace["controller"][a]=encoded(controller[a]);trace["command_base"][a]=encoded(command[a]);
      const d::Vec shoulder=torso.pose.inverse()*robot[a].shoulder.pose.translation();const d::Vec source(0.,a==0?.15:-.15,.42);
      command[a]=d::sourceCoordinates(command[a],torso,shoulder,source);elbow[a]=d::sourceCoordinates(elbow[a],torso,shoulder,source);
      trace["command_wire"][a]=encoded(command[a]);
    }
    Target t;t.header.stamp.fromSec(stamp);t.header.seq=++seq_;t.header.frame_id="ik_session_base";
    t.source=Target::SOURCE_QUEST_CONTROLLER;t.session_id=session_;t.generation=1;t.control_state=state_;t.control_valid=true;t.absolute_state=2;t.arm_hold=arm_hold_;t.arm_operation_seq=arm_operation_seq_;
    t.left_valid=t.right_valid=t.left_elbow_hint_valid=t.right_elbow_hint_valid=t.estimated_twists_valid=true;
    t.left_controlled_frame=c_.controlled_frames[0];t.right_controlled_frame=c_.controlled_frames[1];
    t.left_tcp_pose=pose(command[0].pose);t.right_tcp_pose=pose(command[1].pose);t.left_tcp_twist=twist(command[0]);t.right_tcp_twist=twist(command[1]);
    t.left_elbow_hint=point(elbow[0].pose.translation());t.right_elbow_hint=point(elbow[1].pose.translation());
    t.left_elbow_velocity=twist(elbow[0]).linear;t.right_elbow_velocity=twist(elbow[1]).linear;
    // The unmodified IK requires a fresh chest sample even with torso disabled.
    // Pass through the observed chest; never synthesize a valid skeleton or use
    // this channel to generate hand targets. Body control remains disabled.
    t.body_tracking_frame=c_.input_frame;t.chest_pose=pose(chest.pose);t.chest_valid=true;t.body_high_confidence=true;
    t.body_coordinate_epoch=1;t.torso_control_enabled=false;t.mapper_receive_stamp=receive;t.mapper_publish_stamp=ros::Time::now();t.mapper_processing_time=(t.mapper_publish_stamp-receive).toSec();t.source_relative_age=(t.mapper_publish_stamp-t.header.stamp).toSec();
    // Do not refresh stale data with a publish timestamp after any blocking work.
    if(t.source_relative_age>c_.source_max_age){disable("Mapper sample became stale");return;}
    pub_.publish(t);trace_<<trace.dump()<<'\n';
  }
  ros::NodeHandle nh_;d::Config c_;d::RobotModel model_;ros::Publisher pub_,hands_,hand_driver_;ros::Subscriber refs_,bones_,joys_;ros::ServiceServer initialize_,stop_;ros::WallTimer timer_;
  std::optional<Shadow> reference_;std::optional<std::array<d::Sample,2>> previous_;std::optional<std::array<d::EngagedMapper,2>> maps_;std::ofstream trace_;
  ros::WallTime last_receive_,request_time_,armed_at_;double last_stamp_=0.,engage_=0.;uint64_t session_=0;uint32_t seq_=0;uint8_t state_=Target::CONTROL_INACTIVE;
  wheel_arm_abs_vel::DirectTcpStartup startup_;
  bool arm_hold_=false;uint64_t arm_operation_seq_=0;
  bool initialized_=false,pending_start_=false;
};
}
int main(int argc,char** argv){
 try{
  if(argc==3 && std::string(argv[1])=="--audit"){d::Config c(argv[2]);d::RobotModel m(c);std::cout<<m.audit().dump(2)<<std::endl;return 0;}
  ros::init(argc,argv,"direct_controller_tcp_v9");ros::NodeHandle nh,pn("~");std::string path;check(pn.getParam("config",path),"Missing ~config");d::Config c(path);QuestNode node(nh,c);scheduleMapperThread();ros::spin();return 0;
 }catch(const std::exception& e){
  // NodeHandles unwind before this handler; ROS logging may already be shut
  // down. Always preserve the actual exception on stderr as well.
  std::cerr<<"Direct TCP stopped: "<<e.what()<<std::endl;
  ROS_FATAL_STREAM("Direct TCP stopped: "<<e.what());return 2;
 }
}
