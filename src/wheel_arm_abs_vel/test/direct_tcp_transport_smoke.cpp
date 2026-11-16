#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <wheel_arm_abs_vel/TeleopTcpTarget.h>
#include <wheel_arm_abs_vel/VelocityIkInputDebug.h>
#include <wheel_arm_abs_vel/VelocityIkShadow.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <atomic>
namespace d=wheel_arm_abs_vel::direct_tcp;
using T=wheel_arm_abs_vel::TeleopTcpTarget;using S=wheel_arm_abs_vel::VelocityIkShadow;
void need(bool ok,const char* what){if(!ok)throw std::runtime_error(what);}
int main(int argc,char** argv){try{
 ros::init(argc,argv,"v7_transport_contract_smoke");ros::NodeHandle nh;need(argc>=2,"config arg required");d::Config c(argv[1]);d::RobotModel model(c);
 auto init=nh.advertiseService<std_srvs::Trigger::Request,std_srvs::Trigger::Response>("/humanoid_controller/real_initial_start",[&](auto&,auto& out){out.success=true;return true;});
 auto mode=nh.advertiseService<kuavo_msgs::changeArmCtrlMode::Request,kuavo_msgs::changeArmCtrlMode::Response>("/change_arm_ctrl_mode",[&](auto& req,auto& out){out.result=req.control_mode==2;return true;});
 auto rm=nh.advertiseService<kuavo_msgs::changeArmCtrlMode::Request,kuavo_msgs::changeArmCtrlMode::Response>("/wheel_arm_change_arm_ctrl_mode",[&](auto&,auto& out){out.result=true;return true;});
 auto refs=nh.advertise<S>("/whole_body_velocity_ik/diagnostics/reference",2);
 auto bones=nh.advertise<noitom_hi5_hand_udp_python::PoseInfoList>("/leju_quest_bone_poses",2);auto joys=nh.advertise<kuavo_msgs::JoySticks>("/quest_joystick_data",2);
 S ref;for(const auto& x:c.document.at("seed_joints").items()){ref.dof_names.push_back(x.key());ref.q.push_back(x.value());ref.dq.push_back(0.);}
 ref.measured_q=ref.q;ref.base_feedback_valid=true;ref.valid=true;ref.solver_valid=true;
 model.evaluate(ref.dof_names,ref.q,ref.dq);const auto j7=model.joint7();std::array<d::Pose,2> ctrl;
 for(int a=0;a<2;++a)ctrl[a]=j7[a].pose*c.arms[a].controller_to_virtual.inverse();
 std::atomic<int> qp_accepted{0},qp_rejected_source{0};
 auto debug=nh.subscribe<wheel_arm_abs_vel::VelocityIkInputDebug>("/v7_contract_ik/input_debug",20,[&](const wheel_arm_abs_vel::VelocityIkInputDebug::ConstPtr& s){qp_accepted=s->accepted_count;qp_rejected_source=s->rejected_source_count;});
 std::atomic<int> active{0},armed{0},inactive{0},invalid_state{0};std::atomic<bool> arm_held{false};std::atomic<uint64_t> first_session{0},last_session{0};std::mutex mutex;
 auto output=nh.subscribe<T>("/mm/teleop_tcp_target",100,[&](const T::ConstPtr& t){
  arm_held=t->arm_hold;
  if(t->control_state==T::CONTROL_ACTIVE){++active;need(t->header.frame_id=="ik_session_base","wire frame");need(t->left_elbow_hint_valid && t->right_elbow_hint_valid,"hints");need(!t->torso_control_enabled && !t->root_valid && t->chest_valid,"body contract");need(t->estimated_twists_valid,"twist");need(std::isfinite(t->left_tcp_pose.position.z),"finite");last_session=t->session_id;if(!first_session)first_session=last_session.load();}
  else if(t->control_state==T::CONTROL_ARMED)++armed;else if(t->control_state==T::CONTROL_INACTIVE)++inactive;else ++invalid_state;
  std::lock_guard<std::mutex> lock(mutex);ref.session_id=t->session_id;ref.generation=t->generation;ref.session_anchor_valid=true;ref.control_state=t->control_state;ref.takeover_active=t->control_state==T::CONTROL_ACTIVE;
 });
 ros::AsyncSpinner spinner(1);spinner.start();
 // Only this driver and the mapper run on a separate localhost ROS master.
 const auto deadline=ros::WallTime::now()+ros::WallDuration(10);
 while(bones.getNumSubscribers()!=1){need(ros::WallTime::now()<deadline,"mapper subscription missing");ros::WallDuration(.02).sleep();}
 auto phase=[&](double duration,int buttons,bool valid,bool transmit,int quaternion_fault=0){ros::WallRate rate(100);double start=ros::WallTime::now().toSec();while(ros::WallTime::now().toSec()-start<duration){
  {std::lock_guard<std::mutex> lock(mutex);ref.header.stamp=ros::Time::now();refs.publish(ref);}kuavo_msgs::JoySticks joy;
  if(buttons==1)joy.left_trigger=joy.right_trigger=1.;if(buttons==2)joy.left_first_button_pressed=joy.right_first_button_pressed=true;
  if(buttons==3)joy.left_first_button_pressed=joy.right_second_button_pressed=true;joys.publish(joy);
  if(transmit){noitom_hi5_hand_udp_python::PoseInfoList b;b.timestamp_ms=ros::Time::now().toNSec()/1000000;b.is_high_confidence=true;b.is_hand_tracking=!valid;b.poses.resize(24);b.poses[23].orientation.w=1.;b.poses[23].position.z=1.2;
   for(int a=0;a<2;++a){auto p=ctrl[a];const double t=ros::WallTime::now().toSec()-start;p.translation().x()+=.001*(1-std::cos(2*t));Eigen::Quaterniond q(p.linear());auto& out=b.poses[a+4];out.position.x=p.translation().x();out.position.y=p.translation().y();out.position.z=p.translation().z();out.orientation.x=q.x();out.orientation.y=q.y();out.orientation.z=q.z();out.orientation.w=q.w();}
   if(quaternion_fault==1){auto& q=b.poses[4].orientation;const double k=std::sqrt(.9999989434730134);q.x*=k;q.y*=k;q.z*=k;q.w*=k;}
   if(quaternion_fault==2)b.poses[4].orientation=geometry_msgs::Quaternion();
   if(quaternion_fault==3)b.poses[23].orientation=geometry_msgs::Quaternion();
   bones.publish(b);}
  rate.sleep();}};
 phase(.2,0,true,true);phase(.3,2,true,true);need(active==0 && armed==0,"X+A bypassed first triggers");phase(.1,0,true,true);phase(1.2,1,true,true);phase(.1,0,true,true);need(active>30 && armed>10,"first-trigger ARMED/ACTIVE handshake");
 phase(.1,0,false,true);const int stopped=active;phase(.3,0,true,true);need(active==stopped && inactive>0,"invalid tracking must exit without auto-resume");
 phase(1.,2,true,true);phase(.1,0,true,true);need(active>stopped+20 && last_session!=first_session,"explicit new session restart");
 phase(.3,0,true,false);const int lost=active;phase(.3,0,true,true);need(active==lost,"source loss auto resumed");
 phase(1.,2,true,true);phase(.1,0,true,true);phase(.2,3,true,true);need(arm_held,"X+B did not pause arms");const auto held_session=last_session.load();phase(.2,0,true,true);phase(.2,3,true,true);need(!arm_held && last_session==held_session,"X+B did not resume the same ACTIVE session");phase(.1,0,true,true);need(invalid_state==0,"unexpected HOLD/PREPARE wire state");
 phase(1.,2,true,true);phase(.1,0,true,true);
 const int before_roundoff=active;phase(.4,0,true,true,1);need(active>before_roundoff+20,"float quaternion drift interrupted tracking");
 phase(.1,0,true,true,2);const int bad_hand=active;phase(.3,0,true,true);need(active==bad_hand,"invalid hand auto resumed");need(bones.getNumSubscribers()==1,"invalid hand killed mapper");
 phase(1.,2,true,true);phase(.1,0,true,true);phase(.1,0,true,true,3);const int bad_chest=active;phase(.3,0,true,true);need(active==bad_chest,"invalid chest auto resumed");need(bones.getNumSubscribers()==1,"invalid chest killed mapper");
 phase(1.,2,true,true);phase(.1,0,true,true);need(active>bad_chest+20,"explicit restart after invalid pose failed");phase(.2,3,true,true);need(invalid_state==0,"invalid pose produced HOLD");
 need(qp_accepted>150 && qp_rejected_source==0,"Unmodified QP did not admit real-format targets/stop source");spinner.stop();std::cout<<"QP_accepted="<<qp_accepted<<" rejected_source="<<qp_rejected_source<<" ";std::cout<<"PASS active="<<active<<" armed="<<armed<<" inactive="<<inactive<<" no_hold="<<(invalid_state==0)<<" tracking_loss_exit=1 timeout_exit=1 explicit_restart=1 x_b_pause_resume=1 float_roundoff=1 invalid_hand_exit=1 invalid_chest_exit=1 process_survives=1\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}}
