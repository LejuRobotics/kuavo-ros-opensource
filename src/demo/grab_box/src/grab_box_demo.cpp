#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iterator>
#include "grab_box/package_path.h"
#include "grab_box/localization/closeToStart.hpp"
#include "grab_box/localization/checkStatusOK.hpp"
#include "grab_box/localization/closeToDestination.hpp"
#include "grab_box/localization/boxTagOK.hpp"
#include "grab_box/localization/computeTargetPose.hpp"
#include "grab_box/localization/computeBackTurnPoseFromBoxToTarget.hpp"
#include "grab_box/navigation/navToStart.hpp"
#include "grab_box/navigation/singleStepToDestination.hpp"
#include "grab_box/navigation/moveToDestination.hpp"
#include "grab_box/navigation/makePlan.hpp"
#include "grab_box/navigation/cmdPoseWorldMoveToDestination.hpp"
#include "grab_box/utils/echo.hpp"
#include "grab_box/grasp/graspBox.hpp"
#include "grab_box/grasp/armMoveToReadyPose.hpp"
#include "grab_box/grasp/armMoveToHomePose.hpp"
#include "grab_box/utils/forEachTag.hpp"
#include "grab_box/utils/sleepMs.hpp"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "grab_box/utils/forceCheck.hpp"
#include "grab_box/utils/timingDecorator.hpp"
#include "grab_box/navigation/cmdPoseMoveToDestination.hpp"
#include "grab_box/navigation/planAndMoveToDestination.hpp"
#include "grab_box/utils/getTagMap.hpp"
#include "grab_box/utils/delTagID.hpp"
#include "grab_box/utils/resetState.hpp"
#include "grab_box/utils/headMove.hpp"
#include "grab_box/utils/stopWalking.hpp"
#include <std_srvs/SetBool.h>
#include "grab_box/grasp/clawPart.hpp"
#include "grab_box/grasp/controlClaw.hpp"
#include "grab_box/utils/getPartPose.hpp"   
#include "grab_box/grasp/selectHandSide.hpp"
#include "grab_box/utils/filtered_logger.hpp"

void executeResetState(const BT::BehaviorTreeFactory& factory, const std::string& xml_string, BT::Blackboard::Ptr blackboard) {
  BT::BehaviorTreeFactory factory_reset = factory;
  std::cout << "Loading reset behavior tree from " << xml_string << std::endl;
  auto resetTree = factory_reset.createTreeFromFile(xml_string, blackboard);
  ros::Rate rate(100);
  while (ros::ok()) {
    BT::NodeStatus status = resetTree.tickRoot();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
    {
      ROS_INFO("Reset behavior tree finished with status: %s",
               status == BT::NodeStatus::SUCCESS? "SUCCESS" : "FAILURE");
      break;
    }
    rate.sleep();
    ros::spinOnce();
  }
}

// 定义函数类型
using ResetFunction = std::function<void()>;
ResetFunction globalResetFunc;

void signalHandler(int signum) {
    ROS_INFO("Catch interrupt signal, execute default reset action...");
    if (globalResetFunc) {
      globalResetFunc();  // 执行传入的复位函数
    }
    exit(signum);
}

// #include "grab_box/utils/CSVLogger.hpp"
using namespace GrabBox;

// 添加全局变量
bool g_is_running = false;
BT::Tree* g_tree_ptr = nullptr;

// 添加服务回调函数
bool controlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    g_is_running = req.data;
    if (g_is_running) {
      ROS_INFO("Start grab box demo");
      res.message = "Start grab box demo";
    } else {
      ROS_INFO("Stop grab box demo");
      res.message = "Stop grab box demo";
      // 当停止时，重置行为树
      if (g_tree_ptr) {
        g_tree_ptr->haltTree();
      }
      StopWalking stop_walking("StopWalking", BT::NodeConfiguration());
      stop_walking.tick();
    }
    res.success = true;
    return true;
}

bool resetCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("Reset grab box demo");
  res.message = "Reset grab box demo";
  if(req.data){
    if (globalResetFunc) {
      g_tree_ptr->haltTree();
      globalResetFunc();  // 执行传入的复位函数
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grab_box_demo");
  ros::NodeHandle nh("~");

  // Create a behavior tree factory and register the custom nodes
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CloseToStart>("CloseToStart");
  factory.registerNodeType<CheckStatusOK>("CheckStatusOK");
  factory.registerNodeType<NavToStart>("NavToStart");
  factory.registerNodeType<Echo>("Echo");
  factory.registerNodeType<SingleStepToDestination>("SingleStepToDestination");
  factory.registerNodeType<CloseToDestination>("CloseToDestination");
  factory.registerNodeType<MakePlan>("MakePlan");
  factory.registerNodeType<MoveToDestination>("MoveToDestination");
  factory.registerNodeType<PlanAndMoveToDestination>("PlanAndMoveToDestination");
  factory.registerNodeType<BoxTagOK>("BoxTagOK");
  factory.registerNodeType<GraspBox>("GraspBox");
  factory.registerNodeType<ComputeTargetPose>("ComputeTargetPose");
  factory.registerNodeType<ComputeBackTurnPoseFromBoxToTarget>("ComputeBackTurnPoseFromBoxToTarget");
  factory.registerNodeType<ArmMoveToReadyPose>("ArmMoveToReadyPose");
  factory.registerNodeType<ArmMoveToHomePose>("ArmMoveToHomePose");
  factory.registerNodeType<SleepMs>("SleepMs");
  factory.registerNodeType<ForEachTag>("ForEachTag");
  factory.registerNodeType<TimingDecorator>("TimingDecorator");
  factory.registerNodeType<ForceCheck>("ForceCheck");
  factory.registerNodeType<CmdPoseMoveToDestination>("CmdPoseMoveToDestination");
  factory.registerNodeType<CmdPoseWorldMoveToDestination>("CmdPoseWorldMoveToDestination");

  factory.registerNodeType<GetTagMap>("GetTagMap");
  factory.registerNodeType<DelTagID>("DelTagID");
  factory.registerNodeType<ResetState>("ResetState");
  factory.registerNodeType<HeadMove>("HeadMove");
  factory.registerNodeType<StopWalking>("StopWalking");
  factory.registerNodeType<ClawPart>("ClawPart");
  factory.registerNodeType<ControlClaw>("ControlClaw");
  factory.registerNodeType<GetPartPose>("GetPartPose");
  factory.registerNodeType<SelectHandSide>("SelectHandSide");

  RobotVersion rb_version(4, 2);

  if (nh.hasParam("/robot_version"))
  {
    int rb_version_int;
    nh.getParam("/robot_version", rb_version_int);
    rb_version = RobotVersion::create(rb_version_int);
  }
  auto humanoid_drake_interface = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);

  // Register the custom logger
  CSVLogger::getInstance().initialize("behavior_tree_log.csv");
  // Create a behavior tree from a string
  const std::string version_int_str = "kuavo_v" + rb_version.to_string();
  YAML::Node config = YAML::LoadFile(GrabBox::getPath() + "/cfg/" + version_int_str + "/bt_config.yaml");
  std::string bt_xml_string = config["bt_xml_file"].as<std::string>();
  std::string bt_xml_string_reset = config["bt_xml_file_reset"].as<std::string>();
  std::string path = GrabBox::getPath() + "/cfg/" + version_int_str + "/" + bt_xml_string;
  std::string path_reset = GrabBox::getPath() + "/cfg/" + version_int_str + "/" + bt_xml_string_reset;
  std::cout << "Loading behavior tree from " << path << std::endl;
  std::cout << "Loading reset behavior tree from " << path_reset << std::endl;
  // std::string path = GrabBox::getPath() + "/cfg/grab_box_demo.xml";
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set<YAML::Node>("config", config);
  blackboard->set<HighlyDynamic::HumanoidInterfaceDrake*>("humanoid_drake_interface", humanoid_drake_interface);

  auto tree = factory.createTreeFromFile(path, blackboard);
  g_tree_ptr = &tree;  // 保存树的指针以供服务回调使用
  
  // Load nodes to ignore from the YAML config file for the logger.
  std::vector<std::string> nodes_to_ignore;
  if (config["logger_nodes_to_ignore"])
  {
    ROS_INFO("Loading logger ignore list from bt_config.yaml");
    std::transform(config["logger_nodes_to_ignore"].begin(), 
                   config["logger_nodes_to_ignore"].end(),
                   std::back_inserter(nodes_to_ignore),
                   [](const YAML::Node& node) { return node.as<std::string>(); });
  }
  else
  {
    // Provide a default list if not specified in the YAML file.
    ROS_WARN("Key 'logger_nodes_to_ignore' not found in bt_config.yaml. Using default ignore list.");
    nodes_to_ignore = {};
  }
  GrabBox::FilteredCoutLogger logger_cout(tree, nodes_to_ignore);

  // 创建服务
  ros::ServiceServer service = nh.advertiseService("control_bt", controlCallback);
  ros::ServiceServer service_reset = nh.advertiseService("reset_bt", resetCallback);
  
  // 设置复位函数
  globalResetFunc = [&]() {
    executeResetState(factory, path_reset, blackboard);
  };
  // 注册信号处理函数
  signal(SIGINT, signalHandler);

  // Run the behavior tree until it returns SUCCESS or FAILURE
  ros::Rate rate(100);
  while (ros::ok())
  {
    if (g_is_running) {
      // Tick the behavior tree
      BT::NodeStatus status = tree.tickRoot();
      // Check the status of the behavior tree
      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
      {
        ROS_INFO("行为树执行完成，状态: %s",
                  status == BT::NodeStatus::SUCCESS? "SUCCESS" : "FAILURE");
        globalResetFunc();
        g_is_running = false;  // 执行完成后自动停止
      }
    }
    rate.sleep();
    ros::spinOnce();
  }

  CSVLogger::getInstance().close();

  return 0;
}
