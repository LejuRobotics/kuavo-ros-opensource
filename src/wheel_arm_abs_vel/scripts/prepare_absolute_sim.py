#!/usr/bin/env python3
"""Generate a private MuJoCo translation unit; never edit the shared source.

Exact anchors deliberately fail on upstream lifecycle changes requiring review.
The private ROS actuator adapter restores the joint-side position servo.
Shared sources, model assets and hardware executables remain untouched.
"""
import pathlib
import sys


def prepare(source):
    changes = [
        ('  ros::Subscriber jointCmdSub = g_nh_ptr->subscribe("/joint_cmd", 10, jointCmdCallback);',
         """  ros::Subscriber jointCmdSub = g_nh_ptr->subscribe("/joint_cmd", 10, jointCmdCallback);
  ros::Subscriber absoluteStopSub = g_nh_ptr->subscribe<std_msgs::Bool>(
      "/stop_robot", 1, [sim](const std_msgs::Bool::ConstPtr& stop) {
        if (stop->data && sim) {
          std::unique_lock<std::recursive_mutex> lock(sim->mtx);
          sim->run = false;
          ROS_ERROR("[AbsoluteSimulation] stop_robot: physics paused; explicit restart required");
        }
      });"""),
        ('#include "joint_address.hpp"', '#include "joint_address.hpp"\n#include "wheel_arm_abs_vel/SimulationServo.h"'),
        ('  std::vector<double> joint_tau_cmd;', '  std::vector<double> joint_tau_cmd;\n  AbsoluteSimulationServo absolute_servo;'),
        ('  // controlCommands.push(tau);', '  if (!absolute_servo.encode(*msg, tau)) { ROS_ERROR_THROTTLE(1.0, "Invalid absolute simulation servo command"); return; }\n  // controlCommands.push(tau);'),
        ('  InitRobotState(d);\n  sim->Load(m, d, filename);',
         """  InitRobotState(d);
  std::vector<int> absolute_actuators;
  for (const auto* group : {&LegJointsAddr, &LArmJointsAddr, &RArmJointsAddr, &HeadJointsAddr})
    for (int a : group->ctrladr()) absolute_actuators.push_back(a);
  { std::lock_guard<std::mutex> lock(queueMutex);
    absolute_servo.configure(m, d, *g_nh_ptr, absolute_actuators); }
  sim->Load(m, d, filename);"""),
        ('#include <chrono>', '#include <chrono>\nvoid absolute_stop_callbacks();'),
        ("    MujocoLcm mujocolcm;\n    mujocolcm.startLCMThread();",
         "    // ROS-only absolute simulation: no unused LCM receive worker."),
        ("  mj_deleteData(d);\n  mj_deleteModel(m);",
         "  // Model/data remain live until rendering and all workers finish."),
        ("  signal(SIGINT, signalHandler);",
         "  signal(SIGINT, signalHandler);\n  signal(SIGTERM, signalHandler);"),
        ("""    if (signum == SIGINT) // 捕获Ctrl+C信号
    {
      sim->exitrequest.store(1);
      if(g_dexhand_node) {
        g_dexhand_node->stop();
      }
\x20\x20\x20\x20\x20\x20
      std::cout << "Ctrl+C pressed, exit request sent." << std::endl;
    }""",
         """    // No joins, ROS calls, allocation or stream I/O in a signal handler.
    if (signum == SIGINT || signum == SIGTERM)
      sim->exitrequest.store(1);"""),
        ("  physicsthreadhandle.join();\n\n  return 0;",
         """  physicsthreadhandle.join();
  signal(SIGINT, SIG_IGN);
  signal(SIGTERM, SIG_IGN);
  absolute_stop_callbacks();
  if (g_dexhand_node) g_dexhand_node->stop();
  // Own process: stop callback workers before freeing model-backed resources.
  ros::shutdown();
  sim.reset();
  mj_deleteData(d); d = nullptr;
  mj_deleteModel(m); m = nullptr;
  std::cout << "[AbsoluteSimulation] workers joined, resources released" << std::endl;
  return 0;"""),
    ]
    for old, new in changes:
        if source.count(old) != 1:
            raise ValueError("MuJoCo lifecycle anchor changed; review required: " + old[:80])
        source = source.replace(old, new)
    return source


if __name__ == "__main__":
    pathlib.Path(sys.argv[2]).write_text(prepare(pathlib.Path(sys.argv[1]).read_text()))
