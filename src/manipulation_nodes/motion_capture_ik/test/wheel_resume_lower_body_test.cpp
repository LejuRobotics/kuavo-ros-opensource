#include "motion_capture_ik/WheelOneStageIKEndEffector.h"
#include "motion_capture_ik/drake_parser_compat.hpp"

#include <drake/multibody/parsing/parser.h>
#include <ros/time.h>
#include <iostream>

using namespace HighlyDynamic;

class SeededSolver : public WheelOneStageIKEndEffector {
 public:
  using WheelOneStageIKEndEffector::WheelOneStageIKEndEffector;
  void seed(const Eigen::VectorXd& q) { updateLatestSolution(q); }
};

int main() {
  ros::Time::init();  // Standalone solver test; no ROS master or robot commands.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("kuavo_assets", KUAVO_ASSETS_TEST_PATH);
  (void)motion_capture_ik::drake_parser_compat::AddUrdfModel(
      parser, std::string(KUAVO_ASSETS_TEST_PATH) + "/models/biped_s63/urdf/drake/biped_v3_arm.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"));
  plant.Finalize();

  // Static robot posture after paused TUI motion, taken from the simulation bag.
  Eigen::VectorXd measured(18);
  measured << 0.5235987756, -0.5235987756, 0.0, 0.0,
      0.547, -0.047, 0.258, -1.796, 0.838, -1.078, -0.645,
      -0.207, -0.382, 0.292, -1.168, -1.082, 0.249, -0.623;
  const Eigen::Vector4d previousVrCommand(0.07721914, -0.14237584, 0.08877445, 0.02744815);
  const std::vector<std::string> frames{"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector",
                                        "zarm_l4_link", "zarm_r4_link"};
  WheelPointTrackIKSolverConfig config;
  SeededSolver stale(&plant, frames, config);
  SeededSolver resumed(&plant, frames, config);
  std::vector<PoseData> refs(POSE_DATA_LIST_SIZE_PLUS);
  auto setRef = [&](int index, const std::string& name) {
    const auto pose = resumed.FK(measured, name);
    refs[index] = PoseData(pose.second.toRotationMatrix(), pose.first);
  };
  setRef(POSE_DATA_LIST_INDEX_CHEST, "waist_yaw_link");
  setRef(POSE_DATA_LIST_INDEX_LEFT_SHOULDER, "zarm_l1_link");
  setRef(POSE_DATA_LIST_INDEX_RIGHT_SHOULDER, "zarm_r1_link");
  setRef(POSE_DATA_LIST_INDEX_LEFT_ELBOW, "zarm_l4_link");
  setRef(POSE_DATA_LIST_INDEX_RIGHT_ELBOW, "zarm_r4_link");
  setRef(POSE_DATA_LIST_INDEX_LEFT_LINK6, "zarm_l6_link");
  setRef(POSE_DATA_LIST_INDEX_RIGHT_LINK6, "zarm_r6_link");
  setRef(POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR, "zarm_l7_end_effector");
  setRef(POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR, "zarm_r7_end_effector");
  setRef(POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB, "zarm_l7_virtual_thumb_link");
  setRef(POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB, "zarm_r7_virtual_thumb_link");

  stale.seed(measured);
  if (!stale.solveIK(refs, ArmIdx::BOTH).isSuccess) return 1;
  stale.activateLowerBodyFreeze(previousVrCommand);
  auto bad = stale.solveIK(refs, ArmIdx::BOTH);
  // Let the unchanged production LPF converge over a held-grip interval.
  for (int i = 0; i < 100; ++i) {
    if (!bad.isSuccess) return 1;
    bad = stale.solveIK(refs, ArmIdx::BOTH);
  }
  if (!bad.isSuccess) return 1;
  Eigen::VectorXd executed = bad.solution;
  executed.head<4>() = measured.head<4>();  // VR master off: physical lower body stays at TUI posture.
  const auto badEe = stale.FK(executed, "zarm_l7_end_effector").first;
  const double badError = (badEe - refs[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position).norm();
  if (badError < 0.1) {
    std::cerr << "Failed to reproduce stale lower-body compensation: " << badError << '\n';
    return 2;
  }

  resumed.seed(measured);
  // Populate the production solver's biquad/history at the stationary pose
  // before freezing, as in an already-running VR session.
  if (!resumed.solveIK(refs, ArmIdx::BOTH).isSuccess) return 3;
  resumed.activateLowerBodyFreeze(measured.head<4>());
  for (int i = 0; i < 3; ++i) {
    const auto good = resumed.solveIK(refs, ArmIdx::BOTH);
    if (!good.isSuccess || !good.solution.head<4>().isApprox(measured.head<4>(), 1e-6)) return 3;
    for (auto side : {LEFT, RIGHT}) {
      const int index = side == LEFT ? POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR : POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR;
      const auto ee = resumed.FK(good.solution, side == LEFT ? "zarm_l7_end_effector" : "zarm_r7_end_effector").first;
      if ((ee - refs[index].position).norm() > 0.002) {
        std::cerr << "Static hand moved after measured lower-body re-anchor: "
                  << (ee - refs[index].position).norm() << " m\n";
        return 4;
      }
    }
  }
  std::cout << "Stale lower-body error=" << badError
            << " m; measured freeze preserves both static EE targets\n";
  return 0;
}
