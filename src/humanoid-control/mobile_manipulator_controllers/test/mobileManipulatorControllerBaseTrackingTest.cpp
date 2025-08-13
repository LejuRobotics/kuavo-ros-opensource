#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>
#include <fstream>
#include <iomanip>

using namespace ocs2;
using namespace mobile_manipulator_controller;

// 生成预定义的目标轨迹
TargetTrajectories generatePredefinedTrajectory(MobileManipulatorControllerBase& controller, double totalTime = 10.0, int numPoints = 100) {
  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;

  for (int i = 0; i < numPoints; i++) {
    double t = i * totalTime / (numPoints - 1);
    timeTrajectory.push_back(t);

    // 创建一个sinusoidal轨迹
    vector_t state = vector_t::Zero(20);
    // 基座位置
    state(0) = 0.0 * sin(0.5 * t);  // x
    state(1) = 0.0 * cos(0.5 * t);  // y
    state(2) = 0.0 * sin(0.8 * t);  // z
    // 基座姿态
    state(3) = 0.0 * sin(0.3 * t);  // roll
    state(4) = 0.0 * cos(0.4 * t);  // pitch
    state(5) = 0.0 * sin(0.2 * t);  // yaw
    
    // 左臂关节角度
    for (int j = 0; j < 7; j++) {
      state(6 + j) = 0.5 * sin(0.4 * t + j * 0.3) - M_PI/4;
    }
    // 右臂关节角度,只动肘关节
    state(6+7+3) = -M_PI/2 * (-cos(4 * t) + 1)/2;

    vector_t target = controller.getTargetFromState(state);
    stateTrajectory.push_back(target);
    inputTrajectory.push_back(vector_t::Zero(20));
  }

  return TargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory);
}

// CSV记录器类
class TrajectoryRecorder {
private:
  std::vector<double> timeData;
  std::vector<vector_t> nextStateData;
  std::vector<vector_t> eefPoseData;

public:
  void record(double time, const vector_t& nextState, const vector_t& eefPose) {
    timeData.push_back(time);
    nextStateData.push_back(nextState);
    eefPoseData.push_back(eefPose);
  }

  void saveToCSV(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("Cannot open file: " << filename);
      return;
    }

    // 写入表头
    file << "time";
    for (int i = 0; i < 20; i++) {
      file << ",state_" << i;
    }
    for (int i = 0; i < 14; i++) {  // 2 * 7 for left and right hand pose
      file << ",eef_" << i;
    }
    file << "\n";

    // 写入数据
    for (size_t i = 0; i < timeData.size(); i++) {
      file << std::fixed << std::setprecision(6) << timeData[i];
      for (int j = 0; j < nextStateData[i].size(); j++) {
        file << "," << nextStateData[i](j);
      }
      for (int j = 0; j < eefPoseData[i].size(); j++) {
        file << "," << eefPoseData[i](j);
      }
      file << "\n";
    }
    file.close();
    ROS_INFO_STREAM("Data saved to: " << filename);
  }

  void clear() {
    timeData.clear();
    nextStateData.clear();
    eefPoseData.clear();
  }
};

enum TestMode {
  SINGLE_FRAME = 0,
  TWO_FRAME_WINDOW = 1,
  ALL_FRAMES = 2,
  THREE_FRAME_WITH_PREDICTION = 3  // 新增模式
};

// 四元数辅助函数
Eigen::Quaterniond quaternionFromXYZW(double x, double y, double z, double w) {
  return Eigen::Quaterniond(w, x, y, z);
}

std::tuple<double, double, double, double> xyzwFromQuaternion(const Eigen::Quaterniond& q) {
  return std::make_tuple(q.x(), q.y(), q.z(), q.w());
}

Eigen::Quaterniond interpolateQuaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t) {
  return q1.slerp(t, q2);
}

Eigen::Quaterniond extrapolateQuaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double dt, double extrapolationTime) {
  // 计算相对旋转
  Eigen::Quaterniond delta_q = q1.conjugate() * q2;
  
  // 转换为角轴表示
  Eigen::AngleAxisd aa(delta_q);
  Eigen::Vector3d axis = aa.axis();
  double angle = aa.angle();
  
  // 计算角速度 (rad/s)
  double angularVelocity = angle / dt;
  
  // 计算外推角度
  double extrapolatedAngle = angularVelocity * extrapolationTime;
  
  // 创建外推的四元数
  Eigen::AngleAxisd extrapolatedAA(extrapolatedAngle, axis);
  return q2 * Eigen::Quaterniond(extrapolatedAA);
}

void runTrackingTest(MobileManipulatorControllerBase& controller, 
                     const TargetTrajectories& predefinedTrajectory,
                     TestMode mode,
                     const std::string& outputFilename) {
  
  TrajectoryRecorder recorder;
  ros::Rate rate(100);
  size_t count = 0;
  double dt = 0.01;  // 10ms
  
  vector_t externalState = vector_t::Zero(20);
  vector_t nextState = vector_t::Zero(20);
  vector_t optimizedInput = vector_t::Zero(20);
  
  // 初始化状态
  externalState = nextState;
  
  ROS_INFO_STREAM("Starting tracking test - Mode: " << mode);
  
  const int totalSteps = 1000;  // 10秒测试
  
  for (int step = 0; step < totalSteps && ros::ok(); step++) {
    double currentTime = step * dt;
    
    // 根据测试模式设置目标轨迹
    switch (mode) {
      case SINGLE_FRAME: {
        // 一次只发单帧
        int trajIndex = std::min(step / 10, 99);  // 每100ms更新一次轨迹点
        scalar_array_t timeSeq = {currentTime};
        vector_array_t stateSeq = {predefinedTrajectory.stateTrajectory[trajIndex]};
        vector_array_t inputSeq = {vector_t::Zero(20)};
        TargetTrajectories singleFrameTraj(timeSeq, stateSeq, inputSeq);
        controller.setTargetTrajectory(singleFrameTraj);
        break;
      }
      
      case TWO_FRAME_WINDOW: {
        // 一次发两帧窗口
        int trajIndex1 = std::min(step / 10, 99);
        int trajIndex2 = std::min(trajIndex1 + 1, 99);
        scalar_array_t timeSeq = {currentTime, currentTime + 0.1};
        vector_array_t stateSeq = {
          predefinedTrajectory.stateTrajectory[trajIndex1],
          predefinedTrajectory.stateTrajectory[trajIndex2]
        };
        vector_array_t inputSeq = {vector_t::Zero(20), vector_t::Zero(20)};
        TargetTrajectories twoFrameTraj(timeSeq, stateSeq, inputSeq);
        controller.setTargetTrajectory(twoFrameTraj);
        break;
      }
      
      case ALL_FRAMES: {
        // 一次发全部帧（仅在开始时发送一次）
        if (step == 0) {
          // 调整时间轴从当前时间开始
          scalar_array_t adjustedTimeSeq;
          for (size_t i = 0; i < predefinedTrajectory.timeTrajectory.size(); i++) {
            adjustedTimeSeq.push_back(currentTime + predefinedTrajectory.timeTrajectory[i]);
          }
          TargetTrajectories allFramesTraj(adjustedTimeSeq, 
                                         predefinedTrajectory.stateTrajectory, 
                                         predefinedTrajectory.inputTrajectory);
          controller.setTargetTrajectory(allFramesTraj);
        }
        break;
      }
      case THREE_FRAME_WITH_PREDICTION: {
        // 获取当前和下一时刻的目标索引
        int trajIndex1 = std::min(step / 10, 99);
        int trajIndex2 = std::min(trajIndex1 + 1, 99);
        
        // 获取当前和下一时刻的目标状态（14维末端位姿）
        vector_t target1 = predefinedTrajectory.stateTrajectory[trajIndex1];
        vector_t target2 = predefinedTrajectory.stateTrajectory[trajIndex2];
        
        // 计算时间间隔（确保不为零）
        double t1 = predefinedTrajectory.timeTrajectory[trajIndex1];
        double t2 = predefinedTrajectory.timeTrajectory[trajIndex2];
        double dt_traj = t2 - t1;
        if (dt_traj < 1e-6) dt_traj = 0.1;  // 默认0.1秒
        
        // 创建预测的目标状态
        vector_t predictedTarget = target2;
        
        // 处理左手姿态（索引3-6: x,y,z,w）
        Eigen::Quaterniond q1_left = quaternionFromXYZW(target1(3), target1(4), target1(5), target1(6));
        Eigen::Quaterniond q2_left = quaternionFromXYZW(target2(3), target2(4), target2(5), target2(6));
        Eigen::Quaterniond q_pred_left = extrapolateQuaternion(q1_left, q2_left, dt_traj, 1.0);
        auto [x_left, y_left, z_left, w_left] = xyzwFromQuaternion(q_pred_left);
        predictedTarget(3) = x_left;
        predictedTarget(4) = y_left;
        predictedTarget(5) = z_left;
        predictedTarget(6) = w_left;
        
        // 处理右手姿态（索引10-13: x,y,z,w）
        Eigen::Quaterniond q1_right = quaternionFromXYZW(target1(10), target1(11), target1(12), target1(13));
        Eigen::Quaterniond q2_right = quaternionFromXYZW(target2(10), target2(11), target2(12), target2(13));
        Eigen::Quaterniond q_pred_right = extrapolateQuaternion(q1_right, q2_right, dt_traj, 1.0);
        auto [x_right, y_right, z_right, w_right] = xyzwFromQuaternion(q_pred_right);
        predictedTarget(10) = x_right;
        predictedTarget(11) = y_right;
        predictedTarget(12) = z_right;
        predictedTarget(13) = w_right;
        
        // 构建三帧轨迹
        scalar_array_t timeSeq;
        timeSeq.push_back(currentTime);           // 第1帧：当前时刻
        timeSeq.push_back(currentTime + dt_traj); // 第2帧：下一目标点时刻
        timeSeq.push_back(currentTime + dt_traj + 1.0); // 第3帧：预测1秒后
        
        vector_array_t stateSeq;
        stateSeq.push_back(target1);
        stateSeq.push_back(target2);
        stateSeq.push_back(predictedTarget);
        
        vector_array_t inputSeq(3, vector_t::Zero(20));
        
        TargetTrajectories threeFrameTraj(timeSeq, stateSeq, inputSeq);
        controller.setTargetTrajectory(threeFrameTraj);
        break;
      }
    }
    
    // 执行一步MPC更新
    controller.update(externalState, nextState, optimizedInput);
    
    // 计算末端执行器位姿
    vector_t eefPose = controller.getMMEefPose(nextState);
    
    // 记录数据
    recorder.record(currentTime, nextState, eefPose);
    
    // 更新状态
    externalState = nextState;
    externalState.head(6).setZero();//base固定
    
    // 打印进度
    if (step % 100 == 0) {
      ROS_INFO_STREAM("Progress: " << (step * 100 / totalSteps) << "%");
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  // 保存数据到CSV
  recorder.saveToCSV(outputFilename);
  ROS_INFO_STREAM("Test completed for mode: " << mode);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "mobile_manipulator_controller_tracking_test");
  ros::NodeHandle nodeHandle;

  std::string taskFile, libFolder, urdfFile;
  MpcType mpcType = MpcType::SQP;

  nodeHandle.getParam("/mm/taskFile", taskFile);
  nodeHandle.getParam("/mm/libFolder", libFolder);
  nodeHandle.getParam("/mm/urdfFile", urdfFile);
  ROS_INFO_STREAM("taskFile: " << taskFile);
  ROS_INFO_STREAM("libFolder: " << libFolder);
  ROS_INFO_STREAM("urdfFile: " << urdfFile);

  MobileManipulatorControllerBase controller(nodeHandle, taskFile, libFolder, urdfFile, mpcType, 100, false, true, true);

  // 等待控制器初始化
  ros::Duration(2.0).sleep();
  
  // 生成预定义轨迹
  ROS_INFO("Generating predefined trajectory...");
  TargetTrajectories predefinedTrajectory = generatePredefinedTrajectory(controller);
  ROS_INFO_STREAM("Generated trajectory with " << predefinedTrajectory.size() << " points");

  // Save predefined EEF target trajectory to CSV
  {
    std::ofstream file("/tmp/predefined_eef_target.csv");
    if (!file.is_open()) {
      ROS_ERROR("Failed to open /tmp/predefined_eef_target.csv for writing.");
    } else {
      // Write header
      file << "time";
      for (int i = 0; i < 14; ++i) { // 2 * 7 for left and right hand pose
        file << ",eef_target_" << i;
      }
      file << "\n";

      for (size_t i = 0; i < predefinedTrajectory.timeTrajectory.size(); ++i) {
        double t = predefinedTrajectory.timeTrajectory[i];
        const auto& state = predefinedTrajectory.stateTrajectory[i];

        file << std::fixed << std::setprecision(6) << t;
        for (int j = 0; j < state.size()-6; ++j) {
          file << "," << state(6+j);
        }
        file << "\n";
      }
      file.close();
      ROS_INFO("Saved predefined EEF target trajectory to /tmp/predefined_eef_target.csv");
    }
  }

  // 运行三种测试模式
  std::vector<std::string> modeNames = {"single_frame", "two_frame_window", "all_frames", "three_frame_prediction"};
  
  for (int mode = 0; mode < 4; mode++) {
    ROS_INFO_STREAM("=== Starting Test Mode: " << modeNames[mode] << " ===");
    
    // 重置控制器到初始状态
    vector_t initialState = vector_t::Zero(20);
    controller.reset(initialState);
    ros::Duration(1.0).sleep();  // 等待重置完成
    
    std::string filename = "/tmp/tracking_test_" + modeNames[mode] + ".csv";
    runTrackingTest(controller, predefinedTrajectory, static_cast<TestMode>(mode), filename);
    
    ROS_INFO_STREAM("=== Completed Test Mode: " << modeNames[mode] << " ===");
    ros::Duration(1.0).sleep();  // 模式之间的间隔
  }

  ROS_INFO("All tracking tests completed!");
  ROS_INFO("CSV files saved to /tmp/ directory");
  ROS_INFO("Files: tracking_test_single_frame.csv, tracking_test_two_frame_window.csv, tracking_test_all_frames.csv, predefined_eef_target.csv");

  return 0;
}