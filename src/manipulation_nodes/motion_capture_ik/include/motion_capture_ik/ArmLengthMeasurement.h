#pragma once

#include <string>
#include <vector>

namespace HighlyDynamic {

class ArmLengthMeasurement {
 public:
  ArmLengthMeasurement()
      : measureArmLength_(false),
        measurementCompleted_(false),
        avgLeftUpperArmLength_(0.0),
        avgLeftLowerArmLength_(0.0),
        avgRightUpperArmLength_(0.0),
        avgRightLowerArmLength_(0.0) {}

  void setMeasureArmLength(bool measureArmLength) { measureArmLength_ = measureArmLength; }

  bool isMeasureArmLength() const { return measureArmLength_; }

  bool isMeasurementComplete() const { return measurementCompleted_; }

  void reset() {
    leftUpperArmLengths_.clear();
    leftLowerArmLengths_.clear();
    rightUpperArmLengths_.clear();
    rightLowerArmLengths_.clear();
    avgLeftUpperArmLength_ = 0.0;
    avgLeftLowerArmLength_ = 0.0;
    avgRightUpperArmLength_ = 0.0;
    avgRightLowerArmLength_ = 0.0;
    measureArmLength_ = false;
    measurementCompleted_ = false;
  }

  void updateMeasurement(double humanUpperArmLength, double humanLowerArmLength, const std::string &side);

  void completeMeasurement();

  double getAvgLeftUpperArmLength() const { return avgLeftUpperArmLength_; }

  double getAvgLeftLowerArmLength() const { return avgLeftLowerArmLength_; }

  double getAvgRightUpperArmLength() const { return avgRightUpperArmLength_; }

  double getAvgRightLowerArmLength() const { return avgRightLowerArmLength_; }

  size_t getLeftDataCount() const { return leftUpperArmLengths_.size(); }

  size_t getRightDataCount() const { return rightUpperArmLengths_.size(); }

 private:
  bool measureArmLength_;                     // 是否进行手臂长度测量
  bool measurementCompleted_;                 // 测量是否已完成
  static constexpr int armLengthNum_ = 30;    // 手臂长度测量样本数量
  std::vector<double> leftUpperArmLengths_;   // 左臂上臂长度数据
  std::vector<double> leftLowerArmLengths_;   // 左臂下臂长度数据
  std::vector<double> rightUpperArmLengths_;  // 右臂上臂长度数据
  std::vector<double> rightLowerArmLengths_;  // 右臂下臂长度数据
  double avgLeftUpperArmLength_;              // 左臂平均上臂长度
  double avgLeftLowerArmLength_;              // 左臂平均下臂长度
  double avgRightUpperArmLength_;             // 右臂平均上臂长度
  double avgRightLowerArmLength_;             // 右臂平均下臂长度
};

}  // namespace HighlyDynamic
