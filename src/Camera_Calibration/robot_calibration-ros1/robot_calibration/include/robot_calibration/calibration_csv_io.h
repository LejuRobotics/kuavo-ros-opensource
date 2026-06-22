#ifndef ROBOT_CALIBRATION_CALIBRATION_CSV_IO_H
#define ROBOT_CALIBRATION_CALIBRATION_CSV_IO_H

#include <string>
#include <vector>

#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/ExtendedCameraInfo.h>

namespace robot_calibration
{

struct CalibrationCsvPaths
{
  std::string dir;
  std::string joints_csv;
  std::string features_csv;
  std::string camera_intrinsics_csv;
};

struct CameraIntrinsicsRow
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double z_scaling{1.0};
  double z_offset_mm{0.0};
};

/**
 * @brief Write a calibration dataset to CSV files under one directory.
 *
 * The dataset schema is:
 * - joints.csv: sample_id,joint_name,position
 * - features.csv: sample_id,sensor_name,feature_idx,frame_id,x,y,z
 * - camera_intrinsics.csv: sample_id,sensor_name,fx,fy,cx,cy,z_scaling,z_offset_mm
 */
bool writeCalibrationToCsv(const std::string& out_dir,
                            const std::vector<robot_calibration_msgs::CalibrationData>& data);

/**
 * @brief Load calibration dataset from the CSV directory.
 *
 * If @p out_sample_ids is non-null, it is resized to match @p out_data.size(), and
 * out_sample_ids[i] is the CSV sample_id column for out_data[i].
 */
bool readCalibrationFromCsv(const std::string& in_dir,
                             std::vector<robot_calibration_msgs::CalibrationData>& out_data,
                             std::vector<int>* out_sample_ids = nullptr);

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CALIBRATION_CSV_IO_H

