#include <robot_calibration/calibration_csv_io.h>

#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <sys/stat.h>
#include <iomanip>
#include <vector>

namespace robot_calibration
{
namespace
{

bool ensureDirExists(const std::string& dir)
{
  if (dir.empty())
    return false;

  // Create directory recursively (mkdir -p style).
  std::vector<std::string> parts;
  {
    std::stringstream ss(dir);
    std::string part;
    while (std::getline(ss, part, '/'))
    {
      if (!part.empty())
        parts.push_back(part);
    }
  }

  std::string cur;
  if (!dir.empty() && dir[0] == '/')
    cur = "/";

  for (size_t i = 0; i < parts.size(); ++i)
  {
    if (!cur.empty() && cur.back() != '/')
      cur += "/";
    cur += parts[i];

    struct stat st;
    if (stat(cur.c_str(), &st) == 0)
      continue;

    if (mkdir(cur.c_str(), 0755) != 0)
    {
      ROS_ERROR_STREAM("Failed to mkdir: " << cur);
      return false;
    }
  }

  return true;
}

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> out;
  std::string cur;
  std::stringstream ss(line);
  while (std::getline(ss, cur, ','))
    out.push_back(cur);
  return out;
}

double toDouble(const std::string& s, bool* ok = nullptr)
{
  try
  {
    size_t idx = 0;
    double v = std::stod(s, &idx);
    if (ok)
      *ok = (idx == s.size());
    return v;
  }
  catch (...)
  {
    if (ok)
      *ok = false;
    return 0.0;
  }
}

int toInt(const std::string& s, bool* ok = nullptr)
{
  try
  {
    size_t idx = 0;
    int v = std::stoi(s, &idx);
    if (ok)
      *ok = (idx == s.size());
    return v;
  }
  catch (...)
  {
    if (ok)
      *ok = false;
    return 0;
  }
}

CameraIntrinsicsRow getIntrinsicsRowForSensor(const robot_calibration_msgs::Observation& obs)
{
  CameraIntrinsicsRow row;

  // CameraInfo projection matrix P: [0]=fx, [2]=cx, [5]=fy, [6]=cy
  if (obs.ext_camera_info.camera_info.P.size() >= 12)
  {
    row.fx = obs.ext_camera_info.camera_info.P[0];
    row.fy = obs.ext_camera_info.camera_info.P[5];
    row.cx = obs.ext_camera_info.camera_info.P[2];
    row.cy = obs.ext_camera_info.camera_info.P[6];
  }

  for (size_t i = 0; i < obs.ext_camera_info.parameters.size(); i++)
  {
    const auto& p = obs.ext_camera_info.parameters[i];
    if (p.name == "z_scaling")
      row.z_scaling = p.value;
    else if (p.name == "z_offset_mm")
      row.z_offset_mm = p.value;
  }
  return row;
}

}  // namespace

bool writeCalibrationToCsv(const std::string& out_dir,
                            const std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  if (!ensureDirExists(out_dir))
    return false;

  const std::string joints_csv = out_dir + "/joints.csv";
  const std::string features_csv = out_dir + "/features.csv";
  const std::string camera_intrinsics_csv = out_dir + "/camera_intrinsics.csv";

  std::ofstream joints(joints_csv.c_str(), std::ios::out | std::ios::trunc);
  std::ofstream features(features_csv.c_str(), std::ios::out | std::ios::trunc);
  std::ofstream intrinsics(camera_intrinsics_csv.c_str(), std::ios::out | std::ios::trunc);

  if (!joints.is_open() || !features.is_open() || !intrinsics.is_open())
  {
    ROS_ERROR_STREAM("Failed to open csv files in dir: " << out_dir);
    return false;
  }

  joints << "sample_id,joint_name,position\n";
  features << "sample_id,sensor_name,feature_idx,frame_id,x,y,z\n";
  intrinsics << "sample_id,sensor_name,fx,fy,cx,cy,z_scaling,z_offset_mm\n";

  joints << std::setprecision(17);
  features << std::setprecision(17);
  intrinsics << std::setprecision(17);

  for (int sample_id = 0; sample_id < static_cast<int>(data.size()); ++sample_id)
  {
    const auto& msg = data[sample_id];
    if (msg.observations.empty())
    {
      ROS_WARN_STREAM("CSV sample " << sample_id << " has 0 observations (only joint_states will be written).");
    }
    else
    {
      std::stringstream ss;
      ss << "CSV sample " << sample_id << " observations: ";
      for (size_t oi = 0; oi < msg.observations.size(); ++oi)
      {
        const auto& obs = msg.observations[oi];
        if (oi > 0)
          ss << "; ";
        ss << "[" << oi << "] sensor=" << obs.sensor_name
           << ", features=" << obs.features.size();
      }
      ROS_INFO_STREAM(ss.str());
    }

    // joints.csv
    for (size_t i = 0; i < msg.joint_states.name.size() && i < msg.joint_states.position.size(); i++)
    {
      joints << sample_id << "," << msg.joint_states.name[i] << ","
             << msg.joint_states.position[i] << "\n";
    }

    // features.csv + camera_intrinsics.csv
    for (const auto& obs : msg.observations)
    {
      for (size_t fi = 0; fi < obs.features.size(); fi++)
      {
        const auto& fp = obs.features[fi];
        features << sample_id << "," << obs.sensor_name << "," << fi << ","
                 << fp.header.frame_id << ","
                 << fp.point.x << "," << fp.point.y << "," << fp.point.z << "\n";
      }

      // Always attempt to write intrinsics for this sensor observation (may be unused by some configs).
      // If P is empty, values will remain default 0/1.
      CameraIntrinsicsRow row = getIntrinsicsRowForSensor(obs);
      intrinsics << sample_id << "," << obs.sensor_name << ","
                 << row.fx << "," << row.fy << "," << row.cx << "," << row.cy << ","
                 << row.z_scaling << "," << row.z_offset_mm << "\n";
    }
  }

  joints.close();
  features.close();
  intrinsics.close();

  ROS_INFO_STREAM("Wrote calibration csv to: " << out_dir);
  return true;
}

bool readCalibrationFromCsv(const std::string& in_dir,
                             std::vector<robot_calibration_msgs::CalibrationData>& out_data,
                             std::vector<int>* out_sample_ids)
{
  out_data.clear();
  if (out_sample_ids)
    out_sample_ids->clear();

  const std::string joints_csv = in_dir + "/joints.csv";
  const std::string features_csv = in_dir + "/features.csv";
  const std::string camera_intrinsics_csv = in_dir + "/camera_intrinsics.csv";

  std::ifstream joints(joints_csv.c_str());
  std::ifstream features(features_csv.c_str());
  std::ifstream intrinsics(camera_intrinsics_csv.c_str());

  if (!joints.is_open() || !features.is_open() || !intrinsics.is_open())
  {
    ROS_ERROR_STREAM("Failed to open calibration csv in dir: " << in_dir);
    return false;
  }

  // sample_id -> index in out_data
  std::unordered_map<int, size_t> sample_map;

  auto getOrCreateSample = [&](int sample_id) -> robot_calibration_msgs::CalibrationData&
  {
    auto it = sample_map.find(sample_id);
    if (it == sample_map.end())
    {
      size_t idx = out_data.size();
      out_data.push_back(robot_calibration_msgs::CalibrationData());
      sample_map[sample_id] = idx;
      if (out_sample_ids)
        out_sample_ids->push_back(sample_id);
      return out_data.back();
    }
    return out_data[it->second];
  };

  // Read joints.csv first.
  {
    std::string line;
    bool header_skipped = false;
    while (std::getline(joints, line))
    {
      if (line.empty())
        continue;
      if (!header_skipped)
      {
        header_skipped = true;
        continue;  // skip header
      }

      auto cols = splitCsv(line);
      if (cols.size() < 3)
        continue;

      bool ok = false;
      int sample_id = toInt(cols[0], &ok);
      if (!ok)
        continue;

      const std::string joint_name = cols[1];
      double position = toDouble(cols[2]);

      robot_calibration_msgs::CalibrationData& sample = getOrCreateSample(sample_id);
      sample.joint_states.name.push_back(joint_name);
      sample.joint_states.position.push_back(position);
      sample.joint_states.velocity.push_back(0.0);
      sample.joint_states.effort.push_back(0.0);
    }
  }

  // camera intrinsics: sample_id -> sensor_name -> row
  std::unordered_map<int, std::unordered_map<std::string, CameraIntrinsicsRow>> intrinsics_map;
  {
    std::string line;
    bool header_skipped = false;
    while (std::getline(intrinsics, line))
    {
      if (line.empty())
        continue;
      if (!header_skipped)
      {
        header_skipped = true;
        continue;
      }

      auto cols = splitCsv(line);
      if (cols.size() < 8)
        continue;

      int sample_id = toInt(cols[0]);
      const std::string sensor_name = cols[1];

      CameraIntrinsicsRow row;
      row.fx = toDouble(cols[2]);
      row.fy = toDouble(cols[3]);
      row.cx = toDouble(cols[4]);
      row.cy = toDouble(cols[5]);
      row.z_scaling = toDouble(cols[6]);
      row.z_offset_mm = toDouble(cols[7]);

      intrinsics_map[sample_id][sensor_name] = row;
    }
  }

  // features.csv: sample_id,sensor_name,feature_idx,frame_id,x,y,z
  {
    std::string line;
    bool header_skipped = false;
    while (std::getline(features, line))
    {
      if (line.empty())
        continue;
      if (!header_skipped)
      {
        header_skipped = true;
        continue;
      }

      auto cols = splitCsv(line);
      if (cols.size() < 7)
        continue;

      int sample_id = toInt(cols[0]);
      const std::string sensor_name = cols[1];
      int feature_idx = toInt(cols[2]);
      const std::string frame_id = cols[3];
      double x = toDouble(cols[4]);
      double y = toDouble(cols[5]);
      double z = toDouble(cols[6]);

      robot_calibration_msgs::CalibrationData& sample = getOrCreateSample(sample_id);

      // Find or create observation for this sensor_name
      int obs_index = -1;
      for (size_t oi = 0; oi < sample.observations.size(); ++oi)
      {
        if (sample.observations[oi].sensor_name == sensor_name)
        {
          obs_index = static_cast<int>(oi);
          break;
        }
      }
      if (obs_index < 0)
      {
        robot_calibration_msgs::Observation obs;
        obs.sensor_name = sensor_name;
        sample.observations.push_back(obs);
        obs_index = static_cast<int>(sample.observations.size() - 1);

        // Attach camera intrinsics if available.
        auto it_sensor = intrinsics_map.find(sample_id);
        if (it_sensor != intrinsics_map.end())
        {
          auto it_row = it_sensor->second.find(sensor_name);
          if (it_row != it_sensor->second.end())
          {
            const CameraIntrinsicsRow& row = it_row->second;
            robot_calibration_msgs::ExtendedCameraInfo ext;

            // CameraInfo.P/K are fixed-size boost::array in ROS1.
            for (size_t pi = 0; pi < ext.camera_info.P.size(); ++pi)
              ext.camera_info.P[pi] = 0.0;
            for (size_t ki = 0; ki < ext.camera_info.K.size(); ++ki)
              ext.camera_info.K[ki] = 0.0;

            // P: [0]=fx [2]=cx [5]=fy [6]=cy
            ext.camera_info.P[0] = row.fx;
            ext.camera_info.P[2] = row.cx;
            ext.camera_info.P[5] = row.fy;
            ext.camera_info.P[6] = row.cy;

            // K: [0]=fx [2]=cx [4]=fy [5]=cy (optional)
            ext.camera_info.K[0] = row.fx;
            ext.camera_info.K[2] = row.cx;
            ext.camera_info.K[4] = row.fy;
            ext.camera_info.K[5] = row.cy;

            ext.parameters.resize(2);
            ext.parameters[0].name = "z_offset_mm";
            ext.parameters[0].value = row.z_offset_mm;
            ext.parameters[1].name = "z_scaling";
            ext.parameters[1].value = row.z_scaling;

            sample.observations[obs_index].ext_camera_info = ext;
          }
        }
      }

      if (feature_idx < 0)
        continue;
      if (static_cast<size_t>(feature_idx) >= sample.observations[obs_index].features.size())
        sample.observations[obs_index].features.resize(feature_idx + 1);

      geometry_msgs::PointStamped& fp = sample.observations[obs_index].features[feature_idx];
      fp.header.frame_id = frame_id;
      fp.header.stamp = ros::Time(0);
      fp.point.x = x;
      fp.point.y = y;
      fp.point.z = z;
    }
  }

  ROS_INFO_STREAM("Loaded calibration csv from: " << in_dir
                                                    << ", samples=" << out_data.size());
  return true;
}

}  // namespace robot_calibration

