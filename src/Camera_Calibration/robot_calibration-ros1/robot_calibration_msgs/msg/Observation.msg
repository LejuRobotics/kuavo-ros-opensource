# Name of the "sensor" that generate this data.
string sensor_name

# Features "detected" by the sensor.
geometry_msgs/PointStamped[] features

# Optional 6DOF pose observations (e.g. mocap rigid-body pose).
# Populated by mocap calibration data; empty for camera/checkerboard captures.
geometry_msgs/PoseStamped[] poses

# Sensor information
ExtendedCameraInfo ext_camera_info

# Debugging data (optional)
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image image
