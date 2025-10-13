# Mobile Manipulator Detection Message
# This message provides structured anomaly detection results for mobile manipulator policy tracking

# Header for timestamping
Header header

# Overall anomaly detection result
bool hasAnomaly

# First type detection: Trajectory-level position error metrics
# Compares entire policy trajectory with target trajectory
float64 trajectoryPositionErrorMax
float64 trajectoryPositionErrorAvg
bool trajectoryPositionMaxThresholdExceeded
bool trajectoryPositionAvgThresholdExceeded

# First type detection: Trajectory-level orientation error metrics
# Compares entire policy trajectory with target trajectory
float64 trajectoryOrientationErrorMax
float64 trajectoryOrientationErrorAvg
bool trajectoryOrientationMaxThresholdExceeded
bool trajectoryOrientationAvgThresholdExceeded

# Second type detection: Velocity magnitude threshold detection
# Directly checks policy inputTrajectory velocity magnitudes
float64 velocityMagnitudeAvg
float64 velocityMagnitudeMax
bool velocityAvgThresholdExceeded
bool velocityMaxThresholdExceeded 