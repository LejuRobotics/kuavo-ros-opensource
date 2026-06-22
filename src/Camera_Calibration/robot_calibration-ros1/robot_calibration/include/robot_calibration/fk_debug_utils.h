#ifndef ROBOT_CALIBRATION_FK_DEBUG_UTILS_H
#define ROBOT_CALIBRATION_FK_DEBUG_UTILS_H

#include <string>

#include <sensor_msgs/JointState.h>

#include <robot_calibration_msgs/CalibrationData.h>

namespace robot_calibration
{

class CalibrationOffsetParser;

/**
 * @brief Log forward kinematics root->tip using URDF + KDL (same math as ChainModel).
 *
 * @param urdf_xml   Full robot_description string
 * @param js         Joint positions (names must cover the chain's joints)
 * @param root       Typically base_link
 * @param tip        e.g. head_camera_color_optical_frame or checkerboard_link
 * @param tag        Short label for the log line (e.g. pre_opt, post_opt, capture)
 * @param offsets    Joint/frame offsets; use default-constructed parser for nominal URDF FK
 */
void logChainFkFromUrdf(const std::string& urdf_xml,
                        const sensor_msgs::JointState& js,
                        const std::string& root,
                        const std::string& tip,
                        const std::string& tag,
                        const CalibrationOffsetParser& offsets);

/**
 * @brief Two lines only (no Kabsch): board in base from (1) URDF tag_to_base chain FK,
 *        (2) camera-chain FK times CSV camera observation of board origin (feature 0 =
 *        checkerboard frame origin, same as capture).
 */
void logBoardInBaseUrdfVsCameraCsv(const std::string& urdf_xml,
                                   const sensor_msgs::JointState& js,
                                   const std::string& fk_base,
                                   const std::string& fk_tip_camera,
                                   const std::string& fk_tip_board,
                                   const robot_calibration_msgs::CalibrationData& data,
                                   const std::string& camera_sensor_name,
                                   const CalibrationOffsetParser& offsets,
                                   const std::string& tag);

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_FK_DEBUG_UTILS_H
