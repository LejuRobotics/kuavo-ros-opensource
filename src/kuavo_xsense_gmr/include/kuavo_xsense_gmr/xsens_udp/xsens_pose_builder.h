#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_XSENS_POSE_BUILDER_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_XSENS_POSE_BUILDER_H

#include <string>

#include <kuavo_msgs/xsensePoseInfoList.h>

#include "kuavo_xsense_gmr/xsens_udp/elbow_landmark_cache.h"
#include "kuavo_xsense_gmr/xsens_udp/packet.h"

namespace kuavo_xsense_gmr
{

bool buildXsensPoseInfoList(const ParsedPacket& packet,
                            const ElbowLandmarkOffsets& elbow_offsets,
                            const std::string& frame_id,
                            kuavo_msgs::xsensePoseInfoList* msg,
                            std::string* error);

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_XSENS_POSE_BUILDER_H
