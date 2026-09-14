#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_PARSER_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_PARSER_H

#include <cstdint>
#include <vector>

#include "kuavo_xsense_gmr/xsens_udp/packet.h"

namespace kuavo_xsense_gmr
{

class XsensPacketParser
{
public:
  XsensPacketHeader parseHeader(const std::vector<uint8_t>& datagram) const;
  ParsedPacket parsePacket(const std::vector<uint8_t>& datagram) const;

  static uint16_t readU16BE(const uint8_t* data);
  static int32_t readI32BE(const uint8_t* data);
  static uint32_t readU32BE(const uint8_t* data);
  static float readF32BE(const uint8_t* data);
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_PARSER_H
