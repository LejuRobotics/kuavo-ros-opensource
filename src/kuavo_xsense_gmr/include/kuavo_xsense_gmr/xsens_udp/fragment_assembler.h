#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_FRAGMENT_ASSEMBLER_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_FRAGMENT_ASSEMBLER_H

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "kuavo_xsense_gmr/xsens_udp/parser.h"

namespace kuavo_xsense_gmr
{

class FragmentAssembler
{
public:
  explicit FragmentAssembler(double timeout_seconds = 1.0);

  bool addDatagram(const std::vector<uint8_t>& datagram,
                   std::vector<uint8_t>* complete_datagram,
                   std::string* error);

private:
  struct Key
  {
    uint8_t character_id = 0;
    std::string message_type;
    uint32_t sample_counter = 0;

    bool operator<(const Key& other) const;
  };

  struct PendingPacket
  {
    XsensPacketHeader first_header;
    std::map<uint8_t, std::vector<uint8_t> > payload_by_fragment;
    std::map<uint8_t, uint8_t> item_count_by_fragment;
    bool has_last_fragment = false;
    uint8_t last_fragment_index = 0;
    std::chrono::steady_clock::time_point last_update;
  };

  static std::vector<uint8_t> payloadOf(const std::vector<uint8_t>& datagram,
                                        const XsensPacketHeader& header);
  static void writeU16BE(uint16_t value, uint8_t* data);
  static bool buildDatagram(const PendingPacket& pending,
                            std::vector<uint8_t>* complete_datagram,
                            std::string* error);

  void cleanupExpired();

  XsensPacketParser parser_;
  std::chrono::steady_clock::duration timeout_;
  std::map<Key, PendingPacket> pending_;
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_FRAGMENT_ASSEMBLER_H
