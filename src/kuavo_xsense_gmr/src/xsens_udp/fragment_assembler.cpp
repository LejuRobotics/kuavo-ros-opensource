#include "kuavo_xsense_gmr/xsens_udp/fragment_assembler.h"

#include <algorithm>
#include <sstream>

namespace kuavo_xsense_gmr
{

FragmentAssembler::FragmentAssembler(const double timeout_seconds)
  : timeout_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_seconds)))
{
}

bool FragmentAssembler::Key::operator<(const Key& other) const
{
  if (character_id != other.character_id)
    return character_id < other.character_id;
  if (message_type != other.message_type)
    return message_type < other.message_type;
  return sample_counter < other.sample_counter;
}

bool FragmentAssembler::addDatagram(const std::vector<uint8_t>& datagram,
                                    std::vector<uint8_t>* complete_datagram,
                                    std::string* error)
{
  complete_datagram->clear();
  if (error != nullptr)
    error->clear();

  cleanupExpired();

  const XsensPacketHeader header = parser_.parseHeader(datagram);
  if (!header.valid)
  {
    *complete_datagram = datagram;
    return true;
  }

  if (header.fragment_index() == 0 && header.is_last_fragment())
  {
    *complete_datagram = datagram;
    return true;
  }

  const Key key{header.character_id, header.message_type, header.sample_counter};
  PendingPacket& pending = pending_[key];
  pending.last_update = std::chrono::steady_clock::now();
  if (header.fragment_index() == 0 || pending.payload_by_fragment.empty())
    pending.first_header = header;

  pending.payload_by_fragment[header.fragment_index()] = payloadOf(datagram, header);
  pending.item_count_by_fragment[header.fragment_index()] = header.number_of_items;

  if (header.is_last_fragment())
  {
    pending.has_last_fragment = true;
    pending.last_fragment_index = header.fragment_index();
  }

  if (!pending.has_last_fragment)
    return false;

  for (uint8_t index = 0; index <= pending.last_fragment_index; ++index)
  {
    if (pending.payload_by_fragment.find(index) == pending.payload_by_fragment.end())
      return false;
  }

  const bool ok = buildDatagram(pending, complete_datagram, error);
  pending_.erase(key);
  return ok;
}

std::vector<uint8_t> FragmentAssembler::payloadOf(const std::vector<uint8_t>& datagram,
                                                  const XsensPacketHeader& header)
{
  const std::size_t begin = XsensPacketHeader::kHeaderSize;
  const std::size_t end = std::min(datagram.size(), begin + header.payload_size);
  return std::vector<uint8_t>(datagram.begin() + begin, datagram.begin() + end);
}

void FragmentAssembler::writeU16BE(const uint16_t value, uint8_t* data)
{
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

bool FragmentAssembler::buildDatagram(const PendingPacket& pending,
                                      std::vector<uint8_t>* complete_datagram,
                                      std::string* error)
{
  std::vector<uint8_t> payload;
  uint32_t item_count = 0;

  for (uint8_t index = 0; index <= pending.last_fragment_index; ++index)
  {
    const auto payload_it = pending.payload_by_fragment.find(index);
    if (payload_it == pending.payload_by_fragment.end())
    {
      if (error != nullptr)
        *error = "missing Xsens UDP fragment";
      return false;
    }
    payload.insert(payload.end(), payload_it->second.begin(), payload_it->second.end());

    const auto item_it = pending.item_count_by_fragment.find(index);
    if (item_it != pending.item_count_by_fragment.end())
      item_count += item_it->second;
  }

  if (payload.size() > 65535U)
  {
    if (error != nullptr)
      *error = "reconstructed Xsens payload exceeds uint16 payload size";
    return false;
  }
  if (item_count > 255U)
  {
    if (error != nullptr)
      *error = "reconstructed Xsens item count exceeds uint8 item count";
    return false;
  }

  complete_datagram->assign(XsensPacketHeader::kHeaderSize, 0);
  std::copy(pending.first_header.id_string.begin(), pending.first_header.id_string.end(),
            complete_datagram->begin());

  (*complete_datagram)[6] =
      static_cast<uint8_t>((pending.first_header.sample_counter >> 24) & 0xFF);
  (*complete_datagram)[7] =
      static_cast<uint8_t>((pending.first_header.sample_counter >> 16) & 0xFF);
  (*complete_datagram)[8] =
      static_cast<uint8_t>((pending.first_header.sample_counter >> 8) & 0xFF);
  (*complete_datagram)[9] = static_cast<uint8_t>(pending.first_header.sample_counter & 0xFF);
  (*complete_datagram)[10] = 0x80;
  (*complete_datagram)[11] = static_cast<uint8_t>(item_count);
  (*complete_datagram)[12] =
      static_cast<uint8_t>((pending.first_header.time_code >> 24) & 0xFF);
  (*complete_datagram)[13] =
      static_cast<uint8_t>((pending.first_header.time_code >> 16) & 0xFF);
  (*complete_datagram)[14] =
      static_cast<uint8_t>((pending.first_header.time_code >> 8) & 0xFF);
  (*complete_datagram)[15] = static_cast<uint8_t>(pending.first_header.time_code & 0xFF);
  (*complete_datagram)[16] = pending.first_header.character_id;
  (*complete_datagram)[17] = pending.first_header.body_segments;
  (*complete_datagram)[18] = pending.first_header.props;
  (*complete_datagram)[19] = pending.first_header.finger_tracking_segments;
  writeU16BE(pending.first_header.reserved, &(*complete_datagram)[20]);
  writeU16BE(static_cast<uint16_t>(payload.size()), &(*complete_datagram)[22]);
  complete_datagram->insert(complete_datagram->end(), payload.begin(), payload.end());
  return true;
}

void FragmentAssembler::cleanupExpired()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_.begin(); it != pending_.end();)
  {
    if (now - it->second.last_update > timeout_)
      it = pending_.erase(it);
    else
      ++it;
  }
}

}  // namespace kuavo_xsense_gmr
