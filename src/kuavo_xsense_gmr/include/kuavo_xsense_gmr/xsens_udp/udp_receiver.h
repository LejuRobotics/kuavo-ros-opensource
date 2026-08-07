#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_UDP_RECEIVER_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_UDP_RECEIVER_H

#include <cstdint>
#include <string>
#include <vector>

namespace kuavo_xsense_gmr
{

class UdpReceiver
{
public:
  UdpReceiver();
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;

  bool open(const std::string& listen_ip, uint16_t listen_port, std::string* error);
  void close();
  bool receive(std::vector<uint8_t>* datagram, int timeout_ms, std::string* error);

private:
  int socket_fd_;
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_UDP_RECEIVER_H
