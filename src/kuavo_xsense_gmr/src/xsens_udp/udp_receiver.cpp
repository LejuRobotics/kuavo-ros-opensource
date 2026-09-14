#include "kuavo_xsense_gmr/xsens_udp/udp_receiver.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace kuavo_xsense_gmr
{

UdpReceiver::UdpReceiver()
  : socket_fd_(-1)
{
}

UdpReceiver::~UdpReceiver()
{
  close();
}

bool UdpReceiver::open(const std::string& listen_ip, uint16_t listen_port, std::string* error)
{
  close();

  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    if (error)
      *error = std::string("socket() failed: ") + std::strerror(errno);
    return false;
  }

  int reuse = 1;
  if (::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
  {
    if (error)
      *error = std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(errno);
    close();
    return false;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);

  if (listen_ip.empty() || listen_ip == "0.0.0.0")
  {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  else if (::inet_pton(AF_INET, listen_ip.c_str(), &addr.sin_addr) != 1)
  {
    if (error)
      *error = "invalid IPv4 listen_ip: " + listen_ip;
    close();
    return false;
  }

  if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    if (error)
      *error = std::string("bind() failed: ") + std::strerror(errno);
    close();
    return false;
  }

  return true;
}

void UdpReceiver::close()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool UdpReceiver::receive(std::vector<uint8_t>* datagram, int timeout_ms, std::string* error)
{
  if (!datagram)
    return false;

  datagram->clear();

  if (socket_fd_ < 0)
  {
    if (error)
      *error = "UDP socket is not open";
    return false;
  }

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(socket_fd_, &readfds);

  timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  const int ready = ::select(socket_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
  if (ready < 0)
  {
    if (errno == EINTR)
      return false;
    if (error)
      *error = std::string("select() failed: ") + std::strerror(errno);
    return false;
  }
  if (ready == 0)
    return false;

  std::vector<uint8_t> buffer(65535);
  const ssize_t received = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
  if (received < 0)
  {
    if (error)
      *error = std::string("recvfrom() failed: ") + std::strerror(errno);
    return false;
  }

  buffer.resize(static_cast<std::size_t>(received));
  *datagram = std::move(buffer);
  return true;
}

}  // namespace kuavo_xsense_gmr
