#ifndef UDP_SOCK_TYPE_HPP
#define UDP_SOCK_TYPE_HPP

#include <cstdint>

enum struct udp_sock_type_e : uint32_t {
  CLIENT_UNICAST = 1u,
  SERVER_UNICAST,
  CLIENT_MULTICAST,
  SERVER_MULTICAST,
  CLIENT_BROADCAST,
  SERVER_BROADCAST
};

#endif /* UDP_SOCK_TYPE_HPP */
