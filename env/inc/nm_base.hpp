#ifndef NETWORK_MANAGER_BASE_HPP
#define NETWORK_MANAGER_BASE_HPP

#include <string>
#include <functional>

enum struct nm_networking_t : uint32_t { IPV4, IPV6, IPV4_IPV6 };
struct nm_base_t {
  virtual void run() = 0;
  virtual void stop() = 0;
};

#endif /* NETWORK_MANAGER_BASE_HPP */ 
