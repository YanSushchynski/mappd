#ifndef NETWORK_MANAGER_BASE_HPP
#define NETWORK_MANAGER_BASE_HPP

#include <string>
#include <functional>

struct nm_base_t {
  virtual void run() const = 0;
  virtual void stop() const = 0;
};

#endif /* NETWORK_MANAGER_BASE_HPP */ 
