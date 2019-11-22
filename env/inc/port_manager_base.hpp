#ifndef PORT_MANAGER_BASE_HPP
#define PORT_MANAGER_BASE_HPP

#include "env_base.hpp"
#include "port_node.hpp"

struct port_manager_base_t {
public:
  explicit port_manager_base_t() = default;
  virtual ~port_manager_base_t() = default;

  virtual bool port_exists(const port_info_t &port) const = 0;
  virtual bool is_connected(const port_info_t &first, const port_info_t &second) const = 0;
  virtual const port_node_t &find_port(const std::string &composition_name, const std::string &component_name,
                                       const std::string &port_name) const = 0;
  virtual const port_node_t &find_port(const std::string &composition_name, const std::string &port_name) const = 0;
  virtual void print_info() const = 0;
  virtual const env_base_t *const env() const = 0;
  std::mutex &get_lock() const { return access_mtx_; };

private:
  mutable std::mutex access_mtx_;
};

#endif /* PORT_MANAGER_BASE_HPP */
