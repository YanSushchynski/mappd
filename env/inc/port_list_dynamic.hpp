#ifndef PORT_LIST_DYNAMIC_HPP
#define PORT_LIST_DYNAMIC_HPP

#include "env_utils.hpp"
#include "port_node.hpp"

struct port_list_dynamic_t : std::map<std::string, port_node_t> {
  template <template <typename...> class Container, typename NameType, typename... Ports>
  explicit port_list_dynamic_t(std::pair<NameType, Ports> &&... port_list){};

  virtual ~port_list_dynamic_t() = default;
};

#endif /* PORT_LIST_DYNAMIC_HPP */
