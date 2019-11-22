#ifndef PORT_LIST_CONST_HPP
#define PORT_LIST_CONST_HPP

#include "env_utils.hpp"
#include "port_node.hpp"
#include "tuple.hpp"

template <typename... Ports> struct port_list_static_t : public std::tuple<Ports...> {
private:
  static constexpr bool are_ports = std::are_derived_from_v<port_node_t, typename Ports::base_t...>;
  static_assert(are_ports, "Only ports can be included to this list.");

public:
  static constexpr uint32_t size = sizeof...(Ports);
  using this_t = port_list_static_t<Ports...>;
  using base_t = std::tuple<Ports...>;

  template <typename... Names> explicit port_list_static_t(const Names &... names) : base_t{Ports(names)...} {};

  explicit port_list_static_t(const this_t &) = default;
  explicit port_list_static_t(this_t &&) = default;

  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~port_list_static_t() = default;
  
  void setenv(const struct env_base_t *const p_env) const {
    for (unsigned int i = 0; i < size; i++)
      std::visit_at(
          i, [&p_env](const auto &port) -> void { port.setenv(p_env); }, *this);
  }

  void print_info() const {
    for (uint32_t i = 0; i < size; i++)
      std::visit_at(
          i, [](const auto &element) -> void { element.print_info(); }, *this);
  }

  template <typename NameType> const port_node_t &at(const NameType &name) const {
    const port_node_t *p_port;
    sha256::sha256_hash_type id = sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    bool found = false;
    for (uint32_t i = 0; i < size; i++)
      if (!found)
        std::visit_at(
            i,
            [&p_port, &id, &found](const auto &element) -> void {
              if (id == element.info().name_hash()) {
                p_port = &element;
                found = true;
              }
            },
            *this);
    return (found) ? *p_port : throw std::out_of_range("port_at");
  }
};

#endif /* PORT_LIST_CONST_HPP */
