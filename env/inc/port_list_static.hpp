#ifndef PORT_LIST_CONST_HPP
#define PORT_LIST_CONST_HPP

#include "env_utils.hpp"
#include "port_node.hpp"
#include "tuple.hpp"

template <typename... Ports> struct port_list_static_s : public std::tuple<Ports...> {
private:
  static constexpr bool are_ports = std::are_derived_from_v<port_base_s, typename Ports::base_s...>;
  static_assert(are_ports, "Only ports can be included to this list.");

public:
  static constexpr uint32_t size = sizeof...(Ports);
  using this_s = port_list_static_s<Ports...>;
  using base_s = std::tuple<Ports...>;

  template <typename... Names> explicit port_list_static_s(const Names &... names) : base_s{Ports(names)...} {};

  explicit port_list_static_s(const this_s &) = default;
  explicit port_list_static_s(this_s &&) = default;

  this_s &operator=(const this_s &) = delete;
  this_s &operator=(this_s &&) = delete;

  virtual ~port_list_static_s() = default;

  void setenv(const struct env_base_s *const p_env) const {
    for (unsigned int i = 0; i < size; i++)
      std::visit_at(
          i, [&p_env](const auto &port) -> void { port.setenv(p_env); }, *this);
  }

  void print_info() const {
    for (uint32_t i = 0; i < size; i++)
      std::visit_at(
          i, [](const auto &element) -> void { element.print_info(); }, *this);
  }

  const port_base_s &at(const std::string &name) const {
    const port_base_s *p_port;
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
