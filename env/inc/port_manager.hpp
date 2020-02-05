#ifndef PORT_MANAGER_HPP
#define PORT_MANAGER_HPP

#include "composition_list_static.hpp"
#include "port_manager_base.hpp"
#include "port_node.hpp"

template <typename... Compositions> struct port_manager_t : public port_manager_base_t {
  using base_t = port_manager_base_t;
  static constexpr uint32_t compositions_number = sizeof...(Compositions);

  explicit port_manager_t(const env_base_t *const p_env, const composition_list_static_t<Compositions...> &compositions)
      : base_t(), compositions_(compositions), env_(p_env){};

  virtual ~port_manager_t() = default;
  const env_base_t *const env() const override { return env_; };

  const port_node_t &find_port(const std::string &composition_name, const std::string &component_name,
                               const std::string &port_name) const override {
    const sha256::sha256_hash_type composition_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(composition_name.data()), composition_name.length());
    const sha256::sha256_hash_type component_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(component_name.data()), component_name.length());
    const sha256::sha256_hash_type port_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(port_name.data()), port_name.length());

    const std::lock_guard<std::mutex> lock(this->env()->get_lock());
    return *[&composition_name_hash, &component_name_hash, &port_name_hash, &composition_name, &component_name,
             &port_name](const composition_list_static_t<Compositions...> &compositions) -> const port_node_t * {
      port_node_t *result = nullptr;
      for (uint32_t i = 0; i < compositions_number; i++) {
        if (!result)
          std::visit_at(
              i,
              [&composition_name_hash, &component_name_hash, &port_name_hash,
               &result](auto const &composition) -> void {
                if (composition_name_hash == sha256::sha256_from_string(composition.info().name_hash())) {
                  for (uint32_t i = 0; i < composition.component_list_size; i++) {
                    if (!result)
                      std::visit_at(
                          i,
                          [&component_name_hash, &port_name_hash, &result](const auto &component) -> void {
                            if (component_name_hash == sha256::sha256_from_string(component.info().name_hash())) {
                              for (uint32_t i = 0; i < component.port_list_size; i++) {
                                if (!result)
                                  std::visit_at(
                                      i,
                                      [&port_name_hash, &result](const auto &port) -> void {
                                        if (port_name_hash == sha256::sha256_from_string(port.info().name_hash())) {
                                          result = (port_node_t *)&port;
                                        }
                                      },
                                      component.ports());
                              }
                            }
                          },
                          composition.components());
                  }
                }
              },
              compositions);
      }

      return (result) ? result
                      : throw std::runtime_error(
                            (boost::format("Port \"%3%\" in composition \"%1%\", component \"%2%\" not found\r\n") %
                             composition_name % component_name % port_name)
                                .str());
    }(compositions_);
  }

  const port_node_t &find_port(const std::string &composition_name, const std::string &port_name) const override {
    const sha256::sha256_hash_type composition_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(composition_name.data()), composition_name.length());
    const sha256::sha256_hash_type port_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(port_name.data()), port_name.length());
    const std::lock_guard<std::mutex> lock(this->env()->get_lock());

    return *[&composition_name_hash, &port_name_hash, &composition_name,
             &port_name](const composition_list_static_t<Compositions...> &compositions) -> const port_node_t * {
      port_node_t *result = nullptr;
      for (uint32_t i = 0u; i < compositions_number; i++) {
        if (!result)
          std::visit_at(
              i,
              [&composition_name_hash, &port_name_hash, &result](const auto &composition) -> void {
                if (composition_name_hash == sha256::sha256_from_string(composition.info().name_hash())) {
                  for (uint32_t i = 0u; i < composition.port_list_size; i++) {
                    if (!result)
                      std::visit_at(
                          i,
                          [&port_name_hash, &result](const auto &port) -> void {
                            if (port_name_hash == sha256::sha256_from_string(port.info().name_hash())) {
                              result = (port_node_t *)&port;
                            }
                          },
                          composition.ports());
                  }
                }
              },
              compositions);
      }

      return (result) ? result
                      : throw std::runtime_error((boost::format("Port \"%2%\" in composition \"%1%\" not found\r\n") %
                                                  composition_name % port_name)
                                                     .str());
    }(compositions_);
  }

  bool port_exists(const port_info_t &port) const override { return false; }
  bool is_connected(const port_info_t &first, const port_info_t &second) const override { return false; }
  void print_info() const override {
    const std::lock_guard<std::mutex> lock(this->env()->get_lock());

    for (uint32_t i = 0; i < compositions_number; i++)
      std::visit_at(
          i, [](const auto &composition) -> void { composition.print_info(); }, compositions_);
  }

private:
  const env_base_t *const env_;
  const composition_list_static_t<Compositions...> &compositions_;
};

#endif /* PORT_MANAGER_HPP */
