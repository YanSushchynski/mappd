#ifndef PORT_MANAGER_HPP
#define PORT_MANAGER_HPP

#include "composition_list_static.hpp"
#include "env_base.hpp"
#include "port_node.hpp"

template <typename...> struct pm_s {};

struct pm_sglt_gen_s {
  template <typename Composition, typename... Compositions>
  static pm_s<Composition, Compositions...> &
  pm_inst(const env_base_s *const p_env, const cmps_list_static_s<Composition, Compositions...> &compositions) {
    static struct pm_s<Composition, Compositions...> *inst_ = nullptr;
    if (!inst_) {
      inst_ = new pm_s<Composition, Compositions...>(p_env, compositions);
    }

    return *inst_;
  }
};

template <typename Composition, typename... Compositions> struct pm_s<Composition, Compositions...> {
private:
  using this_s = pm_s<Composition, Compositions...>;
  static constexpr uint32_t cmps_n = sizeof...(Compositions) + 1u;

  friend struct pm_sglt_gen_s;
  friend this_s &pm_sglt_gen_s::pm_inst(const env_base_s *const,
                                        const cmps_list_static_s<Composition, Compositions...> &);

public:
  explicit pm_s(const this_s &) = delete;
  explicit pm_s(this_s &&) = delete;
  this_s &operator=(const this_s &) = delete;
  this_s &operator=(this_s &&) = delete;
  virtual ~pm_s() = default;

  const env_base_s *const env() const { return env_; };
  const struct port_base_s &find_port(const std::string &composition_name, const std::string &component_name,
                                      const std::string &port_name) const {
    const sha256::sha256_hash_type composition_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(composition_name.data()), composition_name.length());
    const sha256::sha256_hash_type component_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(component_name.data()), component_name.length());
    const sha256::sha256_hash_type port_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(port_name.data()), port_name.length());

    const std::lock_guard<std::mutex> lock(this->env()->get_lock());
    return *[
      &composition_name_hash, &component_name_hash, &port_name_hash, &composition_name, &component_name, &port_name
    ](const cmps_list_static_s<Composition, Compositions...> &compositions) -> const struct port_base_s * {
      struct port_base_s *result = nullptr;
      for (uint32_t i = 0; i < cmps_n; i++) {
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
                                          result = (struct port_base_s *)&port;
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
    }
    (compositions_);
  }

  const struct port_base_s &find_port(const std::string &composition_name, const std::string &port_name) const {
    const sha256::sha256_hash_type composition_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(composition_name.data()), composition_name.length());
    const sha256::sha256_hash_type port_name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(port_name.data()), port_name.length());
    const std::lock_guard<std::mutex> lock(this->env()->get_lock());

    return *[&composition_name_hash, &port_name_hash, &composition_name, &port_name ](
               const cmps_list_static_s<Composition, Compositions...> &compositions) -> const struct port_base_s * {
      struct port_base_s *result = nullptr;
      for (uint32_t i = 0u; i < cmps_n; i++) {
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
                              result = (struct port_base_s *)&port;
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
    }
    (compositions_);
  }

  bool port_exists(const struct port_info_s &port) const { return false; }
  bool is_connected(const struct port_info_s &first, const struct port_info_s &second) const { return false; }
  void print_info() const {
    const std::lock_guard<std::mutex> lock(this->env()->get_lock());

    for (uint32_t i = 0; i < cmps_n; i++)
      std::visit_at(
          i, [](const auto &composition) -> void { composition.print_info(); }, compositions_);
  }

  std::mutex &get_lock() const { return mtx_; };

private:
  explicit pm_s(const env_base_s *const p_env, const cmps_list_static_s<Composition, Compositions...> &compositions)
      : compositions_(compositions), env_(p_env){};

  mutable std::mutex mtx_;
  const env_base_s *const env_;
  const cmps_list_static_s<Composition, Compositions...> &compositions_;
};

#endif /* PORT_MANAGER_HPP */
