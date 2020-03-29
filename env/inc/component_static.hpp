#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include "component_node.hpp"
#include "port_list_static.hpp"
#include "runtime_list.hpp"

template <typename...> struct cmp_static_s : public cmp_base_s {};
template <uint32_t runtime_list_size, typename... Ports>
struct cmp_static_s<port_list_static_s<Ports...>, runtime_list_static_s<runtime_list_size>> : public cmp_base_s {
  using base_t = cmp_base_s;
  using this_t = cmp_static_s<Ports...>;
  using name_t = std::string;

  static constexpr uint32_t port_list_size = sizeof...(Ports);

  template <typename PortList, typename RuntimeList>
  explicit cmp_static_s(const std::string &name, const PortList &port_list, const RuntimeList &runtime_list)
      : base_t(name), port_list_(port_list), runtime_list_(runtime_list) {
    update_info();
  };

  virtual ~cmp_static_s() = default;

  void update_info() const {
    std::string type = typestr<this_t>;
    sha256::sha256_hash_type type_hash = sha256::compute(reinterpret_cast<const uint8_t *>(type.data()), type.length());
    this->info().set_component_type_hash(type_hash.data(), type_hash.size());
    this->update_info_base();
    for (unsigned int i = 0; i < port_list_size; i++)
      std::visit_at(
          i,
          [this](const auto &port) mutable -> void {
            port.info().set_composition_id(this->info().composition_id());
            port.info().set_component_id(this->info().id());
            port.update_info();
          },
          port_list_);
  }

  void operator()(int argc, char *argv[], const std::string &signal) const {
    const sha256::sha256_hash_type sighash =
        sha256::compute(reinterpret_cast<const uint8_t *>(signal.data()), signal.length());
    for (unsigned int i = 0; i < runtime_list_size; i++)
      [&sighash, &argc, &argv, this](const auto &runnable) -> void {
        if (std::get<1u>(runnable) == sighash)
          std::get<2u>(runnable)(argc, argv, this->env());
      }(runtime_list_[i]);
  }

  void operator()(int argc, char *argv[], const sha256::sha256_hash_type &signal_id) const {
    for (unsigned int i = 0; i < runtime_list_size; i++)
      [&signal_id, &argc, &argv, this](const auto &runnable) -> void {
        if (std::get<1u>(runnable) == signal_id)
          std::get<2u>(runnable)(argc, argv, this->env());
      }(runtime_list_[i]);
  }

  void setenv(struct env_base_s *const p_env) const {
    this->base_t::setenv(p_env);
    for (unsigned int i = 0; i < port_list_size; i++)
      std::visit_at(
          i, [&p_env](const auto &port) -> void { port.setenv(p_env); }, port_list_);
  }

  const port_list_static_s<Ports...> &ports() const { return port_list_; }
  const runtime_list_static_s<runtime_list_size> &runnables() const { return runtime_list_; }

private:
  const port_list_static_s<Ports...> port_list_;
  const runtime_list_static_s<runtime_list_size> runtime_list_;
};

template <uint32_t runtime_list_size, typename... Ports>
explicit cmp_static_s(const std::string &, const port_list_static_s<Ports...> &,
                      const runtime_list_static_s<runtime_list_size> &)
    ->cmp_static_s<port_list_static_s<Ports...>, runtime_list_static_s<runtime_list_size>>;

#endif /* COMPONENT_HPP */
