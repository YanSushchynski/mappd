#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include "component_node.hpp"
#include "port_list_static.hpp"
#include "runtime_list.hpp"

template <typename...> struct component_static_t : public component_node_t {};
template <uint32_t runtime_list_size, typename... Ports>
struct component_static_t<port_list_static_t<Ports...>, runtime_list_static_t<runtime_list_size>>
    : public component_node_t {
  using base_t = component_node_t;
  using this_t = component_static_t<Ports...>;
  using name_t = std::string;

  static constexpr uint32_t port_list_size = sizeof...(Ports);

  template <typename NameType, typename PortList, typename RuntimeList>
  explicit component_static_t(const NameType &name, const PortList &port_list, const RuntimeList &runtime_list)
      : base_t(name), port_list_(port_list), runtime_list_(runtime_list) {
    update_info();
  };

  virtual ~component_static_t() = default;

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

  void setenv(struct env_base_t *const p_env) const {
    static_cast<const base_t *>(this)->setenv(p_env);
    for (unsigned int i = 0; i < port_list_size; i++)
      std::visit_at(
          i, [&p_env](const auto &port) -> void { port.setenv(p_env); }, port_list_);
  }

  const port_list_static_t<Ports...> &ports() const { return port_list_; }
  const runtime_list_static_t<runtime_list_size> &runnables() const { return runtime_list_; }

private:
  const port_list_static_t<Ports...> port_list_;
  const runtime_list_static_t<runtime_list_size> runtime_list_;
};

template <typename NameType, uint32_t runtime_list_size, typename... Ports>
explicit component_static_t(const NameType &, const port_list_static_t<Ports...> &,
                            const runtime_list_static_t<runtime_list_size> &)
    ->component_static_t<port_list_static_t<Ports...>, runtime_list_static_t<runtime_list_size>>;

#endif /* COMPONENT_HPP */
