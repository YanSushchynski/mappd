#ifndef COMPOSITION_HPP
#define COMPOSITION_HPP

#include "component_list_static.hpp"
#include "composition_node.hpp"
#include "port_list_static.hpp"

template <typename...> struct composition_static_t {};

template <template <typename...> class ComponentList, typename... Components, template <typename...> class PortList,
          typename... Ports>
struct composition_static_t<PortList<Ports...>, ComponentList<Components...>> : public composition_node_t {
  using this_t = composition_static_t<PortList<Ports...>, ComponentList<Components...>>;
  using component_list_static_t = ComponentList<Components...>;
  using port_list_static_t = PortList<Ports...>;
  using base_t = composition_node_t;

  static constexpr uint32_t port_list_size = sizeof...(Ports);
  static constexpr uint32_t component_list_size = sizeof...(Components);

  template <typename NameType>
  explicit composition_static_t(const NameType &name, const port_list_static_t &ports,
                                const component_list_static_t &components)
      : base_t(name), component_list_(components), port_list_(ports) {
    update_info();
  }

  virtual ~composition_static_t() = default;

  void setenv(struct env_base_t *const p_env) const {
    this->base_t::setenv(p_env);
    component_list_.setenv(p_env);
    port_list_.setenv(p_env);
  }

  void update_info() const {

    sha256::sha256_hash_type type_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_t>.data()), typestr<this_t>.length());

    this->info().set_composition_type_hash(type_hash.data(), type_hash.size());
    this->base_t::update_info_base();
    for (unsigned int i = 0; i < port_list_size; i++)
      std::visit_at(
          i,
          [this](const auto &port) mutable -> void {
            port.info().set_composition_id(this->info().id());
            port.info().set_component_id(this->info().id());
            port.update_info();
          },
          port_list_);

    for (unsigned int i = 0; i < component_list_size; i++)
      std::visit_at(
          i,
          [this](const auto &component) mutable -> void {
            component.info().set_composition_id(this->info().id());
            component.update_info();
          },
          component_list_);
  }

  const component_list_static_t &components() const { return component_list_; }
  const port_list_static_t &ports() const { return port_list_; }

private:
  const component_list_static_t component_list_;
  const port_list_static_t port_list_;
};

template <template <typename...> class CompositionList, typename... Compositions,
          template <typename...> class ComponentList, typename... Components, template <typename...> class PortList,
          typename... Ports>
struct composition_static_t<PortList<Ports...>, ComponentList<Components...>, CompositionList<Compositions...>>
    : public composition_node_t {
  using this_t = composition_static_t<PortList<Ports...>, ComponentList<Components...>>;
  using composition_list_static_t = CompositionList<Compositions...>;
  using component_list_static_t = ComponentList<Components...>;
  using port_list_static_t = PortList<Ports...>;
  using base_t = composition_node_t;

  static constexpr uint32_t port_list_size = sizeof...(Ports);
  static constexpr uint32_t component_list_size = sizeof...(Components);
  static constexpr uint32_t composition_list_size = sizeof...(Compositions);

  template <typename NameType>
  explicit composition_static_t(const NameType &name, const port_list_static_t &ports,
                                const component_list_static_t &components,
                                const composition_list_static_t &compositions)
      : base_t(name), composition_list_(compositions), component_list_(components), port_list_(ports) {
    update_info();
  }

  virtual ~composition_static_t() = default;

  void update_info() const {

    sha256::sha256_hash_type type_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_t>.data()), typestr<this_t>.length());
    this->info().set_composition_type_hash(type_hash.data(), type_hash.size());
    this->update_info_base();
    for (unsigned int i = 0; i < port_list_size; i++)
      std::visit_at(
          i,
          [this](const auto &port) mutable -> void {
            port.info().set_composition_id(this->info().id());
            port.info().set_component_id(this->info().id());
            port.update_info();
          },
          port_list_);

    for (unsigned int i = 0; i < component_list_size; i++)
      std::visit_at(
          i,
          [this](const auto &component) mutable -> void {
            component.info().set_composition_id(this->info().id());
            component.update_info();
          },
          component_list_);
  }

  void setenv(struct env_base_t *const p_env) const {
    this->base_t::setenv(p_env);
    component_list_.setenv(p_env);
    port_list_.setenv(p_env);
  }

  const composition_list_static_t &compositions() const { return composition_list_; }
  const component_list_static_t &components() const { return component_list_; }
  const port_list_static_t &ports() const { return port_list_; }

private:
  const composition_list_static_t composition_list_;
  const component_list_static_t component_list_;
  const port_list_static_t port_list_;
};

template <typename NameType, typename... Components, typename... Ports>
explicit composition_static_t(const NameType &, const port_list_static_t<Ports...> &,
                              const component_list_static_t<Components...> &)
    ->composition_static_t<port_list_static_t<Ports...>, component_list_static_t<Components...>>;

template <typename NameType, typename... Compositions, typename... Components, typename... Ports>
explicit composition_static_t(const NameType &, const port_list_static_t<Ports...> &,
                              const component_list_static_t<Components...> &, const std::tuple<Compositions...> &)
    ->composition_static_t<port_list_static_t<Ports...>, component_list_static_t<Components...>,
                           std::tuple<Compositions...>>;

#endif /* COMPOSITION_HPP */
