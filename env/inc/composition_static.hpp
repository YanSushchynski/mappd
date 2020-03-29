#ifndef COMPOSITION_HPP
#define COMPOSITION_HPP

#include "component_list_static.hpp"
#include "composition_node.hpp"
#include "port_list_static.hpp"

template <typename...> struct cmps_static_s {};

template <template <typename...> class ComponentList, typename... Components, template <typename...> class PortList,
          typename... Ports>
struct cmps_static_s<PortList<Ports...>, ComponentList<Components...>> : public cmps_base_s {
  using this_t = cmps_static_s<PortList<Ports...>, ComponentList<Components...>>;
  using component_list_static_t = ComponentList<Components...>;
  using port_list_static_t = PortList<Ports...>;
  using base_s = cmps_base_s;
  
  static constexpr uint32_t port_list_size = sizeof...(Ports);
  static constexpr uint32_t component_list_size = sizeof...(Components);

  explicit cmps_static_s(const std::string &name, const port_list_static_t &ports,
                         const component_list_static_t &components)
      : base_s(name), component_list_(components), port_list_(ports) {
    update_info();
  }

  virtual ~cmps_static_s() = default;

  void setenv(struct env_base_s *const p_env) const {
    this->base_s::setenv(p_env);
    component_list_.setenv(p_env);
    port_list_.setenv(p_env);
  }

  void update_info() const {

    sha256::sha256_hash_type type_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_t>.data()), typestr<this_t>.length());

    this->info().set_composition_type_hash(type_hash.data(), type_hash.size());
    this->base_s::update_info_base();
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
struct cmps_static_s<PortList<Ports...>, ComponentList<Components...>, CompositionList<Compositions...>>
    : public cmps_base_s {
  using this_t = cmps_static_s<PortList<Ports...>, ComponentList<Components...>>;
  using composition_list_static_t = CompositionList<Compositions...>;
  using component_list_static_t = ComponentList<Components...>;
  using port_list_static_t = PortList<Ports...>;
  using base_s = cmps_base_s;

  static constexpr uint32_t port_list_size = sizeof...(Ports);
  static constexpr uint32_t component_list_size = sizeof...(Components);
  static constexpr uint32_t composition_list_size = sizeof...(Compositions);

  explicit cmps_static_s(const std::string &name, const port_list_static_t &ports,
                         const component_list_static_t &components, const composition_list_static_t &compositions)
      : base_s(name), composition_list_(compositions), component_list_(components), port_list_(ports) {
    update_info();
  }

  virtual ~cmps_static_s() = default;

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

  void setenv(struct env_base_s *const p_env) const {
    this->base_s::setenv(p_env);
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

template <typename... Components, typename... Ports>
explicit cmps_static_s(const std::string &, const port_list_static_s<Ports...> &,
                       const cmp_list_static_s<Components...> &)
    ->cmps_static_s<port_list_static_s<Ports...>, cmp_list_static_s<Components...>>;

template <typename... Compositions, typename... Components, typename... Ports>
explicit cmps_static_s(const std::string &, const port_list_static_s<Ports...> &,
                       const cmp_list_static_s<Components...> &, const std::tuple<Compositions...> &)
    ->cmps_static_s<port_list_static_s<Ports...>, cmp_list_static_s<Components...>, std::tuple<Compositions...>>;

#endif /* COMPOSITION_HPP */
