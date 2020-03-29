#ifndef ENV_STATIC_HPP
#define ENV_STATIC_HPP

#include "domain_network_manager.hpp"
#include "env_base.hpp"
#include "env_utils.hpp"
#include "network_manager.hpp"
#include "port_manager.hpp"

template <typename... Compositions> struct env_static_t : public env_base_t {
public:
  using base_t = env_base_t;
  using this_t = env_static_t<Compositions...>;

  template <typename NameType>
  explicit env_static_t(const NameType &name, const composition_list_static_t<Compositions...> &compositions)
      : base_t(name), compositions_(compositions), port_manager_(this, compositions_),
        network_manager_("12345678901234567890123456789000", this),
        domain_network_manager_("12345678901234567890123456789000", this) {
    compositions_.setenv(this);
  }

  explicit env_static_t(const this_t &) = default;
  explicit env_static_t(this_t &&) = default;
  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~env_static_t() = default;

  struct port_manager_base_t &port_manager() { return port_manager_; }
  struct nm_t<> &network_manager() { return network_manager_; }
  struct domain_nm_t<> &domain_network_manager() { return domain_network_manager_; }
  struct composition_list_static_t<Compositions...> &compositions() { return compositions_; }

  virtual void configure(const libconfig::Setting &env_config) override{};
  virtual int32_t run(int32_t argc, char *argv[]) override { return 0; };

private:
  mutable nm_t<> network_manager_;
  mutable domain_nm_t<> domain_network_manager_;
  mutable port_manager_t<Compositions...> port_manager_;
  mutable composition_list_static_t<Compositions...> compositions_;
};

#endif /* ENV_STATIC_HPP */
