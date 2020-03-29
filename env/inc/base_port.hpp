#ifndef BASE_PORT_HPP
#define BASE_PORT_HPP

#include "functor.hpp"
#include "port_node.hpp"
#include "port_type.hpp"
#include "tuple.hpp"
#include "type_traits_ex.hpp"
#include <queue>

template <typename EntityType> struct base_port_t<EntityType> : public port_base_s {
public:
  using base_s = port_base_s;
  using data_t = EntityType;
  using this_s = base_port_t<data_t>;
  using name_t = typename base_s::name_t;

  explicit base_port_t(const std::string &name) : base_s(name){};

  struct env_status_s<this_s> connect_port(const port_base_s &port) const { return connect_port_(port.info().id()); }
  struct env_status_s<this_s> connect_port(const sha256::sha256_hash_type &id) const { return connect_port_(id); }

  struct env_status_s<this_s> disconnect_port(const port_base_s &port) const { return disconnect_port_(port.info().id()); }
  struct env_status_s<this_s> disconnect_port(const sha256::sha256_hash_type &id) const { return disconnect_port_(id); }

  struct env_status_s<this_s> replace_port(const port_base_s &port_which, const port_base_s &port_by) const {
    return replace_port_(port_which.info().id(), port_by.info().id());
  }

  struct env_status_s<this_s> replace_port(const sha256::sha256_hash_type &id_which,
                                    const sha256::sha256_hash_type &id_by) const {
    return replace_port_(id_which, id_by);
  }

  struct env_status_s<this_s> connect_proxy(const port_base_s &port) const { return connect_proxy_(port.info().id()); }
  struct env_status_s<this_s> connect_proxy(const sha256::sha256_hash_type &id) const { return connect_proxy_(id); }

  struct env_status_s<this_s> disconnect_proxy(const port_base_s &port) const { return disconnect_proxy_(port.info().id()); }
  struct env_status_s<this_s> disconnect_proxy(const sha256::sha256_hash_type &id) const { return disconnect_proxy_(id); }

  struct env_status_s<this_s> replace_proxy(const port_base_s &port_which, const port_base_s &port_by) const {
    return replace_proxy_(port_which.info().id(), port_by.info().id());
  };

  struct env_status_s<this_s> replace_proxy(const sha256::sha256_hash_type &id_which,
                                     const sha256::sha256_hash_type &id_by) const {
    return replace_proxy_(id_which, id_by);
  };

private:
  friend struct env_base_s;

  struct env_status_s<this_s> connect_port_(const port_base_s &port) const {
    struct env_status_s<this_s> temp_status(this);

    if (this->connected().size() < this->MAX_CONNECTIONS) {

      if (!this->env()->port_manager().is_connected(this->info(), port.info())) {

        // this->connected().push_back();

        // 		if( !this->env()->port_manager.is_connected(
        // std::forward< struct port_info_s >( info ), std::forward< struct port_info_s >(
        // this->info() ))){

        // 		  auto connect_port_status =
        // this->env()->port_manager.connect_port( std::forward< struct port_info_s >(
        // info ), std::forward< struct port_info_s >( this->info() )); /* TODO: change
        // connection algorithm( use "port_manager" as part of environment ) */
        // 		  temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), env_errno_e::ENV_CLEAR )); 		  goto finish;

        // 		} else {

        // 		  temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), env_errno_e::ENV_CLEAR )); 		  goto finish;
        // 		}
      } else {

        temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_PORT_EXISTS));
        this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                              error_case_e::ERROR_CASE_RUNTIME);
      }
    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_MAX_PORTS_CONNECTED));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
    }

    return temp_status;
  }

  struct env_status_s<this_s> connect_port_(const sha256::sha256_hash_type &id) const {
    struct env_status_s<this_s> temp_status(this);

    if (this->connected().size() < this->MAX_CONNECTIONS) {

      if (!this->env()->port_manager().is_connected(this->get_id(), id)) {

        // this->connected().push_back();

        // 		if( !this->env()->port_manager.is_connected(
        // std::forward< struct port_info_s >( info ), std::forward< struct port_info_s >(
        // this->info() ))){

        // 		  auto connect_port_status =
        // this->env()->port_manager.connect_port( std::forward< struct port_info_s
        // >( info ), std::forward< struct port_info_s >( this->info() )); /* TODO:
        // change connection algorithm( use "port_manager" as part of
        // environment ) */ 		  temp_status.qualifiers.insert(
        // std::make_pair( this->get_id(), env_errno_e::ENV_CLEAR ));
        // goto finish;

        // 		} else {

        // 		  temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), env_errno_e::ENV_CLEAR )); 		  goto
        // finish;
        // 		}
      } else {

        temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_PORT_EXISTS));
        this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                              error_case_e::ERROR_CASE_RUNTIME);
      }
    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_MAX_PORTS_CONNECTED));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
    }

    return temp_status;
  }

  struct env_status_s<this_s> disconnect_port_(const port_base_s &port) const {
    struct env_status_s<this_s> temp_status(this);

    // if (this->env()->port_manager().is_connected(port.info(), this->info())) {

    //   this->env()->port_manager().disconnect_port(port.info(), this->info());
    // } else {

    //   temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_PORT_MISSING));
    //   this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
    //                         error_case_e::ERROR_CASE_RUNTIME);
    // }

    return temp_status;
  }

  struct env_status_s<this_s> disconnect_port_(const sha256::sha256_hash_type &id) const {
    struct env_status_s<this_s> temp_status(this);

    // if( this->env()->port_manager.is_connected( std::forward< struct port_info_s >(
    // info ), std::forward< struct port_info_s >( this->info() ))){

    //   this->env()->port_manager.disconnect_port( std::forward< struct port_info_s >(
    //   info ), std::forward< struct port_info_s >( this->info() ));

    // } else {

    //   temp_status.qualifiers.insert( std::make_pair( this->get_id(),
    //   env_errno_e::ENV_PORT_MISSING )); this->error_handler()(
    //   this->get_id(), temp_status.qualifiers.at( this->get_id() ),
    //   error_case_e::ERROR_CASE_RUNTIME );
    // }

    return temp_status;
  }

  struct env_status_s<this_s> replace_port_(const sha256::sha256_hash_type &id_which,
                                     const sha256::sha256_hash_type &id_by) const {
    struct env_status_s<this_s> temp_status(this);
    struct env_status_s<this_s> disconnect_port_status = disconnect_port_(id_which);

    if (disconnect_port_status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {

      temp_status.qualifiers.insert(
          std::make_pair(this->get_id(), disconnect_port_status.qualifiers.at(this->get_id())));
      return temp_status;

    } else {

      return connect_port_(id_by);
    }
  }

  struct env_status_s<this_s> connect_proxy_(const sha256::sha256_hash_type &id) const {
    struct env_status_s<this_s> temp_status(this);
    return temp_status;
  }

  struct env_status_s<this_s> disconnect_proxy_(const sha256::sha256_hash_type &id) const {
    struct env_status_s<this_s> temp_status(this);
    return temp_status;
  }

  struct env_status_s<this_s> replace_proxy_(const sha256::sha256_hash_type &id_which,
                                      const sha256::sha256_hash_type &id_by) const {
    struct env_status_s<this_s> temp_status(this);
    return temp_status;
  }
};

#endif /* BASE_SR_PORT_HPP */
