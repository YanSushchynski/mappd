#ifndef CLIENT_PORT_HPP
#define CLIENT_PORT_HPP

#include "base_client_port.hpp"

template <typename RetType, typename... Args>
struct port_t<RetType(Args...), port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC>
    : public base_client_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), RetType(Args...)> {
private:
  friend struct env_base_s;

public:
  using data_t = response<RetType>;
  using function_t = RetType(Args...);
  using base_s = base_client_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), function_t>;
  using this_t = port_t<function_t, port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC>;

  explicit port_t(const std::string &name) : base_s(name){};

protected:
  struct env_status_s<base_s> request__(const std::string &server_name, const std::string &call_name,
                                        const Args &... args) const {
    const sha256::sha256_hash_type server_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(server_name.data()), server_name.length());
    const sha256::sha256_hash_type call_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(call_name.data()), call_name.length());
    return this->create_request(server_id, call_id, args...);
  }

  struct env_data_s<data_t, base_s>
  response__() const {
    return this->get_response();
  }
};

template <typename RetType, typename... Args>
struct port_t<RetType(Args...), port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC>
    : public base_client_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), RetType(Args...)> {
private:
  friend struct env_base_s;

public:
  using data_t = response<RetType>;
  using function_t = RetType(Args...);
  using base_s = base_client_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), function_t>;
  using this_t = port_t<function_t, port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC>;

  explicit port_t(const std::string &name) : base_s(name){};

protected:
  struct env_data_s<data_t, base_s> call__(const std::string &server_name, const std::string &call_name,
                                           const Args &... args) const {
    struct env_data_s<data_t, base_s> temp_data(this);
    const sha256::sha256_hash_type server_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(server_name.data()), server_name.length());
    const sha256::sha256_hash_type call_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(call_name.data()), call_name.length());
    struct env_status_s<base_s> request_status = this->create_request(server_id, call_id, args...);

    if (request_status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {
      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), request_status.qualifiers.at(this->get_id())));
      return temp_data;
    } else {
      struct env_status_s<base_s> forwarding_status = this->forward();

      if (forwarding_status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {
        temp_data.status.qualifiers.insert(
            std::make_pair(this->get_id(), forwarding_status.qualifiers.at(this->get_id())));
        return temp_data;
      } else {
        struct env_data_s<data_t, base_s> response_data = this->get_response();

        if (response_data.status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {
          temp_data.status.qualifiers.insert(
              std::make_pair(this->get_id(), response_data.status.qualifiers.at(this->get_id())));
          return temp_data;
        } else {
          temp_data.status.qualifiers.insert(
              std::make_pair(this->get_id(), response_data.status.qualifiers.at(this->get_id())));
          temp_data.data = response_data.data;
          return temp_data;
        }
      }
    }
  }
};

template <typename Prototype>
class sync_client_port_t final : public port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                               port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC> {
private:
  friend struct env_base_s;

public:
  using this_t = sync_client_port_t<Prototype>;
  using base_s = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                        port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC>;
  using function_t = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                            port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC>;
  using data_t = typename port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                 port_types_t::client_port_types_t::PORT_TYPE_CLIENT_SYNC>::data_t;

  explicit sync_client_port_t(const std::string &name) : base_s(name) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());
    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  template <typename... Args>
  struct env_status_s<base_s> request(const std::string &server_name, const std::string &call_name,
                                      const Args &... args) const {
    return this->request__(server_name, call_name, args...);
  };

  struct env_data_s<data_t, base_s> response() const {
    return this->response__();
  }
};

template <typename Prototype>
class async_client_port_t final : public port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                                port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC> {
private:
  friend struct env_base_s;

public:
  using this_t = async_client_port_t<Prototype>;
  using base_s = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                        port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC>;
  using function_t = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                            port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC>;
  using data_t = typename port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                 port_types_t::client_port_types_t::PORT_TYPE_CLIENT_ASYNC>::data_t;

  explicit async_client_port_t(const std::string &name) : base_s(name) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  template <typename... Args>
  struct env_data_s<data_t, base_s> call(const std::string &server_name, const std::string &call_name,
                                         const Args &... args) const {
    return this->call__(server_name, call_name, args...);
  }
};

template <typename Prototype> using sync_client_port_unified_t = sync_client_port_t<Prototype>;
template <typename Prototype> using async_client_port_unified_t = async_client_port_t<Prototype>;

template <typename ProtoType> using sclip_t = sync_client_port_t<ProtoType>;
template <typename ProtoType> using asclip_t = async_client_port_t<ProtoType>;
template <typename Prototype> using usclip_t = sync_client_port_unified_t<Prototype>;
template <typename Prototype> using uasclip_t = async_client_port_unified_t<Prototype>;

#endif /* CLIENT_PORT_HPP */
