#ifndef SERVER_PORT_HPP
#define SERVER_PORT_HPP

#include "base_server_port.hpp"

template <typename RetType, typename... Args>
struct port_t<RetType(Args...), port_types_t::server_port_types_t::PORT_TYPE_SERVER_SYNC>
  : public base_server_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), RetType(Args...)> {
private:
  friend struct env_base_s;

public:
  using function_t = RetType(Args...);
  using base_s = base_server_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), function_t>;
  using this_t = port_t<function_t, port_types_t::server_port_types_t::PORT_TYPE_SERVER_SYNC>;

  explicit port_t(const std::string &name) : base_s(name){};
  explicit port_t(
      const std::string &name,
      const std::initializer_list<const std::pair<const std::string &, const std::function<function_t> &>> calls_list)
      : base_s(name, calls_list){};

  template <typename... Functions>
  explicit port_t(const std::string &name, const std::pair<const std::string &, const Functions &> &... functions)
      : base_s(name, functions...){};
};

template <typename RetType, typename... Args>
struct port_t<RetType(Args...), port_types_t::server_port_types_t::PORT_TYPE_SERVER_ASYNC>
  : public base_server_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), RetType(Args...)> {
private:
  friend struct env_base_s;

public:
  using function_t = RetType(Args...);
  using base_s = base_server_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), function_t>;
  using this_t = port_t<function_t, port_types_t::server_port_types_t::PORT_TYPE_SERVER_ASYNC>;

  explicit port_t(const std::string &name) : base_s(name){};
  explicit port_t(
      const std::string &name,
      const std::initializer_list<const std::pair<const std::string &, const std::function<function_t> &>> &calls_list)
      : base_s(name, calls_list){};

  template <typename... Functions>
  explicit port_t(const std::string &name, const std::pair<const std::string &, const Functions &> &... functions)
      : base_s(name, functions...){};
};

template <typename Prototype>
struct sync_server_port_t final : public port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                                port_types_t::server_port_types_t::PORT_TYPE_SERVER_SYNC> {
private:
  friend struct env_base_s;

public:
  using this_t = sync_server_port_t<Prototype>;
  using base_s = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                        port_types_t::server_port_types_t::PORT_TYPE_SERVER_SYNC>;
  using function_t = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                            port_types_t::server_port_types_t::PORT_TYPE_SERVER_SYNC>;

  explicit sync_server_port_t(const std::string &name) : base_s(name) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  explicit sync_server_port_t(
      const std::string &name,
      const std::initializer_list<
          const std::pair<const std::string &,
                          const std::function<typename std::function_traits::function_traits<Prototype>::function_t> &>>
          &calls_list)
      : base_s(name, calls_list) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  template <typename... Functions>
  explicit sync_server_port_t(const std::string &name,
                              const std::pair<const std::string &, const Functions &> &... functions)
      : base_s(name, functions...) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };
};

template <typename Prototype>
struct async_server_port_t final : public port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                                                 port_types_t::server_port_types_t::PORT_TYPE_SERVER_ASYNC> {
private:
  friend struct env_base_s;

public:
  using this_t = async_server_port_t<Prototype>;
  using base_s = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                        port_types_t::server_port_types_t::PORT_TYPE_SERVER_ASYNC>;
  using function_t = port_t<typename std::function_traits::function_traits<Prototype>::function_t,
                            port_types_t::server_port_types_t::PORT_TYPE_SERVER_ASYNC>;

  explicit async_server_port_t(const std::string &name) : base_s(name) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  explicit async_server_port_t(
      const std::string &name,
      const std::initializer_list<
          const std::pair<const std::string &,
                          const std::function<typename std::function_traits::function_traits<Prototype>::function_t> &>>
          &calls_list)
      : base_s(name, calls_list) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };

  template <typename... Functions>
  explicit async_server_port_t(const std::string &name,
                               const std::pair<const std::string &, const Functions &> &... functions)
      : base_s(name, functions...) {
    std::string this_type = typestr<this_t>;
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(this_type.data()), this_type.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };
};

template <typename Prototype> using sync_server_port_unified_t = sync_server_port_t<Prototype>;
template <typename Prototype> using async_server_port_unified_t = async_server_port_t<Prototype>;

template <typename ProtoType> using sserp_t = sync_server_port_t<ProtoType>;
template <typename ProtoType> using asserp_t = async_server_port_t<ProtoType>;
template <typename Prototype> using usserp_t = sync_server_port_unified_t<Prototype>;
template <typename Prototype> using uasserp_t = async_server_port_unified_t<Prototype>;

#endif /* SERVER_PORT_HPP */
