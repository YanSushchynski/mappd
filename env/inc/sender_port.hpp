#ifndef SENDER_PORT_HPP
#define SENDER_PORT_HPP

#include "base_sender_port.hpp"

template <typename DataType>
struct port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>
    : public base_sender_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), DataType> {
private:
  friend struct env_base_s;

public:
  using data_t = DataType;
  using this_s = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>;
  using base_s = base_sender_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_SYNC), DataType>;

  explicit port_t(const std::string &name) : base_s(name){};
};

template <typename DataType>
struct port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>
    : public base_sender_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), DataType> {
private:
  friend struct env_base_s;

public:
  using data_t = DataType;
  using this_s = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>;
  using base_s = base_sender_port_t<static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC), DataType>;

  explicit port_t(const std::string &name) : base_s(name){};
};

template <typename DataType>
struct sync_sender_port_t final : public port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC> {
private:
  friend struct env_base_s;

public:
  using data_t = DataType;
  using this_s = sync_sender_port_t<data_t>;
  using base_s = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>;

  struct env_status_s<this_s> write(const data_t &data) {
    struct env_status_s<this_s> temp_status(this);
    return this->enqueue_data(data);
  }

  explicit sync_sender_port_t(const std::string &name)
      : base_s(name) {
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_s>.data()), typestr<this_s>.length());
    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };
};

template <typename DataType>
struct async_sender_port_t final : public port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC> {
private:
  friend struct env_base_s;

public:
  using data_t = DataType;
  using this_s = async_sender_port_t<data_t>;
  using base_s = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>;

  struct env_status_s<this_s> write(const data_t &data) {
    struct env_status_s<this_s> temp_status(this);
    auto enqueue_data_status = this -> enqueue_data(data);

    if (temp_status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {
      return temp_status;
    } else {
      return this->forward();
    }
  }

  explicit async_sender_port_t(const std::string &name)
      : base_s(name) {
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_s>.data()), typestr<this_s>.length());

    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };
};

using sync_sender_port_unified_t = sync_sender_port_t<unified_t>;
using async_sender_port_unified_t = async_sender_port_t<unified_t>;

template <typename DataType> using ssenp_t = sync_sender_port_t<DataType>;
template <typename DataType> using assenp_t = async_sender_port_t<DataType>;

using ussenp_t = sync_sender_port_unified_t;
using uassenp_t = async_sender_port_unified_t;

#endif /* SENDER_PORT_HPP */
