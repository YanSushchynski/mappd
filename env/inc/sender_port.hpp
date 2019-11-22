#ifndef SENDER_PORT_HPP
#define SENDER_PORT_HPP

#include "base_sender_port.hpp"

template <typename DataType>
struct port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>
    : public base_sender_port_t<runtime_t::RUNTIME_SYNC, DataType> {
private:
  friend struct env_base_t;

public:
  using data_t = DataType;
  using this_t = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>;
  using base_t = base_sender_port_t<runtime_t::RUNTIME_SYNC, DataType>;

  template <typename NameType> explicit port_t(const NameType &name) : base_t(name){};
};

template <typename DataType>
struct port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>
    : public base_sender_port_t<runtime_t::RUNTIME_ASYNC, DataType> {
private:
  friend struct env_base_t;

public:
  using data_t = DataType;
  using this_t = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>;
  using base_t = base_sender_port_t<runtime_t::RUNTIME_ASYNC, DataType>;

  template <typename NameType> explicit port_t(const NameType &name) : base_t(name){};
};

template <typename DataType>
struct sync_sender_port_t final : public port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC> {
private:
  friend struct env_base_t;

public:
  using data_t = DataType;
  using this_t = sync_sender_port_t<data_t>;
  using base_t = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC>;

  env_status_t<this_t> write(const data_t &data) {
    env_status_t<this_t> temp_status(this);
    return this->enqueue_data(data);
  }

  template <typename NameType> explicit sync_sender_port_t(const NameType &name) : base_t(name) {
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_t>.data()), typestr<this_t>.length());
    this->info().set_port_type_hash(hash.data(), hash.size());
    this->update_info();
  };
};

template <typename DataType>
struct async_sender_port_t final : public port_t<DataType, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC> {
private:
  friend struct env_base_t;

public:
  using data_t = DataType;
  using this_t = async_sender_port_t<data_t>;
  using base_t = port_t<data_t, port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC>;

  env_status_t<this_t> write(const data_t &data) {
    env_status_t<this_t> temp_status(this);
    auto enqueue_data_status = this->enqueue_data(data);

    if (temp_status.qualifiers.at(this->get_id()) != env_errno_t::ENV_CLEAR) {
      return temp_status;
    } else {
      return this->forward();
    }
  }

  template <typename NameType> explicit async_sender_port_t(const NameType &name) : base_t(name) {
    sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(typestr<this_t>.data()), typestr<this_t>.length());

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
