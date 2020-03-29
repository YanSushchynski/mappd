#ifndef RECEIVER_PORT_HPP
#define RECEIVER_PORT_HPP

#include "base_receiver_port.hpp"

template <typename DataType>
struct port_t<DataType, port_types_t::receiver_port_types_t::PORT_TYPE_RECEIVER>
    : public base_receiver_port_t<DataType> {
private:
  friend struct env_base_s;
  static constexpr uint32_t friend_ports[] = {port_types_t::sender_port_types_t::PORT_TYPE_SENDER_SYNC,
                                              port_types_t::sender_port_types_t::PORT_TYPE_SENDER_ASYNC};

public:
  using data_t = DataType;
  using base_s = base_receiver_port_t<data_t>;
  using this_t = port_t<data_t, port_types_t::receiver_port_types_t::PORT_TYPE_RECEIVER>;
  using name_t = typename base_s::name_t;
  static constexpr uint32_t port_type = port_types_t::receiver_port_types_t::PORT_TYPE_RECEIVER;
  explicit port_t(const std::string &name) : base_s(name){};

  template <typename PortType> constexpr bool is_friend(uint32_t type) const {
    bool is_friend = false;
    for (uint32_t i = 0; i < sizeof(friend_ports) / sizeof(friend_ports[0]); i++) {
      if (type != friend_ports[i])
        return false;
    }

    return true;
  }
};

template <typename DataType>
struct receiver_port_t : public port_t<DataType, port_types_t::receiver_port_types_t::PORT_TYPE_RECEIVER> {
private:
  friend struct env_base_s;

public:
  using data_t = DataType;
  using this_t = receiver_port_t<data_t>;
  using base_s = port_t<data_t, port_types_t::receiver_port_types_t::PORT_TYPE_RECEIVER>;
  using name_t = typename base_s::name_t;

  struct env_status_s<this_t> read(data_t &data) {
    struct env_status_s<this_t> temp_status(this);
    auto sender_response = this -> dequeue_data();

    if (sender_response.status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {

      return sender_response.status;
    } else {

      data = sender_response.data;
      return sender_response.status;
    }
  }

  explicit receiver_port_t(const std::string &name)
      : base_s(name) {
    std::string type = typestr<this_t>;
    sha256::sha256_hash_type type_hash = sha256::compute(reinterpret_cast<const uint8_t *>(type.data()), type.length());
    this->info().set_port_type_hash(type_hash.data(), type_hash.size());
    this->update_info();
  };

  virtual ~receiver_port_t() override = default;
};

using receiver_port_unified_t = receiver_port_t<unified_t>;
template <typename DataType> using recp_t = receiver_port_t<DataType>;
using urecp_t = receiver_port_unified_t;

#endif /* RECEIVER_PORT_HPP */
