#ifndef PORT_NODE_HPP
#define PORT_NODE_HPP

#include "env_utils.hpp"

struct env_base_t;

struct port_node_t {
public:
  static constexpr uint32_t BUFFER_SIZE = 8u;
  static constexpr uint32_t MAX_CONNECTIONS = 8u;

  using this_t = port_node_t;
  using name_t = std::string;

  template <typename NameType>
  explicit port_node_t(const NameType &name)
      : error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}),
        name_(name), env_(nullptr){};

  explicit port_node_t(const this_t &) = default;
  explicit port_node_t(this_t &&) = default;

  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~port_node_t() = default;

  template <typename Function> env_status_t<this_t> set_error_handler(const Function &function) {
    return set_error_handler_<Function>(function);
  }

  template <typename OwnerClass, typename Function>
  env_status_t<this_t> set_error_handler(const OwnerClass *const p_class, const Function &function) {
    return set_error_handler_<OwnerClass, Function>(p_class, function);
  }

  port_info_t &info() const { return info_; }
  const std::vector<port_info_t> &connected() const { return connected_; }
  const std::vector<port_info_t>::iterator find_port(const sha256::sha256_hash_type &id,
                                                     const sha256::sha256_hash_type &typehash) const {
    return find_port_(id);
  }

  bool is_connected(const sha256::sha256_hash_type &id) const { return is_connected_(id); }

  const std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> &
  error_handler() const {
    return error_handler_;
  }

  const std::string &get_name() const { return name_; }
  uint32_t max_buffer_size() const { return BUFFER_SIZE; }
  const struct env_base_t *env() const { return env_; }
  void update_info() const { update_info_(); }
  void print_info() const { print_info_(); }
  void setenv(const struct env_base_t *const p_env) const { env_ = const_cast<struct env_base_t *>(p_env); }

private:
  friend struct env_base_t;

  const std::vector<port_info_t>::iterator find_port_(const sha256::sha256_hash_type &id) const {
    return std::find_if(connected_.begin(), connected_.end(),
                        [&id](const port_info_t &part) -> bool { return sha256::sha256_from_string(part.id()) == id; });
  }

  bool is_connected_(const sha256::sha256_hash_type &id) const {
    return (find_port_(id) == connected_.end()) ? false : true;
  }

  void update_info_() const {
    sha256::sha256_hash_type name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name_.data()), name_.length());
    info_.set_name_hash(name_hash.data(), name_hash.size());

    sha256::sha256_hash_type id =
        sha256::sha256_from_string(info_.name_hash()) ^ sha256::sha256_from_string(info_.port_type_hash());

    info_.set_id(id.data(), id.size());
  }

  void print_info_() const {
    std::printf("Port \"%s\" info :\r\n", name_.c_str());
    std::printf("{\r\n\tId : %s,\r\n\tNameHash : %s,\r\n\tTypeHash : "
                "%s,\r\n\tComponentID : "
                "%s,\r\n\tCompositionID : %s\r\n};\r\n",
                info_.id().c_str(), info_.name_hash().c_str(), info_.port_type_hash().c_str(),
                info_.component_id().c_str(), info_.composition_id().c_str());
  }

  template <typename Function> env_status_t<this_t> set_error_handler_(const Function &function) {
    env_status_t temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    update_info_();
    temp_status.qualifiers.insert(std::make_pair(sha256::sha256_from_string(info_.id()), env_errno_t::ENV_CLEAR));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  env_status_t<this_t> set_error_handler_(const OwnerClass *const p_class, const Function &function) {
    env_status_t temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    update_info_();
    temp_status.qualifiers.insert(std::make_pair(sha256::sha256_from_string(info_.id()), env_errno_t::ENV_CLEAR));
    return temp_status;
  }

  mutable struct env_base_t *env_;
  const std::string name_;
  mutable port_info_t info_;
  mutable std::vector<port_info_t> connected_;
  mutable std::vector<port_info_t> proxies_;
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
};

#endif /* PORT_NODE_HPP */
