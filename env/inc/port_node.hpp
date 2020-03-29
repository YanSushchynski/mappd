#ifndef PORT_NODE_HPP
#define PORT_NODE_HPP

#include "env_utils.hpp"

struct env_base_s;

struct port_base_s {
public:
  static constexpr uint32_t BUFFER_SIZE = 8u;
  static constexpr uint32_t MAX_CONNECTIONS = 8u;

  using this_s = port_base_s;
  using name_t = std::string;

  explicit port_base_s(const std::string &name)
      : error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}),
        name_(name), env_(nullptr){};

  explicit port_base_s(const this_s &) = default;
  explicit port_base_s(this_s &&) = default;

  this_s &operator=(const this_s &) = delete;
  this_s &operator=(this_s &&) = delete;

  virtual ~port_base_s() = default;

  template <typename Function> struct env_status_s<this_s> set_error_handler(const Function &function) {
    return set_error_handler_<Function>(function);
  }

  template <typename OwnerClass, typename Function>
  struct env_status_s<this_s> set_error_handler(const OwnerClass *const p_class, const Function &function) {
    return set_error_handler_<OwnerClass, Function>(p_class, function);
  }

  struct port_info_s &
  info() const {
    return info_;
  }
  const std::vector<struct port_info_s> &connected() const { return connected_; }
  const std::vector<struct port_info_s>::iterator find_port(const sha256::sha256_hash_type &id,
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
  const struct env_base_s *env() const { return env_; }
  void update_info() const { update_info_(); }
  void print_info() const { print_info_(); }
  void setenv(const struct env_base_s *const p_env) const { env_ = const_cast<struct env_base_s *>(p_env); }

private:
  friend struct env_base_s;

  const std::vector<struct port_info_s>::iterator find_port_(const sha256::sha256_hash_type &id) const {
    return std::find_if(connected_.begin(), connected_.end(), [&id](const struct port_info_s &part) -> bool {
      return sha256::sha256_from_string(part.id()) == id;
    });
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

  template <typename Function> struct env_status_s<this_s> set_error_handler_(const Function &function) {
    struct env_status_s<this_s> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    update_info_();
    temp_status.qualifiers.insert(std::make_pair(sha256::sha256_from_string(info_.id()),
                                                 static_cast<uint32_t>(env_errno_e::ENV_CLEAR)));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  struct env_status_s<this_s> set_error_handler_(const OwnerClass *const p_class, const Function &function) {
    struct env_status_s<this_s> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    update_info_();
    temp_status.qualifiers.insert(std::make_pair(sha256::sha256_from_string(info_.id()),
                                                 static_cast<uint32_t>(env_errno_e::ENV_CLEAR)));
    return temp_status;
  }

  mutable struct env_base_s *env_;
  const std::string name_;
  mutable struct port_info_s info_;
  mutable std::vector<struct port_info_s> connected_;
  mutable std::vector<struct port_info_s> proxies_;
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
};

#endif /* PORT_NODE_HPP */
