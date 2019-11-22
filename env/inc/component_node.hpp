#ifndef COMPONENT_NODE_HPP
#define COMPONENT_NODE_HPP

#include "env_utils.hpp"
#include "port_node.hpp"
#include "tuple.hpp"

struct component_node_t {
public:
  using this_t = component_node_t;
  template <typename NameType> explicit component_node_t(const NameType &name) : name_(name), env_(nullptr){};

  explicit component_node_t(const this_t &) = default;
  explicit component_node_t(this_t &&) = default;

  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~component_node_t() = default;

  component_info_t &info() const { return info_; }
  const struct env_base_t *env() const { return env_; }
  void print_info() const { print_info_(); }
  void setenv(struct env_base_t *p_env) const { env_ = p_env; }
  const std::string &name() const { return name_; }

  void update_info_base() const {
    sha256::sha256_hash_type name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name_.data()), name_.length());
    info_.set_name_hash(name_hash.data(), name_hash.size());

    sha256::sha256_hash_type id =
        sha256::sha256_from_string(info_.name_hash()) ^ sha256::sha256_from_string(info_.component_type_hash());

    info_.set_id(id.data(), id.size());
  }

private:
  void print_info_() const {
    std::printf("Component \"%s\" info :\r\n", name_.c_str());
    std::printf("{\r\n\tId : %s,\r\n\tNameHash : %s,\r\n\tTypeHash : "
                "%s,\r\n\tCompositionID : %s\r\n};\r\n",
                info_.id().c_str(), info_.name_hash().c_str(), info_.component_type_hash().c_str(), info_.composition_id().c_str());
  }

  const std::string name_;
  mutable struct env_base_t *env_;
  mutable component_info_t info_;
};

#endif /* COMPONENT_NODE_HPP */
