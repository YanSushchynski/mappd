#ifndef COMPOSITION_NODE_HPP
#define COMPOSITION_NODE_HPP

#include "env_utils.hpp"
#include "sha256.hpp"

struct composition_node_t {
public:
  using this_t = composition_node_t;
  template <typename NameType> explicit composition_node_t(const NameType &name) : name_(name), env_(nullptr){};

  explicit composition_node_t(const this_t &) = default;
  explicit composition_node_t(this_t &&) = default;
  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~composition_node_t() = default;

  composition_info_t &info() const { return info_; }

  const struct env_base_t *env() const { return env_; }

  void print_info() const { print_info_(); }
  void setenv(struct env_base_t *p_env) const { env_ = p_env; }
  const std::string &name() const { return name_; }

  void update_info_base() const {
    sha256::sha256_hash_type name_hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name_.data()), name_.length());

    info_.set_name_hash(name_hash.data(), name_hash.size());

    sha256::sha256_hash_type id =
        sha256::sha256_from_string(info_.name_hash()) ^ sha256::sha256_from_string(info_.composition_type_hash());

    info_.set_id(id.data(), id.size());
  }

private:
  void print_info_() const {
    std::printf("Composition \"%s\" info :\r\n", name_.c_str());
    std::printf("{\r\n\tId : %s,\r\n\tNameHash : %s,\r\n\tTypeHash : "
                "%s,\r\n",
                info_.id().c_str(), info_.name_hash().c_str(), info_.composition_type_hash().c_str());
  }
  
  const std::string name_;
  mutable struct env_base_t *env_;
  mutable composition_info_t info_;
};

#endif /* COMPOSITION_NODE_HPP */
