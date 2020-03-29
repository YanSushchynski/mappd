#ifndef COMPONENT_BASE_HPP
#define COMPONENT_BASE_HPP

#include "env_utils.hpp"
#include "port_node.hpp"
#include "tuple.hpp"

struct cmp_base_s {
public:
  using this_t = cmp_base_s;
  explicit cmp_base_s(const std::string &name) : name_(name), env_(nullptr){};

  explicit cmp_base_s(const this_t &) = default;
  explicit cmp_base_s(this_t &&) = default;

  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~cmp_base_s() = default;

  struct cmp_info_s &info() const {
    return info_;
  }
  const struct env_base_s *env() const { return env_; }
  void print_info() const { print_info_(); }
  void setenv(struct env_base_s *p_env) const { env_ = p_env; }
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
                info_.id().c_str(), info_.name_hash().c_str(), info_.component_type_hash().c_str(),
                info_.composition_id().c_str());
  }

  const std::string name_;
  mutable struct env_base_s *env_;
  mutable struct cmp_info_s info_;
};

#endif /* COMPONENT_BASE_HPP */
