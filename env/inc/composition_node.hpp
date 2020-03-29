#ifndef COMPOSITION_BASE_HPP
#define COMPOSITION_BASE_HPP

#include "env_utils.hpp"
#include "sha256.hpp"

struct cmps_base_s {
public:
  using this_s = cmps_base_s;
  explicit cmps_base_s(const std::string &name) : name_(name), env_(nullptr){};

  explicit cmps_base_s(const this_s &) = default;
  explicit cmps_base_s(this_s &&) = default;
  this_s &operator=(const this_s &) = delete;
  this_s &operator=(this_s &&) = delete;

  virtual ~cmps_base_s() = default;

  struct cmps_info_s &info() const { return info_; }

  const struct env_base_s *env() const { return env_; }

  void print_info() const { print_info_(); }
  void setenv(struct env_base_s *p_env) const { env_ = p_env; }
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
  mutable struct env_base_s *env_;
  mutable struct cmps_info_s info_;
};

#endif /* COMPOSITION_BASE_HPP */
