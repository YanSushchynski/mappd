#ifndef PROPERTY_HPP
#define PROPERTY_HPP

#include "hook.hpp"

template <typename PropertyType> struct property_t {
public:
  explicit property_t() : value_(PropertyType()){};
  explicit property_t(const PropertyType &val) : value_(val){};

  explicit property_t(const property_t<PropertyType> &other)
      : value_(other.get_value()), on_read_hook_(other.on_read_hook()), on_write_hook_(other.on_write_hook()){};

  explicit property_t(property_t<PropertyType> &&other)
      : value_(other.get_value()), on_read_hook_(other.on_read_hook()), on_write_hook_(other.on_write_hook()){};

  property_t<PropertyType> &operator=(const property_t<PropertyType> &rhs) {
    set_value_(rhs.value);
    return *this;
  }

  property_t<PropertyType> &operator=(property_t<PropertyType> &&rhs) {
    set_value_(rhs.value);
    return *this;
  }

  property_t<PropertyType> &operator=(const PropertyType &rhs) {
    set_value_(rhs);
    return *this;
  }

  property_t<PropertyType> &operator=(PropertyType &&rhs) {
    set_value_(rhs);
    return *this;
  }

  virtual ~property_t() = default;

  hook_t<void(const PropertyType &)> &on_write_hook() const { return on_write_hook_; }
  hook_t<void(const PropertyType &)> &on_read_hook() const { return on_read_hook_; }

  void set_value_(const PropertyType &value) const {
    value_ = value;
    on_write_hook_(value_);
  }

  PropertyType &get_value_() const {
    on_read_hook_(value_);
    return value_;
  }

  void clear() {
    on_write_hook_.clear();
    on_read_hook_.clear();
  }

  operator PropertyType() const { return get_value_(); }
  PropertyType const &operator()() const { return get_value_(); }

  bool operator==(const PropertyType &rhs) const {
	return value_ == rhs;	
  }
  
private:
  mutable hook_t<void(const PropertyType &)> on_read_hook_;
  mutable hook_t<void(const PropertyType &)> on_write_hook_;
  mutable PropertyType value_;
};

#endif /* PROPERTY_HPP */
