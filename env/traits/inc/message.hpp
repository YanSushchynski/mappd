#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <cstdint>
#include <string>

struct message_t {
  friend struct boost::serialization::access;
  virtual std::string description() const { return ""; };
};
