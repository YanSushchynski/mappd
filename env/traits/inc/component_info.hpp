#include "message.hpp"
#include "sha256.hpp"

#define DEFINE_FIELD(Type, Name)

struct component_info_t : public message_t {
  using this_t = component_info_t;
  using base_T = message_t;

  template <typename Archive> void serialize(Archive &ar, const uint32_t version) {

    ar &boost::serialization::base_object<struct message_t>(*this);
  }

  sha256::sha256_hash_type id;
  sha256::sha256_hash_type name_hash;
  sha256::sha256_hash_type component_type_hash;
  sha256::sha256_hash_type composition_id;
};
