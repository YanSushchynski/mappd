#ifndef QUALIFIED_DATA_HPP
#define QUALIFIED_DATA_HPP

#include "function_traits.hpp"
#include "sha256.hpp"
#include "type_traits_ex.hpp"

#include <type_traits>

template <typename EntityType, typename Dummy = void> struct qualified_data_t {};
template <typename EntityType>
struct qualified_data_t<EntityType, typename std::enable_if<!std::is_entity_is_function_v<EntityType>>::type> {
  using this_t = qualified_data_t<EntityType>;

  qualified_data_t()
      : qualifier(0u), dt_hash(sha256::compute(reinterpret_cast<const uint8_t *>(typeid(EntityType).name()),
                                               std::strlen(typeid(EntityType).name()))){};

  qualified_data_t(const EntityType &data)
      : qualifier(0u), dt_hash(sha256::compute(reinterpret_cast<const uint8_t *>(typeid(EntityType).name()),
                                               std::strlen(typeid(EntityType).name()))),
        data(std::forward<EntityType>(data)){};

  virtual ~qualified_data_t() = default;

  EntityType data;
  sha256::sha256_hash_type dt_hash;
  uint32_t qualifier;
};

template <typename EntityType>
struct qualified_data_t<EntityType, typename std::enable_if<std::is_entity_is_function_v<EntityType>>::type> {
  using this_t = qualified_data_t<EntityType>;

  qualified_data_t()
      : qualifier(0u), dt_hash(sha256::compute(reinterpret_cast<const uint8_t *>(typeid(EntityType).name()),
                                               std::strlen(typeid(EntityType).name()))){};

  virtual ~qualified_data_t() = default;

  std::function<EntityType> function;
  sha256::sha256_hash_type dt_hash;
  uint32_t qualifier;
};

#endif /* QUALIFIED_DATA_HPP */
