#ifndef ENV_UTILS_HPP
#define ENV_UTILS_HPP

#include <any>
#include <cstdint>
#include <cxxabi.h>
#include <map>
#include <memory>
#include <tuple>
#include <unistd.h>

#include "function_traits.hpp"
#include "type_traits_ex.hpp"

#include "component_info.pb.h"
#include "composition_info.pb.h"
#include "env_info.pb.h"
#include "port_info.pb.h"
#include "runtime_info.pb.h"

#include "sha256.hpp"

enum error_case_t : uint32_t { ERROR_CASE_CONSTRUCT, ERROR_CASE_DESTRUCT, ERROR_CASE_RUNTIME };
enum runtime_t : uint32_t { RUNTIME_SYNC, RUNTIME_ASYNC, RUNTIME_COMMON };
enum route_t : uint32_t { ROUTE_CONNECT, ROUTE_PROXY, ROUTE_OUT };

enum env_errno_t : uint32_t {
  ENV_NOT_INITED,
  ENV_CLEAR,
  ENV_BUFFER_EMPTY,
  ENV_BUFFER_OVERFLOW,
  ENV_PORT_EXISTS,
  ENV_PORT_MISSING,
  ENV_MAX_PORTS_CONNECTED,
  ENV_NOT_ACCESSIBLE,
  ENV_NOT_AVAILABLE,
  ENV_NO_TRANSACTIONS,
  ENV_CALL_EXISTS,
  ENV_CALL_MISSING
};

enum hook_errno_t : uint32_t { HOOK_CLEAR, HOOK_MISSING, HOOK_EXISTS, HOOK_UNKNOWN };
enum seq_errno_t : uint32_t { SEQ_CLEAR, SEQ_MISSING, SEQ_EXISTS, SEQ_UNKNOWN };
enum funct_errno_t : uint32_t { FUNCT_CLEAR, FUNCT_MISSING, FUNCT_EXISTS, FUNCT_UNKNOWN };

template <typename... Args> struct request {
  request() : server_id(0u), request_id(0u), payload(std::forward_as_tuple(Args()...)){};

  request(const uint64_t &s_id, const uint64_t &r_id, const Args &... args)
      : server_id(s_id), request_id(r_id), payload(std::forward_as_tuple(args...)){};

  ~request() = default;

  uint64_t server_id;
  uint64_t request_id;
  std::tuple<Args...> payload;
};

template <typename RetType> struct response {
  response() : client_id(0u), payload(RetType()){};

  response(uint64_t c_id, const RetType &r_t) : client_id(c_id), payload(r_t){};

  uint64_t client_id;
  RetType payload;

  ~response() = default;
};

template <typename EntityType, typename Dummy = void> struct type_select {};
template <typename EntityType> using type_select_t = typename type_select<EntityType>::type;
template <typename EntityType>
struct type_select<EntityType, typename std::enable_if<std::is_entity_is_function_v<EntityType>>::type> {
  using type = typename std::function_traits::function_traits<EntityType>::return_t;
};

template <typename EntityType>
struct type_select<EntityType, typename std::enable_if<!std::is_entity_is_function_v<EntityType>>::type> {
  using type = EntityType;
};

template <typename RelatedClass> struct env_status_t {
  using this_t = env_status_t;
  using related_class_t = RelatedClass;

  env_status_t() : ptr_related_class(nullptr), id_{0u} {};
  env_status_t(const related_class_t *const p_class) : ptr_related_class(p_class), id_(p_class->get_id()){};

  virtual ~env_status_t() = default;

  const sha256::sha256_hash_type id_;
  const related_class_t *const ptr_related_class;
  std::map<sha256::sha256_hash_type, uint32_t> qualifiers;
};

template <typename EntityType, typename RelatedClass> struct env_data_t {
  using data_t = typename type_select<EntityType>::type;
  using related_class_t = RelatedClass;
  using this_t = env_data_t<EntityType, related_class_t>;

  env_data_t(const related_class_t *const p_class) : status(env_status_t(p_class)){};
  env_data_t(const related_class_t *const p_class, const data_t &init) : status(env_status_t(p_class)), data(init){};

  virtual ~env_data_t() = default;

  data_t data;
  env_status_t<related_class_t> status;
};

extern std::string demangle(const char *name);
template <typename T> std::string typestr = demangle(typeid(T).name());
template <typename... Args>
auto types_hash = []() -> sha256::sha256_hash_type {
  sha256::sha256_hash_type result{0u};
  std::array<sha256::sha256_hash_type, sizeof...(Args)> hashes = {
      sha256::compute(reinterpret_cast<const uint8_t *>(typestr<Args>.data()), typestr<Args>.length())...};
  for (std::size_t i = 0; i < sizeof...(Args); i++)
    result ^= hashes[i];
  return result;
};

struct unified_t {
public:
  unified_t() = default;
  virtual ~unified_t() = default;

  template <typename ValueType> unified_t(const ValueType &val) : value(val) {
    static_assert(!std::is_reference_v<ValueType>, "Value type shall be not a reference!");
  }

  template <typename ValueType> unified_t &operator=(ValueType &val) {
    static_assert(!std::is_reference_v<ValueType>, "Value type shall be not a reference!");
    value = val;
    return *this;
  }

  template <typename ValueType> unified_t &operator=(ValueType &&val) {
    static_assert(!std::is_reference_v<ValueType>, "Value type shall be not a reference!");
    return *this;
  }

  template <typename ValueType> operator ValueType() {
    static_assert(!std::is_reference_v<ValueType>, "Value type shall be not a reference!");
    return value.has_value() ? std::any_cast<std::remove_reference_t<ValueType>>(value) : ValueType();
  }

  template <typename ValueType> ValueType const &operator()() const {
    static_assert(!std::is_reference_v<ValueType>, "Value type shall be not a reference!");
    return value.has_value() ? std::any_cast<std::remove_reference_t<ValueType>>(value) : ValueType();
  }

  std::any value;
};

#endif /* ENV_UTILS_HPP */
