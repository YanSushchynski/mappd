#ifndef FUNCTOR_HPP
#define FUNCTOR_HPP

#include "env_utils.hpp"
#include "hook.hpp"
#include "sequence.hpp"
#include "type_traits_ex.hpp"

template <typename FunctionType, typename Class>
struct functor_t : std::function<typename std::function_traits::function_traits<FunctionType>::function_t> {};
template <typename Class, typename RetType, typename... Args>
struct functor_t<RetType(Args...), Class> : std::function<RetType(Args...)> {
public:
  using return_t = RetType;
  using class_t = Class;
  using function_t = RetType(Args...);
  using this_t = functor_t<RetType(Args...), class_t>;

  template <typename FunctionType>
  explicit functor_t(const FunctionType &function)
      : call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_functor"), std::strlen("unnamed_functor"))),
        last_state_(std::remove_reference_t<return_t>()), hash_(types_hash<Class, RetType, Args...>()),
        base_(std::function_traits::to_std_function(function)), sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename FunctionType>
  explicit functor_t(Class *p_class, const FunctionType &function)
      : call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_functor"), std::strlen("unnamed_functor"))),
        last_state_(std::remove_reference_t<return_t>()), hash_(types_hash<Class, RetType, Args...>()),
        base_(std::function_traits::to_std_function(p_class, function)), sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename NameType, typename FunctionType>
  explicit functor_t(const NameType &name, const FunctionType &function)
      : call_count_(0u), id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        last_state_(std::remove_reference_t<return_t>()), hash_(types_hash<Class, RetType, Args...>()),
        base_(std::function_traits::to_std_function(function)), sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename NameType, typename FunctionType>
  explicit functor_t(const NameType &name, Class *p_class, const FunctionType &function)
      : call_count_(0u), id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        last_state_(std::remove_reference_t<return_t>()), hash_(types_hash<Class, RetType, Args...>()),
        base_(std::function_traits::to_std_function(p_class, function)), sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  virtual ~functor_t() = default;

  const functor_t<RetType(Args...), Class> &assign(functor_t<RetType(Args...), Class> &&other) {
    *this = other;
    return *this;
  }

  return_t operator()(const Args &... args) const {
    hooks_before_(args...);
    last_state_ = sequence_(args...);
    hooks_after_(args...);
    call_count_++;
    return last_state_;
  }

  const hook_t<function_t> &get_hooks_before() const { return hooks_before_; }
  const hook_t<function_t> &get_hooks_after() const { return hooks_after_; }
  const sequence_t<function_t, Class> &get_sequence() const { return sequence_; }
  const std::function<function_t> &get_base() const { return base_; }

  template <typename Function> env_status_t<this_t> set_error_handler(const Function &function) {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), funct_errno_t::FUNCT_CLEAR));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  env_status_t<this_t> set_error_handler(OwnerClass *p_class, const Function &function) {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), funct_errno_t::FUNCT_CLEAR));
    return temp_status;
  }

  const uint64_t &call_count() const { return call_count_; }
  const sha256::sha256_hash_type &get_id() const { return id_; }

private:
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
  std::function<function_t> base_;
  sequence_t<function_t, Class> sequence_;
  hook_t<function_t> hooks_before_;
  hook_t<function_t> hooks_after_;
  mutable uint64_t call_count_;
  mutable std::remove_reference_t<return_t> last_state_;
  sha256::sha256_hash_type id_;
  sha256::sha256_hash_type hash_;
};

template <typename Class, typename... Args> struct functor_t<void(Args...), Class> : std::function<void(Args...)> {
public:
  using class_t = Class;
  using function_t = void(Args...);
  using this_t = functor_t<void(Args...), class_t>;

  template <typename FunctionType>
  explicit functor_t(const FunctionType &function)
      : call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_functor"), std::strlen("unnamed_functor"))),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(function)),
        sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename FunctionType>
  explicit functor_t(Class *p_class, const FunctionType &function)
      : call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_functor"), std::strlen("unnamed_functor"))),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename NameType, typename FunctionType>
  explicit functor_t(const NameType &name, const FunctionType &function)
      : call_count_(0u), id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(function)),
        sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  template <typename NameType, typename FunctionType>
  explicit functor_t(const NameType &name, Class *p_class, const FunctionType &function)
      : call_count_(0u), id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        sequence_(sequence_t(base_)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &,
                          const uint32_t & = error_case_t::ERROR_CASE_RUNTIME) -> void {}){};

  virtual ~functor_t() = default;

  const functor_t<void(Args...), Class> &assign(functor_t<void(Args...), Class> &&other) {
    *this = std::move(other);
    return *this;
  }

  void operator()(const Args &... args) const {
    hooks_before_(args...);
    sequence_(args...);
    hooks_after_(args...);
    call_count_++;
  }

  const hook_t<function_t> &get_hooks_before() const { return hooks_before_; }
  const hook_t<function_t> &get_hooks_after() const { return hooks_after_; }
  const sequence_t<function_t, Class> &get_sequence() const { return sequence_; }
  const std::function<function_t> &get_base() const { return base_; }

  template <typename Function> env_status_t<this_t> set_error_handler(const Function &function) {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), funct_errno_t::FUNCT_CLEAR));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  env_status_t<this_t> set_error_handler(OwnerClass *p_class, const Function &function) {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), funct_errno_t::FUNCT_CLEAR));
    return temp_status;
  }

  const uint64_t &call_count() const { return call_count_; }
  const sha256::sha256_hash_type &get_id() const { return id_; }

private:
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
  std::function<function_t> base_;
  sequence_t<function_t, Class> sequence_;
  hook_t<function_t> hooks_before_;
  hook_t<function_t> hooks_after_;
  mutable uint64_t call_count_;
  sha256::sha256_hash_type id_;
  sha256::sha256_hash_type hash_;
};

template <typename FunctionType>
explicit functor_t(FunctionType)
    ->functor_t<typename std::function_traits::function_traits<FunctionType>::function_t, void>;
template <typename Class, typename FunctionType>
explicit functor_t(Class *, FunctionType)
    ->functor_t<typename std::function_traits::function_traits<FunctionType>::function_t, Class>;

#endif /* FUNCTOR_HPP */
