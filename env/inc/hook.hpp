#ifndef HOOK_HPP
#define HOOK_HPP

#include "env_utils.hpp"
#include "sha256.hpp"
#include "type_traits_ex.hpp"
#include <functional>
#include <map>

template <typename FunctionType>
struct hook_t : std::function<typename std::function_traits::function_traits<FunctionType>::function_t> {};
template <typename FunctionType>
struct hook_impl_t : std::function<typename std::function_traits::function_traits<FunctionType>::function_t> {};

template <typename... Args> struct hook_impl_t<void(Args...)> : std::function<void(Args...)> {
public:
  using this_t = hook_impl_t<void(Args...)>;

  explicit hook_impl_t()
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_hook"), std::strlen("unnamed_hook"))),
        hash_(types_hash<Args...>()),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename NameType>
  explicit hook_impl_t(const NameType &name)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Args...>()),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  virtual ~hook_impl_t() { clear(); };

  template <typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add(const std::string &name, const FunctionType &function) const {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());

    if constexpr (std::is_entity_is_function_v<FunctionType>)
      return add_(unique_id, std::function_traits::to_std_function(function));
    else
      return add_(unique_id, function);
  }

  template <typename Class, typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add(const std::string &name, Class *p_class, const FunctionType &function) const {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, std::function_traits::to_std_function(p_class, function));
  }

  template <typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add(const std::string &name, const uint64_t &number, const FunctionType &function) const {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());

    if constexpr (std::is_entity_is_function_v<FunctionType>)
      return add_(unique_id, number, std::function_traits::to_std_function(function));
    else
      return add_(unique_id, number, function);
  }

  template <typename Class, typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add(const std::string &name, const uint64_t &number, Class *p_class, const FunctionType &function) const {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, number, std::function_traits::to_std_function(p_class, function));
  }

  env_data_t<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove(const std::string &name) const {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return remove_(unique_id);
  }

  template <typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  replace(const std::string &name, const std::string &new_name, const FunctionType &function) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_data(this);
    env_data_t<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != hook_errno_t::HOOK_CLEAR) {

      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;

    } else {

      return add(new_name, remove_result.data.second, function);
    }
  }

  template <typename Class, typename FunctionType>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  replace(const std::string &name, const std::string &new_name, Class *p_parent, const FunctionType &function) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_data(this);
    env_data_t<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != hook_errno_t::HOOK_CLEAR) {

      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;

    } else {

      return add(new_name, remove_result.data.second, p_parent, function);
    }
  }

  virtual void operator()(const Args &... args) const { run_(args...); }
  env_status_t<this_t> clear() const { return clear_(); }
  uint64_t call_count() { return call_count_; }
  
  template <typename FunctionType> env_status_t<this_t> set_error_handler(const FunctionType &function) const {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    temp_status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
    return temp_status;
  }

  template <typename Class, typename FunctionType>
  env_status_t<this_t> set_error_handler(Class *p_class, const FunctionType &function) const {
    env_status_t<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    temp_status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
    return temp_status;
  }

  const sha256::sha256_hash_type &get_id() const { return id_; }
  bool is_empty() const { return functions_.empty(); }

private:
  mutable std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
  mutable std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<void(Args...)>>> functions_;
  mutable uint64_t current_number_;
  mutable uint64_t call_count_;
  sha256::sha256_hash_type id_;
  sha256::sha256_hash_type hash_;

  struct concept_ {
    virtual void call(Args &&... args) = 0;
    virtual ~concept_() = default;
  };

  template <typename Callable> struct model_ : concept_ {
    model_(Callable &&callable) : callable_(std::move(callable)) {}
    void call(Args &&... args) override { callable_(std::forward<Args>(args)...); }
    Callable callable_;
  };

  const typename std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<void(Args...)>>>::iterator
  find_function_(const sha256::sha256_hash_type &id) const {
    return std::find_if(functions_.begin(), functions_.end(),
                        [&id](const auto &function) -> bool { return function.second.first == id; });
  }

  bool function_exists_(const sha256::sha256_hash_type &id) const {
    if (find_function_(id) == functions_.end())
      return false;
    else
      return true;
  }

  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const std::function<void(Args...)> &function) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_result(this);

    if (!function_exists_(id)) {

      functions_.insert(std::make_pair(current_number_++, std::make_pair(id, function)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;

    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_EXISTS));
      error_handler_(id_, temp_result.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;
    }
  }

  template <typename Callable>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add_(const sha256::sha256_hash_type &id, Callable &&callable) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_result(this);
    std::function<void(Args...)> func = [f = std::make_shared<model_<Callable>>(std::forward<Callable>(callable))](
                                            Args... args) -> void { f->call(std::forward<Args>(args)...); };

    if (!function_exists_(id)) {

      functions_.insert(std::make_pair(current_number_++, std::make_pair(id, std::move(func))));
      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;

    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_EXISTS));
      error_handler_(id_, temp_result.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;
    }
  }

  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const uint64_t &number, const std::function<void(Args...)> &function) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_result(this);

    if ((!function_exists_(id)) && (functions_.find(number) == functions_.end())) {

      functions_.insert(std::make_pair(number, std::make_pair(id, function)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;

    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_EXISTS));
      error_handler_(id_, temp_result.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;
    }
  }

  template <typename Callable>
  env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const uint64_t &number, const Callable &callable) const {
    env_data_t<std::pair<sha256::sha256_hash_type, std::function<void(Args...)> *>, this_t> temp_result(this);
    std::function<void(Args...)> func = [f = const_cast<Callable *>(&callable)](Args... args) -> void {
      static_cast<void>((*f)(std::forward<Args>(args)...));
    };

    if ((!function_exists_(id)) && (functions_.find(number) == functions_.end())) {

      functions_.insert(std::make_pair(number, std::make_pair(id, std::move(func))));
      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;

    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_EXISTS));
      error_handler_(id_, temp_result.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_result.data = std::make_pair(id, &find_function_(id)->second.second);
      return temp_result;
    }
  }

  env_data_t<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_(const sha256::sha256_hash_type &id) const {
    env_data_t<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> temp_result(this);
    uint64_t number = 0u;

    if (function_exists_(id)) {

      number = find_function_(id)->first;
      functions_.erase(find_function_(id));
      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
      temp_result.data = std::make_pair(id, number);
      return temp_result;

    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_MISSING));
      error_handler_(id_, temp_result.status.qualifiers.at(id_), error_case_t::ERROR_CASE_RUNTIME);
      temp_result.data = std::make_pair(id, number);
      return temp_result;
    }
  }

  void run_(const Args &... args) const {
    for (const auto &element : functions_)
      static_cast<void>((element.second.second)(args...));
    call_count_++;
  }

  env_status_t<this_t> clear_() const {
    env_status_t<this_t> temp_status(this);
    temp_status.qualifiers.insert(std::make_pair(id_, hook_errno_t::HOOK_CLEAR));
    current_number_ = 0u;
    functions_.clear();
    return temp_status;
  }
};

template <typename... Args> struct hook_t<void(Args...)> : hook_impl_t<void(Args...)> {};
template <typename RetType, typename... Args> struct hook_t<RetType(Args...)> : hook_impl_t<void(Args...)> {};

#endif /* HOOK_HPP */
