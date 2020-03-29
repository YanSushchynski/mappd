#ifndef SEQUENCE_HPP
#define SEQUENCE_HPP

#include "env_utils.hpp"
#include "type_traits_ex.hpp"
#include <functional>
#include <map>

template <typename FunctionType, typename Class>
struct sequence_t : std::function<typename std::function_traits::function_traits<FunctionType>::function_t> {};
template <typename Class, typename RetType, typename... Args>
struct sequence_t<RetType(Args...), Class> : std::function<RetType(Args...)> {
public:
  using return_t = RetType;
  using class_t = Class;
  using function_t = RetType(Args...);
  using part_t = return_t &(return_t &);
  using this_t = sequence_t<RetType(Args...), Class>;

  template <typename FunctionType>
  explicit sequence_t(const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_sequence"), std::strlen("unnamed_sequence"))),
        hash_(types_hash<Class, RetType, Args...>()), base_(std::function_traits::to_std_function(function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(Class *p_class, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_sequence"), std::strlen("unnamed_sequence"))),
        hash_(types_hash<Class, RetType, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(const std::string &name, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, RetType, Args...>()), base_(std::function_traits::to_std_function(function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(const std::string &name, Class *p_class, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, RetType, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  virtual ~sequence_t() { clear(); };

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, std::function_traits::to_std_function(function));
  }

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const uint64_t &number, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, number, std::function_traits::to_std_function(function));
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, OwnerClass *p_class, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, std::function_traits::to_std_function(p_class, function));
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const uint64_t &number, OwnerClass *p_class, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, number, std::function_traits::to_std_function(p_class, function));
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t>
  remove(const std::string &name) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return remove_(unique_id);
  }

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  replace(const std::string &name, const std::string &new_name, const Function &function) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)) {

      struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_data(this);
      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;

    } else {

      return add(new_name, remove_result.data.second, function);
    }
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  replace(const std::string &name, const std::string &new_name, OwnerClass *p_class, const Function &function) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)) {

      struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_data(this);
      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;

    } else {

      return add(new_name, remove_result.data.second, p_class, function);
    }
  }

  return_t
  operator()(const Args &... args) const {
    return run_(args...);
  }
  struct env_status_s<this_t> clear() {
    return clear_();
  } const uint64_t &call_count() const {
    return call_count_;
  }
  const std::function<function_t> &get_base() const { return base_; }

  template <typename Function> struct env_status_s<this_t> set_error_handler(const Function &function) {
    struct env_status_s<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  struct env_status_s<this_t> set_error_handler(OwnerClass *p_class, const Function &function) {
    struct env_status_s<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }

  const sha256::sha256_hash_type &
  get_id() const {
    return id_;
  }

private:
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
  std::function<function_t> base_;
  std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<part_t>>> parts_;
  uint64_t current_number_;
  mutable uint64_t call_count_;
  sha256::sha256_hash_type id_;
  sha256::sha256_hash_type hash_;

  const typename std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<part_t>>>::const_iterator
  find_part_(const sha256::sha256_hash_type &id) const {
    return std::find_if(parts_.begin(), parts_.end(),
                        [&id](const auto &part) -> bool { return part.second.first == id; });
  }

  bool part_exists_(const sha256::sha256_hash_type &id) const {
    if (find_part_(id) == parts_.end())
      return false;
    else
      return true;
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const std::function<part_t> &part) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_result(this);

    if (!part_exists_(id)) {

      parts_.insert(std::make_pair(current_number_++, std::make_pair(id, part)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_EXISTS)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    }
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const uint64_t &number, const std::function<part_t> &part) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_result(this);

    if ((!part_exists_(id)) && (parts_.find(number) == parts_.end())) {

      parts_.insert(std::make_pair(number, std::make_pair(id, part)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_EXISTS)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    }
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t>
  remove_(const sha256::sha256_hash_type &id) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> temp_result(this);
    uint64_t number = 0u;

    if (part_exists_(id)) {

      number = find_part_(id)->first;
      parts_.erase(find_part_(id));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data = std::make_pair(id, number);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_MISSING)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data = std::make_pair(id, number);
      return temp_result;
    }
  }

  return_t
  run_(const Args &... args) const {
    return_t temp = base_(args...);
    for (auto it = parts_.begin(); it != parts_.end(); it++)
      temp = it->second.second(temp);
    call_count_++;
    return temp;
  }

  struct env_status_s<this_t> clear_() {
    struct env_status_s<this_t> temp_status(this);
    current_number_ = 0u;
    parts_.clear();
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }
};

template <typename Class, typename... Args> struct sequence_t<void(Args...), Class> : std::function<void(Args...)> {
public:
  using function_t = void(Args...);
  using part_t = void(void);
  using this_t = sequence_t<void(Args...), Class>;

  template <typename FunctionType>
  explicit sequence_t(const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_sequence"), std::strlen("unnamed_sequence"))),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(Class *p_class, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>("unnamed_sequence"), std::strlen("unnamed_sequence"))),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(const std::string &name, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  template <typename FunctionType>
  explicit sequence_t(const std::string &name, Class *p_class, const FunctionType &function)
      : current_number_(0u), call_count_(0u),
        id_(sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length())),
        hash_(types_hash<Class, void, Args...>()), base_(std::function_traits::to_std_function(p_class, function)),
        error_handler_([](const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &) -> void {}){};

  virtual ~sequence_t() { clear(); };

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, std::function_traits::to_std_function(function));
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, OwnerClass *p_class, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, std::function_traits::to_std_function(p_class, function));
  }

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const uint64_t &number, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, number, std::function_traits::to_std_function(function));
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add(const std::string &name, const uint64_t &number, OwnerClass *p_class, const Function &function) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return add_(unique_id, number, std::function_traits::to_std_function(p_class, function));
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t>
  remove(const std::string &name) {
    const sha256::sha256_hash_type unique_id =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return remove_(unique_id);
  }

  template <typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  replace(const std::string &name, const std::string &new_name, const Function &function) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)) {

      struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_data(this);
      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;
    } else {

      return add(new_name, remove_result.data.second, function);
    }
  }

  template <typename OwnerClass, typename Function>
  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  replace(const std::string &name, const std::string &new_name, OwnerClass *p_class, const Function &function) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> remove_result = remove(name);

    if (remove_result.status.qualifiers.at(id_) != static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)) {

      struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_data(this);
      temp_data.status.qualifiers.insert(std::make_pair(id_, remove_result.status.qualifiers.at(id_)));
      error_handler_(id_, temp_data.status.qualifiers.at(id_), static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_data.data = std::make_pair(remove_result.data.first, nullptr);
      return temp_data;
    } else {

      return add(new_name, remove_result.data.second, p_class, function);
    }
  }

  void
  operator()(const Args &... args) const {
    run_(args...);
  }
  struct env_status_s<this_t> clear() {
    return clear_();
  } const uint64_t &call_count() const {
    return call_count_;
  }
  const std::function<function_t> &get_base() const { return base_; }

  template <typename Function> struct env_status_s<this_t> set_error_handler(const Function &function) {
    struct env_status_s<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(function);
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }

  template <typename OwnerClass, typename Function>
  struct env_status_s<this_t> set_error_handler(OwnerClass *p_class, const Function &function) {
    struct env_status_s<this_t> temp_status(this);
    error_handler_ = std::function_traits::to_std_function(p_class, function);
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }

  const sha256::sha256_hash_type &
  get_id() const {
    return id_;
  }

private:
  std::function<void(const sha256::sha256_hash_type &, const uint32_t &, const uint32_t &)> error_handler_;
  std::function<function_t> base_;
  std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<part_t>>> parts_;
  uint64_t current_number_;
  mutable uint64_t call_count_;
  sha256::sha256_hash_type id_;
  sha256::sha256_hash_type hash_;

  const typename std::map<uint64_t, std::pair<sha256::sha256_hash_type, std::function<part_t>>>::const_iterator
  find_part_(const sha256::sha256_hash_type &id) const {
    return std::find_if(parts_.begin(), parts_.end(),
                        [id](const auto &part) -> bool { return part.second.first == id; });
  }

  bool part_exists_(const sha256::sha256_hash_type &id) const {
    if (find_part_(id) == parts_.end())
      return false;
    else
      return true;
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const std::function<part_t> &part) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_result(this);

    if (!part_exists_(id)) {

      parts_.insert(std::make_pair(current_number_++, std::make_pair(id, part)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_EXISTS)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    }
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t>
  add_(const sha256::sha256_hash_type &id, const uint64_t &number, const std::function<part_t> &part) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, std::function<part_t> *>, this_t> temp_result(this);

    if ((!part_exists_(id)) && (parts_.find(number) == parts_.end())) {

      parts_.insert(std::make_pair(number, std::make_pair(id, part)));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_EXISTS)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data.first = id;
      temp_result.data.second = const_cast<std::function<part_t> *>(&find_part_(id)->second.second);
      return temp_result;
    }
  }

  struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t>
  remove_(const sha256::sha256_hash_type &id) {
    struct env_data_s<std::pair<sha256::sha256_hash_type, uint64_t>, this_t> temp_result(this);
    uint64_t number = 0u;

    if (part_exists_(id)) {

      number = find_part_(id)->first;
      parts_.erase(find_part_(id));
      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
      temp_result.data = std::make_pair(id, number);
      return temp_result;
    } else {

      temp_result.status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_MISSING)));
      error_handler_(id_, temp_result.status.qualifiers.at(id_),
                     static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME));
      temp_result.data = std::make_pair(id, number);
      return temp_result;
    }
  }

  void
  run_(const Args &... args) const {
    base_(args...);
    for (auto it = parts_.begin(); it != parts_.end(); it++)
      it->second.second();
    call_count_++;
  }

  struct env_status_s<this_t> clear_() {
    struct env_status_s<this_t> temp_status(this);
    current_number_ = 0u;
    parts_.clear();
    temp_status.qualifiers.insert(std::make_pair(id_, static_cast<uint32_t>(seq_errno_e::SEQ_CLEAR)));
    return temp_status;
  }
};

template <typename FunctionType>
explicit sequence_t(const FunctionType &function)
    ->sequence_t<typename std::function_traits::function_traits<FunctionType>::function_t, void>;
template <typename Class, typename FunctionType>
explicit sequence_t(Class *, const FunctionType &function)
    ->sequence_t<typename std::function_traits::function_traits<FunctionType>::function_t, Class>;

#endif /* SEQUENCE_HPP */
