#ifndef BASE_SERVER_PORT_HPP
#define BASE_SERVER_PORT_HPP

#include "base_port.hpp"
#include "tuple.hpp"

template <uint32_t runtime_type, typename ReturnType, typename... Args>
struct base_server_port_t<runtime_type, ReturnType(Args...)> : base_port_t<ReturnType(Args...)> {
  static_assert(!std::any_is_reference_v<ReturnType, Args...>, "There are shall be no references!");

public:
  using function_t = ReturnType(Args...);
  using base_s = base_port_t<function_t>;
  using this_t = base_server_port_t<runtime_type, function_t>;

private:
  friend struct env_base_s;

  static constexpr bool is_sync_ = (runtime_type == static_cast<uint32_t>(runtime_e::RUNTIME_SYNC));
  static constexpr bool is_async_ = (runtime_type == static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC));

  using response_t_ = response<ReturnType>;
  using request_t_ = request<Args...>;

  using response_buffer_t_ = std::queue<response_t_>;
  using request_buffer_t_ = std::queue<request_t_>;

  uint64_t transaction_count_;
  uint64_t response_queue_size_;
  uint64_t request_queue_size_;
  hook_t<void(const Args &...)> before_call_hook_;
  hook_t<void(const Args &...)> after_call_hook_;
  response_buffer_t_ response_buffer_;
  request_buffer_t_ request_buffer_;
  std::map<sha256::sha256_hash_type, std::function<ReturnType(Args...)>> calls_;

  const uint64_t &response_buffer_size_() const { return response_queue_size_; }
  const uint64_t &request_buffer_size_() const { return request_queue_size_; }

  const hook_t<void(const Args &...)> &before_call_() const { return before_call_hook_; }
  const hook_t<void(const Args &...)> &after_call_() const { return after_call_hook_; }

  const uint64_t &get_call_count_() const { return before_call_hook_.call_count(); }

  bool call_exists_(const std::string &name) const {
    const sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return (calls_.find(hash) != calls_.end());
  }

  bool call_exists_(const sha256::sha256_hash_type &id) const { return (calls_.find(id) != calls_.end()); }

  template <typename Function>
  struct env_status_s<this_t> register_call_(const std::string &name, const Function &function) {
    static_assert(
        std::is_same_v<typename std::function_traits::function_traits<Function>::function_t, ReturnType(Args...)> ||
            std::is_convertible_v<std::function<typename std::function_traits::function_traits<Function>::function_t>,
                                  std::function<ReturnType(Args...)>>,
        "Function types must be same of unified!");

    const sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return register_(hash, std::function_traits::to_std_function(function));
  }

  template <typename Class, typename Function>
  struct env_status_s<this_t> register_call_(const std::string &name, Class *p_class, const Function &function) {
    static_assert(
        std::is_same_v<typename std::function_traits::function_traits<Function>::function_t, ReturnType(Args...)> ||
            std::is_convertible_v<std::function<typename std::function_traits::function_traits<Function>::function_t>,
                                  std::function<ReturnType(Args...)>>,
        "Function types must be same of unified!");

    const sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return register_(hash, std::function_traits::to_std_function(p_class, function));
  }

  struct env_status_s<this_t>
  remove_call_(const std::string &name) {
    const sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    return remove_(hash);
  }

  struct env_status_s<this_t>
  register_(const sha256::sha256_hash_type &id, const std::function<ReturnType(Args...)> &function) {
    struct env_status_s<this_t> temp_status(this);

    if (!call_exists_(id)) {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      calls_.insert(std::make_pair(id, function));
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CALL_EXISTS));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  struct env_status_s<this_t>
  register_(const std::pair<const std::string &, const std::function<ReturnType(Args...)> &> &element) {
    const sha256::sha256_hash_type hash =
        sha256::compute(reinterpret_cast<const uint8_t *>(element.first.data()), element.first.length());
    return register_(hash, element.second);
  }

  struct env_status_s<this_t>
  remove_(const sha256::sha256_hash_type &id) {
    struct env_status_s<this_t> temp_status(this);

    if (call_exists_(id)) {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      calls_.erase(id);
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CALL_MISSING));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  struct env_status_s<this_t>
  create_response_(const sha256::sha256_hash_type &call_id, const std::tuple<Args...> &args) {
    struct env_status_s<this_t> temp_status(this);

    if (response_buffer_.size() < response_queue_size_) {

      if (calls_.find(call_id) != calls_.end()) {

        temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
        std::apply(before_call_hook_, args);
        response_buffer_.push(response_t_(this->get_id(), std::apply(calls_.at(call_id), args)));
        std::apply(after_call_hook_, args);
        transaction_count_++;
        return temp_status;

      } else {

        temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CALL_MISSING));
        this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                              error_case_e::ERROR_CASE_RUNTIME);
        return temp_status;
      }
    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  struct env_data_s<response_t_, this_t>
  get_response_() const {
    struct env_data_s<response_t_, this_t> temp_data(this);

    if (response_buffer_.empty()) {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp_data.status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_data;

    } else {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      temp_data.data = response_buffer_.back();
      response_buffer_.pop();
      return temp_data;
    }
  }

  struct env_status_s<this_t>
  push_request_(const request_t_ &request) {
    struct env_status_s<this_t> temp_status(this);

    if (request_buffer_.size() < request_queue_size_) {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      request_buffer_.push(request);
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  struct env_data_s<request_t_, this_t>
  get_request_() const {
    struct env_data_s<request_t_, this_t> temp_data(this);

    if (request_buffer_.empty()) {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp_data.status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_data;

    } else {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      temp_data.data = request_buffer_.back();
      request_buffer_.pop();
      return temp_data;
    }
  }

  public : const uint64_t &
           response_buffer_size() const {
    return response_buffer_size_();
  }
  const uint64_t &request_buffer_size() const { return request_buffer_size_(); }

  const uint64_t &get_call_count() const { return get_call_count_(); }
  const hook_t<void(const Args &...)> &before_call() const { return before_call_(); }
  const hook_t<void(const Args &...)> &after_call() const { return after_call_(); }

  template <typename Function>
  struct env_status_s<this_t> register_call(const std::string &name, const Function &function) {
    return register_call_(name, function);
  }

  template <typename Class, typename Function>
  struct env_status_s<this_t> register_call(const std::string &name, Class *p_class, const Function &function) {
    return register_call_(name, p_class, function);
  }

  struct env_status_s<this_t>
  remove_call(const std::string &name) {
    return remove_call_(name);
  } struct env_status_s<this_t> create_response(const sha256::sha256_hash_type &call_id,
                                                const std::tuple<Args...> &args) {
    return create_response_(call_id, args);
  } struct env_data_s<response_t_, this_t> get_response() const {
    return get_response_();
  } struct env_status_s<this_t> push_request(const request_t_ &request) {
    return push_request_(request);
  } struct env_data_s<request_t_, this_t> get_request() const {
    return get_request_();
  }

  explicit base_server_port_t(const std::string &name)
      : base_s(name), transaction_count_(0u), response_queue_size_(this->BUFFER_SIZE),
        request_queue_size_(this->BUFFER_SIZE){};

  explicit base_server_port_t(
      const std::string &name,
      const std::initializer_list<const std::pair<const std::string &, const std::function<ReturnType(Args...)> &>>
          &calls_list)
      : base_s(name), transaction_count_(0u), response_queue_size_(this->BUFFER_SIZE),
        request_queue_size_(this->BUFFER_SIZE) {
    std::for_each(
        calls_list.begin(), calls_list.end(), [this](const auto &element) -> auto { return register_(element); });
  }

  template <typename... Functions>
  explicit base_server_port_t(const std::string &name,
                              const std::pair<const std::string &, const Functions &> &... functions)
      : base_s(name), transaction_count_(0u), response_queue_size_(this->BUFFER_SIZE),
        request_queue_size_(this->BUFFER_SIZE) {
    std::tuple<const std::pair<const std::string &, const std::function<typename std::function_traits::function_traits<
                                                        Functions>::function_t> &> &...>
        temp_tp = std::make_tuple(
            (std::make_pair(functions.first, std::function_traits::to_std_function(functions.second)))...);

    for (unsigned int i = 0; i < sizeof...(functions); i++) {

      struct env_status_s<this_t> register_status = std::visit_at(
          i, [i, this](const auto &element) -> struct env_status_s<this_t> {
        return register_(element); }, temp_tp);

      if (register_status.qualifiers.at(this->get_id()) != env_errno_e::ENV_CLEAR) {

        this->error_handler()(this->get_id(), register_status.qualifiers.at(this->get_id()),
                              error_case_e::ERROR_CASE_RUNTIME);
        return;
      }
    }
  };

  virtual ~base_server_port_t() override = default;
};

#endif /* BASE_SERVER_PORT_HPP */
