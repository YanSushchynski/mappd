#ifndef BASE_CLIENT_PORT_HPP
#define BASE_CLIENT_PORT_HPP

#include "base_port.hpp"

template <uint32_t runtime_type, typename ReturnType, typename... Args>
struct base_client_port_t<runtime_type, ReturnType(Args...)> : base_port_t<ReturnType(Args...)> {
  static_assert(!std::any_is_reference_v<ReturnType, Args...>, "There are shall be no references!");

public:
  using return_t = ReturnType;
  using function_t = ReturnType(Args...);
  using base_t = base_port_t<ReturnType(Args...)>;
  using this_t = base_client_port_t<runtime_type, function_t>;
  
private:
  friend struct env_base_t;

  static constexpr bool is_sync_ = (runtime_type == runtime_t::RUNTIME_SYNC);
  static constexpr bool is_async_ = (runtime_type == runtime_t::RUNTIME_ASYNC) && !is_sync_;

  using response_t_ = response<return_t>;
  using request_t_ = request<Args...>;
  
  using response_buffer_t_ = std::vector<response_t_>;
  using request_buffer_t_ = std::vector<request_t_>;

  mutable uint64_t transaction_count_;
  uint64_t response_queue_size_;
  uint64_t request_queue_size_;
  hook_t<void(const Args &...)> before_call_hook_;
  hook_t<void(const Args &...)> after_call_hook_;
  response_buffer_t_ response_queue_;
  request_buffer_t_ request_queue_;

  const uint64_t &response_buffer_size_() const { return response_queue_size_; }
  const uint64_t &request_buffer_size_() const { return request_queue_size_; }

  const hook_t<void(const Args &...)> &before_call_() const { return before_call_hook_; }
  const hook_t<void(const Args &...)> &after_call_() const { return after_call_hook_; }
  const uint64_t &get_call_count_() const { return before_call_hook_.call_count(); }

  env_status_t<this_t> create_request_(const sha256::sha256_hash_type &server_id,
                                       const sha256::sha256_hash_type &request_id, const Args &... args) {
    env_status_t<this_t> temp_status(this);

    if (request_queue_.size() < request_queue_size_) {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_CLEAR));
      request_queue_.push_back(request_t_(server_id, request_id, args...));
      transaction_count_++;
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_t::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  env_data_t<request_t_, this_t> get_request_() const {
    env_data_t<request_t_, this_t> temp_data(this);

    if (request_queue_.empty()) {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp_data.status.qualifiers.at(this->get_id()),
                            error_case_t::ERROR_CASE_RUNTIME);
      return temp_data;

    } else {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_CLEAR));
      temp_data.data = request_queue_.front();
      request_queue_.erase(request_queue_.begin());
      return temp_data;
    }
  }

  env_status_t<this_t> push_response_(const response_t_ &resp) {
    env_status_t<this_t> temp_status(this);

    if (response_queue_.size() < response_queue_size_) {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_CLEAR));
      response_queue_.push_back(resp);
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_t::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  env_data_t<response_t_, this_t> get_response_() const {
    env_data_t<response_t_, this_t> temp_data(this);

    if (response_queue_.empty()) {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp_data.status.qualifiers.at(this->get_id()),
                            error_case_t::ERROR_CASE_RUNTIME);
      return temp_data;

    } else {

      temp_data.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_CLEAR));
      temp_data.data = response_queue_.front();
      response_queue_.erase(response_queue_.begin());
      return temp_data;
    }
  }

  env_status_t<this_t> forward_() const {
    env_status_t<this_t> temp_status(this);

    if (transaction_count_) {

      while (transaction_count_) {

        // env_data_t< request_t_, this_t > temp_data = get_request_();

        // if( this->is_connected( temp_data.data.server_id )){

        // friend_port_t * p_server = this->connected().at(
        // temp_data.data.server_id ); env_status_t< typename
        // friend_port_t::base_t > push_request_status = p_server->push_request(
        // std::forward< request_t_ >( temp_data.data ));

        // if( push_request_status.qualifiers.at( p_server->get_id() ) !=
        // env_errno_t::ENV_CLEAR ){

        // temp_status.qualifiers.insert( std::make_pair( this->get_id(),
        // push_request_status.qualifiers.at( p_server->get_id() ))); goto next;

        // } else {

        // env_status_t< typename friend_port_t::base_t > create_response_status
        // = p_server->create_response( temp_data.data.request_id, std::forward<
        // std::tuple< Args ... >>( temp_data.data.payload ));

        // 	if( create_response_status.qualifiers.at( p_server->get_id() )
        // != env_errno_t::ENV_CLEAR ){

        // 	  temp_status.qualifiers.insert( std::make_pair( this->get_id(),
        // create_response_status.qualifiers.at( p_server->get_id() ))); 	  goto
        // next;

        // 	} else {

        // 	  env_data_t< response_t_, typename friend_port_t::base_t >
        // response_data = p_server->get_response();

        // 	  if( response_data.status.qualifiers.at( p_server->get_id() )
        // != env_errno_t::ENV_CLEAR ){

        // 		temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), response_data.status.qualifiers.at(
        // p_server->get_id() ))); 		goto next;

        // 	  } else {

        // 		env_status_t< this_t > push_response_status =
        // push_response_( std::forward< response_t_ >( response_data.data ));

        // 		if( push_response_status.qualifiers.at( this->get_id() ) !=
        // env_errno_t::ENV_CLEAR ){

        // 		  temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), push_response_status.qualifiers.at( this->get_id()
        // ))); 		  this->error_handler()( this->get_id(),
        // push_response_status.qualifiers.at( this->get_id() ),
        // error_case_t::ERROR_CASE_RUNTIME ); 		  goto next;

        // 		} else {

        // 		  temp_status.qualifiers.insert( std::make_pair(
        // this->get_id(), env_errno_t::ENV_CLEAR )); 		  goto next;
        // 		}
        // 	  }
        // 	}
        //   }
        // } else {

        //   temp_status.qualifiers.insert( std::make_pair( this->get_id(),
        //   env_errno_t::ENV_PORT_MISSING )); this->error_handler()(
        //   this->get_id(), temp_status.qualifiers.at( this->get_id() ),
        //   error_case_t::ERROR_CASE_RUNTIME ); goto next;
        // }

        // next:

        transaction_count_--;
      }

      return temp_status;
    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_t::ENV_NO_TRANSACTIONS));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_t::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

public:
  const uint64_t &response_buffer_size() const { return response_buffer_size_(); }
  const uint64_t &request_buffer_size() const { return request_buffer_size_(); }
  const uint64_t &get_call_count() const { return get_call_count_(); }

  const hook_t<void(const Args &...)> &before_call() const { return before_call_(); }
  const hook_t<void(const Args &...)> &after_call() const { return after_call_(); }

  env_status_t<this_t> create_request(const sha256::sha256_hash_type &server_id,
                                      const sha256::sha256_hash_type &request_id, const Args &... args) {
    return create_request_(server_id, request_id, args...);
  }

  env_data_t<request_t_, this_t> get_request() const { return get_request_(); }
  env_status_t<this_t> push_response(const response_t_ &resp) { return push_response_(resp); }
  env_data_t<response_t_, this_t> get_response() const { return get_response_(); }
  env_status_t<this_t> forward() const { return forward_(); }

  template <typename NameType>
  explicit base_client_port_t(const NameType &name)
      : base_t(name), response_queue_size_(this->BUFFER_SIZE), request_queue_size_(this->BUFFER_SIZE),
        transaction_count_(0u){};

  virtual ~base_client_port_t() override = default;
};

#endif /* BASE_CLIENT_PORT_HPP */
