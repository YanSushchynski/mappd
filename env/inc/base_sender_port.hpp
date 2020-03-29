#ifndef BASE_SENDER_PORT_HPP
#define BASE_SENDER_PORT_HPP

#include "base_port.hpp"

template <uint32_t runtime_type, typename DataType>
struct base_sender_port_t<runtime_type, DataType> : base_port_t<DataType> {
  static_assert(!std::is_reference_v<DataType>, "Data type shall be not a reference!");

public:
  using data_t = DataType;
  using this_t = base_sender_port_t<runtime_type, data_t>;
  using base_t = base_port_t<data_t>;

private:
  friend struct env_base_s;

  static constexpr bool is_sync_ = (runtime_type == static_cast<uint32_t>(runtime_e::RUNTIME_SYNC));
  static constexpr bool is_async_ = (runtime_type == static_cast<uint32_t>(runtime_e::RUNTIME_ASYNC)) && !is_sync_;

  using buffer_t_ = std::vector<data_t>;

  uint64_t queue_size_;
  uint64_t transaction_count_;
  hook_t<void(const data_t &)> hook_;
  functor_t<data_t &(data_t &), this_t> functor_;
  buffer_t_ buffer_;

  const uint64_t &buffer_size_() const { return buffer_.size(); }
  const uint64_t &write_count_() const { return functor_.call_count(); }
  const hook_t<void(const data_t &)> &on_send_() const { return hook_; }
  const functor_t<data_t &(data_t &), this_t> &writer_() const { return functor_; }

  template <typename RetType = uint64_t>
  const typename std::enable_if<is_sync_, RetType>::type &get_transaction_count_() {
    return transaction_count_;
  }

  template <typename Function> struct env_status_s<this_t> set_writer_(Function function) {
    struct env_status_s<this_t> temp_status(this);
    functor_.assign(functor_t<data_t &(data_t &), this_t>(function));
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
    return temp_status;
  }

  struct env_status_s<this_t>
  reset_writer_() {
    struct env_status_s<this_t> temp_status(this);
    functor_.assign(functor_t<data_t &(data_t &), this_t>([](data_t &lvr_data) -> data_t & { return lvr_data; }));
    temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
    return temp_status;
  }

  struct env_status_s<this_t>
  enqueue_data_(const data_t &data) {
    struct env_status_s<this_t> temp_status(this);

    if (buffer_.size() < queue_size_) {

      buffer_.push_back(functor_(data));
      transaction_count_++;
      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  struct env_data_s<data_t, this_t>
  dequeue_data_() {
    struct env_data_s<data_t, this_t> temp_status(this);

    if (buffer_.empty()) {

      temp_status.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp_status.status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;

    } else {

      temp_status.data = buffer_.front();
      temp_status.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      buffer_.erase(buffer_.begin());
      return temp_status;
    }
  }

  struct env_status_s<this_t>
  forward_() {
    struct env_status_s<this_t> temp_status(this);

    if (transaction_count_) {

      while (transaction_count_) {

        for (struct port_info_s &connected : this->connected()) {

          struct env_data_s<data_t, this_t> temp_data = dequeue_data_();

          if (temp_data.status.qualifiers.at(this->get_id()) == env_errno_e::ENV_CLEAR) {

            temp_status.qualifiers.insert(
                std::make_pair(this->get_id(), temp_data.status.qualifiers.at(this->get_id())));
            on_send_()(temp_data.data);
            // auto receiver_status = connected.second->enqueue_data(
            // std::forward< data_t >( temp_data.data )); /* TODO: change
            // connection algorithm( use "port_manager" as part of environment )
            // */

            // if( receiver_status.qualifiers.at( connected.second->get_id() )
            // != env_errno_e::ENV_CLEAR ){ /* TODO: change connection
            // algorithm( use "port_manager" as part of environment ) */

            //   temp_status.qualifiers.insert( std::make_pair( this->get_id(),
            //   receiver_status.qualifiers.at( connected.second->get_id() )));
            //   /* TODO: change connection algorithm( use "port_manager" as
            //   part of environment ) */ goto next;

            // } else {

            //   temp_status.qualifiers.insert( std::make_pair( this->get_id(),
            //   env_errno_e::ENV_CLEAR )); connected.second->on_receive()(
            //   temp_data.data ); /* TODO: change connection algorithm( use
            //   "port_manager" as part of environment ) */ goto next;
            // }
          } else {

            temp_status.qualifiers.insert(
                std::make_pair(this->get_id(), temp_data.status.qualifiers.at(this->get_id())));
            this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                                  error_case_e::ERROR_CASE_RUNTIME);
            goto next;
          }
        }

      next:

        transaction_count_--;
      }

      return temp_status;

    } else {

      temp_status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_NO_TRANSACTIONS));
      this->error_handler()(this->get_id(), temp_status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp_status;
    }
  }

  public : const uint64_t &
           buffer_size() const {
    return buffer_size_();
  }
  const uint64_t &write_count() const { return write_count_(); }

  template <typename RetType = uint64_t>
  const typename std::enable_if<is_sync_, RetType>::type &get_transaction_count() {
    return get_transaction_count_();
  }

  template <typename Function> struct env_status_s<this_t> set_writer(Function function) {
    return set_writer_(function);
  }

  struct env_status_s<this_t>
  reset_writer() {
    return reset_writer_();
  } struct env_status_s<this_t> enqueue_data(const data_t &data) {
    return enqueue_data_(data);
  } struct env_data_s<data_t, this_t> dequeue_data() {
    return dequeue_data_();
  } const hook_t<void(const data_t &)> &on_send() const {
    return on_send_();
  }
  const functor_t<data_t &(data_t &), this_t> &writer() const { return writer_(); }
  struct env_status_s<this_t> forward() {
    return forward_();
  }

  explicit base_sender_port_t(const std::string &name)
      : base_port_t<data_t>(name), transaction_count_(0u), queue_size_(this->BUFFER_SIZE),
        functor_(functor_t([](data_t &lvr_data) -> data_t & { return lvr_data; })){};

  virtual ~base_sender_port_t() override = default;
};

#endif /* BASE_SENDER_PORT_HPP */
