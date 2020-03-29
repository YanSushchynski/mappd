#ifndef BASE_RECEIVER_PORT_HPP
#define BASE_RECEIVER_PORT_HPP

#include "base_port.hpp"

template <typename DataType> struct base_receiver_port_t<DataType> : public base_port_t<DataType> {
  static_assert(!std::is_reference_v<DataType>, "Data type shall be not a reference!");

public:
  using data_t = DataType;
  using this_t = base_receiver_port_t<data_t>;
  using base_s = base_port_t<data_t>;
  using name_t = typename base_s::name_t;

private:
  friend struct env_base_s;

  using buffer_t_ = std::vector<data_t>;

  uint64_t queue_size_;
  hook_t<void(const data_t &)> hook_;
  functor_t<data_t &(data_t &), this_t> reader_;
  buffer_t_ buffer_;

  const uint64_t &buffer_size_() const { return buffer_.size(); }
  const uint64_t &read_count_() const { return reader_.call_count(); }
  const hook_t<void(const data_t &)> &get_on_receive_() const { return hook_; }
  const functor_t<data_t &(data_t &), this_t> &get_reader_() const { return reader_; }

  template <typename Function> struct env_status_s<this_t> set_reader_(Function function) {
    struct env_status_s<this_t> temp(this);
    reader_.assign(functor_t<data_t &(data_t &), this_t>(function));
    temp.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
    return temp;
  }

  struct env_status_s<this_t>
  reset_reader_() {
    struct env_status_s<this_t> temp(this);
    reader_.assign(functor_t<data_t &(data_t &), this_t>([](data_t &lvr_data) -> data_t & { return lvr_data; }));
    temp.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
    return temp;
  }

  struct env_data_s<data_t, this_t>
  dequeue_data_() {
    struct env_data_s<data_t, this_t> temp(this);

    if (buffer_.empty()) {

      temp.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_EMPTY));
      this->error_handler()(this->get_id(), temp.status.qualifiers.at(this->get_id()),
                            error_case_e::ERROR_CASE_RUNTIME);
      return temp;

    } else {

      temp.status.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      temp.data = reader_(buffer_.front());
      buffer_.erase(buffer_.begin());
      return temp;
    }
  }

  struct env_status_s<this_t>
  enqueue_data_(const data_t &data) {
    struct env_status_s<this_t> temp(this);

    if (buffer_.size() < queue_size_) {

      buffer_.push_back(data);
      temp.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_CLEAR));
      return temp;

    } else {

      temp.qualifiers.insert(std::make_pair(this->get_id(), env_errno_e::ENV_BUFFER_OVERFLOW));
      this->error_handler()(this->get_id(), temp.qualifiers.at(this->get_id()), error_case_e::ERROR_CASE_RUNTIME);
      return temp;
    }
  }

  public : const uint64_t &
           buffer_size() const {
    return buffer_size_();
  }
  const uint64_t &read_count() const { return read_count_(); }
  template <typename Function> struct env_status_s<this_t> set_reader(Function function) {
    return set_reader_(function);
  }

  struct env_status_s<this_t>
  reset_reader() {
    return reset_reader_();
  } struct env_data_s<data_t, this_t> dequeue_data() {
    return dequeue_data_();
  } struct env_status_s<this_t> enqueue_data(const data_t &data) {
    return enqueue_data_(data);
  } const hook_t<void(const data_t &)> &on_receive() const {
    return get_on_receive_();
  }
  const functor_t<data_t &(data_t &), this_t> &reader() const { return reader_(); }

  explicit base_receiver_port_t(const std::string &name)
      : base_s(name), queue_size_(this->BUFFER_SIZE),
        reader_(functor_t([](data_t &lvr_data) -> data_t & { return lvr_data; })){};
};

#endif /* BASE_RECEIVER_PORT_HPP */
