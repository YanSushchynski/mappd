#ifndef DOMAIN_TCP_SOCK_HPP
#define DOMAIN_TCP_SOCK_HPP

#include "base_socket.hpp"
#include "tcp_sock_type.hpp"

#include <errno.h>
#include <thread>

template <uint32_t family, tcp_sock_t socket_class>
struct domain_tcp_socket_impl : public base_socket<family, SOCK_STREAM, IPPROTO_TCP> {
public:
  static constexpr int32_t socktype = SOCK_STREAM;
  static constexpr int32_t protocol = IPPROTO_TCP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_t : int32_t { CONNECTED, DISCONNECTED, LISTENING, CONNECTING, STOPPED };
  enum struct recv_behavior_t : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };
  enum struct connect_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };

  using this_t = domain_tcp_socket_impl<family, socket_class>;
  using base_t = base_socket<family, socktype, protocol>;

  using connected_peer_info_t = std::conditional_t<
      socket_class == tcp_sock_t::CLIENT_UNICAST, struct sockaddr_un,
      std::conditional_t<socket_class == tcp_sock_t::SERVER_UNICAST, std::map<int32_t, struct sockaddr_un>, void *>>;

  template <tcp_sock_t sc = socket_class>
  explicit domain_tcp_socket_impl(
      typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, tcp_sock_t>::type * = nullptr)
      : base_t(), state_(state_t::DISCONNECTED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_t::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  template <tcp_sock_t sc = socket_class>
  explicit domain_tcp_socket_impl(
      const std::string &path, typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, tcp_sock_t>::type * = nullptr)
      : base_t(path), state_(state_t::STOPPED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_t::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  const auto &on_connect() const { return on_connect_; }
  const auto &on_disconnect() const { return on_disconnect_; }
  const auto &on_receive() const { return on_receive_; }
  const auto &on_send() const { return on_send_; }

  void stop_threads() const { return this->base_t::stop_threads(); }
  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connect(const std::string &path) {
    return setup_<cb>(path);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type disconnect() {
    clear_();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type disconnect(const std::string &path) {
    return disconnect_peer_(path);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type setup() {
    return setup_();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type listening() const {
    return state_ == state_t::LISTENING;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type connecting() const {
    return state_ == state_t::CONNECTING;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connected() const {
    return state_ == state_t::CONNECTED;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  is_peer_connected(const std::string &path) const {
    int32_t rc = get_connected_peer_(path, nullptr);
    return rc > 0;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type is_peer_connected(int32_t fd) const {
    struct sockaddr_un *path;
    int32_t rc = get_connected_peer_(fd, &path);
    return path != nullptr;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash) const {
    for (const auto &peer : connected_) {
      const sockaddr_un &peer_path = peer.second;
      sha256::sha256_hash_type peer_addr_hash =
          sha256::compute(reinterpret_cast<const uint8_t *>(peer_path.sun_path), sizeof(peer_path.sun_path));

      if (hash == peer_addr_hash)
        return true;
    }

    return false;
  }

  void reset() {
    clear_();
    clear_hooks_();
  }

  virtual ~domain_tcp_socket_impl() {
    reset();
    clear_epoll_();
  };

  void set_blocking__(int32_t fd) const { set_blocking_(fd); }
  void set_non_blocking__(int32_t fd) const { set_non_blocking_(fd); }

  void set_on_connect_internal_hook__(const std::function<void(int32_t)> &new_hook) {
    on_connect_internal_hook_ = new_hook;
  }

  void reset_on_connect_internal_hook__() {
    on_connect_internal_hook_ = [](int32_t) {};
  }

  void set_on_disconnect_internal_hook__(const std::function<void(int32_t)> &new_hook) {
    on_disconnect_internal_hook_ = new_hook;
  }

  void reset_on_disconnect_internal_hook__() {
    on_disconnect_internal_hook_ = [](int32_t) {};
  }

protected:
  const connected_peer_info_t &connected__() const { return connected_; }
  const int32_t &fd__() const { return sock_fd_; }
  const int32_t &epfd__() const { return epfd_; };
  struct epoll_event *events__() const {
    return events_;
  }

  std::atomic_bool &listen_enabled__() const { return listen_enabled_; }
  std::thread &listen_thread__() const { return listen_thread_; }
  std::atomic<state_t> &state__() const { return state_; }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  handle_incoming_data__(int32_t fd, RecvFunction recv_function = ::recv) {
    return this->template handle_incoming_data_<rb, RecvFunction>(fd, recv_function);
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_t::HOOK_ON, int32_t, std::pair<struct sockaddr_un, int32_t>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type handle_incoming_peer__() const {
    return this->template handle_incoming_peer_<cb>();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer__(const std::string &addr, uint16_t port, struct sockaddr_un **peer_addr) const {
    return get_connected_peer_(addr, port, peer_addr);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer__(int32_t fd, struct sockaddr_un **peer_addr) const {
    return get_connected_peer_(fd, peer_addr);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type disconnect_peer__(int32_t fd) const {
    return disconnect_peer_(fd);
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send__(int32_t peer_fd, const struct sockaddr_un *const peer_addr, const void *const msg, size_t size,
         SendFunction send_function = ::send) const {
    return send_<sb, SendFunction>(peer_fd, peer_addr, msg, size, send_function);
  }

private:
  int32_t sock_fd_;
  mutable std::recursive_mutex sock_fd_lock_;

  int32_t epfd_;
  mutable std::recursive_mutex epoll_fd_lock_;

  struct epoll_event *events_;
  mutable connected_peer_info_t connected_;
  mutable std::recursive_mutex connected_info_lock_;

  mutable std::thread listen_thread_;
  mutable std::recursive_mutex listen_thread_lock_;

  mutable std::atomic_bool listen_enabled_;
  mutable std::atomic<state_t> state_;

  /* Hooks interface */
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_t *)> on_receive_;
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_t *)> on_send_;
  const hook_t<void(struct sockaddr_un, const this_t *)> on_connect_;
  const hook_t<void(struct sockaddr_un, const this_t *)> on_disconnect_;

  /* Internal hooks, serve to customizing of behavior */
  std::function<void(int32_t)> on_connect_internal_hook_;
  std::function<void(int32_t)> on_disconnect_internal_hook_;

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type set_non_blocking_(int32_t fd) const {
    int32_t flags;
    if ((flags = ::fcntl(fd, F_GETFL, 0)) < 0)
      throw std::runtime_error(fmt::format("Set non-blocking error (errno = {0}), ({1}), {2}:{3}\r\n", strerror(errno),
                                           __func__, __FILE__, __LINE__));
    if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
      throw std::runtime_error(fmt::format("Set non-blocking error (errno = {0}), ({1}), {2}:{3}\r\n", strerror(errno),
                                           __func__, __FILE__, __LINE__));
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type set_blocking_(int32_t fd) const {
    int32_t flags;
    if ((flags = ::fcntl(fd, F_GETFL, 0)) < 0)
      throw std::runtime_error(fmt::format("Set non-blocking error (errno = {0}), ({1}), {2}:{3}\r\n", strerror(errno),
                                           __func__, __FILE__, __LINE__));
    if (::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0)
      throw std::runtime_error(fmt::format("Set non-blocking error (errno = {0}), ({1}), {2}:{3}\r\n", strerror(errno),
                                           __func__, __FILE__, __LINE__));
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send_(int32_t peer_fd, const struct sockaddr_un *const peer_path, const void *const msg, size_t size,
        SendFunction send_function = ::send) const {
    int32_t rc;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_t::send_timeout() / 1000, 0u};

  send:
    if ((rc = send_function(peer_fd, msg, size, MSG_NOSIGNAL)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
        disconnect:
          connected_info_lock_.lock();
          auto it = connected_.find(peer_fd);
          if (it != connected_.end()) {
            std::thread([this, peer_path = *peer_path]() -> void {
              std::unique_lock<std::mutex> lock(this->mtx());
              this->on_disconnect()(peer_path, this);
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }).detach();
            static_cast<void>(disconnect_peer_(peer_fd));
          }

          connected_info_lock_.unlock();
        } else {
          throw std::runtime_error(
              fmt::format("Sendto error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
        }
      } else {
        FD_ZERO(&write_fd_set);
        FD_SET(sock_fd_, &write_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
          goto disconnect;
        } else
          goto send;
      }
    } else if (!rc) {
      goto disconnect;
    } else {

      if constexpr (sb == send_behavior_t::HOOK_ON) {
        void *data = std::malloc(size * sizeof(char));
        std::memcpy(data, msg, size);
        std::thread([this, peer_path = *peer_path, data, size]() -> void {
          std::unique_lock<std::mutex> lock(this->mtx());
          this->on_send()(peer_path, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }).detach();
      }
    }

    if constexpr (sb == send_behavior_t::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

      return {*peer_path, rc};
    }
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<socket_class == tcp_sock_t::CLIENT_UNICAST, RetType>::type
  send_(const void *const msg, size_t size, SendFunction send_function = ::send) const {
    int32_t rc;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_t::send_timeout() / 1000, 0u};

  send:
    if ((rc = send_function(sock_fd_, msg, size, MSG_NOSIGNAL)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
        disconnect:
          if (state_ == state_t::CONNECTED) {

            connected_info_lock_.lock();
            std::thread([this, connected = connected_]() -> void {
              std::unique_lock<std::mutex> lock(this->mtx());
              this->on_disconnect()(connected, this);
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }).detach();
            std::memset(&connected_, 0x0, sizeof(connected_));
            state_ = state_t::DISCONNECTED;
            connected_info_lock_.unlock();
          }
        } else {
          throw std::runtime_error(
              fmt::format("Sendto error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
        }
      } else {
        FD_ZERO(&write_fd_set);
        FD_SET(sock_fd_, &write_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
          goto disconnect;
        } else
          goto send;
      }
    } else if (!rc) {
      goto disconnect;
    } else {
      if constexpr (sb == send_behavior_t::HOOK_ON) {
        void *data = std::malloc(size * sizeof(char));
        std::memcpy(data, msg, size);
        connected_info_lock_.lock();
        std::thread([this, connected = connected_, data, size]() -> void {
          std::unique_lock<std::mutex> lock(this->mtx());
          this->on_send()(connected, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }).detach();
        connected_info_lock_.unlock();
      }
    }

    if constexpr (sb == send_behavior_t::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

      return {connected_, rc};
    }
  }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>>, void>>>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type
  recv_(RecvFunction recv_function = ::recv) const {
    int32_t num_ready;
    int32_t recvd_size = 0;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_t::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>> ret;

    if ((num_ready = ::epoll_wait(epfd_, events_, base_t::epoll_max_events(), base_t::receive_timeout())) < 0)
      throw std::runtime_error(
          fmt::format("Epoll wait error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    for (uint32_t i = 0u; i < num_ready; i++) {
      if ((events_[i].data.fd == sock_fd_) && ((events_[i].events & EPOLLIN) == EPOLLIN)) {
        int32_t rc, bytes_pending, recvd;

        if ((rc = ::ioctl(sock_fd_, FIONREAD, &bytes_pending)) < 0) {
          throw std::runtime_error(
              fmt::format("IOctl error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
        }

        void *data = std::malloc(bytes_pending);
      recv:
        if ((recvd = recv_function(sock_fd_, data, bytes_pending, 0u)) < 0) {
          if (errno != EAGAIN) {

            std::free(data);
            if ((errno == ECONNRESET) || (errno == ENOTCONN)) {

            disconnect:
              if (state_ == state_t::CONNECTED) {
                connected_info_lock_.lock();
                std::thread([this, connected = connected_]() -> void {
                  std::unique_lock<std::mutex> lock(this->mtx());
                  this->on_disconnect()(connected, this);
                  std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                }).detach();
                std::memset(&connected_, 0x0, sizeof(connected_));
                state_ = state_t::DISCONNECTED;
                connected_info_lock_.unlock();
              }

              if constexpr (rb == recv_behavior_t::HOOK) {

                return recvd_size;
              } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

                return std::move(ret);
              }
            } else {
              throw std::runtime_error(fmt::format("Receiveing error (errno = {0}), ({1}), {2}:{3}", strerror(errno),
                                                   __func__, __FILE__, __LINE__));
            }
          } else {
            FD_ZERO(&read_fd_set);
            FD_SET(sock_fd_, &read_fd_set);

            if ((rc = ::select(sock_fd_ + 1, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
              goto disconnect;
            } else
              goto recv;
          }
        } else {
          if constexpr (rb == recv_behavior_t::HOOK) {

            connected_info_lock_.lock();
            std::thread([this, connected = connected_, data, size = recvd]() -> void {
              std::unique_lock<std::mutex> lock(this->mtx());
              this->on_receive()(connected,
                                 std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                 this);
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }).detach();

            connected_info_lock_.unlock();
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            ret.push_back(std::make_tuple(
                recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), connected_));

            if constexpr (rb == recv_behavior_t::HOOK_RET) {
              connected_info_lock_.lock();
              std::thread([this, connected = connected_, size = recvd, data]() -> void {
                std::unique_lock<std::mutex> lock(this->mtx());
                this->on_receive()(connected,
                                   std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                   this);
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }).detach();

              connected_info_lock_.unlock();
            }
          }

          recvd_size += recvd;
        }
      }
    }

    if constexpr (rb == recv_behavior_t::HOOK) {

      return recvd_size;
    } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

      return std::move(ret);
    }
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type bind_(const struct sockaddr_un *sock_path) {
    int32_t rc;

    if ((rc = ::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&sock_path), sizeof(sock_path))) != 0) {
      clear_();
      throw std::runtime_error(fmt::format("Could not bind TCP socket (errno = {0}), ({1}), {2}:{3}", strerror(rc),
                                           __func__, __FILE__, __LINE__));
    }

    return rc;
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connect_(const std::string &path) {
    struct epoll_event event;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, path.c_str(), sizeof(sock_path.sun_path) - 1u);

    state_ = state_t::CONNECTING;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents | EPOLLOUT;
    event.data.fd = sock_fd_;

    if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    int32_t rc;
    if ((rc = ::connect(sock_fd_, reinterpret_cast<struct sockaddr *>(&sock_path), sizeof(sock_path))) < 0) {
      if (errno != EINPROGRESS) {
        throw std::runtime_error(fmt::format("Connection error (errno = {0}), ({1}), {2}:{3}", strerror(errno),
                                             __func__, __FILE__, __LINE__));
      }
    }

    int32_t num_ready;
    if ((num_ready = ::epoll_wait(epfd_, events_, base_t::epoll_max_events(), base_t::connect_timeout())) < 0) {
      throw std::runtime_error(
          fmt::format("Epoll wait error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    for (int32_t i = 0; i < num_ready; i++) {
      if (events_[i].data.fd == sock_fd_) {
        int32_t epoll_error = 0u, rc;
        socklen_t epoll_errlen = sizeof(epoll_error);
        if ((rc = ::getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&epoll_error), &epoll_errlen)) <
            0) {
          throw std::runtime_error(fmt::format("getsockopt() error (errno = {0}) ({1}), {2}:{3}", strerror(errno),
                                               __func__, __FILE__, __LINE__));
        }

        struct sockaddr_un server;
        std::memcpy(&server, &sock_path, sizeof(sock_path));

        std::memset(&event, 0x0, sizeof(event));
        event.events = epollevents;
        event.data.fd = sock_fd_;

        if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
          throw std::runtime_error(fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno),
                                               __func__, __FILE__, __LINE__));

        if (epoll_error) {
          std::thread([this, server]() -> void {
            std::unique_lock<std::mutex> lock(this->mtx());
            this->on_disconnect()(server, this);
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }).detach();
          disconnect();
          state_ = state_t::DISCONNECTED;
        } else {

          connected_info_lock_.lock();
          std::memcpy(&connected_, &server, sizeof(server));
          connected_info_lock_.unlock();

          if constexpr (cb == connect_behavior_t::HOOK_ON) {
            std::thread([this, server]() -> void {
              std::unique_lock<std::mutex> lock(this->mtx());
              this->on_connect()(server, this);
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }).detach();
          }

          state_ = state_t::CONNECTED;
        }
      }
    }

    return state_ == state_t::CONNECTED ? 0 : -1;
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type setup_(const std::string &path) {
    int32_t rc, trueflag = 1, try_count = 0;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, path.c_str(), path.length());

    open_();
    struct timeval recv_timeout = {base_t::receive_timeout(), 0u};
    struct timeval send_timeout = {base_t::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      perror("setsockopt() error\r\n");
      std::exit(1);
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      perror("setsockopt() error\r\n");
      std::exit(1);
    }

    return connect_<cb>(path);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type setup_() {
    int32_t rc, trueflag = 1, try_count = 0;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, this->path().c_str(), this->path().length());

    open_();
    struct timeval recv_timeout = {base_t::receive_timeout(), 0u};
    struct timeval send_timeout = {base_t::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      perror("setsockopt() error\r\n");
      std::exit(1);
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      perror("setsockopt() error\r\n");
      std::exit(1);
    }

    return bind_(&sock_path);
  }

  void open_() {
    if ((sock_fd_ = ::socket(family, socktype | SOCK_CLOEXEC | SOCK_NONBLOCK, protocol)) < 0) {
      clear_();
      throw std::runtime_error(fmt::format("Could not create socket, ({0}), {1}:{2}", __func__, __FILE__, __LINE__));
    }

    struct epoll_event event;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents;
    event.data.fd = sock_fd_;

    if (::epoll_ctl(epfd_, EPOLL_CTL_ADD, sock_fd_, &event) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
  }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  handle_incoming_data_(int32_t fd, RecvFunction recv_function = ::recv) {
    int32_t recvd, rc, bytes_pending;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_t::receive_timeout() / 1000, 0u};

    if ((rc = ::ioctl(fd, FIONREAD, &bytes_pending)) < 0) {
      throw std::runtime_error(
          fmt::format("IOctl error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    void *data = std::malloc(bytes_pending);
  recv:
    if ((recvd = recv_function(fd, data, bytes_pending, 0u)) < 0) {
      if (errno != EAGAIN) {

        std::free(data);
        if ((errno == ECONNRESET) || (errno == ENOTCONN)) {

        disconnect:
          connected_info_lock_.lock();
          for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
            if (fd == it->first) {
              std::thread([this, connected = it->second]() -> void {
                std::unique_lock<std::mutex> lock(this->mtx());
                this->on_disconnect()(connected, this);
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }).detach();
              static_cast<void>(disconnect_peer_(fd));
              connected_info_lock_.unlock();

              if constexpr (rb == recv_behavior_t::HOOK) {

                return recvd;
              } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

                return {recvd, std::shared_ptr<void>(), sockaddr_un()};
              }
            }
          }

          connected_info_lock_.unlock();
          if constexpr (rb == recv_behavior_t::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_un()};
          }
        } else {
          throw std::runtime_error(fmt::format("Receiveing error (errno = {0}), ({1}), {2}:{3}", strerror(errno),
                                               __func__, __FILE__, __LINE__));
        }
      } else {
        FD_ZERO(&read_fd_set);
        FD_SET(sock_fd_, &read_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
          goto disconnect;
        } else
          goto recv;
      }
    } else if (!recvd) {

      connected_info_lock_.lock();
      for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
        if (fd == it->first) {
          std::thread([this, connected = it->second]() -> void {
            std::unique_lock<std::mutex> lock(this->mtx());
            this->on_disconnect()(connected, this);
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }).detach();
          static_cast<void>(disconnect_peer_(fd));
          connected_info_lock_.unlock();

          if constexpr (rb == recv_behavior_t::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_un()};
          }
        }
      }

      connected_info_lock_.unlock();
      std::free(data);
      if constexpr (rb == recv_behavior_t::HOOK) {

        return recvd;
      } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

        return {recvd, std::shared_ptr<void>(), sockaddr_un()};
      }
    } else {
      struct sockaddr_un peer;
      socklen_t peerlen = sizeof(peer);

      if ((rc = ::getpeername(fd, reinterpret_cast<struct sockaddr *>(&peer), &peerlen)) < 0) {
        std::free(data);
        throw std::runtime_error(fmt::format("Get peer name failed (errno = {0}), ({1}), {2}:{3}", strerror(errno),
                                             __func__, __FILE__, __LINE__));
      }

      if constexpr (rb == recv_behavior_t::HOOK) {
        std::thread([this, peer, data, size = recvd]() -> void {
          std::unique_lock<std::mutex> lock(this->mtx());
          this->on_receive()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                             this);
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }).detach();
        return recvd;
      } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

        if constexpr (rb == recv_behavior_t::HOOK_RET) {
          void *data_copy = std::malloc(recvd);
          std::memcpy(data_copy, data, recvd);
          std::thread([this, peer, data_copy, size = recvd]() -> void {
            std::unique_lock<std::mutex> lock(this->mtx());
            this->on_receive()(
                peer, std::shared_ptr<void>(data_copy, [](const auto &data) -> void { std::free(data); }), size, this);
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }).detach();
        }

        return {recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), std::move(peer)};
      }
    }
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_t::HOOK_ON, int32_t, std::pair<struct sockaddr_un, int32_t>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type handle_incoming_peer_() const {
    struct sockaddr_un peer_addr;
    socklen_t peer_addr_size = sizeof(peer_addr);
    uint16_t peer_srv;
    struct epoll_event peer_ev;
    int32_t peer_fd;
    int32_t rc;
    fd_set accept_fd_set;
    struct timeval accept_timeout = {base_t::accept_timeout() / 1000, 0u};

    state_ = state_t::CONNECTING;
  accept:
    if ((peer_fd = ::accept(sock_fd_, reinterpret_cast<struct sockaddr *>(&peer_addr), &peer_addr_size)) < 0) {
      if (errno != EAGAIN) {
        throw std::runtime_error(
            fmt::format("Accept error (errno = {0}), ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
      } else {
        FD_ZERO(&accept_fd_set);
        FD_SET(sock_fd_, &accept_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &accept_fd_set, nullptr, nullptr, &accept_timeout)) <= 0) {
          if constexpr (cb == connect_behavior_t::HOOK_ON) {

            return peer_fd;
          } else if constexpr (cb == connect_behavior_t::HOOK_OFF) {

            return {sockaddr_un(), peer_fd};
          }
        } else
          goto accept;
      }
    }

    set_non_blocking_(peer_fd);
    std::memset(&peer_ev, 0x0, sizeof(peer_ev));
    peer_ev.data.fd = peer_fd;
    peer_ev.events = EPOLLIN | EPOLLET;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_ADD, peer_fd, &peer_ev)) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    connected_info_lock_.lock();
    connected_.insert(std::make_pair(peer_fd, peer_addr));
    connected_info_lock_.unlock();

    if constexpr (cb == connect_behavior_t::HOOK_ON) {
      std::thread([this, peer_addr]() -> void {
        std::unique_lock<std::mutex> lock(this->mtx());
        this->on_connect()(peer_addr, this);
        std::notify_all_at_thread_exit(this->cv(), std::move(lock));
      }).detach();
      return peer_fd;
    } else if constexpr (cb == connect_behavior_t::HOOK_OFF) {

      return {std::move(peer_addr), peer_fd};
    }
  }

  void clear_epoll_() {
    epoll_fd_lock_.lock();
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
    epoll_fd_lock_.unlock();
  }

  void clear_hooks_() {
    this->on_connect().clear();
    this->on_disconnect().clear();
    this->on_receive().clear();
    this->on_send().clear();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type stop() {
    listen_enabled_ = false;
    if (listen_thread_.joinable())
      listen_thread_.join();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type clear_() {
    stop();
    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      std::thread([this, connected = it->second]() -> void {
        std::unique_lock<std::mutex> lock(this->mtx());
        this->on_disconnect()(connected, this);
        std::notify_all_at_thread_exit(this->cv(), std::move(lock));
      }).detach();
      if (::epoll_ctl(epfd_, EPOLL_CTL_DEL, it->first, nullptr) < 0u)
        throw std::runtime_error(
            fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
      ::close(it->first);
    }

    connected_.clear();
    if (::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    ::close(sock_fd_);
    state_ = state_t::STOPPED;
    connected_info_lock_.unlock();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type clear_() {
    connected_info_lock_.lock();
    if (state_ == state_t::CONNECTED) {
      std::thread([this, connected = connected_]() -> void {
        std::unique_lock<std::mutex> lock(this->mtx());
        this->on_disconnect()(connected, this);
        std::notify_all_at_thread_exit(this->cv(), std::move(lock));
      }).detach();
      std::memset(&connected_, 0x0, sizeof(connected_));
    }

    if (::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    on_disconnect_internal_hook_(sock_fd_);
    ::close(sock_fd_);
    state_ = state_t::DISCONNECTED;
    connected_info_lock_.unlock();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(const std::string &path, sockaddr_un **peer_path) const {
    struct sockaddr_un sock_path;
    int32_t rc;
    void *dst_path;

    if ((rc = ::inet_pton(family, path.c_str(), dst_path)) <= 0) {
      return rc;
    }

    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if (!std::memcmp(&sock_path.sun_path, &it->second.sun_path, sizeof(it->second.sun_path))) {

        if (peer_path)
          *peer_path = &it->second;

        connected_info_lock_.unlock();
        return it->first;
      }
    }

    connected_info_lock_.unlock();
    return -1;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(int32_t fd, struct sockaddr_un **peer_path) const {
    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if (fd == it->first) {
        if (peer_path)
          *peer_path = &it->second;

        connected_info_lock_.unlock();
        return;
      }
    }

    if (peer_path)
      *peer_path = nullptr;

    connected_info_lock_.unlock();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  disconnect_peer_(const std::string &path) const {
    int16_t peer_fd;

    if ((peer_fd = get_connected_peer_(path, nullptr)) < 0)
      return peer_fd;
    else {
      ::close(peer_fd);

      connected_info_lock_.lock();
      connected_.erase(peer_fd);
      connected_info_lock_.unlock();

      on_disconnect_internal_hook_(peer_fd);
      return peer_fd;
    }
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type disconnect_peer_(int32_t fd) const {
    if (connected_.find(fd) != connected_.end()) {
      ::close(fd);

      connected_info_lock_.lock();
      connected_.erase(fd);
      connected_info_lock_.unlock();

      on_disconnect_internal_hook_(fd);
      return fd;
    }

    return -1;
  }
};

template <tcp_sock_t sc> struct domain_tcp_socket : domain_tcp_socket_impl<AF_UNIX, sc> {
  using domain_tcp_socket_impl<AF_UNIX, sc>::domain_tcp_socket_impl;
};

#endif /* DOMAIN_TCP_SOCK_HPP */
