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
  std::atomic<state_t> &state() const { return state_; }

  void stop_threads() const { return static_cast<const base_t *>(this)->stop_tp(); }
  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connect(const std::string &addr,
                                                                                   uint16_t port) {
    return setup_<cb>(port, addr);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type disconnect() {
    clear_();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type disconnect(const std::string &addr,
                                                                                      uint16_t srv) {
    return disconnect_peer_(addr, srv);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type setup(uint16_t port) {
    return setup_(port);
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
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type is_peer_connected(const std::string &addr,
                                                                                             uint16_t srv) const {
    int32_t rc = get_connected_peer_(addr, srv, nullptr);
    return rc > 0;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type is_peer_connected(int32_t fd) const {
    struct sockaddr_un *addr;
    int32_t rc = get_connected_peer_(fd, &addr);
    return addr != nullptr;
  }

  void reset() {
    clear_();
    clear_hooks_();
  }

  virtual ~domain_tcp_socket_impl() {
    reset();
    clear_epoll_();
  };

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
  mutable property_t<state_t> state_;

  /* Hooks interface */
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_t *)> on_receive_;
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_t *)> on_send_;
  const hook_t<void(struct sockaddr_un, const this_t *)> on_connect_;
  const hook_t<void(struct sockaddr_un, const this_t *)> on_disconnect_;

  /* Internal hooks, serve to customizing of behavior */
  std::function<void(int32_t)> on_connect_internal_hook_;
  std::function<void(int32_t)> on_disconnect_internal_hook_;

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send_(int32_t peer_fd, const struct sockaddr_un *const peer_addr, const void *const msg, size_t size,
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
            this->tp().push([this, peer_addr = *peer_addr]() -> void { this->on_disconnect()(peer_addr, this); });
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
        this->tp().push([this, peer_addr = *peer_addr, data, size]() -> void {
          this->on_send()(peer_addr, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
        });
      }
    }

    if constexpr (sb == send_behavior_t::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

      return {*peer_addr, rc};
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
            this->tp().push([this, connected = connected_]() -> void { this->on_disconnect()(connected, this); });
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
        this->tp().push([this, connected = connected_, data, size]() -> void {
          this->on_send()(connected, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
        });
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
                this->tp().push([this, connected = connected_]() -> void { this->on_disconnect()(connected, this); });
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
            this->tp().push([this, connected = connected_, data, size = recvd]() -> void {
              this->on_receive()(connected,
                                 std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                 this);
            });

            connected_info_lock_.unlock();
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            ret.push_back(std::make_tuple(
                recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), connected_));

            if constexpr (rb == recv_behavior_t::HOOK_RET) {
              connected_info_lock_.lock();
              this->tp().push([this, connected = connected_, size = recvd, data]() -> void {
                this->on_receive()(connected,
                                   std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                   this);
              });

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
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type bind_(const char *path) {
    int32_t rc;
    struct sockaddr_un addr;
    std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1u);

    if ((rc = ::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr))) != 0) {
      clear_();
      throw std::runtime_error(fmt::format("Could not bind TCP socket (errno = {0}), ({1}), {2}:{3}", strerror(rc),
                                           __func__, __FILE__, __LINE__));
    }

    return rc;
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connect_(const char *path) {
    struct epoll_event event;
    struct sockaddr_un addr;
    std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1u);

    state_ = state_t::CONNECTING;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents | EPOLLOUT;
    event.data.fd = sock_fd_;

    if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    int32_t rc;
    if ((rc = ::connect(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr))) < 0) {
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
        std::memcpy(&server, &addr, sizeof(addr));

        std::memset(&event, 0x0, sizeof(event));
        event.events = epollevents;
        event.data.fd = sock_fd_;

        if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
          throw std::runtime_error(fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno),
                                               __func__, __FILE__, __LINE__));

        if (epoll_error) {
          this->tp().push([this, server]() -> void { this->on_disconnect()(server, this); });
          disconnect();
          state_ = state_t::DISCONNECTED;
        } else {

          connected_info_lock_.lock();
          std::memcpy(&connected_, &server, sizeof(server));
          connected_info_lock_.unlock();

          if constexpr (cb == connect_behavior_t::HOOK_ON) {
            this->tp().push([this, server]() -> void { this->on_connect()(server, this); });
          }

          state_ = state_t::CONNECTED;
        }
      }
    }

    return state_ == state_t::CONNECTED ? 0 : -1;
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON> int32_t setup_(const std::string &path = "") {
    int32_t rc, trueflag = 1, try_count = 0;

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

    if constexpr (socket_class == tcp_sock_t::CLIENT_UNICAST) {
      rc = connect_<cb>(path);
    } else if constexpr (socket_class == tcp_sock_t::SERVER_UNICAST) {
      rc = bind_(path);
    }

    return rc;
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
      this->tp().push([this, connected = it->second]() -> void { this->on_disconnect()(connected, this); });
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

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(const std::string &addr, uint16_t port, sockaddr_un **peer_addr) const {
    sockaddr_un net_addr;
    int32_t rc;
    void *dst_addr, *dst_port;

    if ((rc = ::inet_pton(family, addr.c_str(), dst_addr)) <= 0) {
      return rc;
    }

    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if (!std::memcmp(&net_addr.sun_path, &it->second.sun_path, sizeof(it->second.sun_path))) {

        if (peer_addr)
          *peer_addr = &it->second;

        connected_info_lock_.unlock();
        return it->first;
      }
    }

    connected_info_lock_.unlock();
    return -1;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(int32_t fd, sockaddr_un **peer_addr) const {
    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if (fd == it->first) {
        if (peer_addr)
          *peer_addr = &it->second;

        connected_info_lock_.unlock();
        return;
      }
    }

    if (peer_addr)
      *peer_addr = nullptr;

    connected_info_lock_.unlock();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type disconnect_peer_(const std::string &addr,
                                                                                            uint16_t srv) const {
    int16_t peer_fd;

    if ((peer_fd = get_connected_peer_(addr, srv, nullptr)) < 0)
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
};

template <tcp_sock_t sc> struct domain_tcp_socket : domain_tcp_socket_impl<AF_UNIX, sc> {
  using domain_tcp_socket_impl<AF_UNIX, sc>::domain_tcp_socket_impl;
};

#endif /* DOMAIN_TCP_SOCK_HPP */
