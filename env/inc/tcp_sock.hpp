#ifndef TCP_SOCK_HPP
#define TCP_SOCK_HPP

#include "base_socket.hpp"
#include "function_traits.hpp"
#include "tcp_sock_type.hpp"

#include <errno.h>
#include <future>
#include <thread>

/* @brief: This template class is wrapper of Berkley sockets */
template <uint32_t family, tcp_sock_t socket_class>
struct tcp_socket : public base_socket<family, SOCK_STREAM, IPPROTO_TCP> {
public:
  static constexpr int32_t socktype = SOCK_STREAM;
  static constexpr int32_t protocol = IPPROTO_TCP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_t : int32_t { CONNECTED, DISCONNECTED, LISTENING, CONNECTING, STOPPED };
  enum struct recv_behavior_t : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };
  enum struct connect_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };

  using this_t = tcp_socket<family, socket_class>;
  using base_t = base_socket<family, socktype, protocol>;

  static constexpr bool is_ipv6 = base_t::is_ipv6;
  static constexpr int32_t addrlen = base_t::addrlen;

  using sockaddr_inet_t = std::conditional_t<base_t::is_ipv6, struct sockaddr_in6, struct sockaddr_in>;
  using inet_addr_t = std::conditional_t<base_t::is_ipv6, struct in6_addr, struct in_addr>;

  using connected_peer_info_t = std::conditional_t<
      socket_class == tcp_sock_t::CLIENT_UNICAST, sockaddr_inet_t,
      std::conditional_t<socket_class == tcp_sock_t::SERVER_UNICAST, std::map<int32_t, sockaddr_inet_t>, void *>>;

  template <tcp_sock_t sc = socket_class>
  explicit tcp_socket(const std::string &iface,
                      typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, tcp_sock_t>::type * = nullptr)
      : base_t(iface), state_(state_t::DISCONNECTED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_t::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  template <tcp_sock_t sc = socket_class>
  explicit tcp_socket(const std::string &iface,
                      typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, tcp_sock_t>::type * = nullptr)
      : base_t(iface), state_(state_t::STOPPED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_t::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  const auto &on_connect() const { return on_connect_; }
  const auto &on_disconnect() const { return on_disconnect_; }
  const auto &on_receive() const { return on_receive_; }
  const auto &on_send() const { return on_send_; }
  property_t<state_t> &state_prop() const { return state_; }

  void stop_threads() const { return const_cast<const base_t *>(this)->stop_tp(); }
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
    sockaddr_inet_t *addr;
    int32_t rc = get_connected_peer_(fd, &addr);
    return addr != nullptr;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash, uint16_t srv) const {
    for (const auto &peer : connected_) {
      const sockaddr_inet_t &peer_addr = peer.second;
      char addr[base_t::addrlen];
      uint16_t client_srv;

      if constexpr (base_t::is_ipv6) {

        client_srv = ::htons(peer_addr.sin6_port);
        if (!::inet_ntop(family, &peer_addr.sin6_addr, addr, sizeof(addr))) {
          return false;
        }
      } else {

        client_srv = ::htons(peer_addr.sin_port);
        if (!::inet_ntop(family, &peer_addr.sin_addr, addr, sizeof(addr))) {
          return false;
        }
      }

      sha256::sha256_hash_type peer_addr_hash = sha256::compute(reinterpret_cast<const uint8_t *>(addr), sizeof(addr)) ^
                                                sha256::compute(reinterpret_cast<const uint8_t *>(&srv), sizeof(srv));

      if (hash == peer_addr_hash)
        return true;
    }

    return false;
  }

  int32_t peer_fd(const std::string &addr, uint16_t srv) const { return get_connected_peer_(addr, srv, nullptr); }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const {
    sockaddr_inet_t *peer_addr;
    int32_t peer_fd;

    if ((peer_fd = get_connected_peer_(addr, port, &peer_addr)) < 0) {
      if constexpr (sb == send_behavior_t::HOOK_ON) {

        return -1;
      } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

        return {sockaddr_inet_t(), -1};
      }
    }

    return send_<sb>(peer_fd, peer_addr, msg, size);
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type
  send(const void *const msg, size_t size, SendFunction send_function = ::send) const {
    return send_<sb, SendFunction>(msg, size, send_function);
  }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type
  recv(RecvFunction recv_function = ::recv) const {
    return recv_<rb, RecvFunction>(recv_function);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type start(uint64_t duration_ms = 0) const {
    bool nonblock = duration_ms == 0;
    if (nonblock) {
      listen_thread_ = std::thread([this]() -> void { listen_(); });
    } else {
      this->tp().push(
          [this](int32_t thr_id, uint64_t duration_ms, std::atomic_bool *trigger) -> void {
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
            *trigger = false;
          },
          duration_ms, &listen_enabled_);
      listen_();
    }
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type stop() {
    listen_enabled_ = false;
    if (listen_thread_.joinable())
      listen_thread_.join();
  }

  void reset() {
    clear_();
    clear_hooks_();
  }

  virtual ~tcp_socket() {
    reset();
    clear_epoll_();
  };

protected:
  const connected_peer_info_t &connected__() const { return connected_; }
  const int32_t &fd__() const { return sock_fd_; }
  const int32_t &epfd__() const { return epfd_; };
  struct epoll_event *events__() const {
    return events_;
  }

  std::atomic_bool &listen_enabled__() const { return listen_enabled_; }
  std::thread &listen_thread__() const { return listen_thread_; }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  handle_incoming_data__(int32_t fd, RecvFunction recv_function = ::recv) const {
    return this->template handle_incoming_data_<rb, RecvFunction>(fd, recv_function);
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_t::HOOK_ON, int32_t, std::pair<sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type handle_incoming_peer__() const {
    return this->template handle_incoming_peer_<cb>();
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer__(const std::string &addr, uint16_t port, sockaddr_inet_t **peer_addr) const {
    return get_connected_peer_(addr, port, peer_addr);
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer__(int32_t fd, sockaddr_inet_t **peer_addr) const {
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
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send__(int32_t peer_fd, const sockaddr_inet_t *const peer_addr, const void *const msg, size_t size,
         SendFunction send_function = ::send) const {
    return send_<sb, SendFunction>(peer_fd, peer_addr, msg, size, send_function);
  }

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
  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_t *)> on_receive_;
  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_t *)> on_send_;
  const hook_t<void(sockaddr_inet_t, const this_t *)> on_connect_;
  const hook_t<void(sockaddr_inet_t, const this_t *)> on_disconnect_;

  /* Internal hooks, serve to customizing of behavior */
  std::function<void(int32_t)> on_connect_internal_hook_;
  std::function<void(int32_t)> on_disconnect_internal_hook_;

  template <send_behavior_t sb = send_behavior_t::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t), tcp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  send_(int32_t peer_fd, const sockaddr_inet_t *const peer_addr, const void *const msg, size_t size,
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
            this->tp().push(
                [this, peer_addr = *peer_addr](int32_t thr_id) -> void { this->on_disconnect()(peer_addr, this); });
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
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
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
            this->tp().push(
                [this, connected = connected_](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
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
        this->tp().push([this, connected = connected_, data, size](int32_t thr_id) -> void {
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
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type
  recv_(RecvFunction recv_function = ::recv) const {
    int32_t num_ready;
    int32_t recvd_size = 0;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_t::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>> ret;

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
                this->tp().push(
                    [this, connected = connected_](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
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
            this->tp().push([this, connected = connected_, data, size = recvd](int32_t thr_id) -> void {
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
              this->tp().push([this, connected = connected_, size = recvd, data](int32_t thr_id) -> void {
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

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON>
  int32_t setup_(uint16_t port, const std::string &addr = "") {
    struct addrinfo *addr_info, hints;
    int32_t rc, trueflag = 1, try_count = 0;
    const char *addr_str;
    int32_t ret;

    std::memset(&hints, 0x0, sizeof(hints));
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_protocol = protocol;

    if constexpr (socket_class == tcp_sock_t::CLIENT_UNICAST) {

      addr_str = addr.c_str();
    } else if constexpr (socket_class == tcp_sock_t::SERVER_UNICAST) {

      addr_str = this->iface_info().host_addr.data();
    }

    if ((rc = ::getaddrinfo(addr_str, std::to_string(port).c_str(), &hints, &addr_info)) != 0 || addr_info == nullptr) {
      throw std::runtime_error(fmt::format("Invalid address or port: \"{0}\",\"{1}\" (errno : {2}), ({3}), {4}:{5}\"",
                                           addr, std::to_string(port), gai_strerror(rc), __func__, __FILE__, __LINE__));
    }

    if constexpr (base_t::is_ipv6)
      reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_scope_id = this->iface_info().scopeid;

    open_();
    struct timeval recv_timeout = {base_t::receive_timeout(), 0u};
    struct timeval send_timeout = {base_t::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &trueflag, sizeof(trueflag))) < 0) {
      throw std::runtime_error(
          fmt::format("Setsockopt error (errno = {0}),({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEPORT, &trueflag, sizeof(trueflag))) < 0) {
      throw std::runtime_error(
          fmt::format("Setsockopt error (errno = {0}),({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    trueflag = IP_PMTUDISC_DO;
    if ((rc = ::setsockopt(sock_fd_, protocol, IP_MTU_DISCOVER, &trueflag, sizeof(trueflag))) < 0) {
      throw std::runtime_error(
          fmt::format("Setsockopt error (errno = {0}),({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      throw std::runtime_error(
          fmt::format("Setsockopt error (errno = {0}),({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      throw std::runtime_error(
          fmt::format("Setsockopt error (errno = {0}),({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));
    }

    if constexpr (socket_class == tcp_sock_t::CLIENT_UNICAST) {
      ret = connect_<cb>(addr_info);
    } else if constexpr (socket_class == tcp_sock_t::SERVER_UNICAST) {
      ret = bind_(addr_info);
    }

    ::freeaddrinfo(addr_info);
    return ret;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type bind_(struct addrinfo *addr_info) {
    int32_t rc;
    if ((rc = ::bind(sock_fd_, addr_info->ai_addr, addr_info->ai_addrlen)) != 0) {
      clear_();
      throw std::runtime_error(fmt::format("Could not bind TCP socket (errno = {0}), ({1}), {2}:{3}", strerror(rc),
                                           __func__, __FILE__, __LINE__));
    }

    return rc;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type clear_() {
    stop();
    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      this->tp().push(
          [this, connected = it->second](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
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

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type clear_() {
    connected_info_lock_.lock();
    if (state_ == state_t::CONNECTED) {
      this->tp().push(
          [this, connected = connected_](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
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

  void clear_hooks_() {
    this->on_connect().clear();
    this->on_disconnect().clear();
    this->on_receive().clear();
    this->on_send().clear();
  }

  void clear_epoll_() {
    epoll_fd_lock_.lock();
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
    epoll_fd_lock_.unlock();
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
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  handle_incoming_data_(int32_t fd, RecvFunction recv_function = ::recv) const {
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
              this->tp().push(
                  [this, connected = it->second](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
              static_cast<void>(disconnect_peer_(fd));
              connected_info_lock_.unlock();

              if constexpr (rb == recv_behavior_t::HOOK) {

                return recvd;
              } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

                return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
              }
            }
          }

          connected_info_lock_.unlock();
          if constexpr (rb == recv_behavior_t::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
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
          this->tp().push(
              [this, connected = it->second](int32_t thr_id) -> void { this->on_disconnect()(connected, this); });
          static_cast<void>(disconnect_peer_(fd));
          connected_info_lock_.unlock();

          if constexpr (rb == recv_behavior_t::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
          }
        }
      }

      connected_info_lock_.unlock();
      std::free(data);
      if constexpr (rb == recv_behavior_t::HOOK) {

        return recvd;
      } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

        return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
      }
    } else {
      sockaddr_inet_t peer;
      socklen_t peerlen = sizeof(peer);

      if ((rc = ::getpeername(fd, reinterpret_cast<struct sockaddr *>(&peer), &peerlen)) < 0) {
        std::free(data);
        throw std::runtime_error(fmt::format("Get peer name failed (errno = {0}), ({1}), {2}:{3}", strerror(errno),
                                             __func__, __FILE__, __LINE__));
      }

      if constexpr (rb == recv_behavior_t::HOOK) {
        this->tp().push([this, peer, data, size = recvd](int32_t thr_id) -> void {
          this->on_receive()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                             this);
        });
        return recvd;
      } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

        if constexpr (rb == recv_behavior_t::HOOK_RET) {
          void *data_copy = std::malloc(recvd);
          std::memcpy(data_copy, data, recvd);
          this->tp().push([this, peer, data_copy, size = recvd](int32_t thr_id) -> void {
            this->on_receive()(
                peer, std::shared_ptr<void>(data_copy, [](const auto &data) -> void { std::free(data); }), size, this);
          });
        }

        return {recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), std::move(peer)};
      }
    }
  }

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::CLIENT_UNICAST, RetType>::type connect_(const struct addrinfo *addr_info) {
    struct epoll_event event;

    state_ = state_t::CONNECTING;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents | EPOLLOUT;
    event.data.fd = sock_fd_;

    if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
      throw std::runtime_error(
          fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__, __FILE__, __LINE__));

    int32_t rc;
    if ((rc = ::connect(sock_fd_, addr_info->ai_addr, sizeof(sockaddr_inet_t))) < 0) {
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

        sockaddr_inet_t server;
        std::memcpy(&server, reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr), addr_info->ai_addrlen);

        std::memset(&event, 0x0, sizeof(event));
        event.events = epollevents;
        event.data.fd = sock_fd_;

        if (::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event) < 0u)
          throw std::runtime_error(fmt::format("Epoll ctl error (errno = {0}) ({1}), {2}:{3}", strerror(errno),
                                               __func__, __FILE__, __LINE__));

        if (epoll_error) {
          this->tp().push([this, server](int32_t thr_id) -> void { this->on_disconnect()(server, this); });
          disconnect();
          state_ = state_t::DISCONNECTED;
        } else {

          connected_info_lock_.lock();
          std::memcpy(&connected_, &server, sizeof(server));
          connected_info_lock_.unlock();

          if constexpr (cb == connect_behavior_t::HOOK_ON) {
            this->tp().push([this, server](int32_t thr_id) -> void { this->on_connect()(server, this); });
          }

          state_ = state_t::CONNECTED;
        }
      }
    }

    return state_ == state_t::CONNECTED ? 0 : -1;
  }

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

  template <tcp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(const std::string &addr, uint16_t port, sockaddr_inet_t **peer_addr) const {
    sockaddr_inet_t net_addr;
    int32_t rc;
    void *dst_addr, *dst_port;

    if constexpr (base_t::is_ipv6) {
      dst_addr = &net_addr.sin6_addr;
      net_addr.sin6_port = ::ntohs(port);
    } else {
      dst_addr = &net_addr.sin_addr;
      net_addr.sin_port = ::ntohs(port);
    }

    if ((rc = ::inet_pton(family, addr.c_str(), dst_addr)) <= 0) {
      return rc;
    }

    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if constexpr (base_t::is_ipv6) {
        if (!std::memcmp(&net_addr.sin6_addr, &it->second.sin6_addr, sizeof(it->second.sin6_addr)) &&
            net_addr.sin6_port == it->second.sin6_port) {

          if (peer_addr)
            *peer_addr = &it->second;

          connected_info_lock_.unlock();
          return it->first;
        }
      } else {

        if (net_addr.sin_addr.s_addr == it->second.sin_addr.s_addr && net_addr.sin_port == it->second.sin_port) {
          if (peer_addr)
            *peer_addr = &it->second;

          connected_info_lock_.unlock();
          return it->first;
        }
      }
    }

    connected_info_lock_.unlock();
    return -1;
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type
  get_connected_peer_(int32_t fd, sockaddr_inet_t **peer_addr) const {
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

  template <connect_behavior_t cb = connect_behavior_t::HOOK_ON, tcp_sock_t sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_t::HOOK_ON, int32_t, std::pair<sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type handle_incoming_peer_() const {
    sockaddr_inet_t peer_addr;
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

            return {sockaddr_inet_t(), peer_fd};
          }
        } else
          goto accept;
      }
    }

    if constexpr (base_t::is_ipv6) {

      peer_srv = ::htons(peer_addr.sin6_port);
    } else {

      peer_srv = ::htons(peer_addr.sin_port);
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
      this->tp().push([this, peer_addr](int32_t thr_id) -> void { this->on_connect()(peer_addr, this); });
      return peer_fd;
    } else if constexpr (cb == connect_behavior_t::HOOK_OFF) {

      return {std::move(peer_addr), peer_fd};
    }
  }

  template <tcp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_t::SERVER_UNICAST, RetType>::type listen_() const {
    listen_enabled_ = true;
    int32_t rc;
    if ((rc = ::listen(sock_fd_, SOMAXCONN)) < 0) {
      throw std::runtime_error(fmt::format("Start listening error: (errno = {0}), ({1}), {2}:{3}\"", strerror(errno),
                                           __func__, __FILE__, __LINE__));
    }

    state_ = state_t::LISTENING;
    while (listen_enabled_) {

      int32_t num_ready;
      if ((num_ready = ::epoll_wait(epfd_, events_, base_t::epoll_max_events(), base_t::accept_timeout())) < 0u) {
        throw std::runtime_error(fmt::format("Epoll wait error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__,
                                             __FILE__, __LINE__));
      }

      for (int32_t i = 0; i < num_ready; i++) {
        if (((events_[i].events & EPOLLIN) == EPOLLIN || (events_[i].events & EPOLLET) == EPOLLET) &&
            (events_[i].data.fd == sock_fd_)) {

          handle_incoming_peer_();
        } else if (((events_[i].events & EPOLLIN) == EPOLLIN || (events_[i].events & EPOLLET) == EPOLLET) &&
                   (events_[i].data.fd != sock_fd_)) {

          handle_incoming_data_(events_[i].data.fd);
        }
      }
    }

    state_ = state_t::STOPPED;
  }
};

#endif /* TCP_SOCK_HPP */
