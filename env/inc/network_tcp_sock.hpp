#ifndef NETWORK_TCP_SOCK_HPP
#define NETWORK_TCP_SOCK_HPP

#include "base_socket.hpp"
#include "debug.hpp"
#include "tcp_sock_type.hpp"

#include <errno.h>
#include <thread>

template <uint32_t family, tcp_sock_type_e socket_class, bool multithread>
struct network_tcp_socket_impl_s : public base_sock_s<family, SOCK_STREAM, IPPROTO_TCP, multithread> {
public:
  static constexpr int32_t socktype = SOCK_STREAM;
  static constexpr int32_t protocol = IPPROTO_TCP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_e : int32_t { CONNECTED, DISCONNECTED, LISTENING, CONNECTING, STOPPED };
  enum struct recv_behavior_e : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_e : uint32_t { HOOK_ON, HOOK_OFF };
  enum struct connect_behavior_e : uint32_t { HOOK_ON, HOOK_OFF };

  using this_s = network_tcp_socket_impl_s<family, socket_class, multithread>;
  using base_s = base_sock_s<family, socktype, protocol, multithread>;

  static constexpr bool is_ipv6 = base_s::is_ipv6;
  static constexpr int32_t addrlen = base_s::addrlen;

  using sockaddr_inet_t = std::conditional_t<base_s::is_ipv6, struct sockaddr_in6, struct sockaddr_in>;
  using inet_addr_t = std::conditional_t<base_s::is_ipv6, struct in6_addr, struct in_addr>;

  using connected_peer_info_t = std::conditional_t<
      socket_class == tcp_sock_type_e::CLIENT_UNICAST, sockaddr_inet_t,
      std::conditional_t<socket_class == tcp_sock_type_e::SERVER_UNICAST, std::map<int32_t, sockaddr_inet_t>, void *>>;

  template <tcp_sock_type_e sc = socket_class>
  explicit network_tcp_socket_impl_s(
      const std::string &iface,
      typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, tcp_sock_type_e>::type * = nullptr)
      : base_s(iface), state_(state_e::DISCONNECTED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_s::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  template <tcp_sock_type_e sc = socket_class>
  explicit network_tcp_socket_impl_s(
      const std::string &iface,
      typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, tcp_sock_type_e>::type * = nullptr)
      : base_s(iface), state_(state_e::STOPPED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_s::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  const auto &on_connect() const noexcept { return on_connect_; }
  const auto &on_disconnect() const noexcept { return on_disconnect_; }
  const auto &on_receive() const noexcept { return on_receive_; }
  const auto &on_send() const noexcept { return on_send_; }

  void stop_threads() noexcept { return this->base_s::stop_threads(); }
  template <connect_behavior_e cb = connect_behavior_e::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type connect(const std::string &addr,
                                                                                        uint16_t port) noexcept {
    return setup_<cb>(port, addr);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type disconnect() noexcept {
    clear_();
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type disconnect(const std::string &addr,
                                                                                           uint16_t srv) noexcept {
    return disconnect_peer_(addr, srv);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type setup(uint16_t port) noexcept {
    return setup_(port);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type listening() const noexcept {
    return state_ == state_e::LISTENING;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type connecting() const noexcept {
    return state_ == state_e::CONNECTING;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type connected() const noexcept {
    return state_ == state_e::CONNECTED;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  is_peer_connected(const std::string &addr, uint16_t srv) const noexcept {
    int32_t rc = get_connected_peer_(addr, srv, nullptr);
    return rc > 0;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type is_peer_connected(int32_t fd) const
      noexcept {
    sockaddr_inet_t *addr;
    int32_t rc = get_connected_peer_(fd, &addr);
    return addr != nullptr;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash, uint16_t srv) const noexcept {
    for (const auto &peer : connected_) {
      const sockaddr_inet_t &peer_addr = peer.second;
      char addr[base_s::addrlen];
      uint16_t client_srv;

      if constexpr (base_s::is_ipv6) {

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

  int32_t peer_fd(const std::string &addr, uint16_t srv) const noexcept {
    return get_connected_peer_(addr, srv, nullptr);
  }

  template <send_behavior_e sb = send_behavior_e::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const noexcept {
    sockaddr_inet_t *peer_addr;
    int32_t peer_fd;

    if ((peer_fd = get_connected_peer_(addr, port, &peer_addr)) < 0) {
      if constexpr (sb == send_behavior_e::HOOK_ON) {

        return -1;
      } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

        return {sockaddr_inet_t(), -1};
      }
    }

    return send_<sb>(peer_fd, peer_addr, msg, size);
  }

  template <send_behavior_e sb = send_behavior_e::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  send(const void *const msg, size_t size, SendFunction send_function = ::send) const noexcept {
    return send_<sb, SendFunction>(msg, size, send_function);
  }

  template <recv_behavior_e rb = recv_behavior_e::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  recv(RecvFunction recv_function = ::recv) const noexcept {
    return recv_<rb, RecvFunction>(recv_function);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type start(uint64_t duration_ms = 0) const
      noexcept {
    bool nonblock = duration_ms == 0;
    if (nonblock) {
      listen_thread_ = std::thread([this]() -> void { listen_(); });
    } else {
      std::thread(
          [this](uint64_t duration_ms, std::atomic_bool *trigger) -> void {
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
            *trigger = false;
            {
              std::unique_lock<std::mutex> lock(this->mtx());
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }
          },
          duration_ms, &listen_enabled_)
          .detach();
      listen_();
    }
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type stop() noexcept {
    int32_t rc;
    if (listen_enabled_) {
      listen_enabled_ = false;
      rc = 0;
    } else {
      rc = -1;
      goto exit;
    }

    if (listen_thread_.joinable()) {
      listen_thread_.join();
      rc = 0;
    } else {
      rc = -1;
      goto exit;
    }

  exit:
    return rc;
  }

  void reset() noexcept {
    clear_();
    clear_hooks_();
  }

  virtual ~network_tcp_socket_impl_s() noexcept {
    reset();
    clear_epoll_();
  };

protected:
  const connected_peer_info_t &connected__() const noexcept { return connected_; }
  const int32_t &fd__() const noexcept { return sock_fd_; }
  const int32_t &epfd__() const noexcept { return epfd_; };
  struct epoll_event *events__() const noexcept {
    return events_;
  }

  std::atomic_bool &listen_enabled__() const noexcept { return listen_enabled_; }
  std::thread &listen_thread__() const noexcept { return listen_thread_; }
  std::atomic<state_e> &state__() const noexcept { return state_; }

  template <recv_behavior_e rb = recv_behavior_e::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  handle_incoming_data__(int32_t fd, RecvFunction recv_function = ::recv) noexcept {
    return this->template handle_incoming_data_<rb, RecvFunction>(fd, recv_function);
  }

  template <connect_behavior_e cb = connect_behavior_e::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_e::HOOK_ON, int32_t, std::pair<sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type handle_incoming_peer__() const
      noexcept {
    return this->template handle_incoming_peer_<cb>();
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer__(const std::string &addr, uint16_t port, sockaddr_inet_t **peer_addr) const noexcept {
    return get_connected_peer_(addr, port, peer_addr);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer__(int32_t fd, sockaddr_inet_t **peer_addr) const noexcept {
    return get_connected_peer_(fd, peer_addr);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type disconnect_peer__(int32_t fd) const
      noexcept {
    return disconnect_peer_(fd);
  }

  template <send_behavior_e sb = send_behavior_e::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  send__(int32_t peer_fd, const sockaddr_inet_t *const peer_addr, const void *const msg, size_t size,
         SendFunction send_function = ::send) const noexcept {
    return send_<sb, SendFunction>(peer_fd, peer_addr, msg, size, send_function);
  }

  void set_blocking__(int32_t fd) const noexcept { set_blocking_(fd); }
  void set_non_blocking__(int32_t fd) const noexcept { set_non_blocking_(fd); }

  void set_on_connect_internal_hook__(const std::function<void(int32_t)> &new_hook) noexcept {
    on_connect_internal_hook_ = new_hook;
  }

  void reset_on_connect_internal_hook__() noexcept {
    on_connect_internal_hook_ = [](int32_t) {};
  }

  void set_on_disconnect_internal_hook__(const std::function<void(int32_t)> &new_hook) noexcept {
    on_disconnect_internal_hook_ = new_hook;
  }

  void reset_on_disconnect_internal_hook__() noexcept {
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
  mutable std::atomic<state_e> state_;

  /* Hooks interface */
  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_s *)> on_receive_;
  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_s *)> on_send_;
  const hook_t<void(sockaddr_inet_t, const this_s *)> on_connect_;
  const hook_t<void(sockaddr_inet_t, const this_s *)> on_disconnect_;

  /* Internal hooks, serve to customizing of behavior */
  std::function<void(int32_t)> on_connect_internal_hook_;
  std::function<void(int32_t)> on_disconnect_internal_hook_;

  template <send_behavior_e sb = send_behavior_e::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  send_(int32_t peer_fd, const sockaddr_inet_t *const peer_addr, const void *const msg, size_t size,
        SendFunction send_function = ::send) const noexcept {
    int32_t rc;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_s::send_timeout() / 1000, 0u};

  send:
    if ((rc = send_function(peer_fd, msg, size, MSG_NOSIGNAL)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
        disconnect:
          connected_info_lock_.lock();
          auto it = connected_.find(peer_fd);
          if (it != connected_.end()) {
            std::thread([this, peer_addr = *peer_addr]() -> void {
              this->on_disconnect()(peer_addr, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
            static_cast<void>(disconnect_peer_(peer_fd));
          }

          connected_info_lock_.unlock();
        } else {
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return RetType({sockaddr_inet_t(), rc});
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

      if constexpr (sb == send_behavior_e::HOOK_ON) {
        void *data = std::malloc(size * sizeof(char));
        std::memcpy(data, msg, size);
        std::thread([this, peer_addr = *peer_addr, data, size]() -> void {
          this->on_send()(peer_addr, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
      }
    }

    if constexpr (sb == send_behavior_e::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

      return {*peer_addr, rc};
    }
  }

  template <send_behavior_e sb = send_behavior_e::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<socket_class == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  send_(const void *const msg, size_t size, SendFunction send_function = ::send) const noexcept {
    int32_t rc;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_s::send_timeout() / 1000, 0u};

  send:
    if ((rc = send_function(sock_fd_, msg, size, MSG_NOSIGNAL)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
        disconnect:
          if (state_ == state_e::CONNECTED) {

            connected_info_lock_.lock();
            std::thread([this, connected = connected_]() -> void {
              this->on_disconnect()(connected, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
            std::memset(&connected_, 0x0, sizeof(connected_));
            state_ = state_e::DISCONNECTED;
            connected_info_lock_.unlock();
          }
        } else {
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return RetType({sockaddr_inet_t(), rc});
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
      if constexpr (sb == send_behavior_e::HOOK_ON) {
        void *data = std::malloc(size * sizeof(char));
        std::memcpy(data, msg, size);
        connected_info_lock_.lock();
        std::thread([this, connected = connected_, data, size]() -> void {
          this->on_send()(connected, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
        connected_info_lock_.unlock();
      }
    }

    if constexpr (sb == send_behavior_e::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

      return {connected_, rc};
    }
  }

  template <recv_behavior_e rb = recv_behavior_e::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  recv_(RecvFunction recv_function = ::recv) const noexcept {
    int32_t rc, num_ready, recvd_size = 0;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_s::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>> ret;

    if ((rc = ::epoll_wait(epfd_, events_, base_s::epoll_max_events(), base_s::receive_timeout())) < 0) {
      DEBUG_LOG((boost::format("Epoll wait error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (std::is_same_v<RetType, int32_t>)
        return rc;
      else
        return {{rc, nullptr, sockaddr_inet_t()}};
    }

    num_ready = rc;
    for (uint32_t i = 0u; i < num_ready; i++) {
      if ((events_[i].data.fd == sock_fd_) && ((events_[i].events & EPOLLIN) == EPOLLIN)) {
        int32_t rc, bytes_pending, recvd;

        if ((rc = ::ioctl(sock_fd_, FIONREAD, &bytes_pending)) < 0) {
          DEBUG_LOG((boost::format("IOctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {{rc, nullptr, sockaddr_inet_t()}};
        }

        void *data = std::malloc(bytes_pending);
      recv:
        if ((rc = recv_function(sock_fd_, data, bytes_pending, 0u)) < 0) {
          if (errno != EAGAIN) {

            std::free(data);
            if ((errno == ECONNRESET) || (errno == ENOTCONN)) {

            disconnect:
              if (state_ == state_e::CONNECTED) {
                connected_info_lock_.lock();
                std::thread([this, connected = connected_]() -> void {
                  this->on_disconnect()(connected, this);
                  {
                    std::unique_lock<std::mutex> lock(this->mtx());
                    std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                  }
                }).detach();
                std::memset(&connected_, 0x0, sizeof(connected_));
                state_ = state_e::DISCONNECTED;
                connected_info_lock_.unlock();
              }

              if constexpr (rb == recv_behavior_e::HOOK) {

                return recvd_size;
              } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

                return std::move(ret);
              }
            } else {
              DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                         __FILE__ % __LINE__)
                            .str());
              if constexpr (std::is_same_v<RetType, int32_t>)
                return rc;
              else
                return {{rc, nullptr, sockaddr_inet_t()}};
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
          recvd = rc;
          if constexpr (rb == recv_behavior_e::HOOK) {

            connected_info_lock_.lock();
            std::thread([this, connected = connected_, data, size = recvd]() -> void {
              this->on_receive()(connected,
                                 std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                 this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
            connected_info_lock_.unlock();

          } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

            ret.push_back(std::make_tuple(
                recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), connected_));

            if constexpr (rb == recv_behavior_e::HOOK_RET) {
              connected_info_lock_.lock();
              std::thread([this, connected = connected_, size = recvd, data]() -> void {
                this->on_receive()(connected,
                                   std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                                   this);
                {
                  std::unique_lock<std::mutex> lock(this->mtx());
                  std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                }
              }).detach();

              connected_info_lock_.unlock();
            }
          }

          recvd_size += recvd;
        }
      }
    }

    if constexpr (rb == recv_behavior_e::HOOK) {

      return recvd_size;
    } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

      return std::move(ret);
    }
  }

  template <connect_behavior_e cb = connect_behavior_e::HOOK_ON>
  int32_t setup_(uint16_t port, const std::string &addr = "") noexcept {
    struct addrinfo *addr_info, hints;
    int32_t rc, trueflag = 1, try_count = 0, ret;
    const char *addr_str;

    std::memset(&hints, 0x0, sizeof(hints));
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_protocol = protocol;

    if constexpr (socket_class == tcp_sock_type_e::CLIENT_UNICAST) {

      addr_str = addr.c_str();
    } else if constexpr (socket_class == tcp_sock_type_e::SERVER_UNICAST) {

      addr_str = this->iface().host_addr.data();
    }

    if ((rc = ::getaddrinfo(addr_str, std::to_string(port).c_str(), &hints, &addr_info)) != 0 || addr_info == nullptr) {
      DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2%\" (errno : %3%), (%4%), %5%:%6%\"") % addr %
                 std::to_string(port) % gai_strerror(rc) % __func__ % __FILE__ % __LINE__)
                    .str())
      return -1;
    }

    if constexpr (base_s::is_ipv6)
      reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_scope_id = this->iface().scopeid;

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket: (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str())
      return rc;
    }

    struct timeval recv_timeout = {base_s::receive_timeout(), 0u};
    struct timeval send_timeout = {base_s::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &trueflag, sizeof(trueflag))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return -1;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEPORT, &trueflag, sizeof(trueflag))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return -1;
    }

    trueflag = IP_PMTUDISC_DO;
    if ((rc = ::setsockopt(sock_fd_, protocol, IP_MTU_DISCOVER, &trueflag, sizeof(trueflag))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return -1;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return -1;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return -1;
    }

    if constexpr (socket_class == tcp_sock_type_e::CLIENT_UNICAST) {

      ret = connect_<cb>(addr_info);
    } else if constexpr (socket_class == tcp_sock_type_e::SERVER_UNICAST) {

      ret = bind_(addr_info);
    }

    ::freeaddrinfo(addr_info);
    return ret;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  bind_(struct addrinfo *addr_info) noexcept {
    int32_t rc;
    if ((rc = ::bind(sock_fd_, addr_info->ai_addr, addr_info->ai_addrlen)) != 0) {
      clear_();
      DEBUG_LOG((boost::format("Could not bind TCP socket (errno = %1%), (%2%), %3%:%4%") % strerror(rc) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type clear_() noexcept {
    int32_t rc;
    if ((rc = stop()) < 0) {
      DEBUG_LOG((boost::format("Couldn't stop socket (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      std::thread([this, connected = it->second]() -> void {
        this->on_disconnect()(connected, this);
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();

      if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, it->first, nullptr)) < 0u) {
        DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }

      ::close(it->first);
    }

    connected_.clear();
    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    ::close(sock_fd_);
    state_ = state_e::STOPPED;
    connected_info_lock_.unlock();
    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  disconnect_peer_(const std::string &addr, uint16_t srv) const noexcept {
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

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type disconnect_peer_(int32_t fd) const
      noexcept {
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

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type clear_() noexcept {
    int32_t rc;
    connected_info_lock_.lock();
    if (state_ == state_e::CONNECTED) {
      std::thread([this, connected = connected_]() -> void {
        this->on_disconnect()(connected, this);
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();
      std::memset(&connected_, 0x0, sizeof(connected_));
    }

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    on_disconnect_internal_hook_(sock_fd_);
    ::close(sock_fd_);
    state_ = state_e::DISCONNECTED;
    connected_info_lock_.unlock();

    return rc;
  }

  void clear_hooks_() noexcept {
    this->on_connect().clear();
    this->on_disconnect().clear();
    this->on_receive().clear();
    this->on_send().clear();
  }

  void clear_epoll_() noexcept {
    epoll_fd_lock_.lock();
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
    epoll_fd_lock_.unlock();
  }

  int32_t open_() noexcept {
    int32_t rc;
    if ((rc = ::socket(family, socktype | SOCK_CLOEXEC | SOCK_NONBLOCK, protocol)) < 0) {
      clear_();
      DEBUG_LOG((boost::format("Could not create socket, (%1%), %2%:%3%") % __func__ % __FILE__ % __LINE__).str());
      return rc;
    }

    sock_fd_ = rc;
    struct epoll_event event;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents;
    event.data.fd = sock_fd_;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_ADD, sock_fd_, &event)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <recv_behavior_e rb = recv_behavior_e::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  handle_incoming_data_(int32_t fd, RecvFunction recv_function = ::recv) noexcept {
    int32_t recvd, rc, bytes_pending;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_s::receive_timeout() / 1000, 0u};

    if ((rc = ::ioctl(fd, FIONREAD, &bytes_pending)) < 0) {
      DEBUG_LOG((boost::format("IOctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    void *data = std::malloc(bytes_pending);
  recv:
    if ((rc = recv_function(fd, data, bytes_pending, 0u)) < 0) {
      if (errno != EAGAIN) {

        std::free(data);
        if ((errno == ECONNRESET) || (errno == ENOTCONN)) {

        disconnect:
          connected_info_lock_.lock();
          for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
            if (fd == it->first) {
              std::thread([this, connected = it->second]() -> void {
                this->on_disconnect()(connected, this);
                {
                  std::unique_lock<std::mutex> lock(this->mtx());
                  std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                }
              }).detach();
              static_cast<void>(disconnect_peer_(fd));
              connected_info_lock_.unlock();

              if constexpr (rb == recv_behavior_e::HOOK) {

                return recvd;
              } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

                return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
              }
            }
          }

          connected_info_lock_.unlock();
          if constexpr (rb == recv_behavior_e::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_inet_t()};
          }
        } else {
          DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, nullptr, sockaddr_inet_t()};
        }
      } else {
        FD_ZERO(&read_fd_set);
        FD_SET(sock_fd_, &read_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
          goto disconnect;
        } else
          goto recv;
      }
    } else if (!rc) {

      connected_info_lock_.lock();
      for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
        if (fd == it->first) {
          std::thread([this, connected = it->second]() -> void {
            this->on_disconnect()(connected, this);
            {
              std::unique_lock<std::mutex> lock(this->mtx());
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }
          }).detach();
          static_cast<void>(disconnect_peer_(fd));
          connected_info_lock_.unlock();

          if constexpr (rb == recv_behavior_e::HOOK) {

            return rc;
          } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

            return {rc, std::shared_ptr<void>(), sockaddr_inet_t()};
          }
        }
      }

      connected_info_lock_.unlock();
      std::free(data);
      if constexpr (rb == recv_behavior_e::HOOK) {

        return rc;
      } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

        return {rc, nullptr, sockaddr_inet_t()};
      }
    } else {
      recvd = rc;
      sockaddr_inet_t peer;
      socklen_t peerlen = sizeof(peer);

      if ((rc = ::getpeername(fd, reinterpret_cast<struct sockaddr *>(&peer), &peerlen)) < 0) {
        std::free(data);
        DEBUG_LOG((boost::format("Get peer name failed (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        if constexpr (std::is_same_v<RetType, int32_t>)
          return rc;
        else
          return {rc, nullptr, sockaddr_inet_t()};
      }

      if constexpr (rb == recv_behavior_e::HOOK) {
        std::thread([this, peer, data, size = recvd]() -> void {
          this->on_receive()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                             this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
        return recvd;
      } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

        if constexpr (rb == recv_behavior_e::HOOK_RET) {
          void *data_copy = std::malloc(recvd);
          std::memcpy(data_copy, data, recvd);
          std::thread([this, peer, data_copy, size = recvd]() -> void {
            this->on_receive()(
                peer, std::shared_ptr<void>(data_copy, [](const auto &data) -> void { std::free(data); }), size, this);
            {
              std::unique_lock<std::mutex> lock(this->mtx());
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }
          }).detach();
        }

        return {recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), std::move(peer)};
      }
    }
  }

  template <connect_behavior_e cb = connect_behavior_e::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  connect_(const struct addrinfo *addr_info) noexcept {
    int32_t rc, num_ready;
    struct epoll_event event;

    state_ = state_e::CONNECTING;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents | EPOLLOUT;
    event.data.fd = sock_fd_;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::connect(sock_fd_, addr_info->ai_addr, sizeof(sockaddr_inet_t))) < 0) {
      if (errno != EINPROGRESS) {
        DEBUG_LOG((boost::format("Connection error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }
    }

    if ((rc = ::epoll_wait(epfd_, events_, base_s::epoll_max_events(), base_s::connect_timeout())) < 0) {
      DEBUG_LOG((boost::format("Epoll wait error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    for (int32_t i = 0; i < num_ready; i++) {
      if (events_[i].data.fd == sock_fd_) {
        int32_t epoll_error = 0u;
        socklen_t epoll_errlen = sizeof(epoll_error);
        if ((rc = ::getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&epoll_error), &epoll_errlen)) <
            0) {
          DEBUG_LOG((boost::format("getsockopt() error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          return rc;
        }

        sockaddr_inet_t server;
        std::memcpy(&server, reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr), addr_info->ai_addrlen);

        std::memset(&event, 0x0, sizeof(event));
        event.events = epollevents;
        event.data.fd = sock_fd_;

        if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event)) < 0u) {
          DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          return rc;
        }

        if (epoll_error) {
          std::thread([this, server]() -> void {
            this->on_disconnect()(server, this);
            {
              std::unique_lock<std::mutex> lock(this->mtx());
              std::notify_all_at_thread_exit(this->cv(), std::move(lock));
            }
          }).detach();
          disconnect();
          state_ = state_e::DISCONNECTED;
        } else {

          connected_info_lock_.lock();
          std::memcpy(&connected_, &server, sizeof(server));
          connected_info_lock_.unlock();

          if constexpr (cb == connect_behavior_e::HOOK_ON) {
            std::thread([this, server]() -> void {
              this->on_connect()(server, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
          }

          state_ = state_e::CONNECTED;
        }
      }
    }

    return state_ == state_e::CONNECTED ? 0 : -1;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type set_non_blocking_(int32_t fd) const
      noexcept {
    int32_t flags, rc;
    if ((rc = ::fcntl(fd, F_GETFL, 0)) < 0) {
      DEBUG_LOG((boost::format("Set non-blocking error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) %
                 __func__ % __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    flags = rc;
    if ((rc = ::fcntl(fd, F_SETFL, flags | O_NONBLOCK)) < 0) {
      DEBUG_LOG((boost::format("Set non-blocking error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) %
                 __func__ % __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type set_blocking_(int32_t fd) const
      noexcept {
    int32_t flags, rc;
    if ((rc = ::fcntl(fd, F_GETFL, 0)) < 0) {
      DEBUG_LOG((boost::format("Set non-blocking error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) %
                 __func__ % __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    flags = rc;
    if ((rc = ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK)) < 0) {
      DEBUG_LOG((boost::format("Set non-blocking error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) %
                 __func__ % __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer_(const std::string &addr, uint16_t port, sockaddr_inet_t **peer_addr) const noexcept {
    sockaddr_inet_t net_addr;
    int32_t rc;
    void *dst_addr, *dst_port;

    if constexpr (base_s::is_ipv6) {

      dst_addr = &net_addr.sin6_addr;
      net_addr.sin6_port = ::ntohs(port);
    } else {

      dst_addr = &net_addr.sin_addr;
      net_addr.sin_port = ::ntohs(port);
    }

    if ((rc = ::inet_pton(family, addr.c_str(), dst_addr)) <= 0) {
      DEBUG_LOG((boost::format("inet_pton() failed for addr = %1%\r\n") % addr.c_str()).str());
      return rc;
    }

    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if constexpr (base_s::is_ipv6) {
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
    rc = -1;
    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer_(int32_t fd, sockaddr_inet_t **peer_addr) const noexcept {
    connected_info_lock_.lock();
    for (typename connected_peer_info_t::iterator it = connected_.begin(); it != connected_.end(); it++) {
      if (fd == it->first) {
        if (peer_addr)
          *peer_addr = &it->second;

        connected_info_lock_.unlock();
        return 0;
      }
    }

    if (peer_addr)
      *peer_addr = nullptr;

    connected_info_lock_.unlock();
    return -1;
  }

  template <connect_behavior_e cb = connect_behavior_e::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_e::HOOK_ON, int32_t, std::pair<sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type handle_incoming_peer_() const noexcept {
    sockaddr_inet_t peer_addr;
    socklen_t peer_addr_size = sizeof(peer_addr);
    uint16_t peer_srv;
    struct epoll_event peer_ev;
    int32_t peer_fd, rc;
    fd_set accept_fd_set;
    struct timeval accept_timeout = {base_s::accept_timeout() / 1000, 0u};

    state_ = state_e::CONNECTING;
  accept:
    if ((rc = ::accept(sock_fd_, reinterpret_cast<struct sockaddr *>(&peer_addr), &peer_addr_size)) < 0) {
      if (errno != EAGAIN) {
        DEBUG_LOG((boost::format("Accept error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                   __LINE__)
                      .str());
        if constexpr (cb == connect_behavior_e::HOOK_ON) {

          return rc;
        } else if constexpr (cb == connect_behavior_e::HOOK_OFF) {

          return {sockaddr_inet_t(), rc};
        }
      } else {
        peer_fd = rc;

        FD_ZERO(&accept_fd_set);
        FD_SET(sock_fd_, &accept_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &accept_fd_set, nullptr, nullptr, &accept_timeout)) <= 0) {
          if constexpr (cb == connect_behavior_e::HOOK_ON) {

            return peer_fd;
          } else if constexpr (cb == connect_behavior_e::HOOK_OFF) {

            return {sockaddr_inet_t(), peer_fd};
          }
        } else
          goto accept;
      }
    }

    peer_fd = rc;
    if constexpr (base_s::is_ipv6) {

      peer_srv = ::htons(peer_addr.sin6_port);
    } else {

      peer_srv = ::htons(peer_addr.sin_port);
    }

    set_non_blocking_(peer_fd);
    std::memset(&peer_ev, 0x0, sizeof(peer_ev));
    peer_ev.data.fd = peer_fd;
    peer_ev.events = EPOLLIN | EPOLLET;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_ADD, peer_fd, &peer_ev)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      if constexpr (cb == connect_behavior_e::HOOK_ON) {

        return rc;
      } else if constexpr (cb == connect_behavior_e::HOOK_OFF) {

        return {sockaddr_inet_t(), rc};
      }
    }

    connected_info_lock_.lock();
    connected_.insert(std::make_pair(peer_fd, peer_addr));
    connected_info_lock_.unlock();

    if constexpr (cb == connect_behavior_e::HOOK_ON) {
      std::thread([this, peer_addr]() -> void {
        this->on_connect()(peer_addr, this);
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();
      return peer_fd;
    } else if constexpr (cb == connect_behavior_e::HOOK_OFF) {

      return {std::move(peer_addr), peer_fd};
    }
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type listen_() const noexcept {
    int32_t rc, num_ready;
    listen_enabled_ = true;
    if ((rc = ::listen(sock_fd_, SOMAXCONN)) < 0) {
      DEBUG_LOG((boost::format("Start listening error: (errno = %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    state_ = state_e::LISTENING;
    while (listen_enabled_) {
      if ((rc = ::epoll_wait(epfd_, events_, base_s::epoll_max_events(), base_s::accept_timeout())) < 0u) {
        DEBUG_LOG((boost::format("Epoll wait error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }

      num_ready = rc;
      for (int32_t i = 0; i < num_ready; i++) {
        if (((events_[i].events & EPOLLIN) == EPOLLIN || (events_[i].events & EPOLLET) == EPOLLET) &&
            (events_[i].data.fd == sock_fd_)) {

          static_cast<void>(handle_incoming_peer_());
        } else if (((events_[i].events & EPOLLIN) == EPOLLIN || (events_[i].events & EPOLLET) == EPOLLET) &&
                   (events_[i].data.fd != sock_fd_)) {

          static_cast<void>(handle_incoming_data_(events_[i].data.fd));
        }
      }
    }

    state_ = state_e::STOPPED;
    return 0;
  }
};

template <tcp_sock_type_e sc, bool multithread>
struct network_tcp_socket_ipv4_s : network_tcp_socket_impl_s<AF_INET, sc, multithread> {
  using network_tcp_socket_impl_s<AF_INET, sc, multithread>::network_tcp_socket_impl_s;
};

template <tcp_sock_type_e sc, bool multithread>
struct network_tcp_socket_ipv6_s : network_tcp_socket_impl_s<AF_INET6, sc, multithread> {
  using network_tcp_socket_impl_s<AF_INET6, sc, multithread>::network_tcp_socket_impl_s;
};

#endif /* NETWORK_TCP_SOCK_HPP */
