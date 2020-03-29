#ifndef DOMAIN_TCP_SOCK_HPP
#define DOMAIN_TCP_SOCK_HPP

#include "base_socket.hpp"
#include "debug.hpp"
#include "tcp_sock_type.hpp"

#include <errno.h>
#include <thread>

template <uint32_t family, tcp_sock_type_e socket_class, bool multithread>
struct domain_tcp_socket_impl_s : public base_sock_s<family, SOCK_STREAM, IPPROTO_TCP, multithread> {
public:
  static constexpr int32_t socktype = SOCK_STREAM;
  static constexpr int32_t protocol = IPPROTO_TCP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_s : int32_t { CONNECTED, DISCONNECTED, LISTENING, CONNECTING, STOPPED };
  enum struct recv_behavior_s : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_s : uint32_t { HOOK_ON, HOOK_OFF };
  enum struct connect_behavior_s : uint32_t { HOOK_ON, HOOK_OFF };

  using this_s = domain_tcp_socket_impl_s<family, socket_class, multithread>;
  using base_s = base_sock_s<family, socktype, protocol, multithread>;

  using connected_peer_info_t = std::conditional_t<socket_class == tcp_sock_type_e::CLIENT_UNICAST, struct sockaddr_un,
                                                   std::conditional_t<socket_class == tcp_sock_type_e::SERVER_UNICAST,
                                                                      std::map<int32_t, struct sockaddr_un>, void *>>;

  template <tcp_sock_type_e sc = socket_class>
  explicit domain_tcp_socket_impl_s(
      typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, tcp_sock_type_e>::type * = nullptr)
      : base_s(), state_(state_s::DISCONNECTED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_s::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  template <tcp_sock_type_e sc = socket_class>
  explicit domain_tcp_socket_impl_s(
      const std::string &path,
      typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, tcp_sock_type_e>::type * = nullptr)
      : base_s(path), state_(state_s::STOPPED), epfd_(epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(base_s::epoll_max_events() * sizeof(struct epoll_event)))),
        on_disconnect_internal_hook_([](int32_t) {}), on_connect_internal_hook_([](int32_t) {}) {}

  const auto &on_connect() const noexcept { return on_connect_; }
  const auto &on_disconnect() const noexcept { return on_disconnect_; }
  const auto &on_receive() const noexcept { return on_receive_; }
  const auto &on_send() const noexcept { return on_send_; }

  void stop_threads() const noexcept { return this->base_s::stop_threads(); }
  template <connect_behavior_s cb = connect_behavior_s::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  connect(const std::string &path) noexcept {
    return setup_<cb>(path);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type disconnect() noexcept {
    clear_();
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  disconnect(const std::string &path) noexcept {
    return disconnect_peer_(path);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type setup() noexcept {
    return setup_();
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type listening() const noexcept {
    return state_ == state_s::LISTENING;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type connecting() const noexcept {
    return state_ == state_s::CONNECTING;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type connected() const noexcept {
    return state_ == state_s::CONNECTED;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  is_peer_connected(const std::string &path) const noexcept {
    int32_t rc = get_connected_peer_(path, nullptr);
    return rc > 0;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type is_peer_connected(int32_t fd) const
      noexcept {
    struct sockaddr_un *path;
    int32_t rc = get_connected_peer_(fd, &path);
    return path != nullptr;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash) const noexcept {
    for (const auto &peer : connected_) {
      const sockaddr_un &peer_path = peer.second;
      sha256::sha256_hash_type peer_addr_hash =
          sha256::compute(reinterpret_cast<const uint8_t *>(peer_path.sun_path), sizeof(peer_path.sun_path));

      if (hash == peer_addr_hash)
        return true;
    }

    return false;
  }

  void reset() noexcept {
    clear_();
    clear_hooks_();
  }

  virtual ~domain_tcp_socket_impl_s() noexcept {
    reset();
    clear_epoll_();
  };

  int32_t set_blocking__(int32_t fd) const noexcept { set_blocking_(fd); }
  int32_t set_non_blocking__(int32_t fd) const noexcept { set_non_blocking_(fd); }

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

protected:
  const connected_peer_info_t &connected__() const noexcept { return connected_; }
  const int32_t &fd__() const noexcept { return sock_fd_; }
  const int32_t &epfd__() const noexcept { return epfd_; };
  struct epoll_event *events__() const noexcept {
    return events_;
  }

  std::atomic_bool &listen_enabled__() const noexcept { return listen_enabled_; }
  std::thread &listen_thread__() const noexcept { return listen_thread_; }
  std::atomic<state_s> &state__() const noexcept { return state_; }

  template <recv_behavior_s rb = recv_behavior_s::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_s::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  handle_incoming_data__(int32_t fd, RecvFunction recv_function = ::recv) noexcept {
    return this->template handle_incoming_data_<rb, RecvFunction>(fd, recv_function);
  }

  template <connect_behavior_s cb = connect_behavior_s::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_s::HOOK_ON, int32_t, std::pair<struct sockaddr_un, int32_t>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type handle_incoming_peer__() const
      noexcept {
    return this->template handle_incoming_peer_<cb>();
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer__(const std::string &addr, uint16_t port, struct sockaddr_un **peer_addr) const noexcept {
    return get_connected_peer_(addr, port, peer_addr);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer__(int32_t fd, struct sockaddr_un **peer_addr) const noexcept {
    return get_connected_peer_(fd, peer_addr);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type disconnect_peer__(int32_t fd) const
      noexcept {
    return disconnect_peer_(fd);
  }

  template <send_behavior_s sb = send_behavior_s::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_s::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_s::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  send__(int32_t peer_fd, const struct sockaddr_un *const peer_addr, const void *const msg, size_t size,
         SendFunction send_function = ::send) const noexcept {
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
  mutable std::atomic<state_s> state_;

  /* Hooks interface */
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_s *)> on_receive_;
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_s *)> on_send_;
  const hook_t<void(struct sockaddr_un, const this_s *)> on_connect_;
  const hook_t<void(struct sockaddr_un, const this_s *)> on_disconnect_;

  /* Internal hooks, serve to customizing of behavior */
  std::function<void(int32_t)> on_connect_internal_hook_;
  std::function<void(int32_t)> on_disconnect_internal_hook_;

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

  template <send_behavior_s sb = send_behavior_s::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_s::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_s::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  send_(int32_t peer_fd, const struct sockaddr_un *const peer_path, const void *const msg, size_t size,
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
            std::thread([this, peer_path = *peer_path]() -> void {
              this->on_disconnect()(peer_path, this);
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
          if constexpr (sb == send_behavior_s::HOOK_ON) {

            return rc;
          } else if constexpr (sb == send_behavior_s::HOOK_OFF) {

            return {*peer_path, rc};
          }
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
      if constexpr (sb == send_behavior_s::HOOK_ON) {

        void *data = std::malloc(size * sizeof(char));
        std::memcpy(data, msg, size);
        std::thread([this, peer_path = *peer_path, data, size]() -> void {
          this->on_send()(peer_path, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                          size, this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
      }
    }

    if constexpr (sb == send_behavior_s::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_s::HOOK_OFF) {

      return {*peer_path, rc};
    }
  }

  template <send_behavior_s sb = send_behavior_s::HOOK_ON,
            typename SendFunction = int32_t (*)(int32_t, const void *, size_t, int32_t),
            tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_s::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_s::HOOK_OFF, std::pair<struct sockaddr_un, int32_t>, void>>>
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
          if (state_ == state_s::CONNECTED) {

            connected_info_lock_.lock();
            std::thread([this, connected = connected_]() -> void {
              this->on_disconnect()(connected, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
            std::memset(&connected_, 0x0, sizeof(connected_));
            state_ = state_s::DISCONNECTED;
            connected_info_lock_.unlock();
          }
        } else {
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (sb == send_behavior_s::HOOK_ON) {

            return rc;
          } else if constexpr (sb == send_behavior_s::HOOK_OFF) {

            return {connected_, rc};
          }
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
      if constexpr (sb == send_behavior_s::HOOK_ON) {
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

    if constexpr (sb == send_behavior_s::HOOK_ON) {

      return rc;
    } else if constexpr (sb == send_behavior_s::HOOK_OFF) {

      return {connected_, rc};
    }
  }

  template <recv_behavior_s rb = recv_behavior_s::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_s::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>>, void>>>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  recv_(RecvFunction recv_function = ::recv) const noexcept {
    int32_t recvd_size = 0, num_ready, rc;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_s::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>> ret;

    if ((rc = ::epoll_wait(epfd_, events_, base_s::epoll_max_events(), base_s::receive_timeout())) < 0) {
      DEBUG_LOG((boost::format("Epoll wait error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (rb == recv_behavior_s::HOOK) {

        return rc;
      } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

        return {{rc, nullptr, sockaddr_un()}};
      }
    }

    num_ready = rc;
    for (uint32_t i = 0u; i < num_ready; i++) {
      if ((events_[i].data.fd == sock_fd_) && ((events_[i].events & EPOLLIN) == EPOLLIN)) {
        int32_t rc, bytes_pending, recvd;

        if ((rc = ::ioctl(sock_fd_, FIONREAD, &bytes_pending)) < 0) {
          DEBUG_LOG((boost::format("IOctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (rb == recv_behavior_s::HOOK) {

            return rc;
          } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

            return {{rc, nullptr, sockaddr_un()}};
          }
        }

        void *data = std::malloc(bytes_pending);
      recv:
        if ((recvd = recv_function(sock_fd_, data, bytes_pending, 0u)) < 0) {
          if (errno != EAGAIN) {

            std::free(data);
            if ((errno == ECONNRESET) || (errno == ENOTCONN)) {

            disconnect:
              if (state_ == state_s::CONNECTED) {
                connected_info_lock_.lock();
                std::thread([this, connected = connected_]() -> void {
                  this->on_disconnect()(connected, this);
                  {
                    std::unique_lock<std::mutex> lock(this->mtx());
                    std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                  }
                }).detach();
                std::memset(&connected_, 0x0, sizeof(connected_));
                state_ = state_s::DISCONNECTED;
                connected_info_lock_.unlock();
              }

              if constexpr (rb == recv_behavior_s::HOOK) {

                return recvd_size;
              } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

                return std::move(ret);
              }
            } else {
              DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                         __FILE__ % __LINE__)
                            .str());
              if constexpr (rb == recv_behavior_s::HOOK) {

                return rc;
              } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

                return {{rc, nullptr, sockaddr_un()}};
              }
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
          if constexpr (rb == recv_behavior_s::HOOK) {

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
          } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

            ret.push_back(std::make_tuple(
                recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), connected_));

            if constexpr (rb == recv_behavior_s::HOOK_RET) {
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

    if constexpr (rb == recv_behavior_s::HOOK) {

      return recvd_size;
    } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

      return std::move(ret);
    }
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  bind_(const struct sockaddr_un *sock_path) noexcept {
    int32_t rc;

    if ((rc = ::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&sock_path), sizeof(sock_path))) != 0) {
      clear_();
      DEBUG_LOG((boost::format("Could not bind TCP socket (errno = %1%), (%2%), %3%:%4%") % strerror(rc) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <connect_behavior_s cb = connect_behavior_s::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  connect_(const std::string &path) noexcept {
    int32_t rc, num_ready;
    struct epoll_event event;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, path.c_str(), sizeof(sock_path.sun_path) - 1u);

    state_ = state_s::CONNECTING;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents | EPOLLOUT;
    event.data.fd = sock_fd_;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_MOD, sock_fd_, &event)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::connect(sock_fd_, reinterpret_cast<struct sockaddr *>(&sock_path), sizeof(sock_path))) < 0) {
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

    num_ready = rc;
    for (int32_t i = 0; i < num_ready; i++) {
      if (events_[i].data.fd == sock_fd_) {
        int32_t epoll_error = 0u, rc;
        socklen_t epoll_errlen = sizeof(epoll_error);
        if ((rc = ::getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&epoll_error), &epoll_errlen)) <
            0) {
          DEBUG_LOG((boost::format("getsockopt() error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          return rc;
        }

        struct sockaddr_un server;
        std::memcpy(&server, &sock_path, sizeof(sock_path));
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
          state_ = state_s::DISCONNECTED;
        } else {

          connected_info_lock_.lock();
          std::memcpy(&connected_, &server, sizeof(server));
          connected_info_lock_.unlock();

          if constexpr (cb == connect_behavior_s::HOOK_ON) {
            std::thread([this, server]() -> void {
              this->on_connect()(server, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
          }

          state_ = state_s::CONNECTED;
        }
      }
    }

    return state_ == state_s::CONNECTED ? 0 : -1;
  }

  template <connect_behavior_s cb = connect_behavior_s::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type
  setup_(const std::string &path) noexcept {
    int32_t rc, trueflag = 1, try_count = 0;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, path.c_str(), path.length());

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    struct timeval recv_timeout = {base_s::receive_timeout(), 0u};
    struct timeval send_timeout = {base_s::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt() error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt() error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return connect_<cb>(path);
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type setup_() noexcept {
    int32_t rc, trueflag = 1, try_count = 0;
    struct sockaddr_un sock_path;
    std::strncpy(sock_path.sun_path, this->path().c_str(), this->path().length());

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    struct timeval recv_timeout = {base_s::receive_timeout(), 0u};
    struct timeval send_timeout = {base_s::send_timeout(), 0u};

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return bind_(&sock_path);
  }

  int32_t open_() noexcept {
    int32_t rc;
    struct epoll_event event;

    if ((rc = ::socket(family, socktype | SOCK_CLOEXEC | SOCK_NONBLOCK, protocol)) < 0) {
      clear_();
      DEBUG_LOG((boost::format("Could not create socket, (%1%), %2%:%3%") % __func__ % __FILE__ % __LINE__).str());
      return rc;
    }

    sock_fd_ = rc;
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

  template <recv_behavior_s rb = recv_behavior_s::HOOK,
            typename RecvFunction = int32_t (*)(int32_t, void *, size_t, int32_t), tcp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                rb == recv_behavior_s::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>, void>>>
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

              if constexpr (rb == recv_behavior_s::HOOK) {

                return rc;
              } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

                return {rc, nullptr, sockaddr_un()};
              }
            }
          }

          connected_info_lock_.unlock();
          if constexpr (rb == recv_behavior_s::HOOK) {

            return rc;
          } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

            return {rc, nullptr, sockaddr_un()};
          }
        } else {
          DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          return rc;
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

          if constexpr (rb == recv_behavior_s::HOOK) {

            return recvd;
          } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

            return {recvd, std::shared_ptr<void>(), sockaddr_un()};
          }
        }
      }

      connected_info_lock_.unlock();
      std::free(data);
      if constexpr (rb == recv_behavior_s::HOOK) {

        return rc;
      } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

        return {rc, std::shared_ptr<void>(), sockaddr_un()};
      }
    } else {
      struct sockaddr_un peer;
      socklen_t peerlen = sizeof(peer);

      recvd = rc;
      if ((rc = ::getpeername(fd, reinterpret_cast<struct sockaddr *>(&peer), &peerlen)) < 0) {
        std::free(data);
        DEBUG_LOG((boost::format("Get peer name failed (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }

      if constexpr (rb == recv_behavior_s::HOOK) {
        std::thread([this, peer, data, size = recvd]() -> void {
          this->on_receive()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                             this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
        return recvd;
      } else if constexpr (rb == recv_behavior_s::RET || rb == recv_behavior_s::HOOK_RET) {

        if constexpr (rb == recv_behavior_s::HOOK_RET) {
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

  template <connect_behavior_s cb = connect_behavior_s::HOOK_ON, tcp_sock_type_e sc = socket_class,
            typename RetType =
                std::conditional_t<cb == connect_behavior_s::HOOK_ON, int32_t, std::pair<struct sockaddr_un, int32_t>>>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type handle_incoming_peer_() const noexcept {
    struct sockaddr_un peer_addr;
    socklen_t peer_addr_size = sizeof(peer_addr);
    uint16_t peer_srv;
    struct epoll_event peer_ev;
    int32_t peer_fd, rc;
    fd_set accept_fd_set;
    struct timeval accept_timeout = {base_s::accept_timeout() / 1000, 0u};

    state_ = state_s::CONNECTING;
  accept:
    if ((rc = ::accept(sock_fd_, reinterpret_cast<struct sockaddr *>(&peer_addr), &peer_addr_size)) < 0) {
      if (errno != EAGAIN) {
        DEBUG_LOG((boost::format("Accept error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                   __LINE__)
                      .str())
        if constexpr (cb == connect_behavior_s::HOOK_ON) {

          return rc;
        } else if constexpr (cb == connect_behavior_s::HOOK_OFF) {

          return {peer_addr, rc};
        }
      } else {
        FD_ZERO(&accept_fd_set);
        FD_SET(sock_fd_, &accept_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &accept_fd_set, nullptr, nullptr, &accept_timeout)) <= 0) {
          if constexpr (cb == connect_behavior_s::HOOK_ON) {

            return rc;
          } else if constexpr (cb == connect_behavior_s::HOOK_OFF) {

            return {peer_addr, rc};
          }
        } else
          goto accept;
      }
    }

    peer_fd = rc;
    set_non_blocking_(peer_fd);
    std::memset(&peer_ev, 0x0, sizeof(peer_ev));
    peer_ev.data.fd = peer_fd;
    peer_ev.events = EPOLLIN | EPOLLET;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_ADD, peer_fd, &peer_ev)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ % __FILE__ %
                 __LINE__)
                    .str());
      if constexpr (cb == connect_behavior_s::HOOK_ON) {

        return rc;
      } else if constexpr (cb == connect_behavior_s::HOOK_OFF) {

        return {peer_addr, rc};
      }
    }

    connected_info_lock_.lock();
    connected_.insert(std::make_pair(peer_fd, peer_addr));
    connected_info_lock_.unlock();

    if constexpr (cb == connect_behavior_s::HOOK_ON) {
      std::thread([this, peer_addr]() -> void {
        this->on_connect()(peer_addr, this);
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();
      return peer_fd;
    } else if constexpr (cb == connect_behavior_s::HOOK_OFF) {

      return {std::move(peer_addr), peer_fd};
    }
  }

  void clear_epoll_() noexcept {
    epoll_fd_lock_.lock();
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
    epoll_fd_lock_.unlock();
  }

  void clear_hooks_() noexcept {
    this->on_connect().clear();
    this->on_disconnect().clear();
    this->on_receive().clear();
    this->on_send().clear();
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

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type clear_() noexcept {
    int32_t rc;
    stop();
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
    state_ = state_s::STOPPED;
    connected_info_lock_.unlock();
    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::CLIENT_UNICAST, RetType>::type clear_() noexcept {
    int32_t rc;
    connected_info_lock_.lock();
    if (state_ == state_s::CONNECTED) {
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
    state_ = state_s::DISCONNECTED;
    connected_info_lock_.unlock();
    return rc;
  }

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer_(const std::string &path, sockaddr_un **peer_path) const noexcept {
    struct sockaddr_un sock_path;
    int32_t rc;
    void *dst_path;

    if ((rc = ::inet_pton(family, path.c_str(), dst_path)) <= 0) {
      DEBUG_LOG((boost::format("inet_pton() failed for addr = %1%\r\n") % path.c_str()).str());
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

  template <tcp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  get_connected_peer_(int32_t fd, struct sockaddr_un **peer_path) const noexcept {
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

  template <tcp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_type_e::SERVER_UNICAST, RetType>::type
  disconnect_peer_(const std::string &path) const noexcept {
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
};

template <tcp_sock_type_e sc, bool multithread>
struct domain_tcp_socket : domain_tcp_socket_impl_s<AF_UNIX, sc, multithread> {
  using domain_tcp_socket_impl_s<AF_UNIX, sc, multithread>::domain_tcp_socket_impl_s;
};

#endif /* DOMAIN_TCP_SOCK_HPP */
