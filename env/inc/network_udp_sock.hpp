#ifndef NETWORK_UDP_SOCK_HPP
#define NETWORK_UDP_SOCK_HPP

#include "base_socket.hpp"
#include "debug.hpp"
#include "udp_sock_type.hpp"

#include <errno.h>
#include <future>
#include <thread>

template <uint32_t family, udp_sock_t socket_class>
struct network_udp_socket_impl : public base_socket<family, SOCK_DGRAM, IPPROTO_UDP> {
public:
  static constexpr int32_t socktype = SOCK_DGRAM;
  static constexpr int32_t protocol = IPPROTO_UDP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_t : int32_t { RUNNING, STOPPED };
  enum struct recv_behavior_t : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };

  using this_t = network_udp_socket_impl<family, socket_class>;
  using base_t = base_socket<family, socktype, protocol>;

  static constexpr bool is_ipv6 = base_t::is_ipv6;

  using sockaddr_inet_t = std::conditional_t<is_ipv6, struct sockaddr_in6, struct sockaddr_in>;
  using inet_addr_t = std::conditional_t<is_ipv6, struct in6_addr, struct in_addr>;

  template <udp_sock_t sc = socket_class>
  explicit network_udp_socket_impl(
      const std::string &iface,
      typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST) ||
                                  sc == udp_sock_t::SERVER_MULTICAST,
                              udp_sock_t>::type * = nullptr)
      : base_t(iface), epfd_(::epoll_create1(EPOLL_CLOEXEC)), state_(state_t::STOPPED),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(this->epoll_max_events() * sizeof(struct epoll_event)))) {}

  template <udp_sock_t sc = socket_class>
  explicit network_udp_socket_impl(
      const std::string &iface,
      typename std::enable_if<sc == udp_sock_t::CLIENT_UNICAST || (!is_ipv6 && sc == udp_sock_t::CLIENT_BROADCAST),
                              udp_sock_t>::type * = nullptr)
      : base_t(iface), epfd_(::epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(this->epoll_max_events() * sizeof(struct epoll_event)))) {
    setup_();
  }

  template <udp_sock_t sc = socket_class>
  explicit network_udp_socket_impl(
      const std::string &iface, const std::string &mcast_gr_addr, uint16_t srv,
      typename std::enable_if<sc == udp_sock_t::CLIENT_MULTICAST, udp_sock_t>::type * = nullptr)
      : base_t(iface), epfd_(::epoll_create1(EPOLL_CLOEXEC)),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(this->epoll_max_events() * sizeof(struct epoll_event)))) {
    setup_(mcast_gr_addr, srv);
  }

  template <udp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_t::SERVER_MULTICAST, RetType>::type setup(const std::string &mcast_gr_addr,
                                                                                   uint16_t port) noexcept {
    setup_(mcast_gr_addr, port);
  }

  template <udp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST),
                          RetType>::type
  setup(uint16_t port) noexcept {
    setup_(port);
  }

  const auto &on_receive() const noexcept { return on_receive_; }
  const auto &on_send() const noexcept { return on_send_; }

  void stop_threads() const noexcept { return this->base_t::stop_threads(); }
  template <udp_sock_t sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST) ||
                              sc == udp_sock_t::SERVER_MULTICAST,
                          RetType>::type
  running() const noexcept {
    return state_ == state_t::RUNNING;
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON, udp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<int32_t, sockaddr_inet_t>, void>>>
  typename std::enable_if<!is_ipv6 && (sc == udp_sock_t::CLIENT_BROADCAST || sc == udp_sock_t::SERVER_BROADCAST),
                          RetType>::type
  send(uint16_t port, const void *const msg, size_t size) const noexcept {
    struct addrinfo *dgram_addrinfo, hints;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_t::send_timeout() / 1000, 0u};

    std::memset(&hints, 0x0, sizeof(hints));
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_protocol = protocol;

    int32_t rc;
    if ((rc = ::getaddrinfo(this->iface().broadcast.data(), std::to_string(port).c_str(), &hints, &dgram_addrinfo)) !=
            0 ||
        dgram_addrinfo == nullptr) {
      DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2% (errno = %3%), (%4%), %5%:%6%\"") %
                 this->iface().broadcast.data() % std::to_string(port) % gai_strerror(rc) % __func__ % __FILE__ %
                 __LINE__)
                    .str())

      if constexpr (std::is_same_v<RetType, int32_t>)
        return rc;
      else
        return {rc, sockaddr_inet_t()};
    }

  sendto:
    if ((rc = ::sendto(sock_fd_, msg, size, 0u, dgram_addrinfo->ai_addr, dgram_addrinfo->ai_addrlen)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
          ::freeaddrinfo(dgram_addrinfo);
          DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        } else {
          ::freeaddrinfo(dgram_addrinfo);
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        }
      } else {
        FD_ZERO(&write_fd_set);
        FD_SET(sock_fd_, &write_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
          ::freeaddrinfo(dgram_addrinfo);
          DEBUG_LOG((boost::format("Send timeout of select() error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) %
                     __func__ % __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        } else
          goto sendto;
      }
    } else if (!rc) {
      ::freeaddrinfo(dgram_addrinfo);
      DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (std::is_same_v<RetType, int32_t>)
        return rc;
      else
        return {rc, sockaddr_inet_t()};
    } else {
      if constexpr (sb == send_behavior_t::HOOK_ON) {

        sockaddr_inet_t to;
        std::memcpy(&to, reinterpret_cast<sockaddr_inet_t *>(dgram_addrinfo->ai_addr), dgram_addrinfo->ai_addrlen);
        ::freeaddrinfo(dgram_addrinfo);
        void *data = std::calloc(size, sizeof(char));
        std::memcpy(data, msg, size);
        std::thread([this, to, size, data]() -> void {
          this->on_send()(to, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                          this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();

        return rc;
      } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

        sockaddr_inet_t to;
        std::memcpy(&to, reinterpret_cast<sockaddr_inet_t *>(dgram_addrinfo->ai_addr), dgram_addrinfo->ai_addrlen);
        ::freeaddrinfo(dgram_addrinfo);
        return {rc, std::move(to)};
      }
    }
  }

  template <send_behavior_t sb = send_behavior_t::HOOK_ON, udp_sock_t sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_t::HOOK_OFF, std::pair<int32_t, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || sc == udp_sock_t::CLIENT_UNICAST ||
                              sc == udp_sock_t::SERVER_MULTICAST || sc == udp_sock_t::CLIENT_MULTICAST,
                          RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const noexcept {
    int32_t rc;
    struct addrinfo *dgram_addrinfo, hints;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_t::send_timeout() / 1000, 0u};

    std::memset(&hints, 0x0, sizeof(hints));
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_protocol = protocol;

    if ((rc = ::getaddrinfo(addr.c_str(), std::to_string(port).c_str(), &hints, &dgram_addrinfo)) != 0 ||
        dgram_addrinfo == nullptr) {
      DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2% (errno = %3%), (%4%), %5%:%6%\"") % addr %
                 std::to_string(port) % ::gai_strerror(rc) % __func__ % __FILE__ % __LINE__)
                    .str());
      if constexpr (std::is_same_v<RetType, int32_t>)
        return rc;
      else
        return {rc, sockaddr_inet_t()};
    }

  sendto:
    if ((rc = ::sendto(sock_fd_, msg, size, 0, dgram_addrinfo->ai_addr, dgram_addrinfo->ai_addrlen)) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
          DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        } else {
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        }
      } else {
        FD_ZERO(&write_fd_set);
        FD_SET(sock_fd_, &write_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
          ::freeaddrinfo(dgram_addrinfo);
          DEBUG_LOG((boost::format("Send timeout of select() error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) %
                     __func__ % __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {rc, sockaddr_inet_t()};
        } else
          goto sendto;
      }
    } else if (!rc) {
      DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (std::is_same_v<RetType, int32_t>)
        return rc;
      else
        return {rc, sockaddr_inet_t()};
    } else {
      if constexpr (sb == send_behavior_t::HOOK_ON) {

        sockaddr_inet_t to;
        std::memcpy(&to, reinterpret_cast<sockaddr_inet_t *>(dgram_addrinfo->ai_addr), dgram_addrinfo->ai_addrlen);
        ::freeaddrinfo(dgram_addrinfo);
        void *data = std::calloc(size, sizeof(char));
        std::memcpy(data, msg, size);
        std::thread([this, to, data, size]() -> void {
          this->on_send()(to, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                          this);
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
      } else if constexpr (sb == send_behavior_t::HOOK_OFF) {

        sockaddr_inet_t to;
        std::memcpy(&to, reinterpret_cast<sockaddr_inet_t *>(dgram_addrinfo->ai_addr), dgram_addrinfo->ai_addrlen);
        ::freeaddrinfo(dgram_addrinfo);
        return {rc, std::move(to)};
      }
    }
  }

  template <recv_behavior_t rb = recv_behavior_t::HOOK,
            typename RetType = std::conditional_t<
                rb == recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  RetType recv() const noexcept {
    int32_t recvd_size = 0, num_ready, rc, recvd;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_t::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>> ret;
    void *data;

    if ((rc = ::epoll_wait(epfd_, events_, this->epoll_max_events(), base_t::receive_timeout())) < 0) {
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
        sockaddr_inet_t from;
        socklen_t fromlen = sizeof(from);
        int32_t bytes_pending;

        if ((rc = ::ioctl(sock_fd_, FIONREAD, &bytes_pending)) < 0) {
          DEBUG_LOG((boost::format("IOctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (std::is_same_v<RetType, int32_t>)
            return rc;
          else
            return {{rc, nullptr, sockaddr_inet_t()}};
        }

        data = std::malloc(bytes_pending + 1u);
      recv:
        if ((rc = ::recvfrom(sock_fd_, data, bytes_pending, 0u, reinterpret_cast<struct sockaddr *>(&from), &fromlen)) <
            0) {
          if (errno != EAGAIN) {
            std::free(data);
            DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                       __FILE__ % __LINE__)
                          .str());
            if constexpr (std::is_same_v<RetType, int32_t>)
              return rc;
            else
              return {{rc, nullptr, sockaddr_inet_t()}};
          } else {
            FD_ZERO(&read_fd_set);
            FD_SET(sock_fd_, &read_fd_set);

            if ((rc = ::select(sock_fd_ + 1, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
              DEBUG_LOG((boost::format("Send timeout of select() error (errno = %1%), (%2%), %3%:%4%") %
                         strerror(errno) % __func__ % __FILE__ % __LINE__)
                            .str());
              if constexpr (std::is_same_v<RetType, int32_t>)
                return rc;
              else
                return {{rc, nullptr, sockaddr_inet_t()}};

            } else
              goto recv;
          }
        } else if (rc) {
          recvd = rc;
          *(reinterpret_cast<char *>(data) + recvd) = '\0';
          if constexpr (rb == recv_behavior_t::HOOK) {
            std::thread([this, from, data, size = recvd]() -> void {
              this->on_receive()(from, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                                 size, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
          } else if constexpr (rb == recv_behavior_t::RET || rb == recv_behavior_t::HOOK_RET) {

            ret.push_back(
                std::make_tuple(recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                                std::move(from)));

            if constexpr (rb == recv_behavior_t::HOOK_RET) {
              std::thread([this, peer = std::get<2u>(ret.back()), data = std::get<1u>(ret.back()),
                           size = std::get<0u>(ret.back())]() -> void {
                this->on_receive()(peer, data, size, this);
                {
                  std::unique_lock<std::mutex> lock(this->mtx());
                  std::notify_all_at_thread_exit(this->cv(), std::move(lock));
                }
              }).detach();
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

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || sc == udp_sock_t::SERVER_MULTICAST ||
                              sc == udp_sock_t::SERVER_BROADCAST,
                          RetType>::type
  start(uint64_t duration_ms = 0) const noexcept {
    int32_t rc;
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

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || sc == udp_sock_t::SERVER_MULTICAST ||
                              (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST),
                          RetType>::type
  stop() noexcept {
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
    clear_<socket_class>();
    clear_hooks_();
  }

  virtual ~network_udp_socket_impl() noexcept {
    reset();
    clear_epoll_();
  }

protected:
  const int32_t &fd__() const noexcept { return sock_fd_; }
  std::atomic_bool &listen_enabled__() const noexcept { return listen_enabled_; }
  std::atomic<state_t> &state__() const noexcept { return state_; }
  std::thread &listen_thread__() const noexcept { return listen_thread_; }

private:
  int32_t sock_fd_;
  int32_t epfd_;
  struct epoll_event *events_;
  mutable std::thread listen_thread_;

  mutable std::atomic_bool listen_enabled_;
  mutable std::mutex mtx_;
  mutable std::atomic<state_t> state_;

  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_t *)> on_receive_;
  const hook_t<void(sockaddr_inet_t, std::shared_ptr<void>, size_t, const this_t *)> on_send_;

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || sc == udp_sock_t::CLIENT_UNICAST, RetType>::type
  setup_(uint16_t port = 0u) noexcept {
    struct addrinfo *addr_info, hints;
    int32_t rc, trueflag = 1;
    const char *addr_str;

    if constexpr (sc == udp_sock_t::SERVER_UNICAST) {
      std::memset(&hints, 0x0, sizeof(hints));
      hints.ai_family = family;
      hints.ai_socktype = socktype;
      hints.ai_protocol = protocol;
      addr_str = this->iface().host_addr.data();

      if ((rc = ::getaddrinfo(addr_str, std::to_string(port).c_str(), &hints, &addr_info)) != 0 ||
          addr_info == nullptr) {
        DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2% (errno : %3%), (%4%), %5%:%6%\"") % addr_str %
                   std::to_string(port) % gai_strerror(rc) % __func__ % __FILE__ % __LINE__)
                      .str());
        return rc;
      }

      if constexpr (is_ipv6)
        reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_scope_id = this->iface().scopeid;
    }

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEPORT, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if constexpr (sc == udp_sock_t::SERVER_UNICAST) {
      bind_(addr_info);
      ::freeaddrinfo(addr_info);
    }

    return rc;
  }

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<!is_ipv6 && (sc == udp_sock_t::SERVER_BROADCAST || sc == udp_sock_t::CLIENT_BROADCAST),
                          RetType>::type
  setup_(uint16_t port = 0u) noexcept {
    struct addrinfo *addr_info, hints;
    int32_t rc, trueflag = 1;

    if constexpr (sc == udp_sock_t::SERVER_BROADCAST) {
      std::memset(&hints, 0x0, sizeof(hints));
      hints.ai_family = family;
      hints.ai_socktype = socktype;
      hints.ai_protocol = protocol;

      if ((rc = ::getaddrinfo(this->iface().broadcast.data(), std::to_string(port).c_str(), &hints, &addr_info)) != 0 ||
          addr_info == nullptr) {
        DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2% (errno : %3%), (%4%), %5%:%6%\"") %
                   this->iface().broadcast.data() % std::to_string(port) % gai_strerror(rc) % __func__ % __FILE__ %
                   __LINE__)
                      .str());
        return rc;
      }

      if constexpr (is_ipv6)
        reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_scope_id = this->iface().scopeid;
    }

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEPORT, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_BROADCAST, &trueflag, sizeof(trueflag))) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if constexpr (sc == udp_sock_t::SERVER_BROADCAST) {

      bind_(addr_info);
      ::freeaddrinfo(addr_info);
    }

    return rc;
  }

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_MULTICAST || sc == udp_sock_t::CLIENT_MULTICAST, RetType>::type
  setup_(const std::string &multicast_group_addr, uint16_t port) noexcept {
    using inet_mreq_t = std::conditional_t<is_ipv6, struct ipv6_mreq, struct ip_mreq>;

    struct addrinfo *addr_info, hints;
    int32_t rc, trueflag = 1;
    inet_addr_t *p_mcast_group_addr, p_mcast_req_multiaddr;
    inet_mreq_t mcast_req;

    std::memset(&hints, 0x0, sizeof(hints));
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_protocol = protocol;

    if ((rc = ::getaddrinfo(multicast_group_addr.c_str(), std::to_string(port).c_str(), &hints, &addr_info)) != 0 ||
        addr_info == nullptr) {
      DEBUG_LOG((boost::format("Invalid address or port: \"%1%\",\"%2% (errno : %3%), (%4%), %5%:%6%\"") %
                 multicast_group_addr % std::to_string(port) % gai_strerror(rc) % __func__ % __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if constexpr (is_ipv6)
      reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_scope_id = this->iface().scopeid;

    if ((rc = open_()) < 0) {
      DEBUG_LOG((boost::format("Couldn't open socket (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str())
      return rc;
    }

    if ((rc = ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEPORT, &trueflag, sizeof trueflag)) < 0) {
      DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    if constexpr (is_ipv6) {
      p_mcast_group_addr = &reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin6_addr;
      std::memcpy(&mcast_req.ipv6mr_multiaddr, p_mcast_group_addr, sizeof(mcast_req.ipv6mr_multiaddr));
      mcast_req.ipv6mr_interface = 0u;

    } else {
      p_mcast_group_addr = &reinterpret_cast<sockaddr_inet_t *>(addr_info->ai_addr)->sin_addr;
      std::memcpy(&mcast_req.imr_multiaddr, p_mcast_group_addr, sizeof(mcast_req.imr_multiaddr));
      mcast_req.imr_interface.s_addr = ::htonl(INADDR_ANY);
    }

    if constexpr (is_ipv6) {
      if ((rc = ::setsockopt(sock_fd_, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, reinterpret_cast<char *>(&mcast_req),
                             sizeof(mcast_req))) != 0) {
        DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }
    } else {
      if ((rc = ::setsockopt(sock_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<char *>(&mcast_req),
                             sizeof(mcast_req))) != 0) {
        DEBUG_LOG((boost::format("Setsockopt error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }
    }

    if constexpr (sc == udp_sock_t::SERVER_MULTICAST) {
      bind_(addr_info);
    }

    ::freeaddrinfo(addr_info);
    return rc;
  }

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST) ||
                              sc == udp_sock_t::SERVER_MULTICAST,
                          RetType>::type
  clear_() noexcept {
    int32_t rc;
    if ((rc = stop()) < 0) {
      DEBUG_LOG((boost::format("Stop() socket error (errno = %1%),(%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
    }

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    ::close(sock_fd_);
    return rc;
  }

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::CLIENT_UNICAST || (!is_ipv6 && sc == udp_sock_t::CLIENT_BROADCAST) ||
                              sc == udp_sock_t::CLIENT_MULTICAST,
                          RetType>::type
  clear_() noexcept {
    int32_t rc;
    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    ::close(sock_fd_);
    return rc;
  }

  void clear_hooks_() noexcept {
    this->on_receive().clear();
    this->on_send().clear();
  }

  void clear_epoll_() noexcept {
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
  }

  template <udp_sock_t sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST) ||
                              sc == udp_sock_t::SERVER_MULTICAST,
                          RetType>::type
  bind_(const struct addrinfo *addr_info) noexcept {
    int32_t rc;
    if ((rc = ::bind(sock_fd_, addr_info->ai_addr, addr_info->ai_addrlen)) != 0) {
      clear_<socket_class>();
      DEBUG_LOG((boost::format("Could not bind UDP socket (errno = %1%), (%2%), %3%:%4%\"") % strerror(rc) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  int32_t open_() noexcept {
    int32_t rc;
    struct epoll_event event;

    if ((rc = ::socket(family, socktype | SOCK_CLOEXEC | SOCK_NONBLOCK, protocol)) < 0) {
      clear_<socket_class>();
      DEBUG_LOG((boost::format("Could not create socket, (%1%), %2%:%3%") % __func__ % __FILE__ % __LINE__).str());
      return rc;
    }

    sock_fd_ = rc;
    std::memset(&event, 0x0, sizeof(event));
    event.events = epollevents;
    event.data.fd = sock_fd_;

    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_ADD, sock_fd_, &event)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <udp_sock_t sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_t::SERVER_UNICAST || (!is_ipv6 && sc == udp_sock_t::SERVER_BROADCAST) ||
                              sc == udp_sock_t::SERVER_MULTICAST,
                          RetType>::type
  listen_() const noexcept {
    listen_enabled_ = true;
    state_ = state_t::RUNNING;

    while (listen_enabled_)
      static_cast<void>(recv());
    state_ = state_t::STOPPED;
  }
};

template <udp_sock_t sc> struct network_udp_socket_ipv4 : network_udp_socket_impl<AF_INET, sc> {
  using network_udp_socket_impl<AF_INET, sc>::network_udp_socket_impl;
};

template <udp_sock_t sc> struct network_udp_socket_ipv6 : network_udp_socket_impl<AF_INET6, sc> {
  using network_udp_socket_impl<AF_INET6, sc>::network_udp_socket_impl;
};

#endif /* NETWORK_UDP_SOCK_HPP */
