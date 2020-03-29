#ifndef DOMAIN_UDP_SOCK_HPP
#define DOMAIN_UDP_SOCK_HPP

#include "base_socket.hpp"
#include "debug.hpp"
#include "udp_sock_type.hpp"

#include <errno.h>
#include <future>
#include <thread>

template <uint32_t family, udp_sock_type_e socket_class, bool multithread>
struct domain_udp_socket_impl_s : public base_sock_s<family, SOCK_DGRAM, IPPROTO_UDP, multithread> {
public:
  static_assert(socket_class != udp_sock_type_e::CLIENT_BROADCAST &&
                    socket_class != udp_sock_type_e::CLIENT_MULTICAST &&
                    socket_class != udp_sock_type_e::SERVER_BROADCAST &&
                    socket_class != udp_sock_type_e::SERVER_MULTICAST,
                "Only unicast is allowed in domain network");

  static constexpr int32_t socktype = SOCK_DGRAM;
  static constexpr int32_t protocol = IPPROTO_UDP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_e : int32_t { RUNNING, STOPPED };
  enum struct recv_behavior_e : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_e : uint32_t { HOOK_ON, HOOK_OFF };

  using this_e = domain_udp_socket_impl_s<family, socket_class, multithread>;
  using base_e = base_sock_s<family, socktype, protocol, multithread>;

  template <udp_sock_type_e sc = socket_class>
  explicit domain_udp_socket_impl_s(
      const std::string &path,
      typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, udp_sock_type_e>::type * = nullptr)
      : base_e(path), epfd_(::epoll_create1(EPOLL_CLOEXEC)), state_(state_e::STOPPED),
        events_(reinterpret_cast<struct epoll_event *>(
            std::malloc(this->epoll_max_events() * sizeof(struct epoll_event)))) {}

  template <udp_sock_type_e sc = socket_class>
  explicit domain_udp_socket_impl_s(
      typename std::enable_if<sc == udp_sock_type_e::CLIENT_UNICAST, udp_sock_type_e>::type * = nullptr)
      : base_e(), epfd_(::epoll_create1(EPOLL_CLOEXEC)), events_(reinterpret_cast<struct epoll_event *>(std::malloc(
                                                             this->epoll_max_events() * sizeof(struct epoll_event)))) {
    setup_();
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type setup() {
    setup_();
  }

  const auto &on_receive() const { return on_receive_; }
  const auto &on_send() const { return on_send_; }

  void stop_threads() const { return this->base_e::stop_threads(); }
  template <udp_sock_type_e sc = socket_class, typename RetType = bool>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type running() const {
    return state_ == state_e::RUNNING;
  }

  template <send_behavior_e sb = send_behavior_e::HOOK_ON, udp_sock_type_e sc = socket_class,
            typename RetType = std::conditional_t<
                sb == send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == send_behavior_e::HOOK_OFF, std::pair<int32_t, struct sockaddr_un>, void>>>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST || sc == udp_sock_type_e::CLIENT_UNICAST, RetType>::type
  send(const std::string &path, const void *const msg, size_t size) const {
    struct sockaddr_un addr;
    int32_t rc;
    fd_set write_fd_set;
    struct timeval write_timeout = {base_e::send_timeout() / 1000, 0u};

    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1u);
  sendto:
    if ((rc = ::sendto(sock_fd_, msg, size, 0, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr))) < 0) {
      if (errno != EAGAIN) {
        if (errno == ECONNREFUSED || errno == EHOSTUNREACH || errno == ENETUNREACH || errno == ECONNRESET ||
            errno == ECONNABORTED || errno == EPIPE) {
          DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());

          if constexpr (sb == send_behavior_e::HOOK_ON) {

            return rc;
          } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

            return {rc, addr};
          }
        } else {
          DEBUG_LOG((boost::format("Sendto error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (sb == send_behavior_e::HOOK_ON) {

            return rc;
          } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

            return {rc, addr};
          }
        }
      } else {
        FD_ZERO(&write_fd_set);
        FD_SET(sock_fd_, &write_fd_set);

        if ((rc = ::select(sock_fd_ + 1, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
          DEBUG_LOG((boost::format("Send timeout of select() error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) %
                     __func__ % __FILE__ % __LINE__)
                        .str());
          if constexpr (sb == send_behavior_e::HOOK_ON) {

            return rc;
          } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

            return {rc, addr};
          }
        } else
          goto sendto;
      }
    } else if (!rc) {
      DEBUG_LOG((boost::format("Network error (errno = %1%), (%2%), %3%:%4%\r\n") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (sb == send_behavior_e::HOOK_ON) {

        return rc;
      } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

        return {rc, addr};
      }
    } else {
      if constexpr (sb == send_behavior_e::HOOK_ON) {

        struct sockaddr_un to;
        std::memcpy(&to, &addr, sizeof(addr));
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
      } else if constexpr (sb == send_behavior_e::HOOK_OFF) {

        struct sockaddr_un to;
        std::memcpy(&to, &addr, sizeof(addr));
        return {rc, std::move(to)};
      }
    }
  }

  template <recv_behavior_e rb = recv_behavior_e::HOOK,
            typename RetType = std::conditional_t<
                rb == recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>>, void>>>
  RetType recv() const {
    int32_t recvd_size = 0, num_ready, rc;
    fd_set read_fd_set;
    struct timeval read_timeout = {base_e::receive_timeout() / 1000, 0u};
    std::vector<std::tuple<int32_t, std::shared_ptr<void>, struct sockaddr_un>> ret;

    if ((rc = ::epoll_wait(epfd_, events_, this->epoll_max_events(), base_e::receive_timeout())) < 0) {
      DEBUG_LOG((boost::format("Epoll wait error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      if constexpr (rb == recv_behavior_e::HOOK) {

        return rc;
      } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

        return {{rc, nullptr, sockaddr_un()}};
      }
    }

    num_ready = rc;
    for (uint32_t i = 0u; i < num_ready; i++) {
      if ((events_[i].data.fd == sock_fd_) && ((events_[i].events & EPOLLIN) == EPOLLIN)) {
        struct sockaddr_un from;
        socklen_t fromlen = sizeof(from);
        int32_t bytes_pending, rc;

        if ((rc = ::ioctl(sock_fd_, FIONREAD, &bytes_pending)) < 0) {
          DEBUG_LOG((boost::format("IOctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                     __FILE__ % __LINE__)
                        .str());
          if constexpr (rb == recv_behavior_e::HOOK) {

            return rc;
          } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

            return {{rc, nullptr, sockaddr_un()}};
          }
        }

        void *data = std::malloc(bytes_pending + 1u);
        int32_t recvd;
      recv:
        if ((recvd = ::recvfrom(sock_fd_, data, bytes_pending, 0u, reinterpret_cast<struct sockaddr *>(&from),
                                &fromlen)) < 0) {
          if (errno != EAGAIN) {
            std::free(data);
            DEBUG_LOG((boost::format("Receiveing error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                       __FILE__ % __LINE__)
                          .str());
            if constexpr (rb == recv_behavior_e::HOOK) {

              return rc;
            } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

              return {{rc, nullptr, sockaddr_un()}};
            }
          } else {
            FD_ZERO(&read_fd_set);
            FD_SET(sock_fd_, &read_fd_set);

            if ((rc = ::select(sock_fd_ + 1, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
              DEBUG_LOG((boost::format("Send timeout of select() error (errno = %1%), (%2%), %3%:%4%") %
                         strerror(errno) % __func__ % __FILE__ % __LINE__)
                            .str());

              if constexpr (rb == recv_behavior_e::HOOK) {

                return rc;
              } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

                return {{rc, nullptr, sockaddr_un()}};
              }
            } else
              goto recv;
          }
        } else if (recvd) {
          *(reinterpret_cast<char *>(data) + recvd) = '\0';
          if constexpr (rb == recv_behavior_e::HOOK) {
            std::thread([this, from, data, size = recvd]() -> void {
              this->on_receive()(from, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                                 size, this);
              {
                std::unique_lock<std::mutex> lock(this->mtx());
                std::notify_all_at_thread_exit(this->cv(), std::move(lock));
              }
            }).detach();
          } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

            ret.push_back(
                std::make_tuple(recvd, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }),
                                std::move(from)));

            if constexpr (rb == recv_behavior_e::HOOK_RET) {
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

    if constexpr (rb == recv_behavior_e::HOOK) {

      return recvd_size;
    } else if constexpr (rb == recv_behavior_e::RET || rb == recv_behavior_e::HOOK_RET) {

      return std::move(ret);
    }
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type start(uint64_t duration_ms = 0) const {
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

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type stop() {
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

  void reset() {
    clear_<socket_class>();
    clear_hooks_();
  }

  virtual ~domain_udp_socket_impl_s() {
    reset();
    clear_epoll_();
  }

protected:
  const int32_t &fd__() const { return sock_fd_; }
  std::atomic_bool &listen_enabled__() const { return listen_enabled_; }
  std::atomic<state_e> &state__() const { return state_; }
  std::thread &listen_thread__() const { return listen_thread_; }

private:
  int32_t sock_fd_;
  int32_t epfd_;
  struct epoll_event *events_;
  mutable std::thread listen_thread_;

  mutable std::atomic_bool listen_enabled_;
  mutable std::mutex mtx_;
  mutable std::atomic<state_e> state_;
  struct addrinfo *target;

  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_e *)> on_receive_;
  const hook_t<void(struct sockaddr_un, std::shared_ptr<void>, size_t, const this_e *)> on_send_;

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type setup_() {
    struct sockaddr_un path;
    std::strncpy(path.sun_path, this->path().c_str(), this->path().size());
    int32_t rc, trueflag = 1;

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

    if ((rc = bind_(&path)) < 0) {
      DEBUG_LOG((boost::format("Couldn't bind socket (errno : %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::CLIENT_UNICAST, RetType>::type setup_() {
    int32_t rc, trueflag = 1;

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

    return rc;
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type clear_() {
    int32_t rc;
    stop();
    if ((rc = ::epoll_ctl(epfd_, EPOLL_CTL_DEL, sock_fd_, nullptr)) < 0u) {
      DEBUG_LOG((boost::format("Epoll ctl error (errno = %1%), (%2%), %3%:%4%") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    ::close(sock_fd_);
    return rc;
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::CLIENT_UNICAST, RetType>::type clear_() {
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

  void clear_hooks_() {
    this->on_receive().clear();
    this->on_send().clear();
  }

  void clear_epoll_() {
    ::close(epfd_);
    std::free(events_);
    events_ = nullptr;
  }

  template <udp_sock_type_e sc = socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type bind_(const struct sockaddr_un *path) {
    int32_t rc;

    if ((rc = ::bind(sock_fd_, reinterpret_cast<const struct sockaddr *>(path), sizeof(path->sun_path))) != 0) {
      clear_<socket_class>();
      DEBUG_LOG((boost::format("Can't bind UDP socket (errno = %1%), (%2%), %3%:%4%\"") % strerror(rc) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    return rc;
  }

  int32_t open_() {
    int32_t rc;
    if ((rc = ::socket(family, socktype | SOCK_CLOEXEC | SOCK_NONBLOCK, 0u)) < 0) {
      clear_<socket_class>();
      DEBUG_LOG((boost::format("Could not create socket, (%1%), %2%:%3%, (errno = %4%)") % __func__ % __FILE__ %
                 __LINE__ % ::strerror(errno))
                    .str());
      return rc;
    }

    sock_fd_ = rc;
    struct epoll_event event;
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

  template <udp_sock_type_e sc = socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_type_e::SERVER_UNICAST, RetType>::type listen_() const {
    listen_enabled_ = true;
    state_ = state_e::RUNNING;

    while (listen_enabled_)
      static_cast<void>(recv());
    state_ = state_e::STOPPED;
  }
};

template <udp_sock_type_e sc, bool multithread>
struct domain_udp_socket_s : domain_udp_socket_impl_s<AF_UNIX, sc, multithread> {
  using domain_udp_socket_impl_s<AF_UNIX, sc, multithread>::domain_udp_socket_impl_s;
};

#endif /* DOMAIN_UDP_SOCK_HPP */
