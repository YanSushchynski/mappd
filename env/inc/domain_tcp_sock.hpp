#ifndef DOMAIN_TCP_SOCK_HPP
#define DOMAIN_TCP_SOCK_HPP

#include "base_socket.hpp"
#include "tcp_sock_type.hpp"

#include <errno.h>
#include <thread>

template <tcp_sock_t socket_class>
struct domain_tcp_socket_impl : public base_socket<AF_UNIX, SOCK_STREAM, IPPROTO_TCP, false> {
public:
  static constexpr int32_t family = AF_UNIX;
  static constexpr int32_t socktype = SOCK_STREAM;
  static constexpr int32_t protocol = IPPROTO_TCP;
  static constexpr int32_t epollevents = EPOLLIN | EPOLLET;

  enum struct state_t : int32_t { CONNECTED, DISCONNECTED, LISTENING, CONNECTING, STOPPED };
  enum struct recv_behavior_t : uint32_t { HOOK, RET, HOOK_RET };
  enum struct send_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };
  enum struct connect_behavior_t : uint32_t { HOOK_ON, HOOK_OFF };

  using this_t = domain_tcp_socket_impl<socket_class>;
  using base_t = base_socket<family, socktype, protocol, false>;

  static constexpr bool is_ipv6 = base_t::is_ipv6;
  static constexpr int32_t addrlen = base_t::addrlen;

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
  int32_t setup_(const std::string &path = "") {
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
      rc = connect_<cb>(addr_info);
    } else if constexpr (socket_class == tcp_sock_t::SERVER_UNICAST) {
      rc = bind_(addr_info);
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
};

static int32_t sock_fd, epfd;
static constexpr uint32_t recv_timeout_s = 1u;
static struct epoll_event events[16u]{0x0};

int32_t setup_client(const std::string &addr) {
  int32_t rc, trueflag = 1;
  struct sockaddr_un server_addr;

  epfd = epoll_create1(EPOLL_CLOEXEC);

  std::memset(&server_addr, '\0', sizeof(server_addr));
  server_addr.sun_family = AF_UNIX;
  std::strcpy(server_addr.sun_path, addr.c_str());

  if ((sock_fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0u)) < 0) {
    perror("socket() error\r\n");
    std::exit(1);
  }

  struct epoll_event event;
  std::memset(&event, 0x0, sizeof(event));
  event.events = EPOLLIN | EPOLLET;
  event.data.fd = sock_fd;

  if (::epoll_ctl(epfd, EPOLL_CTL_ADD, sock_fd, &event) < 0u) {
    perror("epoll_ctl() error\r\n");
    std::exit(1);
  }

  struct timeval recv_timeout = {recv_timeout_s, 0u};
  struct timeval send_timeout = {recv_timeout_s, 0u};

  if ((rc = ::setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout))) < 0) {
    perror("setsockopt() error\r\n");
    std::exit(1);
  }

  if ((rc = ::setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout))) < 0) {
    perror("setsockopt() error\r\n");
    std::exit(1);
  }

  std::memset(&event, 0x0, sizeof(event));
  event.events = EPOLLIN | EPOLLET | EPOLLOUT;
  event.data.fd = sock_fd;

  if (::epoll_ctl(epfd, EPOLL_CTL_MOD, sock_fd, &event) < 0u) {
    perror("epoll_ctl() error\r\n");
    std::exit(1);
  }

  if ((rc = ::connect(sock_fd, reinterpret_cast<struct sockaddr *>(&server_addr), sizeof(struct sockaddr_un))) < 0) {
    if (errno != EINPROGRESS) {
      perror("connect() error\r\n");
      std::exit(1);
    }
  }

  int32_t ready;
  if ((ready = ::epoll_wait(epfd, events, 16u, recv_timeout_s)) < 0) {
    perror("epoll_wait() error\r\n");
    std::exit(1);
  }

  for (int32_t i = 0; i < ready; i++) {
    if (events[i].data.fd == sock_fd && (events[i].events & EPOLLOUT) == EPOLLOUT) {
      int32_t error = 0u, rc;
      socklen_t errlen = sizeof(error);

      if ((rc = ::getsockopt(sock_fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&error), &errlen)) < 0) {
        perror("epoll_wait() error\r\n");
        std::exit(1);
      }

      std::memset(&event, 0x0, sizeof(event));
      event.events = EPOLLIN | EPOLLET;
      event.data.fd = sock_fd;

      if (::epoll_ctl(epfd, EPOLL_CTL_MOD, sock_fd, &event) < 0u) {
        perror("epoll_ctl() error\r\n");
        std::exit(1);
      }

      rc = error;
    }
  }

  return rc;
}

void reset_client(){
  if (::epoll_ctl(epfd, EPOLL_CTL_DEL, sock_fd, nullptr) < 0u) {
    perror("epoll_ctl() error\r\n");
    std::exit(1);
  }

  ::close(epfd);
  ::close(sock_fd);
}

int32_t recv(void (*recv_callback)(const void *const, size_t)) {
  int32_t ready, recvd_size = 0;

  if ((ready = ::epoll_wait(epfd, events, 16u, recv_timeout_s * 1000)) < 0) {
    perror("epoll_wait() error\r\n");
    std::exit(1);
  }

  for (uint32_t i = 0u; i < ready; i++) {
    if ((events[i].data.fd == sock_fd) && ((events[i].events & EPOLLIN) == EPOLLIN)) {
      int32_t rc, bytes_pending, recvd;

      if ((rc = ::ioctl(sock_fd, FIONREAD, &bytes_pending)) < 0) {
        perror("ioctl() error\r\n");
        std::exit(1);
      }

      void *data = std::malloc(bytes_pending);
    recv:
      if ((recvd = ::recv(sock_fd, data, bytes_pending, MSG_CMSG_CLOEXEC)) < 0) {
        if (errno != EAGAIN) {
          perror("recv() error\r\n");
          std::exit(1);
        } else
          goto recv;
      } else {

        recvd_size += recvd;
        recv_callback(data, recvd);
        std::free(data);
      }
    }
  }

  return recvd_size;
}

int32_t send(const void *const msg, size_t size, void (*send_callback)(size_t, const void *const, size_t)) {
send:
  int32_t rc;
  if ((rc = ::send(sock_fd, msg, size, MSG_NOSIGNAL)) < 0) {
    if (errno != EAGAIN) {
      perror("send() error\r\n");
      std::exit(1);
    } else
      goto send;
  } else if (!rc) {
    std::printf("Connection refused\r\n");
    std::exit(1);
  } else {
    send_callback(rc, msg, size);
    return rc;
  }
}

void on_receive(const void *const msg, size_t size) {
  char *to_print = reinterpret_cast<char *>(std::malloc(size)), *start = to_print;
  std::memcpy(to_print, msg, size);
  std::printf("Received message! size = %lu, msg = \r\n", size);
  for (char *max = to_print + size; to_print != max; std::printf("%c ", *to_print++))
    ;
  std::printf("\r\n");
  std::free(start);
}

void on_send(size_t snd_size, const void *const msg, size_t size) {
  char *to_print = reinterpret_cast<char *>(std::malloc(size)), *start = to_print;
  std::memcpy(to_print, msg, size);
  std::printf("Sent message! size = %lu, msg = \r\n", size);
  for (char *max = to_print + size; to_print != max; std::printf("%c ", *to_print++))
    ;
  std::printf("\r\n");
  std::free(start);  
}

int32_t main(int32_t argc, char *argv[]) {
  char msg[] = {0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x0, 0x0, 0x0, 'l', 'o', 'c', 'a', 'l', 'h', 'o', 's', 't', 0x0}; // request
  char *service_select = &msg[4u];

  for (uint32_t i = 0x0; i < 0xf; i++, (*service_select)++) {
    setup_client("/var/run/nscd/socket");               // connect to socket
    static_cast<void>(send(msg, sizeof(msg), on_send)); // send request
    static_cast<void>(recv(on_receive));                // receive response
	reset_client();
  }
}

#endif /* DOMAIN_SOCK_HPP */
