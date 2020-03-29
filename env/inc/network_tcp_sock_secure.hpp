#ifndef NETWORK_TCP_SOCK_SECURE_HPP
#define NETWORK_TCP_SOCK_SECURE_HPP

#include "network_tcp_sock.hpp"
#include "secure_layer.hpp"
#include "tcp_sock_secure_type.hpp"

template <uint32_t family, tcp_sock_secure_type_e secure_socket_class, bool multithread>
struct network_tcp_socket_secure_impl_s
    : protected network_tcp_socket_impl_s<
          family, static_cast<tcp_sock_type_e>(static_cast<uint32_t>(secure_socket_class) % 2u), multithread> {
public:
  static constexpr uint32_t aes_key_size_bits = 256u;
  static constexpr uint32_t rsa_key_size_bits = 2048u;

  using base_s =
      network_tcp_socket_impl_s<family, static_cast<tcp_sock_type_e>(static_cast<uint32_t>(secure_socket_class) % 2u),
                                multithread>;
  using this_s = network_tcp_socket_secure_impl_s<family, secure_socket_class, multithread>;

  static constexpr bool is_ipv6 = base_s::is_ipv6;
  static constexpr int32_t addrlen = base_s::addrlen;

  using sockaddr_inet_t = typename base_s::sockaddr_inet_t;
  using inet_addr_t = typename base_s::inet_addr_t;
  using connected_peer_info_t = typename base_s::connected_peer_info_t;

  template <tcp_sock_secure_type_e sc = secure_socket_class>
  explicit network_tcp_socket_secure_impl_s(
      const std::string &iface, const std::string &ca_cert_file, const std::string &ca_priv_key_file,
      typename tls_sl_t<sc, rsa_key_size_bits>::x509_cert_info_t cert_info, uint64_t exp_time,
      typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS ||
                                  sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS,
                              tcp_sock_secure_type_e>::type * = nullptr)
      : base_s(iface), sl_(ca_cert_file, ca_priv_key_file, cert_info, exp_time) {

    /* Add hook to base class for clearing credentials in secure layer after disconnect case */
    if constexpr (sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS)
      this->set_on_disconnect_internal_hook__([this](int32_t fd) -> void { sl_.clear_peer_creds(fd); });
  }

  virtual ~network_tcp_socket_secure_impl_s() = default;

  const auto &on_connect() const { return this->base_s::on_connect(); }
  const auto &on_disconnect() const { return this->base_s::on_disconnect(); }
  const auto &on_receive() const { return this->base_s::on_receive(); }
  const auto &on_send() const { return this->base_s::on_send(); }

  void stop_threads() { return this->base_s::stop_threads(); }
  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type setup(uint16_t port) {
    return this->base_s::setup(port);
  }

  template <typename base_s::connect_behavior_e cb = base_s::connect_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  connect(const std::string &addr, uint16_t port) {
    int32_t rc;

    /* Connect underlaying client socket */
    if ((rc = this->base_s::template connect<base_s::connect_behavior_e::HOOK_OFF>(addr, port))) {
      return rc;
    }

    this->state__() = base_s::state_e::CONNECTING;

    /* Do TLS connection */
    if ((rc = sl_.connect(this->fd__()))) {
      if constexpr (cb == base_s::connect_behavior_e::HOOK_ON) {
        std::thread([this, peer = this->connected__()]() -> void {
          this->on_disconnect()(peer, static_cast<const base_s *>(this));
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
      }

      this->state__() = base_s::state_e::DISCONNECTED;
      return -1;
    } else {
      if constexpr (cb == base_s::connect_behavior_e::HOOK_ON) {
        std::thread([this, peer = this->connected__()]() -> void {
          this->on_connect()(peer, static_cast<const base_s *>(this));
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
      }

      this->state__() = base_s::state_e::CONNECTED;
      return 0;
    }
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type disconnect() {
    /* Disconnect from pure socket */
    this->base_s::disconnect();

    /* Clear TLS credentials */
    sl_.clear(this->fd__());
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  disconnect(const std::string &addr, uint16_t srv) {
    /* Disconnect from pure socket */
    this->base_s::disconnect(addr, srv);

    /* Clear TLS credentials */
    sl_.clear_peer_creds(addr, srv);
  }

  template <typename base_s::send_behavior_e sb = base_s::send_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_s::send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == base_s::send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const {
    return send_<sb>(addr, port, msg, size);
  }

  template <typename base_s::send_behavior_e sb = base_s::send_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_s::send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == base_s::send_behavior_e::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  send(const void *const msg, size_t size) const {
    return send_<sb>(msg, size);
  }

  template <typename base_s::recv_behavior_e rb = base_s::recv_behavior_e::HOOK,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_s::recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type recv() const {
    return recv_<rb>();
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type running() const {
    return this->base_s::running();
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type connected() const {
    return this->base_s::connected();
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type connecting() const {
    return this->state__() == base_s::state_e::CONNECTING;
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(const std::string &addr, uint16_t srv) const {
    int32_t rc = this->get_connected_peer__(addr, srv, nullptr);
    return rc > 0;
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(int32_t fd) const {
    sockaddr_inet_t *addr;
    int32_t rc = this->get_connected_peer__(fd, &addr);
    return addr != nullptr;
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash, uint16_t srv) const {
    return this->base_s::is_peer_connected(hash, srv);
  }

  int32_t peer_fd(const std::string &addr, uint16_t srv) const { return this->base_s::peer_fd(addr, srv, nullptr); }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  start(uint64_t duration_ms = 0) {
    bool nonblock = duration_ms == 0;
    if (nonblock) {
      this->listen_thread__() = std::thread([this]() -> void { listen_(); });

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
          duration_ms, &this->listen_enabled__())
          .detach();
      listen_();
    }
  }

  template <tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type stop() {
    int32_t rc;
    if (this->listen_enabled__()) {
      this->listen_enabled__() = false;
      rc = 0;
    } else {
      rc = -1;
      goto exit;
    }

    if (this->listen_thread__().joinable()) {
      this->listen_thread__().join();
      rc = 0;
    } else {
      rc = -1;
      goto exit;
    }

  exit:
    return rc;
  }

  void reset() { this->base_s::reset(); }
  const auto &iface() const { return this->base_s::iface(); }

private:
  mutable secure_layer_t<(is_secure_tcp_aes_type(secure_socket_class))
                             ? aes_key_size_bits
                             : (is_secure_tcp_tls_x509_type(secure_socket_class)) ? rsa_key_size_bits : 0u,
                         tcp_sock_secure_type_e, secure_socket_class>
      sl_;

  template <typename base_s::send_behavior_e sb = base_s::send_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_s::send_behavior_e::HOOK_ON, int32_t,
                std::conditional_t<sb == base_s::send_behavior_e::HOOK_OFF,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  RetType send_(const void *const msg, size_t size) const {
    return this->base_s::template send<sb>(
        msg, size, [this](int32_t fd, const void *buffer, size_t size, int32_t flags) -> int32_t {
          static_cast<void>(flags);
          return sl_.send(fd, buffer, size);
        });
  }

  template <typename base_s::recv_behavior_e rb = base_s::recv_behavior_e::HOOK,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_s::recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type recv_() const {
    return this->base_s::template recv<rb>([this](int32_t fd, void *buffer, size_t size, int32_t flags) -> int32_t {
      static_cast<void>(flags);
      return sl_.recv(fd, buffer, size);
    });
  }

  template <typename base_s::recv_behavior_e rb = base_s::recv_behavior_e::HOOK,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_s::recv_behavior_e::HOOK, int32_t,
                std::conditional_t<rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  handle_incoming_data_(int32_t fd) {
    return this->template handle_incoming_data__<rb>(
        fd, [this](int32_t fd, void *buffer, size_t size, int32_t flags) -> int32_t {
          static_cast<void>(flags);
          return sl_.recv(fd, buffer, size);
        });
  }

  template <typename base_s::connect_behavior_e cb = base_s::connect_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class,
            typename RetType = std::conditional_t<cb == base_s::connect_behavior_e::HOOK_ON, int32_t,
                                                  std::pair<typename base_s::sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  handle_incoming_peer_() {
    int32_t rc;

    /* Underlaying peer handling */
    auto [peer, peer_fd] = this->template handle_incoming_peer__<base_s::connect_behavior_e::HOOK_OFF>();

    this->state__() = base_s::state_e::CONNECTING;
    if (peer_fd <= 0) {
      if constexpr (cb == base_s::connect_behavior_e::HOOK_ON) {

        this->state__() = base_s::state_e::DISCONNECTED;
        return -1;
      } else if constexpr (cb == base_s::connect_behavior_e::HOOK_OFF) {

        this->state__() = base_s::state_e::DISCONNECTED;
        return {sockaddr_inet_t(), -1};
      }
    }

    /* Handle peer on secure layer */
    if ((rc = sl_.register_client(peer_fd))) {
      /*Disconnect*/
      this->disconnect_peer__(peer_fd);
      sl_.clear_peer_creds(peer_fd);

      if constexpr (cb == base_s::connect_behavior_e::HOOK_ON) {
        std::thread([this, peer = peer]() -> void {
          this->on_disconnect()(peer, static_cast<const base_s *>(this));
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();
        this->state__() = base_s::state_e::DISCONNECTED;
        return -1;
      } else if constexpr (cb == base_s::connect_behavior_e::HOOK_OFF) {

        this->state__() = base_s::state_e::DISCONNECTED;
        return {sockaddr_inet_t(), -1};
      }
    }

    if constexpr (cb == base_s::connect_behavior_e::HOOK_ON) {
      std::thread([this, peer = peer]() -> void {
        this->on_connect()(peer, static_cast<const base_s *>(this));
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();
      this->state__() = base_s::state_e::CONNECTED;
      return peer_fd;
    } else if constexpr (cb == base_s::connect_behavior_e::HOOK_OFF) {

      this->state__() = base_s::state_e::CONNECTED;
      return {std::move(peer), peer_fd};
    }
  }

  template <typename base_s::connect_behavior_e cb = base_s::connect_behavior_e::HOOK_ON,
            tcp_sock_secure_type_e sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type listen_() {
    int32_t num_ready, rc;
    this->listen_enabled__() = true;
    if ((rc = ::listen(this->fd__(), SOMAXCONN)) < 0) {
      DEBUG_LOG((boost::format("Start listening error: (errno = %1%), (%2%), %3%:%4%\"") % strerror(errno) % __func__ %
                 __FILE__ % __LINE__)
                    .str());
      return rc;
    }

    this->state__() = base_s::state_e::LISTENING;
    while (this->listen_enabled__()) {
      if ((rc = ::epoll_wait(this->epfd__(), this->events__(), base_s::epoll_max_events(), base_s::accept_timeout())) <
          0u) {
        DEBUG_LOG((boost::format("Epoll wait error (errno = %1%) (%2%), %3%:%4%") % strerror(errno) % __func__ %
                   __FILE__ % __LINE__)
                      .str());
        return rc;
      }

      num_ready = rc;
      for (int32_t i = 0; i < num_ready; i++) {
        if (((this->events__()[i].events & EPOLLIN) == EPOLLIN || (this->events__()[i].events & EPOLLET) == EPOLLET) &&
            (this->events__()[i].data.fd == this->fd__())) {

          static_cast<void>(handle_incoming_peer_());
        } else if (((this->events__()[i].events & EPOLLIN) == EPOLLIN ||
                    (this->events__()[i].events & EPOLLET) == EPOLLET) &&
                   (this->events__()[i].data.fd != this->fd__())) {

          static_cast<void>(handle_incoming_data_(this->events__()[i].data.fd));
        }
      }
    }

    this->state__() = base_s::state_e::STOPPED;
    return 0;
  }
};

template <tcp_sock_secure_type_e sc, bool multithread>
struct network_tcp_socket_secure_ipv4_s : network_tcp_socket_secure_impl_s<AF_INET, sc, multithread> {
  using network_tcp_socket_secure_impl_s<AF_INET, sc, multithread>::network_tcp_socket_secure_impl_s;
};

template <tcp_sock_secure_type_e sc, bool multithread>
struct network_tcp_socket_secure_ipv6_s : network_tcp_socket_secure_impl_s<AF_INET6, sc, multithread> {
  using network_tcp_socket_secure_impl_s<AF_INET6, sc, multithread>::network_tcp_socket_secure_impl_s;
};

#endif /* NETWORK_TCP_SOCK_SECURE_HPP */
