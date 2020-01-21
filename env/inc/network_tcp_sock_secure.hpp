#ifndef NETWORK_TCP_SOCK_SECURE_HPP
#define NETWORK_TCP_SOCK_SECURE_HPP

#include "network_tcp_sock.hpp"
#include "secure_layer.hpp"
#include "tcp_sock_secure_type.hpp"

template <uint32_t family, tcp_sock_secure_t secure_socket_class>
struct network_tcp_socket_secure_impl
    : protected network_tcp_socket_impl<family,
                                        static_cast<tcp_sock_t>(static_cast<uint32_t>(secure_socket_class) % 2u)> {
public:
  static constexpr uint32_t aes_key_size_bits = 256u;
  static constexpr uint32_t rsa_key_size_bits = 2048u;

  using base_t =
      network_tcp_socket_impl<family, static_cast<tcp_sock_t>(static_cast<uint32_t>(secure_socket_class) % 2u)>;
  using this_t = network_tcp_socket_secure_impl<family, secure_socket_class>;

  static constexpr bool is_ipv6 = base_t::is_ipv6;
  static constexpr int32_t addrlen = base_t::addrlen;

  using sockaddr_inet_t = typename base_t::sockaddr_inet_t;
  using inet_addr_t = typename base_t::inet_addr_t;
  using connected_peer_info_t = typename base_t::connected_peer_info_t;

  template <tcp_sock_secure_t sc = secure_socket_class>
  explicit network_tcp_socket_secure_impl(
      const std::string &iface, const std::string &ca_cert_file, const std::string &ca_priv_key_file,
      typename tls_sl_t<sc, rsa_key_size_bits>::x509_cert_info_t cert_info, uint64_t exp_time,
      typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS ||
                                  sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS,
                              tcp_sock_secure_t>::type * = nullptr)
      : base_t(iface), sl_(ca_cert_file, ca_priv_key_file, cert_info, exp_time) {

    /* Add hook to base class for clearing credentials in secure layer after disconnect case */
    if constexpr (sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS)
      this->set_on_disconnect_internal_hook__([this](int32_t fd) -> void { sl_.clear_peer_creds(fd); });
  }

  virtual ~network_tcp_socket_secure_impl() = default;

  const auto &on_connect() const { return static_cast<const base_t *>(this)->on_connect(); }
  const auto &on_disconnect() const { return static_cast<const base_t *>(this)->on_disconnect(); }
  const auto &on_receive() const { return static_cast<const base_t *>(this)->on_receive(); }
  const auto &on_send() const { return static_cast<const base_t *>(this)->on_send(); }

  void stop_threads() { return static_cast<typename base_t::base_t *>(this)->stop_tp(); }
  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type setup(uint16_t port) {
    return static_cast<base_t *>(this)->setup(port);
  }

  template <typename base_t::connect_behavior_t cb = base_t::connect_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  connect(const std::string &addr, uint16_t port) {
    int32_t rc;

    /* Connect underlaying client socket */
    if ((rc = static_cast<base_t *>(this)->template connect<base_t::connect_behavior_t::HOOK_OFF>(addr, port))) {
      return rc;
    }

    this->state() = base_t::state_t::CONNECTING;

    /* Do TLS connection */
    if ((rc = sl_.connect(this->fd__()))) {
      if constexpr (cb == base_t::connect_behavior_t::HOOK_ON) {
        this->tp().push([this, peer = this->connected__()]() -> void {
          this->on_disconnect()(peer, static_cast<const base_t *>(this));
        });
      }

      this->state() = base_t::state_t::DISCONNECTED;
      return -1;
    } else {
      if constexpr (cb == base_t::connect_behavior_t::HOOK_ON) {
        this->tp().push([this, peer = this->connected__()]() -> void {
          this->on_connect()(peer, static_cast<const base_t *>(this));
        });
      }

      this->state() = base_t::state_t::CONNECTED;
      return 0;
    }
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type disconnect() {
    /* Disconnect from pure socket */
    static_cast<base_t *>(this)->disconnect();

    /* Clear TLS credentials */
    sl_.clear(this->fd__());
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  disconnect(const std::string &addr, uint16_t srv) {
    /* Disconnect from pure socket */
    static_cast<base_t *>(this)->disconnect(addr, srv);

    /* Clear TLS credentials */
    sl_.clear_peer_creds(addr, srv);
  }

  template <typename base_t::send_behavior_t sb = base_t::send_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_t::send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == base_t::send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const {
    return send_<sb>(addr, port, msg, size);
  }

  template <typename base_t::send_behavior_t sb = base_t::send_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_t::send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == base_t::send_behavior_t::HOOK_OFF, std::pair<sockaddr_inet_t, int32_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type send(const void *const msg,
                                                                                                  size_t size) const {
    return send_<sb>(msg, size);
  }

  template <typename base_t::recv_behavior_t rb = base_t::recv_behavior_t::HOOK,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_t::recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type recv() const {
    return recv_<rb>();
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type running() const {
    return static_cast<const base_t *>(this)->running();
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type connected() const {
    return static_cast<const base_t *>(this)->connected();
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type connecting() const {
    return this->state() == base_t::state_t::CONNECTING;
  }

  std::atomic<typename base_t::state_t> &state() const { return this->state(); }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(const std::string &addr, uint16_t srv) const {
    int32_t rc = this->get_connected_peer__(addr, srv, nullptr);
    return rc > 0;
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(int32_t fd) const {
    sockaddr_inet_t *addr;
    int32_t rc = this->get_connected_peer__(fd, &addr);
    return addr != nullptr;
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  is_peer_connected(const sha256::sha256_hash_type &hash, uint16_t srv) const {
    return static_cast<const base_t *>(this)->is_peer_connected(hash, srv);
  }

  int32_t peer_fd(const std::string &addr, uint16_t srv) const {
    return static_cast<const base_t *>(this)->peer_fd(addr, srv, nullptr);
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  start(uint64_t duration_ms = 0) {
    bool nonblock = duration_ms == 0;
    if (nonblock) {
      this->listen_thread__() = std::thread([this]() -> void { listen_(); });

    } else {
      this->tp().push(
          [this](uint64_t duration_ms, std::atomic_bool *trigger) -> void {
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
            *trigger = false;
          },
          duration_ms, &this->listen_enabled__());
      listen_();
    }
  }

  template <tcp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type stop() {
    this->listen_enabled__() = false;
    if (this->listen_thread__().joinable())
      this->listen_thread__().join();
  }

  void reset() { static_cast<base_t *>(this)->reset(); }
  const auto &iface() const { return static_cast<const base_t *>(this)->iface(); }

private:
  mutable secure_layer_t<(is_secure_tcp_aes_type(secure_socket_class))
                             ? aes_key_size_bits
                             : (is_secure_tcp_tls_x509_type(secure_socket_class)) ? rsa_key_size_bits : 0u,
                         tcp_sock_secure_t, secure_socket_class>
      sl_;

  template <typename base_t::send_behavior_t sb = base_t::send_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                sb == base_t::send_behavior_t::HOOK_ON, int32_t,
                std::conditional_t<sb == base_t::send_behavior_t::HOOK_OFF,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  RetType send_(const void *const msg, size_t size) const {
    return static_cast<const base_t *>(this)->template send<sb>(
        msg, size, [this](int32_t fd, const void *buffer, size_t size, int32_t flags) -> int32_t {
          static_cast<void>(flags);
          return sl_.send(fd, buffer, size);
        });
  }

  template <typename base_t::recv_behavior_t rb = base_t::recv_behavior_t::HOOK,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_t::recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET,
                                   std::vector<std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS, RetType>::type recv_() const {
    return static_cast<const base_t *>(this)->template recv<rb>(
        [this](int32_t fd, void *buffer, size_t size, int32_t flags) -> int32_t {
          static_cast<void>(flags);
          return sl_.recv(fd, buffer, size);
        });
  }

  template <typename base_t::recv_behavior_t rb = base_t::recv_behavior_t::HOOK,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<
                rb == base_t::recv_behavior_t::HOOK, int32_t,
                std::conditional_t<rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET,
                                   std::tuple<int32_t, std::shared_ptr<void>, sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type
  handle_incoming_data_(int32_t fd) {
    return this->template handle_incoming_data__<rb>(
        fd, [this](int32_t fd, void *buffer, size_t size, int32_t flags) -> int32_t {
          static_cast<void>(flags);
          return sl_.recv(fd, buffer, size);
        });
  }

  template <typename base_t::connect_behavior_t cb = base_t::connect_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class,
            typename RetType = std::conditional_t<cb == base_t::connect_behavior_t::HOOK_ON, int32_t,
                                                  std::pair<typename base_t::sockaddr_inet_t, int32_t>>>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type handle_incoming_peer_() {
    int32_t rc;

    /* Underlaying peer handling */
    auto [peer, peer_fd] = this->template handle_incoming_peer__<base_t::connect_behavior_t::HOOK_OFF>();

    this->state() = base_t::state_t::CONNECTING;
    if (peer_fd <= 0) {
      if constexpr (cb == base_t::connect_behavior_t::HOOK_ON) {

        this->state() = base_t::state_t::DISCONNECTED;
        return -1;
      } else if constexpr (cb == base_t::connect_behavior_t::HOOK_OFF) {

        this->state() = base_t::state_t::DISCONNECTED;
        return {sockaddr_inet_t(), -1};
      }
    }

    /* Handle peer on secure layer */
    if ((rc = sl_.register_client(peer_fd))) {
      /*Disconnect*/
      this->disconnect_peer__(peer_fd);
      sl_.clear_peer_creds(peer_fd);

      if constexpr (cb == base_t::connect_behavior_t::HOOK_ON) {
        this->tp().push(
            [this, peer = peer]() -> void { this->on_disconnect()(peer, static_cast<const base_t *>(this)); });
        this->state() = base_t::state_t::DISCONNECTED;
        return -1;
      } else if constexpr (cb == base_t::connect_behavior_t::HOOK_OFF) {

        this->state() = base_t::state_t::DISCONNECTED;
        return {sockaddr_inet_t(), -1};
      }
    }

    if constexpr (cb == base_t::connect_behavior_t::HOOK_ON) {
      this->tp().push([this, peer = peer]() -> void { this->on_connect()(peer, static_cast<const base_t *>(this)); });
      this->state() = base_t::state_t::CONNECTED;
      return peer_fd;
    } else if constexpr (cb == base_t::connect_behavior_t::HOOK_OFF) {

      this->state() = base_t::state_t::CONNECTED;
      return {std::move(peer), peer_fd};
    }
  }

  template <typename base_t::connect_behavior_t cb = base_t::connect_behavior_t::HOOK_ON,
            tcp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS, RetType>::type listen_() {
    this->listen_enabled__() = true;
    int32_t rc;
    if ((rc = ::listen(this->fd__(), SOMAXCONN)) < 0) {
      throw std::runtime_error(fmt::format("Start listening error: (errno = {0}), ({1}), {2}:{3}\"", strerror(errno),
                                           __func__, __FILE__, __LINE__));
    }

    this->state() = base_t::state_t::LISTENING;
    while (this->listen_enabled__()) {

      int32_t num_ready;
      if ((num_ready = ::epoll_wait(this->epfd__(), this->events__(), base_t::epoll_max_events(),
                                    base_t::accept_timeout())) < 0u) {
        throw std::runtime_error(fmt::format("Epoll wait error (errno = {0}) ({1}), {2}:{3}", strerror(errno), __func__,
                                             __FILE__, __LINE__));
      }

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

    this->state() = base_t::state_t::STOPPED;
  }
};

template <tcp_sock_secure_t sc> struct network_tcp_socket_secure_ipv4 : network_tcp_socket_secure_impl<AF_INET, sc> {
  using network_tcp_socket_secure_impl<AF_INET, sc>::network_tcp_socket_secure_impl;
};

template <tcp_sock_secure_t sc> struct network_tcp_socket_secure_ipv6 : network_tcp_socket_secure_impl<AF_INET6, sc> {
  using network_tcp_socket_secure_impl<AF_INET6, sc>::network_tcp_socket_secure_impl;
};

#endif /* NETWORK_TCP_SOCK_SECURE_HPP */
