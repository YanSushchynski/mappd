#ifndef UDP_SOCK_SECURE_HPP
#define UDP_SOCK_SECURE_HPP

#include "secure_layer.hpp"
#include "udp_sock.hpp"
#include "udp_sock_secure_type.hpp"

template <uint32_t family, udp_sock_secure_t secure_socket_class>
struct udp_socket_secure
    : protected udp_socket<family, static_cast<udp_sock_t>(static_cast<uint32_t>(secure_socket_class))> {
public:
  static constexpr uint32_t dgram_aes_key_size_bits = 256u;
  using base_t = udp_socket<family, static_cast<udp_sock_t>(static_cast<uint32_t>(secure_socket_class))>;
  using this_t = udp_socket_secure<family, secure_socket_class>;
  static constexpr bool is_ipv6 = base_t::is_ipv6;

  template <udp_sock_secure_t sc = secure_socket_class>
  explicit udp_socket_secure(
      const std::string &iface, const char (&dgram_aes_key)[(dgram_aes_key_size_bits / 8u) + 1u],
      typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                                  (!is_ipv6 && sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES) ||
                                  sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES,
                              udp_sock_secure_t>::type * = nullptr)
      : base_t(iface), sl_(dgram_aes_key) {}

  template <udp_sock_secure_t sc = secure_socket_class>
  explicit udp_socket_secure(
      const std::string &iface, const char (&dgram_aes_key)[(dgram_aes_key_size_bits / 8u) + 1u],
      typename std::enable_if<sc == udp_sock_secure_t::CLIENT_UNICAST_SECURE_AES ||
                                  (!is_ipv6 && sc == udp_sock_secure_t::CLIENT_BROADCAST_SECURE_AES),
                              udp_sock_secure_t>::type * = nullptr)
      : base_t(iface), sl_(dgram_aes_key) {}

  template <udp_sock_secure_t sc = secure_socket_class>
  explicit udp_socket_secure(const std::string &iface, const std::string &mcast_gr_addr, uint16_t srv,
                             const char (&dgram_aes_key)[(dgram_aes_key_size_bits / 8u) + 1u],
                             typename std::enable_if<sc == udp_sock_secure_t::CLIENT_MULTICAST_SECURE_AES,
                                                     udp_sock_secure_t>::type * = nullptr)
      : base_t(iface, mcast_gr_addr, srv), sl_(dgram_aes_key) {}

  virtual ~udp_socket_secure() = default;

  void stop_threads() const { return const_cast<const typename base_t::base_t *>(this)->stop_tp(); }
  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES, RetType>::type
  setup(const std::string &mcast_gr_addr, uint16_t port) {
    static_cast<base_t *>(this)->setup(mcast_gr_addr, port);
  }

  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  setup(uint16_t port) {
    static_cast<const base_t *>(this)->setup(port);
  }

  const auto &on_receive() const { return static_cast<const base_t *>(this)->on_receive(); }
  const auto &on_send() const { return static_cast<const base_t *>(this)->on_send(); }

  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES) ||
                              sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES,
                          RetType>::type
  running() const {
    return static_cast<const base_t *>(this)->running();
  }

  template <typename base_t::send_behavior_t sb = base_t::send_behavior_t::HOOK_OFF,
            udp_sock_secure_t sc = secure_socket_class,
            typename RetType =
                std::conditional_t<sb == base_t::send_behavior_t::HOOK_ON, int32_t,
                                   std::conditional_t<sb == base_t::send_behavior_t::HOOK_OFF,
                                                      std::pair<int32_t, typename base_t::sockaddr_inet_t>, void>>>
  typename std::enable_if<!is_ipv6 && (sc == udp_sock_secure_t::CLIENT_BROADCAST_SECURE_AES ||
                                       sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  send(uint16_t port, const void *const msg, size_t size) const {
    auto [enc_msg, cipher_size] = sl_.encrypt(msg, size);
    if constexpr (sb == base_t::send_behavior_t::HOOK_ON) {

      int32_t snd_size = static_cast<const base_t *>(this)->template send<base_t::send_behavior_t::HOOK_ON>(
          port, enc_msg.get(), cipher_size);
      return snd_size;
    } else if constexpr (sb == base_t::send_behavior_t::HOOK_OFF) {

      auto [snd_size, to] = static_cast<const base_t *>(this)->template send<base_t::send_behavior_t::HOOK_OFF>(
          port, enc_msg.get(), cipher_size);

      void *data = std::calloc(size, sizeof(char));
      std::memcpy(data, msg, size);
      this->tp().push([this, data, peer = to, size](int32_t thr_id) -> void {
        this->on_send()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                        static_cast<const base_t *>(this));
      });

      return {snd_size, to};
    }
  }

  template <typename base_t::send_behavior_t sb = base_t::send_behavior_t::HOOK_OFF,
            udp_sock_secure_t sc = secure_socket_class,
            typename RetType =
                std::conditional_t<sb == base_t::send_behavior_t::HOOK_ON, int32_t,
                                   std::conditional_t<sb == base_t::send_behavior_t::HOOK_OFF,
                                                      std::pair<int32_t, typename base_t::sockaddr_inet_t>, void>>>
  typename std::enable_if<
      sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES || sc == udp_sock_secure_t::CLIENT_UNICAST_SECURE_AES ||
          sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES || sc == udp_sock_secure_t::CLIENT_MULTICAST_SECURE_AES,
      RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) const {
    auto [enc_msg, cipher_size] = sl_.encrypt(msg, size);
    if constexpr (sb == base_t::send_behavior_t::HOOK_ON) {

      int32_t snd_size = static_cast<const base_t *>(this)->template send<base_t::send_behavior_t::HOOK_ON>(
          addr, port, enc_msg.get(), cipher_size);
      return snd_size;
    } else if constexpr (sb == base_t::send_behavior_t::HOOK_OFF) {

      auto [snd_size, to] = static_cast<const base_t *>(this)->template send<base_t::send_behavior_t::HOOK_OFF>(
          addr, port, enc_msg.get(), cipher_size);

      void *data = std::calloc(size, sizeof(char));
      std::memcpy(data, msg, size);
      this->tp().push([this, peer = to, data, size](int32_t thr_id) -> void {
        this->on_send()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                        static_cast<const base_t *>(this));
      });

      return {snd_size, to};
    }
  }

  template <typename base_t::recv_behavior_t rb = base_t::recv_behavior_t::HOOK,
            typename RetType = std::conditional_t<
                rb == base_t::recv_behavior_t::HOOK, int32_t,
                std::conditional_t<
                    rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET,
                    std::vector<std::tuple<int32_t, std::shared_ptr<void>, typename base_t::sockaddr_inet_t>>, void>>>
  RetType recv() const {
    int32_t recvd_size;
    auto res = static_cast<const base_t *>(this)->template recv<base_t::recv_behavior_t::RET>();
    for (auto &transaction : res) {
      auto [dec_data, plain_size] = sl_.decrypt(std::get<1u>(transaction).get(), std::get<0u>(transaction));

      if constexpr (rb == base_t::recv_behavior_t::HOOK) {
        this->tp().push(
            [this, peer = std::get<2u>(transaction), data = dec_data, size = plain_size](int32_t thr_id) -> void {
              this->on_receive()(peer, data, size, static_cast<const base_t *>(this));
            });

      } else if constexpr (rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET) {

        std::get<0u>(transaction) = plain_size;
        std::get<1u>(transaction).reset();
        std::get<1u>(transaction) = std::move(dec_data);
      }

      if constexpr (rb == base_t::recv_behavior_t::HOOK)
        recvd_size += plain_size;
    }

    if constexpr (rb == base_t::recv_behavior_t::HOOK) {

      return recvd_size;
    } else if constexpr (rb == base_t::recv_behavior_t::RET || rb == base_t::recv_behavior_t::HOOK_RET) {

      return std::move(res);
    }
  }

  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES ||
                              sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES,
                          RetType>::type
  start(uint64_t duration_ms = 0) const {
    bool nonblock = duration_ms == 0;
    if (nonblock) {
      this->listen_thread__() = std::thread([this]() -> void { listen_(); });

    } else {
      this->tp().push(
          [this](int32_t thr_id, uint64_t duration_ms, std::atomic_bool *trigger) -> void {
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
            *trigger = false;
          },
          duration_ms, &this->listen_enabled__());

      listen_();
    }
  }

  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  stop() {
    return static_cast<base_t *>(this)->stop();
  }

  void reset() { static_cast<base_t *>(this)->reset(); }

private:
  const secure_layer_t<dgram_aes_key_size_bits, udp_sock_secure_t, secure_socket_class> sl_;

  template <udp_sock_secure_t sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_t::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_t::SERVER_BROADCAST_SECURE_AES) ||
                              sc == udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES,
                          RetType>::type
  listen_() const {
    this->listen_enabled__() = true;
    this->state__() = base_t::state_t::RUNNING;

    while (this->listen_enabled__())
      static_cast<void>(recv());
    this->state__() = base_t::state_t::STOPPED;
  }
};

#endif /* UDP_SOCK_SECURE_HPP */
