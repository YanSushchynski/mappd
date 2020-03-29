#ifndef NETWORK_UDP_SOCK_SECURE_HPP
#define NETWORK_UDP_SOCK_SECURE_HPP

#include "network_udp_sock.hpp"
#include "secure_layer.hpp"
#include "udp_sock_secure_type.hpp"

template <uint32_t family, udp_sock_secure_type_e secure_socket_class, bool multithread, uint32_t aes_key_size_bits>
struct network_udp_socket_secure_impl_s
    : protected network_udp_socket_impl_s<
          family, static_cast<udp_sock_type_e>(static_cast<uint32_t>(secure_socket_class)), multithread> {
public:
  using base_s =
      network_udp_socket_impl_s<family, static_cast<udp_sock_type_e>(static_cast<uint32_t>(secure_socket_class)),
                                multithread>;
		using this_t = network_udp_socket_secure_impl_s<family, secure_socket_class, multithread, aes_key_size_bits>;
  static constexpr bool is_ipv6 = base_s::is_ipv6;

  template <udp_sock_secure_type_e sc = secure_socket_class>
  explicit network_udp_socket_secure_impl_s(
      const std::string &iface, const char (&dgram_aes_key)[(aes_key_size_bits / 8u) + 1u],
      typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                                  (!is_ipv6 && sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES) ||
                                  sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES,
                              udp_sock_secure_type_e>::type * = nullptr)
      : base_s(iface), sl_(dgram_aes_key) {}

  template <udp_sock_secure_type_e sc = secure_socket_class>
  explicit network_udp_socket_secure_impl_s(
      const std::string &iface, const char (&dgram_aes_key)[(aes_key_size_bits / 8u) + 1u],
      typename std::enable_if<sc == udp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES ||
                                  (!is_ipv6 && sc == udp_sock_secure_type_e::CLIENT_BROADCAST_SECURE_AES),
                              udp_sock_secure_type_e>::type * = nullptr)
      : base_s(iface), sl_(dgram_aes_key) {}

  template <udp_sock_secure_type_e sc = secure_socket_class>
  explicit network_udp_socket_secure_impl_s(
      const std::string &iface, const std::string &mcast_gr_addr, uint16_t srv,
      const char (&dgram_aes_key)[(aes_key_size_bits / 8u) + 1u],
      typename std::enable_if<sc == udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES, udp_sock_secure_type_e>::type
          * = nullptr)
      : base_s(iface, mcast_gr_addr, srv), sl_(dgram_aes_key) {}

  virtual ~network_udp_socket_secure_impl_s() = default;

  void stop_threads() const { return const_cast<const typename base_s::base_s *>(this)->stop_threads(); }
  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES, RetType>::type
  setup(const std::string &mcast_gr_addr, uint16_t port) {
    this->base_s::setup(mcast_gr_addr, port);
  }

  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  setup(uint16_t port) {
    this->base_s::setup(port);
  }

  const auto &on_receive() const { return this->base_s::on_receive(); }
  const auto &on_send() const { return this->base_s::on_send(); }

  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = bool>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES) ||
                              sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES,
                          RetType>::type
  running() const {
    return this->base_s::running();
  }

  template <typename base_s::send_behavior_e sb = base_s::send_behavior_e::HOOK_OFF,
            udp_sock_secure_type_e sc = secure_socket_class,
            typename RetType =
                std::conditional_t<sb == base_s::send_behavior_e::HOOK_ON, int32_t,
                                   std::conditional_t<sb == base_s::send_behavior_e::HOOK_OFF,
                                                      std::pair<int32_t, typename base_s::sockaddr_inet_t>, void>>>
  typename std::enable_if<!is_ipv6 && (sc == udp_sock_secure_type_e::CLIENT_BROADCAST_SECURE_AES ||
                                       sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  send(uint16_t port, const void *const msg, size_t size) const {
    auto [enc_msg, cipher_size] = sl_.encrypt(msg, size);
    if constexpr (sb == base_s::send_behavior_e::HOOK_ON) {

      int32_t snd_size =
          this->base_s::template send<base_s::send_behavior_e::HOOK_ON>(port, enc_msg.get(), cipher_size);
      return snd_size;
    } else if constexpr (sb == base_s::send_behavior_e::HOOK_OFF) {

      auto [snd_size, to] =
          this->base_s::template send<base_s::send_behavior_e::HOOK_OFF>(port, enc_msg.get(), cipher_size);

      void *data = std::calloc(size, sizeof(char));
      std::memcpy(data, msg, size);
      std::thread([this, data, peer = to, size]() -> void {
        this->on_send()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                        static_cast<const base_s *>(this));
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();

      return {snd_size, to};
    }
  }

  template <typename base_s::send_behavior_e sb = base_s::send_behavior_e::HOOK_OFF,
            udp_sock_secure_type_e sc = secure_socket_class,
            typename RetType =
                std::conditional_t<sb == base_s::send_behavior_e::HOOK_ON, int32_t,
                                   std::conditional_t<sb == base_s::send_behavior_e::HOOK_OFF,
                                                      std::pair<int32_t, typename base_s::sockaddr_inet_t>, void>>>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES,
                          RetType>::type
  send(const std::string &addr, uint16_t port, const void *const msg, size_t size) {
    auto [enc_msg, cipher_size] = sl_.encrypt(msg, size);
    if constexpr (sb == base_s::send_behavior_e::HOOK_ON) {

      int32_t snd_size =
          this->base_s::template send<base_s::send_behavior_e::HOOK_ON>(addr, port, enc_msg.get(), cipher_size);
      return snd_size;
    } else if constexpr (sb == base_s::send_behavior_e::HOOK_OFF) {

      auto [snd_size, to] =
          this->base_s::template send<base_s::send_behavior_e::HOOK_OFF>(addr, port, enc_msg.get(), cipher_size);

      void *data = std::calloc(size, sizeof(char));
      std::memcpy(data, msg, size);
      std::thread([this, peer = to, data, size]() -> void {
        this->on_send()(peer, std::shared_ptr<void>(data, [](const auto &data) -> void { std::free(data); }), size,
                        static_cast<const base_s *>(this));
        {
          std::unique_lock<std::mutex> lock(this->mtx());
          std::notify_all_at_thread_exit(this->cv(), std::move(lock));
        }
      }).detach();

      return {snd_size, to};
    }
  }

  template <typename base_s::recv_behavior_e rb = base_s::recv_behavior_e::HOOK,
            typename RetType = std::conditional_t<
                rb == base_s::recv_behavior_e::HOOK, int32_t,
                std::conditional_t<
                    rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET,
                    std::vector<std::tuple<int32_t, std::shared_ptr<void>, typename base_s::sockaddr_inet_t>>, void>>>
  RetType recv() {
    int32_t recvd_size;
    auto res = this->base_s::template recv<base_s::recv_behavior_e::RET>();
    for (auto &transaction : res) {
      auto [dec_data, plain_size] = sl_.decrypt(std::get<1u>(transaction).get(), std::get<0u>(transaction));

      if constexpr (rb == base_s::recv_behavior_e::HOOK) {
        std::thread([this, peer = std::get<2u>(transaction), data = dec_data, size = plain_size]() -> void {
          this->on_receive()(peer, data, size, static_cast<const base_s *>(this));
          {
            std::unique_lock<std::mutex> lock(this->mtx());
            std::notify_all_at_thread_exit(this->cv(), std::move(lock));
          }
        }).detach();

      } else if constexpr (rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET) {

        std::get<0u>(transaction) = plain_size;
        std::get<1u>(transaction).reset();
        std::get<1u>(transaction) = std::move(dec_data);
      }

      if constexpr (rb == base_s::recv_behavior_e::HOOK)
        recvd_size += plain_size;
    }

    if constexpr (rb == base_s::recv_behavior_e::HOOK) {

      return recvd_size;
    } else if constexpr (rb == base_s::recv_behavior_e::RET || rb == base_s::recv_behavior_e::HOOK_RET) {

      return std::move(res);
    }
  }

  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES,
                          RetType>::type
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

  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = int32_t>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES),
                          RetType>::type
  stop() {
    return this->base_s::stop();
  }

  void reset() { this->base_s::reset(); }

private:
  const secure_layer_s<aes_key_size_bits, udp_sock_secure_type_e, secure_socket_class> sl_;

  template <udp_sock_secure_type_e sc = secure_socket_class, typename RetType = void>
  typename std::enable_if<sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
                              (!is_ipv6 && sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES) ||
                              sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES,
                          RetType>::type
  listen_() {
    this->listen_enabled__() = true;
    this->state__() = base_s::state_e::RUNNING;

    while (this->listen_enabled__())
      static_cast<void>(recv());
    this->state__() = base_s::state_e::STOPPED;
  }
};

template <udp_sock_secure_type_e sc, bool multithread, uint32_t key_size>
struct network_udp_socket_secure_ipv4_s : network_udp_socket_secure_impl_s<AF_INET, sc, multithread, key_size> {
  using network_udp_socket_secure_impl_s<AF_INET, sc, multithread, key_size>::network_udp_socket_secure_impl_s;
};

template <udp_sock_secure_type_e sc, bool multithread, uint32_t key_size>
struct network_udp_socket_secure_ipv6_s : network_udp_socket_secure_impl_s<AF_INET6, sc, multithread, key_size> {
  using network_udp_socket_secure_impl_s<AF_INET6, sc, multithread, key_size>::network_udp_socket_secure_impl_s;
};

#endif /* NETWORK_UDP_SOCK_SECURE_HPP */
