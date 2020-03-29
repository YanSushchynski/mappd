#ifndef SECURE_LAYER_HPP
#define SECURE_LAYER_HPP

#include "aes.hpp"
#include "tls.hpp"

#include "tcp_sock_secure_type.hpp"
#include "udp_sock_secure_type.hpp"

#include <future>
#include <type_traits>

static constexpr bool is_secure_tcp_server_type(tcp_sock_secure_type_e sc) {
  return sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS ||
         sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES;
}

static constexpr bool is_secure_tcp_client_type(tcp_sock_secure_type_e sc) {
  return sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS ||
         sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES;
}

static constexpr bool is_secure_tcp_aes_type(tcp_sock_secure_type_e sc) {
  return sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES ||
         sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES;
}

static constexpr bool is_secure_tcp_tls_x509_type(tcp_sock_secure_type_e sc) {
  return sc == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS ||
         sc == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS;
}

static constexpr bool is_secure_udp_server_type(udp_sock_secure_type_e sc) {
  return sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES;
}

static constexpr bool is_secure_udp_client_type(udp_sock_secure_type_e sc) {
  return sc == udp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::CLIENT_BROADCAST_SECURE_AES;
}

static constexpr bool is_secure_udp_unicast_type(udp_sock_secure_type_e sc) {
  return sc == udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES;
}

static constexpr bool is_secure_udp_multicast_type(udp_sock_secure_type_e sc) {
  return sc == udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES;
}

static constexpr bool is_secure_udp_broadcast_type(udp_sock_secure_type_e sc) {
  return sc == udp_sock_secure_type_e::SERVER_BROADCAST_SECURE_AES ||
         sc == udp_sock_secure_type_e::CLIENT_BROADCAST_SECURE_AES;
}

template <uint32_t, typename SockClass, SockClass sc, typename Dummy = void> struct secure_layer_s {};
template <uint32_t cipher_size_bits, typename SockClass, SockClass sc>
struct secure_layer_s<cipher_size_bits, SockClass, sc,
                      typename std::enable_if<std::is_same_v<SockClass, tcp_sock_secure_type_e> &&
                                              is_secure_tcp_tls_x509_type(sc)>::type> {
  explicit secure_layer_s(const std::string &ca_cert_file, const std::string &ca_priv_key_file,
                          typename tls_sl_t<sc, cipher_size_bits>::x509_cert_info_t cert_info, uint64_t exp_time)
      : sl_(ca_cert_file, ca_priv_key_file, cert_info, exp_time) {}
  virtual ~secure_layer_s() = default;

  template <tcp_sock_secure_type_e sock_type = sc, typename RetType = int32_t>
  typename std::enable_if<sock_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  connect(int32_t fd) {
    return sl_.connect(fd);
  }

  template <tcp_sock_secure_type_e sock_type = sc, typename RetType = int32_t>
  typename std::enable_if<sock_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  clear(int32_t fd) {
    return sl_.clear(fd);
  }

  template <tcp_sock_secure_type_e sock_type = sc, typename RetType = int32_t>
  typename std::enable_if<sock_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  register_client(int32_t fd) {
    return sl_.register_client(fd);
  }

  template <tcp_sock_secure_type_e sock_type = sc, typename RetType = int32_t>
  typename std::enable_if<sock_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  clear_peer_creds(int32_t fd) {
    return sl_.clear_peer_creds(fd);
  }

  int32_t recv(int32_t fd, void *buffer, size_t nbytes) const { return sl_.recv(fd, buffer, nbytes); }
  int32_t send(int32_t fd, const void *const msg, size_t msg_size) const { return sl_.send(fd, msg, msg_size); }

private:
  tls_sl_t<sc, cipher_size_bits> sl_;
};

template <uint32_t cipher_size_bits, typename SockClass, SockClass sc>
struct secure_layer_s<
    cipher_size_bits, SockClass, sc,
    typename std::enable_if<std::is_same_v<SockClass, tcp_sock_secure_type_e> && is_secure_tcp_aes_type(sc)>::type> {
  explicit secure_layer_s(const char (&aes_key)[(cipher_size_bits / 8u) + 1u]) : sl_(aes_key) {}
  virtual ~secure_layer_s() = default;

  std::pair<std::shared_ptr<void>, int32_t> encrypt(const void *const plaintext, size_t plaintext_len) const {
    return sl_.encrypt(plaintext, plaintext_len);
  }

  std::pair<std::shared_ptr<void>, int32_t> decrypt(const void *const ciphertext, size_t ciphertext_len) const {
    return sl_.decrypt(ciphertext, ciphertext_len);
  }

private:
  aes_ssl_t<cipher_size_bits> sl_;
};

template <uint32_t cipher_size_bits, typename SockClass, SockClass sc>
struct secure_layer_s<cipher_size_bits, SockClass, sc,
                      typename std::enable_if<std::is_same_v<SockClass, udp_sock_secure_type_e> &&
                                              (is_secure_udp_client_type(sc) || is_secure_udp_server_type(sc))>::type> {
  explicit secure_layer_s(const char (&key)[(cipher_size_bits / 8u) + 1u]) : sl_(key) {}
  virtual ~secure_layer_s() = default;

  std::pair<std::shared_ptr<void>, int32_t> encrypt(const void *const plaintext, size_t plaintext_len) const {
    return sl_.encrypt(plaintext, plaintext_len);
  }

  std::pair<std::shared_ptr<void>, int32_t> decrypt(const void *const ciphertext, size_t ciphertext_len) const {
    return sl_.decrypt(ciphertext, ciphertext_len);
  }

private:
  aes_ssl_t<cipher_size_bits> sl_;
};

#endif /* SECURE_LAYER_HPP */
