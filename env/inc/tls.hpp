#ifndef TLS_HPP
#define TLS_HPP

#include <algorithm>
#include <csignal>
#include <cstdint>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <future>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <openssl/asn1.h>
#include <openssl/asn1err.h>
#include <openssl/conf.h>
#include <openssl/conferr.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/evperr.h>
#include <openssl/pem.h>
#include <openssl/pemerr.h>
#include <openssl/rand.h>
#include <openssl/randerr.h>
#include <openssl/rsa.h>
#include <openssl/rsaerr.h>
#include <openssl/ssl.h>
#include <openssl/ssl3.h>
#include <openssl/sslerr.h>
#include <openssl/store.h>
#include <openssl/x509v3.h>
#include <openssl/x509v3err.h>

#include <boost/format.hpp>

#include "tcp_sock_secure_type.hpp"
#include "udp_sock_secure_type.hpp"

template <tcp_sock_secure_type_e secure_socket_type, uint32_t bits> struct tls_sl_t {
  using this_t = tls_sl_t<secure_socket_type, bits>;

private:
  template <uint32_t N> struct x509_cert_info_impl_t_ : std::array<std::string, N> {
    using this_t = x509_cert_info_impl_t_<N>;
    using base_s = std::array<std::string, N>;
    static constexpr const char *parser_split_tokens = ",;.";

    x509_cert_info_impl_t_(const std::string &bytes) : base_s(from_string_(bytes, parser_split_tokens)) {}
    virtual ~x509_cert_info_impl_t_() = default;

    const char *country_value() { return (*this)[0].c_str(); }; /* C */
    const char *state_value() { return (*this)[1].c_str(); };
    const char *locality_name_value() { return (*this)[2].c_str(); };
    const char *organization_name_value() { return (*this)[3].c_str(); };
    const char *organizational_unit_name_value() { return (*this)[4].c_str(); };
    const char *common_name_value() { return (*this)[5].c_str(); };

    const char *country_token() { return "C"; }; /* C */
    const char *state_token() { return "ST"; };
    const char *locality_name_token() { return "L"; };
    const char *organization_name_token() { return "O"; };
    const char *organizational_unit_name_token() { return "OU"; };
    const char *common_name_token() { return "CN"; };

  private:
    std::array<std::string, N> from_string_(const std::string &bytes, const char *delimeters) {
      char *split_list;
      std::array<std::string, N> ret;
      std::string data_copy = bytes;

      if (!(split_list = std::strtok(const_cast<char *>(data_copy.data()), delimeters))) {
        throw std::runtime_error(
            (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
      }

      do {
        if (!std::strncmp(split_list, (std::string(country_token()) + "=").c_str(),
                          std::strlen(country_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            throw std::runtime_error(
                (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
          ret[0u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else if (!std::strncmp(split_list, (std::string(state_token()) + "=").c_str(),
                                 std::strlen(state_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            throw std::runtime_error(
                (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
          ret[1u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else if (!std::strncmp(split_list, (std::string(locality_name_token()) + "=").c_str(),
                                 std::strlen(locality_name_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            throw std::runtime_error(
                (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
          ret[2u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else if (!std::strncmp(split_list, (std::string(organization_name_token()) + "=").c_str(),
                                 std::strlen(organization_name_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            throw std::runtime_error(
                (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
          ret[3u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else if (!std::strncmp(split_list, (std::string(organizational_unit_name_token()) + "=").c_str(),
                                 std::strlen(organizational_unit_name_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            throw std::runtime_error(
                (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
          ret[4u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else if (!std::strncmp(split_list, (std::string(common_name_token()) + "=").c_str(),
                                 std::strlen(common_name_token()) + 1u)) {

          char *pch = std::strtok(split_list, "=");
          if (!pch)
            goto error;
          ret[5u] = (pch = std::strtok(nullptr, "="))
                        ? pch
                        : throw std::runtime_error(
                              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
        } else
        error:
          throw std::runtime_error(
              (boost::format("Invalid certificate subject (%1%:%2%)\r\n") % __FILE__ % __LINE__).str());
      } while ((split_list = std::strtok(nullptr, delimeters)));

      return std::move(ret);
    }
  };

  static constexpr uint32_t fields_count_ = 6u;
  static constexpr uint32_t tls_handshake_timeout_ms = 1000u;

public:
  using x509_cert_info_t = x509_cert_info_impl_t_<fields_count_>;

  explicit tls_sl_t(const std::string &ca_cert_file, const std::string &ca_key_file, x509_cert_info_t cert_info,
                    uint64_t exp_time)
      : ca_cert_file_(ca_cert_file), ca_key_file_(ca_key_file), cert_info_(cert_info), exp_time_(exp_time) {
    init_(ca_cert_file, ca_key_file);
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  register_client(int32_t fd) {
    return register_client_(fd);
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  clear_peer_creds(int32_t fd) {
    return clear_peer_creds_(fd);
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type connect(int32_t fd) {
    return connect_(fd);
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type clear(int32_t fd) {
    return clear_(fd);
  }

  int32_t recv(int32_t fd, void *buffer, size_t nbytes) const { return recv_(fd, buffer, nbytes); }
  int32_t send(int32_t fd, const void *const msg, size_t msg_size) const { return send_(fd, msg, msg_size); }

  virtual ~tls_sl_t() = default;

private:
  static void raise_error_() {
    int32_t crypto_err = ERR_get_error();
    ERR_clear_error();
    throw std::runtime_error(
        (boost::format("%1% (errno = %2%)\r\n") % ERR_error_string(crypto_err, nullptr) % strerror(errno)).str());
  }

  static void raise_ssl_error_(SSL *ssl_handle, int32_t rc) {
    int32_t ssl_err = SSL_get_error(ssl_handle, rc);
    const char *error_reason = ERR_reason_error_string(ERR_get_error());
    const char *state = SSL_state_string_long(ssl_handle);
    ERR_clear_error();

    if (error_reason && state) {
      std::printf("%s", (boost::format("Error: %1%, state: %2%\r\n") % error_reason % state).str().c_str());
    }

    throw std::runtime_error(
        (boost::format("%1%, (errno = %2%)\r\n") % ERR_error_string(ssl_err, nullptr) % strerror(errno)).str());
  }

  static void print_error_() {
    int32_t crypto_err = ERR_get_error();
    ERR_clear_error();
    std::printf(
        "%s", (boost::format("Error: %1% (errno = %2%)\r\n") % ERR_error_string(crypto_err, nullptr) % strerror(errno))
                  .str()
                  .c_str());
  }

  static void print_ssl_error_(SSL *ssl_handle, int32_t rc) {
    int32_t ssl_err = SSL_get_error(ssl_handle, rc);
    const char *error_reason = ERR_reason_error_string(ERR_get_error());
    const char *state = SSL_state_string_long(ssl_handle);

    ERR_clear_error();

    if (error_reason) {
      std::printf("%s", (boost::format("SSL error reason: %1%\r\n") % error_reason).str().c_str());
    }

    if (state) {
      std::printf("%s", (boost::format("SSL state: %1%\r\n") % state).str().c_str());
    }

    std::printf("%s", (boost::format("Errno: %1%\r\n") % strerror(errno)).str().c_str());
  }

  std::shared_ptr<SSL_CTX> create_tls_context_() {
    const SSL_METHOD *method = nullptr;
    SSL_CTX *ctx = nullptr;

    if constexpr (secure_socket_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS)

      method = TLS_server_method();
    else if constexpr (secure_socket_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS)

      method = TLS_client_method();
    if (!(ctx = SSL_CTX_new(method))) {
      raise_error_();
    }

    return std::shared_ptr<SSL_CTX>(ctx, [](const auto &data) -> void { SSL_CTX_free(data); });
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  init_tls_context_(SSL_CTX *ctx, X509 *cert, RSA *key) {
    EVP_PKEY *priv_key;

    SSL_CTX_set_ecdh_auto(ctx, 1);
    SSL_CTX_set_mode(ctx, SSL_MODE_AUTO_RETRY);
    SSL_CTX_set_options(ctx, SSL_OP_ALL | SSL_OP_NO_RENEGOTIATION | SSL_OP_NO_TICKET | SSL_OP_SINGLE_DH_USE |
                                 SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_NO_TLSv1 | SSL_OP_NO_TLSv1_1 |
                                 SSL_OP_NO_TLSv1_2);

    SSL_CTX_set_ciphersuites(ctx, "TLS_AES_256_GCM_SHA384");

    /* Set the key and cert */
    if (SSL_CTX_use_certificate(ctx, cert) <= 0) {
      raise_error_();
    }

    if (!(priv_key = EVP_PKEY_new())) {
      raise_error_();
    }

    EVP_PKEY_assign_RSA(priv_key, key);
    if (SSL_CTX_use_PrivateKey(ctx, priv_key) <= 0) {
      raise_error_();
    }

    /* Verify private key */
    if (!SSL_CTX_check_private_key(ctx)) {
      raise_error_();
    }

    return 0;
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  init_tls_context_(int32_t fd, SSL_CTX *ctx, X509 *cert, RSA *key) {
    EVP_PKEY *priv_key;

    SSL_CTX_set_ecdh_auto(ctx, 1);
    SSL_CTX_set_mode(ctx, SSL_MODE_AUTO_RETRY);
    SSL_CTX_set_options(ctx, SSL_OP_ALL | SSL_OP_NO_RENEGOTIATION | SSL_OP_NO_TICKET |
                                 SSL_OP_NO_SESSION_RESUMPTION_ON_RENEGOTIATION | SSL_OP_SINGLE_DH_USE |
                                 SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_NO_TLSv1 | SSL_OP_NO_TLSv1_1 |
                                 SSL_OP_NO_TLSv1_2);

    SSL_CTX_set_ciphersuites(ctx, "TLS_AES_256_GCM_SHA384");

    /* Set the key and cert */
    if (SSL_CTX_use_certificate(ctx, cert) <= 0) {
      raise_error_();
    }

    if (!(priv_key = EVP_PKEY_new())) {
      raise_error_();
    }

    EVP_PKEY_assign_RSA(priv_key, key);
    if (SSL_CTX_use_PrivateKey(ctx, priv_key) <= 0) {
      raise_error_();
    }

    /* Verify private key */
    if (!SSL_CTX_check_private_key(ctx)) {
      raise_error_();
    }

    return 0;
  }

  int32_t sig_verify_(X509 *cert, X509 *ca_cert) const {
    EVP_PKEY *pubkey;
    if (!(pubkey = X509_get_pubkey(ca_cert))) {
      raise_error_();
    }

    int32_t result = X509_verify(cert, pubkey);
    EVP_PKEY_free(pubkey);
    return result;
  }

  static int32_t verify_cb_(int32_t preverify_ok, X509_STORE_CTX *certs_ctx) { return 1; }
  int32_t set_blocking_(int32_t fd) const {
    int32_t flags;
    if ((flags = ::fcntl(fd, F_GETFL, 0)) < 0)
      return -1;

    if (::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0)
      return -1;
    return 0;
  }

  int32_t set_non_blocking_(int32_t fd) const {
    int32_t flags;
    if ((flags = ::fcntl(fd, F_GETFL, 0)) < 0)
      return -1;

    if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
      return -1;

    return 0;
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  register_client_(int32_t fd) {
    SSL *ssl_handle;
    int32_t rc, handshake_status;
    X509 *peer_cert;

    ::signal(SIGPIPE, SIG_IGN);

    tls_context_lock_.lock();
    typename decltype(tls_context_)::iterator it = tls_context_.find(fd);
    if (it == tls_context_.end()) {

      tls_context_lock_.unlock();
      auto tls_context = create_tls_context_();

      tls_handle_lock_.lock();
      typename decltype(tls_handle_)::iterator it = tls_handle_.find(fd);
      if (it == tls_handle_.end()) {

        /* Generate signed private key and certificate */
        tls_handle_lock_.unlock();
        auto [x509_cert, rsa_priv_key] = gen_signed_key_cert_();

        /* Initialize context */
        init_tls_context_(fd, tls_context.get(), x509_cert.get(), rsa_priv_key.get());

        if (!(ssl_handle = SSL_new(tls_context.get()))) {
          raise_ssl_error_(ssl_handle, rc);
        }

        /* Prepare for accepting */
        SSL_set_shutdown(ssl_handle, SSL_SENT_SHUTDOWN | SSL_RECEIVED_SHUTDOWN);
        SSL_set_verify(ssl_handle, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, verify_cb_);
        SSL_set_verify_depth(ssl_handle, 4);
        SSL_set_fd(ssl_handle, fd);
        SSL_set_accept_state(ssl_handle);

        /* File descriptor sets, needed to handle nonblocking socket using ::select() */
        fd_set read_fd_set;
        fd_set write_fd_set;
        fd_set error_fd_set;
        struct timeval accept_timeout = {1u, 0u};

        /* Do accept procedure */
        do {
          switch ((rc = ::SSL_get_error(ssl_handle, (rc = SSL_accept(ssl_handle))))) {
          case SSL_ERROR_NONE:
            break;

          case SSL_ERROR_WANT_WRITE:
            FD_ZERO(&write_fd_set);
            FD_SET(fd, &write_fd_set);

            if ((rc = ::select(fd + 1, &write_fd_set, nullptr, nullptr, &accept_timeout)) <= 0) {
              /* Write timeout */
              rc = -1;
              break;
            } else
              /* Ok, selected successfully, continuing */
              continue;
          case SSL_ERROR_WANT_READ:
            FD_ZERO(&read_fd_set);
            FD_SET(fd, &read_fd_set);

            if ((rc = ::select(fd + 1, &read_fd_set, nullptr, nullptr, &accept_timeout)) <= 0) {
              /* Read timeout */
              rc = -1;
              break;
            } else
              /* Ok, selected successfully, continuing */
              continue;
          case SSL_ERROR_WANT_X509_LOOKUP:
          case SSL_ERROR_WANT_ACCEPT:
            continue;

            /* Unrecoverable errors */
          case SSL_ERROR_ZERO_RETURN:
          case SSL_ERROR_SYSCALL:
          case SSL_ERROR_SSL:
          default:
            rc = -1;
            break;
          }
        } while (rc != SSL_ERROR_NONE && rc != -1);

        /* Interrupt if accept handled unsuccessfully */
        if (rc) {

          SSL_clear(ssl_handle);
          SSL_free(ssl_handle);
          return rc;
        }

        /* Get peer certificate */
        if (!(peer_cert = SSL_get_peer_certificate(ssl_handle))) {

          /* Get vertificate error, clear memory, return */
          rc = -1;
          set_blocking_(fd);
          SSL_shutdown(ssl_handle);
          set_non_blocking_(fd);
          SSL_free(ssl_handle);
          raise_ssl_error_(ssl_handle, rc);
        } else {
          if ((rc = sig_verify_(peer_cert, ca_cert_.get())) > 0) {

            /* Verified, init private members */
            tls_handle_lock_.lock();
            tls_handle_.insert(
                std::make_pair(fd, std::shared_ptr<SSL>(ssl_handle, [](const auto &data) -> void { SSL_free(data); })));
            tls_handle_lock_.unlock();

            tls_context_lock_.lock();
            tls_context_.insert(std::make_pair(fd, tls_context));
            tls_context_lock_.unlock();

            cert_lock_.lock();
            cert_.insert(std::make_pair(fd, std::move(x509_cert)));
            cert_lock_.unlock();

            rsa_priv_key_lock_.lock();
            rsa_priv_key_.insert(std::make_pair(fd, std::move(rsa_priv_key)));
            rsa_priv_key_lock_.unlock();
            rc = 0;
          } else {

            /* Check failed, clear memory */
            set_blocking_(fd);
            SSL_shutdown(ssl_handle);
            set_non_blocking_(fd);
            SSL_free(ssl_handle);
            rc = -1;
          }

          X509_free(peer_cert);
        }

        return rc;
      } else {

        tls_handle_lock_.unlock();
        return 1;
      }
    } else {

      tls_context_lock_.unlock();
      return 1;
    }
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  clear_peer_creds_(int32_t fd) {
    int32_t rc;
    /* Erase leftovers from peer by FD (if they are exist)*/
    if (tls_handle_.find(fd) != tls_handle_.end())
      tls_handle_.erase(fd);

    if (tls_context_.find(fd) != tls_context_.end())
      tls_context_.erase(fd);

    if (cert_.find(fd) != cert_.end())
      cert_.erase(fd);

    if (rsa_priv_key_.find(fd) != rsa_priv_key_.end())
      rsa_priv_key_.erase(fd);

    return 0;
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type connect_(int32_t fd) {
    int32_t rc, err, handshake_status;
    SSL *ssl_handle;
    X509 *peer_cert;

    ::signal(SIGPIPE, SIG_IGN);

    /* Generate new signed by ca_key certificate and private key */
    auto [x509_ca_cert, x509_cert, rsa_ca_priv_key, rsa_priv_key] = gen_signed_key_cert_(ca_cert_file_, ca_key_file_);
    auto tls_context = create_tls_context_();

    /* Initialize new TLS context */
    init_tls_context_(tls_context.get(), x509_cert.get(), rsa_priv_key.get());

    if (!(ssl_handle = SSL_new(tls_context.get()))) {
      raise_ssl_error_(ssl_handle, rc);
    }

    /* Prepare SSL handle for connecting */
    SSL_set_shutdown(ssl_handle, SSL_SENT_SHUTDOWN | SSL_RECEIVED_SHUTDOWN);
    SSL_set_verify(ssl_handle, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, verify_cb_);
    SSL_set_verify_depth(ssl_handle, 4);
    SSL_set_fd(ssl_handle, fd);
    SSL_set_connect_state(ssl_handle);

    /* Used for handling nonblocking socket */
    fd_set read_fd_set;
    fd_set write_fd_set;
    struct timeval connect_timeout = {1u, 0u};

    /* Do connect procedure */
    do {
      switch ((rc = ::SSL_get_error(ssl_handle, (rc = SSL_connect(ssl_handle))))) {
      case SSL_ERROR_NONE:
        break;

      case SSL_ERROR_WANT_WRITE:
        FD_ZERO(&write_fd_set);
        FD_SET(fd, &write_fd_set);

        if ((rc = ::select(fd + 1, &write_fd_set, nullptr, nullptr, &connect_timeout)) <= 0) {
          /* Write timeout */
          rc = -1;
          break;
        } else
          /* Ok, continuing */
          continue;
      case SSL_ERROR_WANT_READ:
        FD_ZERO(&read_fd_set);
        FD_SET(fd, &read_fd_set);

        if ((rc = ::select(fd + 1, &read_fd_set, nullptr, nullptr, &connect_timeout)) <= 0) {
          /* Read timeout */
          rc = -1;
          break;
        } else
          /* Ok, continuing */
          continue;
      case SSL_ERROR_WANT_X509_LOOKUP:
      case SSL_ERROR_WANT_CONNECT:
        continue;

        /* Some unrecoverable errors */
      case SSL_ERROR_ZERO_RETURN:
      case SSL_ERROR_SSL:

        rc = -1;
        break;
      case SSL_ERROR_SYSCALL:
        /* Handle EAGAIN */
        if (errno == EAGAIN)
          continue;

        /* If other unrecoverable error occured */
      default:
        rc = -1;
        break;
      }
    } while (rc != SSL_ERROR_NONE && rc != -1);

    /* Clear memory and return if connection failed */
    if (rc) {

      SSL_clear(ssl_handle);
      SSL_free(ssl_handle);
      return rc;
    }

    /* Get peer certificate */
    if (!(peer_cert = SSL_get_peer_certificate(ssl_handle))) {

      /* Error getting peer credentials */
      rc = -1;
      set_blocking_(fd);
      SSL_shutdown(ssl_handle);
      set_non_blocking_(fd);
      SSL_free(ssl_handle);
      raise_ssl_error_(ssl_handle, rc);
    } else {
      /* Verify signature of peer certificate */
      if ((rc = sig_verify_(peer_cert, ca_cert_.get())) > 0) {

        /* Ok, fill private members */
        tls_handle_lock_.lock();
        tls_handle_ = std::shared_ptr<SSL>(ssl_handle, [](const auto &data) -> void { SSL_free(data); });
        tls_handle_lock_.unlock();

        tls_context_lock_.lock();
        tls_context_ = std::move(tls_context);
        tls_context_lock_.unlock();

        cert_lock_.lock();
        cert_ = std::move(x509_cert);
        cert_lock_.unlock();

        rsa_priv_key_lock_.lock();
        rsa_priv_key_ = std::move(rsa_priv_key);
        rsa_priv_key_lock_.unlock();
        rc = 0;
      } else {

        /* Signature verifying error, clear memory */
        set_blocking_(fd);
        SSL_shutdown(ssl_handle);
        set_non_blocking_(fd);
        SSL_free(ssl_handle);
        rc = -1;
      }

      X509_free(peer_cert);
    }

    return rc;
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type clear_(int32_t fd) {
    if (tls_handle_) {
      set_blocking_(fd);
      SSL_shutdown(tls_handle_.get());
      set_non_blocking_(fd);
    }

    cert_.reset();
    rsa_priv_key_.reset();
    tls_context_.reset();
    tls_handle_.reset();
    return 0;
  }

  void init_(const std::string &ca_cert_file, const std::string &ca_priv_key_file) {
    /* Generate signed with CA certificate and private key */
    auto [x509_ca_cert, rsa_ca_priv_key] = load_ca_(ca_cert_file, ca_priv_key_file);

    /* Init private members */
    ca_files_lock_.lock();
    ca_cert_ = std::move(x509_ca_cert);
    rsa_ca_priv_key_ = std::move(rsa_ca_priv_key);
    ca_files_lock_.unlock();
  }

  std::pair<std::shared_ptr<X509>, std::shared_ptr<RSA>> load_ca_(const std::string &ca_cert_file,
                                                                  const std::string &ca_priv_key_file) const {
    BIO *bio = nullptr;
    X509 *ca_cert = nullptr;
    RSA *ca_rsa_priv_key = nullptr;

    /* Allocate buffer */
    if (!(bio = BIO_new(BIO_s_file()))) {
      raise_error_();
    }

    /* Read certificate file to buffer */
    if (!BIO_read_filename(bio, ca_cert_file.c_str())) {
      BIO_free_all(bio);
      raise_error_();
    }

    /* Read CA certificate from buffer to X509 */
    if (!(ca_cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr))) {
      BIO_free_all(bio);
      raise_error_();
    }

    /* Free buffer */
    BIO_free_all(bio);

    /* Re-allocate buffer */
    if (!(bio = BIO_new(BIO_s_file()))) {
      raise_error_();
    }

    /* Read CA private key from file */
    if (!BIO_read_filename(bio, ca_priv_key_file.c_str())) {
      BIO_free_all(bio);
      raise_error_();
    }

    /* Read private key from buffer to RSA */
    if (!(ca_rsa_priv_key = PEM_read_bio_RSAPrivateKey(bio, &ca_rsa_priv_key, nullptr, nullptr))) {
      BIO_free_all(bio);
      raise_error_();
    }

    /* Free buffer */
    BIO_free_all(bio);
    return {std::shared_ptr<X509>(ca_cert, [](const auto &data) -> void { X509_free(data); }),
            std::shared_ptr<RSA>(ca_rsa_priv_key, [](const auto &data) -> void { RSA_free(data); })};
  }

  std::tuple<std::shared_ptr<X509>, std::shared_ptr<X509>, std::shared_ptr<RSA>, std::shared_ptr<RSA>>
  gen_signed_key_cert_(const std::string &ca_cert_file, const std::string &ca_priv_key_file) const {
    X509 *x509_cert;
    EVP_PKEY *req_pkey, *ca_priv_key;
    int32_t rc;

    /* Generate unsigned certificate and rsa key */
    auto [x509_req, rsa_priv_key] = gen_key_csr_();

    /* Load CA certificate and private key */
    auto [x509_ca_cert, rsa_ca_priv_key] = load_ca_(ca_cert_file, ca_priv_key_file);
    /* Create new X509 certificate */
    if (!(x509_cert = X509_new())) {
      raise_error_();
    }

    /* Allocate memory for CA private key */
    if (!(ca_priv_key = EVP_PKEY_new())) {
      raise_error_();
    }

    /* Configure certificate */
    X509_set_version(x509_cert, 2u);
    set_rnd_sn_(&x509_cert);
    X509_set_issuer_name(x509_cert, X509_get_subject_name(x509_ca_cert.get()));
    X509_gmtime_adj(X509_get_notBefore(x509_cert), 0u);
    X509_gmtime_adj(X509_get_notAfter(x509_cert), exp_time_);
    X509_set_subject_name(x509_cert, X509_REQ_get_subject_name(x509_req.get()));

    /* Set public key for new certificate */
    req_pkey = X509_REQ_get_pubkey(x509_req.get());
    X509_set_pubkey(x509_cert, req_pkey);

    /* Assign CA private key to allocated memory */
    if (!EVP_PKEY_assign_RSA(ca_priv_key, rsa_ca_priv_key.get())) {
      raise_error_();
    }

    /* Sign our new certificate with Sha256 hash and CA private key */
    if (!X509_sign(x509_cert, ca_priv_key, EVP_sha256())) {
      raise_error_();
    }

    /* Verify signature */
    if ((rc = sig_verify_(x509_cert, x509_ca_cert.get())) <= 0) {
      raise_error_();
    }

    EVP_PKEY_free(req_pkey);
    return {std::move(x509_ca_cert),
            std::shared_ptr<X509>(x509_cert, [](const auto &data) -> void { X509_free(data); }),
            std::move(rsa_ca_priv_key), std::move(rsa_priv_key)};

  error:
    raise_error_();
    return {std::shared_ptr<X509>(), std::shared_ptr<X509>(), std::shared_ptr<RSA>(), std::shared_ptr<RSA>()};
  }

  std::tuple<std::shared_ptr<X509>, std::shared_ptr<RSA>> gen_signed_key_cert_() const {
    X509 *x509_cert;
    EVP_PKEY *req_pkey, *ca_priv_key;
    int32_t rc;

    /* Generate unsigned certificate and rsa key */
    auto [x509_req, rsa_priv_key] = gen_key_csr_();

    /* Create new X509 certificate */
    if (!(x509_cert = X509_new())) {
      raise_error_();
    }

    /* Allocate memory for CA private key */
    if (!(ca_priv_key = EVP_PKEY_new())) {
      raise_error_();
    }

    std::shared_ptr<X509> ca_cert(X509_dup(ca_cert_.get()), [](const auto &data) -> void { X509_free(data); });

    /* Configure certificate */
    X509_set_version(x509_cert, 2u);
    set_rnd_sn_(&x509_cert);
    X509_set_issuer_name(x509_cert, X509_get_subject_name(ca_cert.get()));
    X509_gmtime_adj(X509_get_notBefore(x509_cert), 0u);
    X509_gmtime_adj(X509_get_notAfter(x509_cert), exp_time_);
    X509_set_subject_name(x509_cert, X509_REQ_get_subject_name(x509_req.get()));

    /* Set public key for new certificate */
    req_pkey = X509_REQ_get_pubkey(x509_req.get());
    X509_set_pubkey(x509_cert, req_pkey);

    /* Assign CA private key to allocated memory */
    std::shared_ptr<RSA> rsa_ca_priv_key(RSAPrivateKey_dup(rsa_ca_priv_key_.get()),
                                         [](const auto &data) -> void { RSA_free(data); });

    if (!EVP_PKEY_assign_RSA(ca_priv_key, rsa_ca_priv_key.get())) {
      raise_error_();
    }

    /* Sign our new certificate with Sha256 hash and CA private key */
    if (!X509_sign(x509_cert, ca_priv_key, EVP_sha256())) {
      raise_error_();
    }

    /* Verify signature */
    if ((rc = sig_verify_(x509_cert, ca_cert.get())) <= 0) {
      raise_error_();
    }

    EVP_PKEY_free(req_pkey);
    return {std::shared_ptr<X509>(x509_cert, [](const auto &data) -> void { X509_free(data); }),
            std::move(rsa_priv_key)};

  error:
    raise_error_();
    return {std::shared_ptr<X509>(), std::shared_ptr<RSA>()};
  }

  std::tuple<std::shared_ptr<X509_REQ>, std::shared_ptr<RSA>> gen_key_csr_() const {
    int32_t rc;
    RSA *rsa_keypair, *rsa_pub_key, *rsa_priv_key;
    BIGNUM *bn;
    EVP_PKEY *pk, *privk;
    X509_REQ *x509_req;
    X509_NAME *x509_name;

    /* Generate keypair */
    if (!(bn = BN_new())) {
      raise_error_();
    }

    if ((rc = BN_set_word(bn, RSA_F4)) != 1) {
      raise_error_();
    }

    if (!(pk = EVP_PKEY_new())) {
      raise_error_();
    }

    if (!(privk = EVP_PKEY_new())) {
      raise_error_();
    }

    if (!(rsa_keypair = RSA_new())) {
      raise_error_();
    }

    if ((rc = RSA_generate_key_ex(rsa_keypair, bits, bn, nullptr)) != 1) {
      raise_error_();
    }

    if (!(rsa_pub_key = RSAPublicKey_dup(rsa_keypair))) {
      raise_error_();
    }

    if (!(rsa_priv_key = RSAPrivateKey_dup(rsa_keypair))) {
      raise_error_();
    }

    if (!EVP_PKEY_assign_RSA(pk, rsa_pub_key)) {
      raise_error_();
    }

    if (!EVP_PKEY_assign_RSA(privk, rsa_priv_key)) {
      raise_error_();
    }

    /* Generate CSR */
    if (!(x509_req = X509_REQ_new())) {
      raise_error_();
    }

    if (!X509_REQ_set_pubkey(x509_req, pk)) {
      raise_error_();
    }

    /* Configure reqest. */
    x509_name = X509_REQ_get_subject_name(x509_req);
    X509_NAME_add_entry_by_txt(x509_name, cert_info_.country_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.country_value()), -1, -1, 0);

    X509_NAME_add_entry_by_txt(x509_name, cert_info_.state_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.state_value()), -1, -1, 0);

    X509_NAME_add_entry_by_txt(x509_name, cert_info_.locality_name_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.locality_name_value()), -1, -1, 0);

    X509_NAME_add_entry_by_txt(x509_name, cert_info_.organization_name_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.organization_name_value()), -1, -1, 0);

    X509_NAME_add_entry_by_txt(x509_name, cert_info_.organizational_unit_name_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.organizational_unit_name_value()), -1, -1,
                               0);

    X509_NAME_add_entry_by_txt(x509_name, cert_info_.common_name_token(), MBSTRING_ASC,
                               reinterpret_cast<const uint8_t *>(cert_info_.common_name_value()), -1, -1, 0);

    /* Self - signing request by sha256 hash and generated private key */
    if (!X509_REQ_sign(x509_req, privk, EVP_sha256())) {
      raise_error_();
    }

    BN_free(bn);
    RSA_free(rsa_keypair);
    return {std::shared_ptr<X509_REQ>(x509_req, [](const auto &data) -> void { X509_REQ_free(data); }),
            std::shared_ptr<RSA>(rsa_priv_key, [](const auto &data) -> void { RSA_free(data); })};

  error:
    BN_free(bn);
    RSA_free(rsa_keypair);
    RSA_free(rsa_pub_key);
    RSA_free(rsa_priv_key);
    X509_REQ_free(x509_req);
    raise_error_();
    return {std::shared_ptr<X509_REQ>(), std::shared_ptr<RSA>()};
  }

  void set_rnd_sn_(X509 **crt) const {

    uint8_t sn[20u];
    BIGNUM *bn;
    ASN1_INTEGER *serial;

    /* Set random bytes to array */
    if (!(RAND_bytes(sn, sizeof(sn)))) {
      raise_error_();
    }

    if (!(bn = BN_new())) {
      raise_error_();
    }

    if (!BN_bin2bn(sn, sizeof(sn), bn)) {
      raise_error_();
    }

    if (!(serial = ASN1_INTEGER_new())) {
      raise_error_();
    }

    if (!BN_to_ASN1_INTEGER(bn, serial)) {
      raise_error_();
    }

    /* Set random serial number to X509 certificate */
    if (!X509_set_serialNumber(*crt, serial)) {
      raise_error_();
    }

    ASN1_INTEGER_free(serial);
    BN_free(bn);
    return;

  error:
    ASN1_INTEGER_free(serial);
    BN_free(bn);
    raise_error_();
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  recv_(int32_t fd, void *buffer, size_t nbytes) const {
    int32_t rc, recvd;
    fd_set read_fd_set;
    fd_set generic_fd_set;
    struct timeval read_timeout = {1u, 0u};

    if (tls_handle_) {
      SSL *ssl_handle = tls_handle_.get();
      do {
        switch (rc = SSL_get_error(ssl_handle, (recvd = SSL_read(ssl_handle, buffer, nbytes)))) {
        case SSL_ERROR_NONE:
          break;
        case SSL_ERROR_WANT_READ:

          /* Handle this state */
          FD_ZERO(&read_fd_set);
          FD_SET(fd, &read_fd_set);

          if ((rc = ::select(fd + 1u, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_WANT_ACCEPT:
        case SSL_ERROR_WANT_CONNECT:
        case SSL_ERROR_WANT_X509_LOOKUP:
        case SSL_ERROR_WANT_WRITE:

          /* Handle generic state */
          FD_ZERO(&read_fd_set);
          FD_SET(fd, &read_fd_set);

          if ((rc = ::select(fd + 1u, &generic_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_ZERO_RETURN:
        case SSL_ERROR_SSL:
        case SSL_ERROR_SYSCALL:
        default:
          rc = -1;
          break;
        }
      } while (rc != SSL_ERROR_NONE && rc != -1);
      if (rc)
        return rc;

      return recvd;
    } else {

      return -1;
    }
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, RetType>::type
  send_(int32_t fd, const void *const msg, size_t msg_size) const {
    int32_t rc, sent;
    fd_set write_fd_set;
    fd_set generic_fd_set;
    struct timeval write_timeout = {1u, 0u};

    if (tls_handle_) {
      SSL *ssl_handle = tls_handle_.get();
      do {
        switch (rc = SSL_get_error(ssl_handle, (sent = SSL_write(ssl_handle, msg, msg_size)))) {
        case SSL_ERROR_NONE:
          break;
        case SSL_ERROR_WANT_WRITE:

          /* Handle this state */
          FD_ZERO(&write_fd_set);
          FD_SET(fd, &write_fd_set);

          if ((rc = ::select(fd + 1u, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_WANT_ACCEPT:
        case SSL_ERROR_WANT_CONNECT:
        case SSL_ERROR_WANT_X509_LOOKUP:
        case SSL_ERROR_WANT_READ:

          /* Handle generic state */
          FD_ZERO(&write_fd_set);
          FD_SET(fd, &write_fd_set);

          if ((rc = ::select(fd + 1u, &generic_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }
        case SSL_ERROR_ZERO_RETURN:
        case SSL_ERROR_SSL:
        case SSL_ERROR_SYSCALL:
        default:
          rc = -1;
          break;
        }
      } while (rc != SSL_ERROR_NONE && rc != -1);
      if (rc)
        return rc;

      return sent;
    } else {

      return -1;
    }
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  recv_(int32_t fd, void *buffer, size_t nbytes) const {
    int32_t rc, recvd;
    fd_set read_fd_set;
    fd_set generic_fd_set;
    struct timeval read_timeout = {1u, 0u};

    if (auto tls_handle = tls_handle_.find(fd); tls_handle != tls_handle_.end()) {
      SSL *ssl_handle = tls_handle->second.get();
      do {
        switch (rc = SSL_get_error(ssl_handle, (recvd = SSL_read(ssl_handle, buffer, nbytes)))) {
        case SSL_ERROR_NONE:
          break;
        case SSL_ERROR_WANT_READ:

          /* Handle this state */
          FD_ZERO(&read_fd_set);
          FD_SET(fd, &read_fd_set);

          if ((rc = ::select(fd + 1u, &read_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_WANT_ACCEPT:
        case SSL_ERROR_WANT_CONNECT:
        case SSL_ERROR_WANT_X509_LOOKUP:
        case SSL_ERROR_WANT_WRITE:

          /* Handle generic state */
          FD_ZERO(&read_fd_set);
          FD_SET(fd, &read_fd_set);

          if ((rc = ::select(fd + 1u, &generic_fd_set, nullptr, nullptr, &read_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_ZERO_RETURN:
        case SSL_ERROR_SSL:
        case SSL_ERROR_SYSCALL:
        default:
          rc = -1;
          break;
        }
      } while (rc != SSL_ERROR_NONE && rc != -1);
      if (rc)
        return rc;

      return recvd;
    } else {

      return -1;
    }
  }

  template <tcp_sock_secure_type_e st = secure_socket_type, typename RetType = int32_t>
  typename std::enable_if<st == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, RetType>::type
  send_(int32_t fd, const void *const msg, size_t msg_size) const {
    int32_t rc, sent;
    fd_set write_fd_set;
    fd_set generic_fd_set;
    struct timeval write_timeout = {1u, 0u};

    if (auto tls_handle = tls_handle_.find(fd); tls_handle != tls_handle_.end()) {
      SSL *ssl_handle = tls_handle->second.get();
      do {
        switch (rc = SSL_get_error(ssl_handle, (sent = SSL_write(ssl_handle, msg, msg_size)))) {
        case SSL_ERROR_NONE:
          break;
        case SSL_ERROR_WANT_WRITE:

          /* Handle this state */
          FD_ZERO(&write_fd_set);
          FD_SET(fd, &write_fd_set);

          if ((rc = ::select(fd + 1u, &write_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }

        case SSL_ERROR_WANT_ACCEPT:
        case SSL_ERROR_WANT_CONNECT:
        case SSL_ERROR_WANT_X509_LOOKUP:
        case SSL_ERROR_WANT_READ:

          /* Handle generic state */
          FD_ZERO(&write_fd_set);
          FD_SET(fd, &write_fd_set);

          if ((rc = ::select(fd + 1u, &generic_fd_set, nullptr, nullptr, &write_timeout)) <= 0) {
            /* Error or timeout */
            rc = -1;
            break;
          } else {
            /* Ok, continuing */
            continue;
          }
        case SSL_ERROR_ZERO_RETURN:
        case SSL_ERROR_SSL:
        case SSL_ERROR_SYSCALL:
        default:
          rc = -1;
          break;
        }
      } while (rc != SSL_ERROR_NONE && rc != -1);
      if (rc)
        return rc;

      return sent;
    } else {

      return -1;
    }
  }

  const std::string ca_cert_file_;
  const std::string ca_key_file_;

  std::shared_ptr<X509> ca_cert_;
  std::shared_ptr<RSA> rsa_ca_priv_key_;
  mutable std::mutex ca_files_lock_;

  mutable x509_cert_info_t cert_info_;
  const uint64_t exp_time_;

  std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, std::shared_ptr<X509>,
                     std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS,
                                        std::map<int32_t, std::shared_ptr<X509>>, void *>>
      cert_;
  mutable std::mutex cert_lock_;

  std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, std::shared_ptr<RSA>,
                     std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS,
                                        std::map<int32_t, std::shared_ptr<RSA>>, void *>>
      rsa_priv_key_;
  mutable std::mutex rsa_priv_key_lock_;

  std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, std::shared_ptr<SSL_CTX>,
                     std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS,
                                        std::map<int32_t, std::shared_ptr<SSL_CTX>>, void *>>
      tls_context_;
  mutable std::mutex tls_context_lock_;

  std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, std::shared_ptr<SSL>,
                     std::conditional_t<secure_socket_type == tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS,
                                        std::map<int32_t, std::shared_ptr<SSL>>, void *>>
      tls_handle_;
  mutable std::mutex tls_handle_lock_;
};

#endif /* TLS_HPP */
