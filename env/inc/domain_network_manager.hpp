#ifndef DOMAIN_NETWORK_MANAGER_HPP
#define DOMAIN_NETWORK_MANAGER_HPP

#include "domain_tcp_sock_secure.hpp"
#include "domain_udp_sock_secure.hpp"
#include "env_base.hpp"
#include "network_manager_base.hpp"
#include "sha256.hpp"

template <nm_networking_t nt = nm_networking_t::DOMAIN> struct domain_nm_t : public nm_base_t {
public:
  using this_t = domain_nm_t<nt>;
  using base_t = nm_base_t;
  static constexpr uint32_t known_env_lifetime_s = 5u;

  explicit domain_nm_t(
      const char (&dgram_aes_key)
          [(domain_udp_socket_secure<udp_sock_secure_t::SERVER_UNICAST_SECURE_AES>::dgram_aes_key_size_bits / 8u) + 1u],
      const struct env_base_t *const p_env)
      : p_env_(p_env),

        domain_dgram_srv(domain_udp_socket_secure<udp_sock_secure_t::SERVER_UNICAST_SECURE_AES>(
            p_env->info().domain_udp_socket_path(), dgram_aes_key)),

        domain_dgram_cli(domain_udp_socket_secure<udp_sock_secure_t::CLIENT_UNICAST_SECURE_AES>(dgram_aes_key)),

        domain_stream_srv(domain_tcp_socket_secure<tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS>(
            p_env->info().domain_stream_socket_path(), p_env->info().env_ca_cert_file(),
            p_env->info().env_ca_priv_key_file(), p_env->info().env_cert_info(), p_env->info().env_cert_exp_time())) {

    /* Initialize OpenSSL */
    SSL_load_error_strings();
    SSL_library_init();
    OpenSSL_add_all_algorithms();
    ERR_load_BIO_strings();

    host_hash_ = sha256::compute(reinterpret_cast<const uint8_t *>(domain_stream_srv.path().data()),
                                 domain_stream_srv.path().size());
  }

  virtual ~domain_nm_t() {
    ERR_free_strings();
    EVP_cleanup();
  };

  virtual void run() override final {
    domain_stream_srv.setup();
    domain_dgram_srv.setup();

    domain_dgram_srv.on_receive().add(
        "OnReceiveProbe",
        [this](struct sockaddr_un peer_path, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          env_config_header_t header;
          header.ParseFromArray(data.get(), size);

          /* Calculate hashes */
          sha256::sha256_hash_type invite_hash = sha256::sha256_from_string(header.env_invite());

          sha256::sha256_hash_type peer_path_hash =
              sha256::compute(reinterpret_cast<const uint8_t *>(peer_path.sun_path), sizeof(peer_path.sun_path));

          /* Add this env to known (not connected in both sides) */
          if (peer_path_hash != host_hash_) {
            std::lock_guard<std::recursive_mutex> lock_envs(known_envs_lock_);
            std::lock_guard<std::recursive_mutex> lock_peers(peers_lock_);

            if (std::find(known_envs_.begin(), known_envs_.end(), peer_path_hash) == known_envs_.end()) {

              known_envs_.push_back(peer_path_hash);
              std::thread([this, peer_path_hash]() -> void {
                std::this_thread::sleep_for(std::chrono::seconds(known_env_lifetime_s));

                {
                  std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
                  if (auto it = std::find(known_envs_.begin(), known_envs_.end(), peer_path_hash);
                      it != known_envs_.end()) {
                    known_envs_.erase(it);
                  }
                }
              }).detach();
            }

            if (invite_hash == host_hash_ && !out_connection_established_(peer_path_hash)) {
              static_cast<void>(handle_probe_(peer_path.sun_path, peer_path_hash, header));
            }
          }
        });

    domain_stream_srv.on_connect().add("OnConnectHook", [this](struct sockaddr_un peer, auto *unit) -> void {
      std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
      std::printf("%s", (boost::format("Server: peer %1% connected!\r\n") % peer.sun_path).str().c_str());
    });

    domain_stream_srv.on_disconnect().add("OnDisonnectHook", [this](struct sockaddr_un peer, auto *unit) -> void {
      std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
      std::printf("%s", (boost::format("Server: peer %1%:%2% disconnected!\r\n") % peer.sun_path).str().c_str());
    });

    domain_stream_srv.on_receive().add(
        "OnReceiveHook", [this](struct sockaddr_un peer, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          std::printf("%s", (boost::format("Server: received message \"%1%\" from peer %2%:%3%\r\n") %
                             std::string(reinterpret_cast<char *>(data.get()), size) % peer.sun_path)
                                .str()
                                .c_str());
        });

    domain_stream_srv.start();
    domain_dgram_srv.start();
    auto run_thr = std::thread(&this_t::run_probing_, this, 60u, 1000u);

    run_thr.join();
    stop();
  }

  virtual void stop() override final {
    domain_dgram_srv.stop();
    domain_dgram_srv.reset();

    domain_dgram_cli.reset();

    domain_stream_srv.stop();
    domain_stream_srv.reset();

    std::vector<domain_tcp_socket_secure<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS> *> peers;

    peers_lock_.lock();
    for (auto &peer : peers_) {
      peers.push_back(std::get<2u>(peer.second).get());
    }

    peers_lock_.unlock();
    for (auto &peer_ptr : peers) {
      peer_ptr->stop_threads();
      peer_ptr->reset();
    }
  }

private:
  mutable domain_udp_socket_secure<udp_sock_secure_t::SERVER_UNICAST_SECURE_AES> domain_dgram_srv;
  mutable domain_udp_socket_secure<udp_sock_secure_t::CLIENT_UNICAST_SECURE_AES> domain_dgram_cli;
  mutable domain_tcp_socket_secure<tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS> domain_stream_srv;

  mutable std::vector<sha256::sha256_hash_type> known_envs_;
  mutable std::map<sha256::sha256_hash_type,
                   std::tuple<env_config_header_t, std::string,
                              std::shared_ptr<domain_tcp_socket_secure<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>>>>
      peers_;

  mutable std::recursive_mutex known_envs_lock_;
  mutable std::recursive_mutex peers_lock_;

  sha256::sha256_hash_type host_hash_;
  const struct env_base_t *const p_env_;
  const struct env_base_t *const env_() const { return p_env_; };

  bool out_connection_established_(const sha256::sha256_hash_type &peer_hash) const {
    return peers_.find(peer_hash) != peers_.end();
  }

  bool in_connection_established_(const sha256::sha256_hash_type &hash) const {
    return domain_stream_srv.is_peer_connected(hash);
  }

  void run_probing_(uint64_t times, uint64_t interval_ms) {

    /* Get some info from host environment */
    void *payload = nullptr;
    this->env_()->get_lock().lock();
    env_config_header_t header = this->env_()->info_header();
    const std::string &mcast_addr = this->env_()->info().env_ipv4_multicast_group_addr();
    uint16_t mcast_srv = this->env_()->info().env_ipv4_multicast_port();
    this->env_()->get_lock().unlock();

    for (uint32_t i = 0u; i < times; i++) {
      /* Decide who is next for inviting */
      {
        std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
        for (const auto &hash : known_envs_) {
          if (!in_connection_established_(hash)) {

            header.set_env_invite(reinterpret_cast<const char *>(hash.data()));
            break;
          }
        }
      }

      size_t payload_size = header.ByteSizeLong();
      payload = std::malloc(payload_size);
      header.SerializeToArray(payload, payload_size);
      domain_dgram_cli.send(mcast_addr, payload, payload_size);
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    std::free(payload);
  }

  int32_t handle_probe_(const std::string &peer_path, const sha256::sha256_hash_type &peer_hash,
                        const env_config_header_t &header) const {
    int32_t rc;
    if (peer_path != domain_stream_srv.path()) {
      int32_t pid = header.env_pid();

      typename decltype(peers_)::iterator it;
      if ((it = peers_.find(peer_hash)) == peers_.end()) {
        std::shared_ptr<domain_tcp_socket_secure<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>> unit(
            std::shared_ptr<domain_tcp_socket_secure<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>>(
                new domain_tcp_socket_secure<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>(
                    this->env_()->info().env_ca_cert_file(), this->env_()->info().env_ca_priv_key_file(),
                    this->env_()->info().env_cert_info(), this->env_()->info().env_cert_exp_time())));

        unit->on_connect().add(
            "OnConnectHook",
            [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](struct sockaddr_un peer,
                                                                                   auto *unit) -> void {
              std::printf("%s", (boost::format("Client: connected to %1%\r\n") % peer.sun_path).str().c_str());
            });

        unit->on_disconnect().add(
            "OnDisconnectHook",
            [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](struct sockaddr_un peer,
                                                                                   auto *unit) -> void {
              std::lock_guard<std::recursive_mutex> lock(peers_lock_);
              std::printf("%s", (boost::format("Client: disconnected from %1%\r\n") % peer.sun_path).str().c_str());
              peers_.erase(hash);
            });

        if (!(rc = unit->connect(peer_path))) {

          std::lock_guard<std::recursive_mutex> lock_peers(peers_lock_);
          peers_.insert(std::make_pair(peer_hash, std::make_tuple(header, std::string(peer_path), std::move(unit))));
        } else {

          unit->stop_threads();
          unit.reset();
          rc = -1;
        }
      } else {
        rc = 1;
      }
    } else {

      rc = -1;
    }

    return rc;
  }
};

#endif /* DOMAIN_NETWORK_MANAGER_HPP */
