#ifndef NETWORK_MANAGER_HPP
#define NETWORK_MANAGER_HPP

#include "env_base.hpp"
#include "network_manager_base.hpp"
#include "network_tcp_sock_secure.hpp"
#include "network_udp_sock_secure.hpp"
#include "sha256.hpp"

#include <forward_list>
#include <shared_mutex>
#include <thread>

template <nm_networking_t nt = nm_networking_t::IPV4> struct nm_t : public nm_base_t {
public:
  using this_t = nm_t;
  using base_t = nm_base_t;
  static constexpr uint32_t known_env_lifetime_s = 5u;

  explicit nm_t(
      const char (&dgram_aes_key)
          [(network_udp_socket_secure_ipv4<udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES>::dgram_aes_key_size_bits /
            8u) +
           1u],
      const struct env_base_t *const p_env)
      : p_env_(p_env),

        ipv4_dgram_srv(network_udp_socket_secure_ipv4<udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES>(
            p_env->info().env_network_ifname(), dgram_aes_key)),

        ipv4_dgram_cli(network_udp_socket_secure_ipv4<udp_sock_secure_t::CLIENT_MULTICAST_SECURE_AES>(
            p_env->info().env_network_ifname(), p_env->info().env_ipv4_multicast_group_addr(),
            p_env->info().env_ipv4_multicast_port(), dgram_aes_key)),

        ipv4_stream_srv(network_tcp_socket_secure_ipv4<tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS>(
            p_env->info().env_network_ifname(), p_env->info().env_ca_cert_file(), p_env->info().env_ca_priv_key_file(),
            p_env->info().env_cert_info(), p_env->info().env_cert_exp_time())) {

    /* Initialize OpenSSL */
    SSL_load_error_strings();
    SSL_library_init();
    OpenSSL_add_all_algorithms();
    ERR_load_BIO_strings();

    uint16_t port = this->env_()->info().env_ipv4_stream_port();
    host_hash_ = sha256::compute(reinterpret_cast<const uint8_t *>(ipv4_stream_srv.iface().host_addr.data()),
                                 ipv4_stream_srv.iface().host_addr.size()) ^
                 sha256::compute(reinterpret_cast<const uint8_t *>(&port), sizeof(port));
  }

  virtual ~nm_t() {
    ERR_free_strings();
    EVP_cleanup();
  };

  virtual void run() override final {
    ipv4_stream_srv.setup(this->env_()->info().env_ipv4_stream_port());
    ipv4_dgram_srv.setup(this->env_()->info().env_ipv4_multicast_group_addr(),
                         this->env_()->info().env_ipv4_multicast_port());

    ipv4_dgram_srv.on_receive().add(
        "OnReceiveProbe", [this](struct sockaddr_in peer, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          env_config_header_t header;
          char peer_addr[INET_ADDRSTRLEN] = {0x0};
          uint16_t peer_srv = ::htons(peer.sin_port);
          ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
          header.ParseFromArray(data.get(), size);

          /* Calculate hashes */
          sha256::sha256_hash_type invite_hash = sha256::sha256_from_string(header.env_invite());

          uint16_t port = header.env_ipv4_stream_port();
          sha256::sha256_hash_type peer_addr_hash =
              sha256::compute(reinterpret_cast<const uint8_t *>(peer_addr), sizeof(peer_addr)) ^
              sha256::compute(reinterpret_cast<const uint8_t *>(&port), sizeof(port));

          /* Add this env to known (not connected in both sides) */
          if (peer_addr_hash != host_hash_) {
            std::lock_guard<std::recursive_mutex> lock_envs(known_envs_lock_);
            std::lock_guard<std::recursive_mutex> lock_peers(peers_lock_);

            if (std::find(known_envs_.begin(), known_envs_.end(), std::make_pair(peer_addr_hash, port)) ==
                known_envs_.end()) {

              known_envs_.push_back(std::make_pair(peer_addr_hash, port));
              std::thread([this, peer_addr_hash, port]() -> void {
                std::this_thread::sleep_for(std::chrono::seconds(known_env_lifetime_s));

                {
                  std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
                  if (auto it = std::find(known_envs_.begin(), known_envs_.end(), std::make_pair(peer_addr_hash, port));
                      it != known_envs_.end()) {
                    known_envs_.erase(it);
                  }
                }
              }).detach();
            }

            if (invite_hash == host_hash_ && !out_connection_established_(peer_addr_hash)) {
              static_cast<void>(handle_probe_(peer_addr, peer_addr_hash, header));
            }
          }
        });

    ipv4_stream_srv.on_connect().add("OnConnectHook", [this](struct sockaddr_in peer, auto *unit) -> void {
      std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
      uint16_t peer_srv = ::htons(peer.sin_port);
      char peer_addr[INET_ADDRSTRLEN] = {0x0};
      ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
      fmt::print("Server: peer {0}:{1} connected!\r\n", peer_addr, peer_srv);
    });

    ipv4_stream_srv.on_disconnect().add("OnDisonnectHook", [this](struct sockaddr_in peer, auto *unit) -> void {
      std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
      uint16_t peer_srv = ::htons(peer.sin_port);
      char peer_addr[INET_ADDRSTRLEN] = {0x0};
      ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
      fmt::print("Server: peer {0}:{1} disconnected!\r\n", peer_addr, peer_srv);
    });

    ipv4_stream_srv.on_receive().add(
        "OnReceiveHook", [this](struct sockaddr_in peer, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          uint16_t peer_srv = ::htons(peer.sin_port);
          char peer_addr[INET_ADDRSTRLEN] = {0x0};
          ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
          fmt::print("Server: received message \"{0}\" from peer {1}:{2}\r\n",
                     std::string(reinterpret_cast<char *>(data.get()), size), peer_addr, peer_srv);
        });

    ipv4_stream_srv.start();
    ipv4_dgram_srv.start();
    auto run_thr = std::thread(&this_t::run_probing_, this, 60u, 1000u);

    run_thr.join();
    stop();
  }

  virtual void stop() override final {
    ipv4_dgram_srv.stop();
    ipv4_dgram_srv.reset();

    ipv4_dgram_cli.reset();

    ipv4_stream_srv.stop();
    ipv4_stream_srv.reset();

    std::vector<network_tcp_socket_secure_ipv4<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS> *> peers;

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
  mutable network_udp_socket_secure_ipv4<udp_sock_secure_t::SERVER_MULTICAST_SECURE_AES> ipv4_dgram_srv;
  mutable network_udp_socket_secure_ipv4<udp_sock_secure_t::CLIENT_MULTICAST_SECURE_AES> ipv4_dgram_cli;
  mutable network_tcp_socket_secure_ipv4<tcp_sock_secure_t::SERVER_UNICAST_SECURE_TLS> ipv4_stream_srv;

  mutable std::vector<std::pair<sha256::sha256_hash_type, uint16_t>> known_envs_;
  mutable std::map<
      sha256::sha256_hash_type,
      std::tuple<env_config_header_t, std::string,
                 std::shared_ptr<network_tcp_socket_secure_ipv4<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>>>>
      peers_;

  mutable std::recursive_mutex known_envs_lock_;
  mutable std::recursive_mutex peers_lock_;

  sha256::sha256_hash_type host_hash_;
  const struct env_base_t *const p_env_;
  const struct env_base_t *const env_() const { return p_env_; };

  bool out_connection_established_(const sha256::sha256_hash_type &peer_hash) const {
    return peers_.find(peer_hash) != peers_.end();
  }

  bool in_connection_established_(const sha256::sha256_hash_type &hash, uint16_t port) const {
    return ipv4_stream_srv.is_peer_connected(hash, port);
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
        for (const auto &hash_srv_pair : known_envs_) {
          if (!in_connection_established_(hash_srv_pair.first, hash_srv_pair.second)) {

            header.set_env_invite(reinterpret_cast<const char *>(hash_srv_pair.first.data()));
            break;
          }
        }
      }

      size_t payload_size = header.ByteSizeLong();
      payload = std::malloc(payload_size);
      header.SerializeToArray(payload, payload_size);
      ipv4_dgram_cli.send(mcast_addr, mcast_srv, payload, payload_size);
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    std::free(payload);
  }

  int32_t handle_probe_(const std::string &peer_addr, const sha256::sha256_hash_type &peer_hash,
                        const env_config_header_t &header) const {
    int32_t rc;
    if (peer_addr != ipv4_stream_srv.iface().host_addr.data()) {
      int32_t pid = header.env_pid();

      typename decltype(peers_)::iterator it;
      if ((it = peers_.find(peer_hash)) == peers_.end()) {
        std::shared_ptr<network_tcp_socket_secure_ipv4<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>> unit(
            std::shared_ptr<network_tcp_socket_secure_ipv4<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>>(
                new network_tcp_socket_secure_ipv4<tcp_sock_secure_t::CLIENT_UNICAST_SECURE_TLS>(
                    this->env_()->info().env_network_ifname(), this->env_()->info().env_ca_cert_file(),
                    this->env_()->info().env_ca_priv_key_file(), this->env_()->info().env_cert_info(),
                    this->env_()->info().env_cert_exp_time())));

        unit->on_connect().add("OnConnectHook",
                               [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](
                                   struct sockaddr_in peer, auto *unit) -> void {
                                 uint16_t peer_srv = ::htons(peer.sin_port);
                                 char peer_addr[INET_ADDRSTRLEN] = {0};
                                 ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
                                 fmt::print("Client: connected to {0}:{1}\r\n", peer_addr, peer_srv);
                               });

        unit->on_disconnect().add("OnDisconnectHook",
                                  [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](
                                      struct sockaddr_in peer, auto *unit) -> void {
                                    std::lock_guard<std::recursive_mutex> lock(peers_lock_);
                                    uint16_t peer_srv = ::htons(peer.sin_port);
                                    char peer_addr[INET_ADDRSTRLEN] = {0};
                                    ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
                                    fmt::print("Client: disconnected from {0}:{1}\r\n", peer_addr, peer_srv);
                                    peers_.erase(hash);
                                  });

        if (!(rc = unit->connect(peer_addr, header.env_ipv4_stream_port()))) {

          std::lock_guard<std::recursive_mutex> lock_peers(peers_lock_);
          peers_.insert(std::make_pair(peer_hash, std::make_tuple(header, std::string(peer_addr), std::move(unit))));
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

#endif /* NETWORK_MANAGER_HPP */
