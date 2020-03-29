#ifndef NETWORK_MANAGER_HPP
#define NETWORK_MANAGER_HPP

#include "domain_tcp_sock_secure.hpp"
#include "domain_udp_sock_secure.hpp"
#include "env_base.hpp"
#include "network_tcp_sock_secure.hpp"
#include "network_udp_sock_secure.hpp"
#include "sha256.hpp"

#include <forward_list>
#include <shared_mutex>
#include <thread>

template <env_networking_type_e, typename...> struct nm_s {};

template <env_networking_type_e nt> struct nm_sglt_gen_s {
  template <typename Enabled = void>
  static struct nm_s<nt> &nm_inst(const char (&key)[(aes_key_size_bits / UINT8_WIDTH) + 1u],
                                  const struct env_base_s *const p_env,
                                  typename std::enable_if<is_secure_type(nt), Enabled>::type * = nullptr) {
    static const std::unique_ptr<struct nm_s<nt>> inst { new nm_s<nt>(key, p_env) };
    return *inst;
  }

  template <typename Enabled = void>
  static struct nm_s<nt> &nm_inst(const struct env_base_s *const p_env,
                                  typename std::enable_if<!is_secure_type(nt), Enabled>::type * = nullptr) {
    static const std::unique_ptr<struct nm_s<nt>> inst { new nm_s<nt>(p_env) };
    return *inst;
  }
};

template <env_networking_type_e nt> struct nm_s<nt> {
private:
  using this_s = nm_s;
  static constexpr uint32_t known_env_lifetime_s = 5u;

  friend struct nm_sglt_gen_s<nt>;
  template <typename RetType>
  friend typename std::enable_if<is_secure_type(nt), RetType> &
  nm_sglt_gen_s<nt>::nm_inst(const char (&)[(aes_key_size_bits / UINT8_WIDTH) + 1u], const struct env_base_s *const);

  template <typename RetType>
  friend typename std::enable_if<!is_secure_type(nt), RetType> &
  nm_sglt_gen_s<nt>::nm_inst(const struct env_base_s *const);

public:
  explicit nm_s(const this_s &) = delete;
  explicit nm_s(this_s &&) = delete;
  this_s &operator=(const this_s &) = delete;
  this_s &operator=(this_s &&) = delete;

  virtual ~nm_s() {
    ERR_free_strings();
    EVP_cleanup();
  };

  void run() const {
    tcp_srv.setup(this->env_()->info().env_ipv4_tcp_port());
    udp_srv.setup(this->env_()->info().env_ipv4_multicast_group_addr(), this->env_()->info().env_ipv4_multicast_port());

    udp_srv.on_receive().add(
        "OnReceiveProbe", [this](struct sockaddr_in peer, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          struct env_cfg_header_s header;
          char peer_addr[INET_ADDRSTRLEN];
          uint16_t peer_srv = ::htons(peer.sin_port);
          ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
          header.ParseFromArray(data.get(), size);

          sha256::sha256_hash_type invite_hash = sha256::sha256_from_string(header.env_invite());
          uint16_t port = header.env_ipv4_tcp_port();
          sha256::sha256_hash_type peer_addr_hash =
              sha256::compute(reinterpret_cast<const uint8_t *>(peer_addr), sizeof(peer_addr)) ^
              sha256::compute(reinterpret_cast<const uint8_t *>(&port), sizeof(port));

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

    tcp_srv.on_connect().add("OnConnectHook", [this](struct sockaddr_in peer, auto *unit) -> void {
      uint16_t peer_srv = ::htons(peer.sin_port);
      char peer_addr[INET_ADDRSTRLEN];
      ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
      DEBUG_LOG((boost::format("Server: peer %1%:%2% connected!\r\n") % peer_addr % peer_srv).str());
    });

    tcp_srv.on_disconnect().add("OnDisonnectHook", [this](struct sockaddr_in peer, auto *unit) -> void {
      uint16_t peer_srv = ::htons(peer.sin_port);
      char peer_addr[INET_ADDRSTRLEN];
      ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
      DEBUG_LOG((boost::format("Server: peer %1%:%2% disconnected!\r\n") % peer_addr % peer_srv).str());
    });

    tcp_srv.on_receive().add(
        "OnReceiveHook", [this](struct sockaddr_in peer, std::shared_ptr<void> data, size_t size, auto *unit) -> void {
          uint16_t peer_srv = ::htons(peer.sin_port);
          char peer_addr[INET_ADDRSTRLEN];
          ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
          DEBUG_LOG((boost::format("Server: received message \"%1%\" from peer %2%:%3%\r\n") %
                     std::string(reinterpret_cast<char *>(data.get()), size) % peer_addr % peer_srv)
                        .str());
        });

    tcp_srv.start();
    udp_srv.start();
    auto run_thr = std::thread(&this_s::run_probing_, this, 60u, 1000u);

    run_thr.join();
    stop();
  }

  void stop() const {
    udp_srv.stop();
    udp_srv.reset();

    udp_cli.reset();

    tcp_srv.stop();
    tcp_srv.reset();

    std::vector<network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true> *> peers;

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
  template <typename Enabled = void>
  explicit nm_s(const struct env_base_s *const p_env,
                typename std::enable_if<!is_secure_type(nt), Enabled>::type * = nullptr)
      : p_env_(p_env),

        udp_srv(network_udp_socket_ipv4_s<udp_sock_type_e::SERVER_MULTICAST, true>(p_env->info().env_network_ifname())),

        udp_cli(network_udp_socket_ipv4_s<udp_sock_type_e::CLIENT_MULTICAST, true>(
            p_env->info().env_network_ifname(), p_env->info().env_ipv4_multicast_group_addr(),
            p_env->info().env_ipv4_multicast_port())),

        tcp_srv(network_tcp_socket_ipv4_s<tcp_sock_type_e::SERVER_UNICAST, true>(p_env->info().env_network_ifname())) {

    const uint16_t port = this->env_()->info().env_ipv4_tcp_port();
    host_hash_ = sha256::compute(reinterpret_cast<const uint8_t *>(tcp_srv.iface().host_addr.data()),
                                 tcp_srv.iface().host_addr.size()) ^
                 sha256::compute(reinterpret_cast<const uint8_t *>(&port), sizeof(port));
  }

  template <typename Enabled = void>
  explicit nm_s(const char (&key)[(aes_key_size_bits / UINT8_WIDTH) + 1u], const struct env_base_s *const p_env,
                typename std::enable_if<is_secure_type(nt), Enabled>::type * = nullptr)
      : p_env_(p_env),

        udp_srv(network_udp_socket_secure_ipv4_s<udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES, true,
                                                 aes_key_size_bits>(p_env->info().env_network_ifname(), key)),

        udp_cli(network_udp_socket_secure_ipv4_s<udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES, true,
                                                 aes_key_size_bits>(p_env->info().env_network_ifname(),
                                                                    p_env->info().env_ipv4_multicast_group_addr(),
                                                                    p_env->info().env_ipv4_multicast_port(), key)),

        tcp_srv(network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, true>(
            p_env->info().env_network_ifname(), p_env->info().env_ca_cert_file(), p_env->info().env_ca_priv_key_file(),
            p_env->info().env_cert_info(), p_env->info().env_cert_exp_time())) {

    /* Initialize OpenSSL */
    SSL_load_error_strings();
    SSL_library_init();
    OpenSSL_add_all_algorithms();
    ERR_load_BIO_strings();

    const uint16_t port = this->env_()->info().env_ipv4_tcp_port();
    host_hash_ = sha256::compute(reinterpret_cast<const uint8_t *>(tcp_srv.iface().host_addr.data()),
                                 tcp_srv.iface().host_addr.size()) ^
                 sha256::compute(reinterpret_cast<const uint8_t *>(&port), sizeof(port));
  }

  using domain_udp_srv_sock_t =
      std::conditional_t<is_secure_type(nt),
                         domain_udp_socket_secure_s<udp_sock_secure_type_e::SERVER_UNICAST_SECURE_AES, true>,
                         domain_udp_socket_s<udp_sock_type_e::SERVER_MULTICAST, true>>;

  using domain_udp_cli_sock_t =
      std::conditional_t<is_secure_type(nt),
                         domain_udp_socket_secure_s<udp_sock_secure_type_e::CLIENT_UNICAST_SECURE_AES, true>,
                         domain_udp_socket_s<udp_sock_type_e::CLIENT_MULTICAST, true>>;

  using domain_tcp_srv_sock_t =
      std::conditional_t<is_secure_type(nt),
                         domain_tcp_socket_secure_s<tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, true>,
                         domain_tcp_socket_s<tcp_sock_type_e::SERVER_UNICAST, true>>;

  using domain_tcp_cli_sock_t =
      std::conditional_t<is_secure_type(nt),
                         domain_tcp_socket_secure_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>,
                         domain_tcp_socket_s<tcp_sock_type_e::CLIENT_UNICAST, true>>;

  using ipv4_udp_srv_sock_t = std::conditional_t<
      is_secure_type(nt),
      network_udp_socket_secure_ipv4_s<udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES, true, aes_key_size_bits>,
      network_udp_socket_ipv4_s<udp_sock_type_e::SERVER_MULTICAST, true>>;

  using ipv4_udp_cli_sock_t = std::conditional_t<
      is_secure_type(nt),
      network_udp_socket_secure_ipv4_s<udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES, true, aes_key_size_bits>,
      network_udp_socket_ipv4_s<udp_sock_type_e::CLIENT_MULTICAST, true>>;

  using ipv4_tcp_srv_sock_t =
      std::conditional_t<is_secure_type(nt),
                         network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, true>,
                         network_tcp_socket_ipv4_s<tcp_sock_type_e::SERVER_UNICAST, true>>;

  using ipv4_tcp_cli_sock_t =
      std::conditional_t<is_secure_type(nt),
                         network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>,
                         network_tcp_socket_ipv4_s<tcp_sock_type_e::CLIENT_UNICAST, true>>;

  using ipv6_udp_srv_sock_t = std::conditional_t<
      is_secure_type(nt),
      network_udp_socket_secure_ipv6_s<udp_sock_secure_type_e::SERVER_MULTICAST_SECURE_AES, true, aes_key_size_bits>,
      network_udp_socket_ipv6_s<udp_sock_type_e::SERVER_MULTICAST, true>>;

  using ipv6_udp_cli_sock_t = std::conditional_t<
      is_secure_type(nt),
      network_udp_socket_secure_ipv6_s<udp_sock_secure_type_e::CLIENT_MULTICAST_SECURE_AES, true, aes_key_size_bits>,
      network_udp_socket_ipv6_s<udp_sock_type_e::CLIENT_MULTICAST, true>>;

  using ipv6_tcp_srv_sock_t =
      std::conditional_t<is_secure_type(nt),
                         network_tcp_socket_secure_ipv6_s<tcp_sock_secure_type_e::SERVER_UNICAST_SECURE_TLS, true>,
                         network_tcp_socket_ipv6_s<tcp_sock_type_e::SERVER_UNICAST, true>>;

  using ipv6_tcp_cli_sock_t =
      std::conditional_t<is_secure_type(nt),
                         network_tcp_socket_secure_ipv6_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>,
                         network_tcp_socket_ipv6_s<tcp_sock_type_e::CLIENT_UNICAST, true>>;

  using ipv4_ipv6_udp_srv_sock_t = std::tuple<ipv4_udp_srv_sock_t, ipv6_udp_srv_sock_t>;
  using ipv4_ipv6_udp_cli_sock_t = std::tuple<ipv4_udp_cli_sock_t, ipv6_udp_cli_sock_t>;
  using ipv4_ipv6_tcp_srv_sock_t = std::tuple<ipv4_tcp_srv_sock_t, ipv6_tcp_srv_sock_t>;
  using ipv4_ipv6_tcp_cli_sock_t = std::tuple<ipv4_tcp_cli_sock_t, ipv6_tcp_cli_sock_t>;

  using domain_ipv4_udp_srv_sock_t = std::tuple<domain_udp_srv_sock_t, ipv4_udp_srv_sock_t>;
  using domain_ipv4_udp_cli_sock_t = std::tuple<domain_udp_cli_sock_t, ipv4_udp_cli_sock_t>;
  using domain_ipv4_tcp_srv_sock_t = std::tuple<domain_tcp_srv_sock_t, ipv4_tcp_srv_sock_t>;
  using domain_ipv4_tcp_cli_sock_t = std::tuple<domain_tcp_cli_sock_t, ipv4_tcp_cli_sock_t>;

  using domain_ipv6_udp_srv_sock_t = std::tuple<domain_udp_srv_sock_t, ipv6_udp_srv_sock_t>;
  using domain_ipv6_udp_cli_sock_t = std::tuple<domain_udp_cli_sock_t, ipv6_udp_cli_sock_t>;
  using domain_ipv6_tcp_srv_sock_t = std::tuple<domain_tcp_srv_sock_t, ipv6_tcp_srv_sock_t>;
  using domain_ipv6_tcp_cli_sock_t = std::tuple<domain_tcp_cli_sock_t, ipv6_tcp_cli_sock_t>;

  using domain_ipv4_ipv6_udp_srv_sock_t = std::tuple<domain_udp_srv_sock_t, ipv4_udp_srv_sock_t, ipv6_udp_srv_sock_t>;
  using domain_ipv4_ipv6_udp_cli_sock_t = std::tuple<domain_udp_cli_sock_t, ipv4_udp_cli_sock_t, ipv6_udp_srv_sock_t>;
  using domain_ipv4_ipv6_tcp_srv_sock_t = std::tuple<domain_tcp_srv_sock_t, ipv4_tcp_srv_sock_t, ipv6_udp_srv_sock_t>;
  using domain_ipv4_ipv6_tcp_cli_sock_t = std::tuple<domain_tcp_cli_sock_t, ipv4_tcp_cli_sock_t, ipv6_udp_srv_sock_t>;

  using nm_all_types_t =
      std::tuple<domain_udp_srv_sock_t, domain_udp_cli_sock_t, domain_tcp_srv_sock_t, domain_tcp_cli_sock_t,
                 ipv4_udp_srv_sock_t, ipv4_udp_cli_sock_t, ipv4_tcp_srv_sock_t, ipv4_tcp_cli_sock_t,
                 ipv6_udp_srv_sock_t, ipv6_udp_cli_sock_t, ipv6_tcp_srv_sock_t, ipv6_tcp_cli_sock_t,
                 ipv4_ipv6_udp_srv_sock_t, ipv4_ipv6_udp_cli_sock_t, ipv4_ipv6_tcp_srv_sock_t, ipv4_ipv6_tcp_cli_sock_t,
                 domain_ipv4_udp_srv_sock_t, domain_ipv4_udp_cli_sock_t, domain_ipv4_tcp_srv_sock_t,
                 domain_ipv4_tcp_cli_sock_t, domain_ipv6_udp_srv_sock_t, domain_ipv6_udp_cli_sock_t,
                 domain_ipv6_tcp_srv_sock_t, domain_ipv6_tcp_cli_sock_t, domain_ipv4_ipv6_udp_srv_sock_t,
                 domain_ipv4_ipv6_udp_cli_sock_t, domain_ipv4_ipv6_tcp_srv_sock_t, domain_ipv4_ipv6_tcp_cli_sock_t>;

  static constexpr uint32_t sock_family_types_n = static_cast<uint32_t>(env_networking_type_e::ENV_NETW_TYPE_NUM) / 2u;
  static constexpr uint32_t sock_protocol_types_n = 4u;
  static constexpr uint32_t udp_srv_order_n = 0u;
  using udp_srv_t = std::tuple_element_t<
      ((static_cast<uint32_t>(nt) % sock_family_types_n) * sock_protocol_types_n) + udp_srv_order_n, nm_all_types_t>;

  static constexpr uint32_t udp_cli_order_n = 1u;
  using udp_cli_t = std::tuple_element_t<
      ((static_cast<uint32_t>(nt) % sock_family_types_n) * sock_protocol_types_n) + udp_cli_order_n, nm_all_types_t>;

  static constexpr uint32_t tcp_srv_order_n = 2u;
  using tcp_srv_t = std::tuple_element_t<
      ((static_cast<uint32_t>(nt) % sock_family_types_n) * sock_protocol_types_n) + tcp_srv_order_n, nm_all_types_t>;

  static constexpr uint32_t tcp_cli_order_n = 3u;
  using tcp_cli_t = std::tuple_element_t<
      ((static_cast<uint32_t>(nt) % sock_family_types_n) * sock_protocol_types_n) + tcp_cli_order_n, nm_all_types_t>;

  mutable udp_srv_t udp_srv;
  mutable udp_cli_t udp_cli;
  mutable tcp_srv_t tcp_srv;

  mutable std::vector<std::pair<sha256::sha256_hash_type, uint16_t>> known_envs_;
  mutable std::map<sha256::sha256_hash_type,
                   std::tuple<struct env_cfg_header_s, std::string, std::shared_ptr<tcp_cli_t>>>
      peers_;

  mutable std::recursive_mutex known_envs_lock_;
  mutable std::recursive_mutex peers_lock_;

  sha256::sha256_hash_type host_hash_;
  const struct env_base_s *const p_env_;
  const struct env_base_s *const env_() const { return p_env_; };

  bool out_connection_established_(const sha256::sha256_hash_type &peer_hash) const {
    return peers_.find(peer_hash) != peers_.end();
  }

  bool in_connection_established_(const sha256::sha256_hash_type &hash, uint16_t port) const {
    return tcp_srv.is_peer_connected(hash, port);
  }

  void run_probing_(uint64_t times, uint64_t interval_ms) const {

    /* Get some info from host environment */
    void *payload = nullptr;
    this->env_()->get_lock().lock();
    struct env_cfg_header_s header = this->env_()->info_header();
    const std::string &mcast_addr = this->env_()->info().env_ipv4_multicast_group_addr();
    uint16_t mcast_srv = this->env_()->info().env_ipv4_multicast_port();
    this->env_()->get_lock().unlock();

    for (uint32_t i = 0u; i < times; i++) {
      {
        std::lock_guard<std::recursive_mutex> lock(known_envs_lock_);
        for (const auto &hash_srv_pair : known_envs_) {
          if (!in_connection_established_(hash_srv_pair.first, hash_srv_pair.second)) {

            header.set_env_invite(static_cast<const void *>(hash_srv_pair.first.data()), hash_srv_pair.first.size());
            break;
          }
        }
      }

      size_t payload_size = header.ByteSizeLong();
      payload = std::malloc(payload_size);
      header.SerializeToArray(payload, payload_size);
      udp_cli.send(mcast_addr, mcast_srv, payload, payload_size);
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    std::free(payload);
  }

  int32_t handle_probe_(const std::string &peer_addr, const sha256::sha256_hash_type &peer_hash,
                        const struct env_cfg_header_s &header) const {
    int32_t rc;
    if (peer_addr != tcp_srv.iface().host_addr.data()) {
      int32_t pid = header.env_pid();

      typename decltype(peers_)::iterator it;
      if ((it = peers_.find(peer_hash)) == peers_.end()) {
        std::shared_ptr<network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>> unit(
            std::shared_ptr<network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>>(
                new network_tcp_socket_secure_ipv4_s<tcp_sock_secure_type_e::CLIENT_UNICAST_SECURE_TLS, true>(
                    this->env_()->info().env_network_ifname(), this->env_()->info().env_ca_cert_file(),
                    this->env_()->info().env_ca_priv_key_file(), this->env_()->info().env_cert_info(),
                    this->env_()->info().env_cert_exp_time())));

        unit->on_connect().add(
            "OnConnectHook",
            [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](struct sockaddr_in peer,
                                                                                   auto *unit) -> void {
              uint16_t peer_srv = ::htons(peer.sin_port);
              char peer_addr[INET_ADDRSTRLEN];
              ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
              DEBUG_LOG((boost::format("Client: connected to %1%:%2%\r\n") % peer_addr % peer_srv).str());
            });

        unit->on_disconnect().add(
            "OnDisconnectHook",
            [this, hash = std::remove_reference_t<decltype(peer_hash)>(peer_hash)](struct sockaddr_in peer,
                                                                                   auto *unit) -> void {
              std::lock_guard<std::recursive_mutex> lock(peers_lock_);
              uint16_t peer_srv = ::htons(peer.sin_port);
              char peer_addr[INET_ADDRSTRLEN];
              ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
              DEBUG_LOG((boost::format("Client: disconnected from %1%:%2%\r\n") % peer_addr % peer_srv).str());
              peers_.erase(hash);
            });

        if (!(rc = unit->connect(peer_addr, header.env_ipv4_tcp_port()))) {

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
