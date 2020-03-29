#ifndef ENV_BASE_HPP
#define ENV_BASE_HPP

#include "env_utils.hpp"
#include <boost/format.hpp>

#include <libconfig.h++>
#include <mutex>

enum struct env_networking_type_e : uint32_t {
  IPV4,
  IPV6,
  IPV4_IPV6,
  DOMAIN,
  DOMAIN_IPV4,
  DOMAIN_IPV6,
  DOMAIN_IPV4_IPV6,
  IPV4_SC,
  IPV6_SC,
  IPV4_IPV6_SC,
  DOMAIN_SC,
  DOMAIN_IPV4_SC,
  DOMAIN_IPV6_SC,
  DOMAIN_IPV4_IPV6_SC
};

static constexpr bool is_secure_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::IPV4_SC || nt == env_networking_type_e::IPV6_SC ||
         nt == env_networking_type_e::IPV4_IPV6_SC || nt == env_networking_type_e::DOMAIN_SC ||
         nt == env_networking_type_e::DOMAIN_IPV4_SC || nt == env_networking_type_e::DOMAIN_IPV6_SC ||
         nt == env_networking_type_e::DOMAIN_IPV4_IPV6_SC;
}

static constexpr bool is_ipv4_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::IPV4 || nt == env_networking_type_e::IPV4_SC;
}

static constexpr bool is_ipv6_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::IPV6 || nt == env_networking_type_e::IPV6_SC;
}

static constexpr bool is_ipv4_ipv6_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::IPV4_IPV6 || nt == env_networking_type_e::IPV4_IPV6_SC;
}

static constexpr bool is_domain_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::DOMAIN || nt == env_networking_type_e::DOMAIN_SC;
}

static constexpr bool is_domain_ipv4_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::DOMAIN_IPV4 || nt == env_networking_type_e::DOMAIN_IPV4_SC;
}

static constexpr bool is_domain_ipv6_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::DOMAIN_IPV6 || nt == env_networking_type_e::DOMAIN_IPV6_SC;
}

static constexpr bool is_domain_ipv4_ipv6_type(env_networking_type_e nt) {
  return nt == env_networking_type_e::DOMAIN_IPV4_IPV6 || nt == env_networking_type_e::DOMAIN_IPV4_IPV6_SC;
}

struct env_base_s {
  explicit env_base_s(const std::string &name) : name_(name) {
    info_.set_env_name(name_);

    info_.set_env_headers_path("./headers");
    info_.set_env_mountpoint("./mountpoint");
    info_.set_env_components_path("./components");

    info_.set_env_network_ifname("enp1s0");

    info_.set_env_ipv4_multicast_group_addr("224.0.0.1");
    info_.set_env_ipv4_broadcast_port(4000);
    info_.set_env_ipv4_stream_port(4001);
    info_.set_env_ipv4_multicast_port(4003);

    info_.set_env_ipv6_multicast_group_addr("ff02:1::1");
    info_.set_env_ipv6_stream_port(4003);
    info_.set_env_ipv6_multicast_port(4005);

    info_.set_env_ipv4_enabled(true);
    info_.set_env_ipv6_enabled(true);

    info_.set_domain_udp_socket_path("./udp_socket.socket");
    info_.set_domain_stream_socket_path("./stream_socket.socket");

    info_.set_env_max_buffsize(1024);
    info_.set_env_broadcast_interval_ms(100u);
    info_.set_env_pid(getpid());

    const char *ca_files_path = std::getenv("CA_FILES");
    if (!ca_files_path)
      throw std::runtime_error(boost::format("Env initialization error, set CA_FILES env var first").str());
    std::string ca_files_path_str = ca_files_path;

    if (!ca_files_path_str.empty() && ca_files_path[ca_files_path_str.length() - 1u] == '/')
      ca_files_path_str += "/";

    !ca_files_path_str.empty()
        ? info_.set_env_ca_cert_file(ca_files_path_str + "ca_cert.crt")
        : throw std::runtime_error(boost::format("Env initialization error, set CA_FILE env var first").str());

    !ca_files_path_str.empty()
        ? info_.set_env_ca_priv_key_file(ca_files_path_str + "ca_key.key")
        : throw std::runtime_error(boost::format("Env initialization error, set CA_KEY env var first").str());

    info_.set_env_cert_exp_time(60u);
    info_.set_env_cert_info("C=XX; ST=XXX; L=XXX; O=Mappd_Testing; OU=Developers; CN=mappd.localhost");

    char hostname[32u];
    (void)gethostname(hostname, sizeof(hostname));
    info_.set_env_host_name(hostname);
  };

  virtual ~env_base_s() = default;

  virtual void configure(const libconfig::Setting &env_config) {
    throw std::runtime_error("Runtime configuration allowed only in dynamic evironments");
  };

  virtual int run(int argc, char *argv[]) const = 0;
  const struct env_cfg_s &info() const { return info_; }
  const struct env_cfg_header_s info_header() const { return get_info_header_(info_); }
  const std::string &name() const { return name_; };
  std::mutex &get_lock() const { return access_mtx_; };

private:
  mutable std::mutex access_mtx_;
  struct env_cfg_s info_;
  const std::string name_;

  const struct env_cfg_header_s get_info_header_(const struct env_cfg_s &info) const {
    struct env_cfg_header_s header;
    header.set_env_name(info.env_name());
    header.set_env_ipv4_stream_port(info.env_ipv4_stream_port());
    header.set_env_ipv6_stream_port(info.env_ipv6_stream_port());
    header.set_env_pid(info.env_pid());
    header.set_env_host_name(info.env_host_name());
    header.set_env_invite(
        reinterpret_cast<const char *>(const_cast<const uint8_t *>(sha256::sha256_hash_type{0u}.data())));

    header.set_domain_stream_socket_path(info.domain_stream_socket_path());
    return std::move(header);
  }
};

#endif /* ENV_BASE_HPP */
