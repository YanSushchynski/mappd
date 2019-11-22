#ifndef ENV_BASE_HPP
#define ENV_BASE_HPP

#include "env_utils.hpp"
#include "fmt/format.h"

#include <libconfig.h++>
#include <mutex>

struct env_base_t {
  explicit env_base_t(const std::string &name) : name_(name) {
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

    info_.set_env_max_buffsize(1024);
    info_.set_env_broadcast_interval_ms(100u);
    info_.set_env_pid(getpid());

	const char *ca_files_path = std::getenv("CA_FILES");
	if(!ca_files_path) throw std::runtime_error(fmt::format("Env initialization error, set CA_FILES env var first"));
    std::string ca_files_path_str = ca_files_path;
	
    if (!ca_files_path_str.empty() && ca_files_path[ca_files_path_str.length() - 1u] == '/')
      ca_files_path_str += "/";

    !ca_files_path_str.empty()
        ? info_.set_env_ca_cert_file(ca_files_path_str + "ca_cert.crt")
        : throw std::runtime_error(fmt::format("Env initialization error, set CA_FILE env var first"));

    !ca_files_path_str.empty()
        ? info_.set_env_ca_priv_key_file(ca_files_path_str + "ca_key.key")
        : throw std::runtime_error(fmt::format("Env initialization error, set CA_KEY env var first"));

    info_.set_env_cert_exp_time(60u);
    info_.set_env_cert_info("C=XX; ST=XXX; L=XXX; O=Mappd_Testing; OU=Developers; CN=mappd.localhost");

    char hostname[32u];
    (void)gethostname(hostname, sizeof(hostname));
    info_.set_env_host_name(hostname);
  };

  virtual ~env_base_t() = default;

  virtual void configure(const libconfig::Setting &env_config) const {
    throw std::runtime_error("Runtime configuration allowed only in dynamic evironments");
  };

  virtual int run(int argc, char *argv[]) const = 0;
  const env_config_t &info() const { return info_; }
  const env_config_header_t info_header() const { return get_info_header_(info_); }
  const std::string &name() const { return name_; };
  std::mutex &get_lock() const { return access_mtx_; };

private:
  mutable std::mutex access_mtx_;
  env_config_t info_;
  const std::string name_;
  
  const env_config_header_t get_info_header_(const env_config_t &info) const {
    env_config_header_t header;
    header.set_env_name(info.env_name());
    header.set_env_ipv4_stream_port(info.env_ipv4_stream_port());
    header.set_env_ipv6_stream_port(info.env_ipv6_stream_port());
    header.set_env_pid(info.env_pid());
    header.set_env_host_name(info.env_host_name());
    header.set_env_invite(
        reinterpret_cast<const char *>(const_cast<const uint8_t *>(sha256::sha256_hash_type{0u}.data())));
    return std::move(header);
  }
};

#endif /* ENV_BASE_HPP */
