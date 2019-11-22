#include "gtest/gtest.h"
#include "udp_sock.hpp"

// static constexpr uint32_t interval_ms = 100u;
// static constexpr uint32_t duration_ms = 1000u;
// static constexpr uint16_t listener_port = 9999u;

// static constexpr const char *interface_name = "wlp1s0";
// static constexpr const char *ipv4_multicast_addr = "224.0.0.1";
// static constexpr const char *ipv6_multicast_addr = "ff02:1::1";

// udp_socket<AF_INET, udp_sock_t::CLIENT_UNICAST> ipv4_unicast_client(interface_name);
// udp_socket<AF_INET, udp_sock_t::CLIENT_BROADCAST> ipv4_broadcast_client(interface_name);
// udp_socket<AF_INET, udp_sock_t::CLIENT_MULTICAST> ipv4_multicast_client(interface_name, ipv4_multicast_addr,
// listener_port);

// udp_socket<AF_INET, udp_sock_t::SERVER_UNICAST> ipv4_unicast_server(interface_name);
// udp_socket<AF_INET, udp_sock_t::SERVER_BROADCAST> ipv4_broadcast_server(interface_name);
// udp_socket<AF_INET, udp_sock_t::SERVER_MULTICAST> ipv4_multicast_server(interface_name);

// udp_socket<AF_INET6, udp_sock_t::CLIENT_UNICAST> ipv6_unicast_client(interface_name);
// udp_socket<AF_INET6, udp_sock_t::CLIENT_MULTICAST> ipv6_multicast_client(interface_name, ipv6_multicast_addr,
// listener_port);

// udp_socket<AF_INET6, udp_sock_t::SERVER_UNICAST> ipv6_unicast_server(interface_name);
// udp_socket<AF_INET6, udp_sock_t::SERVER_MULTICAST> ipv6_multicast_server(interface_name);

// auto ipv4_unicast_set_hooks = []() -> void {
//   ipv4_unicast_server.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv4_unicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin_port);
//                                          char peer_addr[INET_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv4_unicast_client.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv4_unicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin_port);
//                                       char peer_addr[INET_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });

//   ipv4_unicast_client.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv4_unicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin_port);
//                                          char peer_addr[INET_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv4_unicast_server.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv4_unicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin_port);
//                                       char peer_addr[INET_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });
// };

// auto ipv4_broadcast_set_hooks = []() -> void {
//   ipv4_broadcast_server.on_receive().add("OnReceiveHook",
//                                          [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                             const decltype(ipv4_broadcast_server)::base_t *socket) -> void {
//                                            const char *payloadstr = static_cast<const char *>(data.get());
//                                            uint16_t peer_port = ::htons(from.sin_port);
//                                            char peer_addr[INET_ADDRSTRLEN] = {0};
//                                            ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                            fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr,
//                                                       peer_addr, peer_port);
//                                          });

//   ipv4_broadcast_client.on_send().add("OnSendHook",
//                                       [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                          const decltype(ipv4_broadcast_client)::base_t *socket) -> void {
//                                         const char *payloadstr = static_cast<const char *>(data.get());
//                                         uint16_t peer_port = ::htons(to.sin_port);
//                                         char peer_addr[INET_ADDRSTRLEN] = {0};
//                                         ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                         fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                    peer_port);
//                                       });

//   ipv4_broadcast_client.on_receive().add("OnReceiveHook",
//                                          [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                             const decltype(ipv4_broadcast_server)::base_t *socket) -> void {
//                                            const char *payloadstr = static_cast<const char *>(data.get());
//                                            uint16_t peer_port = ::htons(from.sin_port);
//                                            char peer_addr[INET_ADDRSTRLEN] = {0};
//                                            ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                            fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr,
//                                                       peer_addr, peer_port);
//                                          });

//   ipv4_broadcast_server.on_send().add("OnSendHook",
//                                       [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                          const decltype(ipv4_broadcast_client)::base_t *socket) -> void {
//                                         const char *payloadstr = static_cast<const char *>(data.get());
//                                         uint16_t peer_port = ::htons(to.sin_port);
//                                         char peer_addr[INET_ADDRSTRLEN] = {0};
//                                         ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                         fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                    peer_port);
//                                       });
// };

// auto ipv4_multicast_set_hooks = []() -> void {
//   ipv4_multicast_server.on_receive().add("OnReceiveHook",
//                                          [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                             const decltype(ipv4_multicast_server)::base_t *socket) -> void {
//                                            const char *payloadstr = static_cast<const char *>(data.get());
//                                            uint16_t peer_port = ::htons(from.sin_port);
//                                            char peer_addr[INET_ADDRSTRLEN] = {0};
//                                            ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                            fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr,
//                                                       peer_addr, peer_port);
//                                          });

//   ipv4_multicast_client.on_send().add("OnSendHook",
//                                       [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                          const decltype(ipv4_multicast_client)::base_t *socket) -> void {
//                                         const char *payloadstr = static_cast<const char *>(data.get());
//                                         uint16_t peer_port = ::htons(to.sin_port);
//                                         char peer_addr[INET_ADDRSTRLEN] = {0};
//                                         ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                         fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                    peer_port);
//                                       });

//   ipv4_multicast_client.on_receive().add("OnReceiveHook",
//                                          [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
//                                             const decltype(ipv4_multicast_server)::base_t *socket) -> void {
//                                            const char *payloadstr = static_cast<const char *>(data.get());
//                                            uint16_t peer_port = ::htons(from.sin_port);
//                                            char peer_addr[INET_ADDRSTRLEN] = {0};
//                                            ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
//                                            fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr,
//                                                       peer_addr, peer_port);
//                                          });

//   ipv4_multicast_server.on_send().add("OnSendHook",
//                                       [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size,
//                                          const decltype(ipv4_multicast_client)::base_t *socket) -> void {
//                                         const char *payloadstr = static_cast<const char *>(data.get());
//                                         uint16_t peer_port = ::htons(to.sin_port);
//                                         char peer_addr[INET_ADDRSTRLEN] = {0};
//                                         ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
//                                         fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                    peer_port);
//                                       });
// };

// auto ipv6_unicast_set_hooks = []() -> void {
//   ipv6_unicast_server.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv6_unicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin6_port);
//                                          char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv6_unicast_client.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in6 to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv6_unicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin6_port);
//                                       char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET6, &to.sin6_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });

//   ipv6_unicast_client.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv6_unicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin6_port);
//                                          char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv6_unicast_server.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in6 to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv6_unicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin6_port);
//                                       char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET6, &to.sin6_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });
// };

// auto ipv6_multicast_set_hooks = []() -> void {
//   ipv6_multicast_server.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv6_multicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin6_port);
//                                          char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv6_multicast_client.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in6 to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv6_multicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin6_port);
//                                       char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET6, &to.sin6_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });

//   ipv6_multicast_client.on_receive().add("OnReceiveHook",
//                                        [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
//                                           const decltype(ipv6_multicast_server)::base_t *socket) -> void {
//                                          const char *payloadstr = static_cast<const char *>(data.get());
//                                          uint16_t peer_port = ::htons(from.sin6_port);
//                                          char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                          ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
//                                          fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr,
//                                                     peer_addr, peer_port);
//                                        });

//   ipv6_multicast_server.on_send().add("OnSendHook",
//                                     [](struct sockaddr_in6 to, std::shared_ptr<void> data, size_t size,
//                                        const decltype(ipv6_multicast_client)::base_t *socket) -> void {
//                                       const char *payloadstr = static_cast<const char *>(data.get());
//                                       uint16_t peer_port = ::htons(to.sin6_port);
//                                       char peer_addr[INET6_ADDRSTRLEN] = {0};
//                                       ::inet_ntop(AF_INET6, &to.sin6_addr, peer_addr, sizeof(peer_addr));
//                                       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr,
//                                                  peer_port);
//                                     });
// };

TEST(UDPNetworking, IPv4_Unicast) {
  // ipv4_unicast_set_hooks();
  // ipv4_unicast_server.setup(listener_port);
  // ipv4_unicast_server.start();

  // for (uint32_t i = 0u; i < 25u; i++) {
  //   ipv4_unicast_client.send(ipv4_unicast_server.iface_info().host_addr.data(), listener_port, "Hello world!",
  //                            std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv4_unicast_server.stop();
}

TEST(UDPNetworking, IPv4_Broadcast) {
  // ipv4_broadcast_set_hooks();
  // ipv4_broadcast_server.setup(listener_port);
  // ipv4_broadcast_server.start();

  // for (uint32_t i = 0u; i < 25u; i++) {
  //   ipv4_broadcast_client.send(listener_port, "Hello world!", std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv4_broadcast_server.stop();
}

TEST(UDPNetworking, IPv4_Multicast) {
  // ipv4_multicast_set_hooks();
  // ipv4_multicast_server.setup(ipv4_multicast_addr, listener_port);
  // ipv4_multicast_server.start();

  // for (uint32_t i = 0u; i < 25u; i++) {
  //   ipv4_multicast_client.send(ipv4_multicast_addr, listener_port, "Hello world!", std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv4_multicast_server.stop();
}

TEST(UDPNetworking, IPv6_Unicast) {
  // ipv6_unicast_set_hooks();
  // ipv6_unicast_server.setup(listener_port);
  // ipv6_unicast_server.start();

  // for (uint32_t i = 0u; i < 25u; i++) {
  //   ipv6_unicast_client.send(ipv6_unicast_server.iface_info().host_addr.data(), listener_port, "Hello world!",
  //                            std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv6_unicast_server.stop();
}

TEST(UDPNetworking, IPv6_Multicast) {
  // ipv6_multicast_set_hooks();
  // ipv6_multicast_server.setup(ipv6_multicast_addr, listener_port);
  // ipv6_multicast_server.start();

  // for (uint32_t i = 0u; i < 25u; i++) {
  //   ipv6_multicast_client.send(ipv6_multicast_addr, listener_port, "Hello world!", std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv6_multicast_server.stop();
}
