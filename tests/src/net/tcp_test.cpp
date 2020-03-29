#include "gtest/gtest.h"
#include "network_tcp_sock.hpp"

TEST(TCPNetworking, IPv4) {
  // static constexpr uint32_t interval_ms = 100u;
  // static constexpr uint32_t duration_ms = 1000u;
  // static constexpr uint16_t listener_port = 4000u;
  // static constexpr const char *interface_name = "wlp1s0";

  // tcp_socket<AF_INET, tcp_sock_type_e::SERVER_UNICAST> ipv4_server(interface_name);
  // tcp_socket<AF_INET, tcp_sock_type_e::CLIENT_UNICAST> ipv4_client(interface_name);

  // ipv4_server.on_receive().add("OnReceiveHook",
  //                              [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
  //                                 const decltype(ipv4_server) *socket) -> void {
  //                                const char *payloadstr = static_cast<const char *>(data.get());
  //                                char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(from.sin_port);
  //                                ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr, peer_addr,
  //                                           peer_port);
  //                              });

  // ipv4_server.on_connect().add("OnConnectHook",
  //                              [](struct sockaddr_in peer, const decltype(ipv4_server) *socket) -> void {
  //                                char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(peer.sin_port);
  //                                ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Server: connected peer {0}:{1}\r\n", peer_addr, peer_port);
  //                              });

  // ipv4_server.on_disconnect().add("OnDisconnectHook",
  //                                 [](struct sockaddr_in peer, const decltype(ipv4_server) *socket) -> void {
  //                                   char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                   uint16_t peer_port = ::htons(peer.sin_port);
  //                                   ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
  //                                   fmt::print("Server: peer {0}:{1} disconnected\r\n", peer_addr, peer_port);
  //                                 });

  // ipv4_client.on_send().add(
  //     "OnSendHook",
  //     [](struct sockaddr_in to, std::shared_ptr<void> data, size_t size, const decltype(ipv4_client) *socket) -> void {
  //       const char *payloadstr = static_cast<const char *>(data.get());
  //       uint16_t peer_port = ::htons(to.sin_port);
  //       char peer_addr[INET_ADDRSTRLEN] = {0};
  //       ::inet_ntop(AF_INET, &to.sin_addr, peer_addr, sizeof(peer_addr));
  //       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr, peer_port);
  //     });

  // ipv4_client.on_receive().add("OnReceiveHook",
  //                              [](struct sockaddr_in from, std::shared_ptr<void> data, size_t size,
  //                                 const decltype(ipv4_client) *socket) -> void {
  //                                const char *payloadstr = static_cast<const char *>(data.get());
  //                                uint16_t peer_port = ::htons(from.sin_port);
  //                                char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                ::inet_ntop(AF_INET, &from.sin_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr, peer_addr,
  //                                           peer_port);
  //                              });

  // ipv4_client.on_connect().add("OnConnectHook",
  //                              [](struct sockaddr_in peer, const decltype(ipv4_client) *socket) -> void {
  //                                char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(peer.sin_port);
  //                                ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Client: connected to {0}:{1}\r\n", peer_addr, peer_port);
  //                              });

  // ipv4_client.on_disconnect().add("OnDisconnectHook",
  //                                 [](struct sockaddr_in peer, const decltype(ipv4_client) *socket) -> void {
  //                                   char peer_addr[INET_ADDRSTRLEN] = {0};
  //                                   uint16_t peer_port = ::htons(peer.sin_port);
  //                                   ::inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr));
  //                                   fmt::print("Client: disconnected from {0}:{1}\r\n", peer_addr, peer_port);
  //                                 });

  // ipv4_server.setup(listener_port);
  // ipv4_server.start();

  // std::this_thread::sleep_for(std::chrono::microseconds(50u));

  // ipv4_client.connect(ipv4_server.iface_info().host_addr.data(), listener_port);

  // for (uint32_t i = 0; i < 1000u; i++) {
  //   ipv4_client.send("Hello world!", std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv4_client.disconnect();
  // ipv4_server.stop();
}

TEST(TCPNetworking, IPv6) {
  // static constexpr uint32_t interval_ms = 100u;
  // static constexpr uint32_t duration_ms = 1000u;
  // static constexpr uint16_t listener_port = 4000u;
  // static constexpr const char *interface_name = "wlp1s0";

  // tcp_socket<AF_INET6, tcp_sock_type_e::SERVER_UNICAST> ipv4_server(interface_name);
  // tcp_socket<AF_INET6, tcp_sock_type_e::CLIENT_UNICAST> ipv4_client(interface_name);

  // ipv4_server.on_receive().add("OnReceiveHook",
  //                              [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
  //                                 const decltype(ipv4_server) *socket) -> void {
  //                                const char *payloadstr = static_cast<const char *>(data.get());
  //                                char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(from.sin6_port);
  //                                ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Server: received string : {0} from {1}:{2}\r\n", payloadstr, peer_addr,
  //                                           peer_port);
  //                              });

  // ipv4_server.on_connect().add("OnConnectHook",
  //                              [](struct sockaddr_in6 peer, const decltype(ipv4_server) *socket) -> void {
  //                                char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(peer.sin6_port);
  //                                ::inet_ntop(AF_INET6, &peer.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Server: connected peer {0}:{1}\r\n", peer_addr, peer_port);
  //                              });

  // ipv4_server.on_disconnect().add("OnDisconnectHook",
  //                                 [](struct sockaddr_in6 peer, const decltype(ipv4_server) *socket) -> void {
  //                                   char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                   uint16_t peer_port = ::htons(peer.sin6_port);
  //                                   ::inet_ntop(AF_INET6, &peer.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                   fmt::print("Server: peer {0}:{1} disconnected\r\n", peer_addr, peer_port);
  //                                 });

  // ipv4_client.on_send().add(
  //     "OnSendHook",
  //     [](struct sockaddr_in6 to, std::shared_ptr<void> data, size_t size, const decltype(ipv4_client) *socket) -> void {
  //       const char *payloadstr = static_cast<const char *>(data.get());
  //       uint16_t peer_port = ::htons(to.sin6_port);
  //       char peer_addr[INET6_ADDRSTRLEN] = {0};
  //       ::inet_ntop(AF_INET6, &to.sin6_addr, peer_addr, sizeof(peer_addr));
  //       fmt::print("Client: sent string {0} to {1}:{2}\r\n", payloadstr, peer_addr, peer_port);
  //     });

  // ipv4_client.on_receive().add("OnReceiveHook",
  //                              [](struct sockaddr_in6 from, std::shared_ptr<void> data, size_t size,
  //                                 const decltype(ipv4_client) *socket) -> void {
  //                                const char *payloadstr = static_cast<const char *>(data.get());
  //                                uint16_t peer_port = ::htons(from.sin6_port);
  //                                char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                ::inet_ntop(AF_INET6, &from.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Client: received string {0} from {1}:{2}\r\n", payloadstr, peer_addr,
  //                                           peer_port);
  //                              });

  // ipv4_client.on_connect().add("OnConnectHook",
  //                              [](struct sockaddr_in6 peer, const decltype(ipv4_client) *socket) -> void {
  //                                char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                uint16_t peer_port = ::htons(peer.sin6_port);
  //                                ::inet_ntop(AF_INET6, &peer.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                fmt::print("Client: connected to {0}:{1}\r\n", peer_addr, peer_port);
  //                              });

  // ipv4_client.on_disconnect().add("OnDisconnectHook",
  //                                 [](struct sockaddr_in6 peer, const decltype(ipv4_client) *socket) -> void {
  //                                   char peer_addr[INET6_ADDRSTRLEN] = {0};
  //                                   uint16_t peer_port = ::htons(peer.sin6_port);
  //                                   ::inet_ntop(AF_INET6, &peer.sin6_addr, peer_addr, sizeof(peer_addr));
  //                                   fmt::print("Client: disconnected from {0}:{1}\r\n", peer_addr, peer_port);
  //                                 });

  // ipv4_server.setup(listener_port);
  // ipv4_server.start();

  // std::this_thread::sleep_for(std::chrono::microseconds(50u));

  // ipv4_client.connect(ipv4_server.iface_info().host_addr.data(), listener_port);

  // for (uint32_t i = 0; i < 25u; i++) {
  //   ipv4_client.send("Hello world!", std::strlen("Hello world!"));
  //   std::this_thread::sleep_for(std::chrono::microseconds(25u));
  // }

  // ipv4_client.disconnect();
  // ipv4_server.stop();
}
