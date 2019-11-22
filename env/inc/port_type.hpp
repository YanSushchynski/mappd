#ifndef PORT_TYPE_HPP
#define PORT_TYPE_HPP

#include <cstdint>
#include <type_traits>

struct uint32 {
  uint32_t i;
  uint32() : i(0u) {}
  operator uint32_t() { return i; }
};

struct port_types_t : uint32 {
  struct receiver_port_types_t {
    enum : uint32_t { PORT_TYPE_RECEIVER = 0u, PORT_TYPE_CONST_RECEIVER };
  };

  struct sender_port_types_t {
    enum : uint32_t {
      PORT_TYPE_SENDER_SYNC = 2u,
      PORT_TYPE_SENDER_ASYNC,
      PORT_TYPE_CONST_SENDER_SYNC,
      PORT_TYPE_CONST_SENDER_ASYNC
    };
  };

  struct client_port_types_t {
    enum : uint32_t {
      PORT_TYPE_CLIENT_SYNC = 6u,
      PORT_TYPE_CLIENT_ASYNC,
      PORT_TYPE_CONST_CLIENT_SYNC,
      PORT_TYPE_CONST_CLIENT_ASYNC
    };
  };

  struct server_port_types_t {
    enum : uint32_t {
      PORT_TYPE_SERVER_SYNC = 10u,
      PORT_TYPE_SERVER_ASYNC,
      PORT_TYPE_CONST_SERVER_SYNC,
      PORT_TYPE_CONST_SERVER_ASYNC
    };
  };
};

template <typename EntityType, uint32_t...> struct port_t {};

template <typename...> struct base_port_t {};
template <typename...> struct base_port_const_t {};

template <uint32_t, typename...> struct base_sender_port_t {};
template <typename...> struct base_receiver_port_t {};

template <uint32_t, typename...> struct base_sender_port_const_t {};
template <typename...> struct base_receiver_port_const_t {};

template <uint32_t, typename...> struct base_server_port_t {};
template <uint32_t, typename...> struct base_client_port_t {};

template <uint32_t, typename...> struct const_base_server_port_t {};
template <uint32_t, typename...> struct const_base_client_port_t {};

#endif /* PORT_TYPE_HPP */
