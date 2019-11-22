#include "client_port.hpp"
#include "server_port.hpp"
#include "gtest/gtest.h"

struct env {
public:
  void AsyncCSPortConnecting() {
    async_client_port_t<std::string(std::string *, std::string *)> client_port("Client");
    async_server_port_t<std::string(std::string *, std::string *)> server_port("Server");

    async_server_port_t<std::string(std::string *, std::string *)> api_server_port(
        "ServerAPI", {{"First", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; }},
                      {"Second", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; }},
                      {"Third", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; }}});

    uint32_t client_error_id = env_errno_t::ENV_CLEAR;
    uint32_t server_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(client_port
                  .set_error_handler([&client_error_id](const uint64_t &, const uint32_t &error_id,
                                                        const uint32_t &) -> void { client_error_id = error_id; })
                  .qualifiers.at(client_port.get_id()),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(server_port
                  .set_error_handler([&server_error_id](const uint64_t &, const uint32_t &error_id,
                                                        const uint32_t &) -> void { server_error_id = error_id; })
                  .qualifiers.at(server_port.get_id()),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(client_port.connect_port(&server_port).qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);

    std::string str1 = "Hello ";
    std::string str2 = "World!";

    EXPECT_EQ(client_port.replace_port("Server", &api_server_port).qualifiers.at(client_port.get_id()),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(server_port.connect_port(&client_port).qualifiers.at(server_port.get_id()), env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        server_port
            .register_call("Call1", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        api_server_port
            .register_call("Call1", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(api_server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        api_server_port
            .register_call("Call2", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(api_server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        api_server_port
            .register_call("Call3", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(api_server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        api_server_port
            .register_call("Call4", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(api_server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        api_server_port
            .register_call("Call5", [](std::string *str1, std::string *str2) -> std::string { return *str1 + *str2; })
            .qualifiers.at(api_server_port.get_id()),
        env_errno_t::ENV_CLEAR);

    {
      auto call_result = client_port.call("ServerAPI", "Call1", &str1, &str2);

      EXPECT_EQ(call_result.status.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(call_result.status.id, client_port.get_id());

      std::string result = call_result.data.payload;

      EXPECT_EQ(result, "Hello World!");
      result.clear();
    }

    {
      auto call_result = client_port.call("Server", "Call", &str1, &str2);

      EXPECT_EQ(call_result.status.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CALL_MISSING);
      EXPECT_EQ(call_result.status.id, client_port.get_id());

      std::string result = call_result.data.payload;

      EXPECT_EQ(result.empty(), true);
      result.clear();

      auto forward_again_result = client_port.forward();

      EXPECT_EQ(forward_again_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_NO_TRANSACTIONS);
      EXPECT_EQ(forward_again_result.id, client_port.get_id());
    }

    EXPECT_EQ(client_port.disconnect_port("Server").qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
    EXPECT_EQ(server_port.disconnect_port("Client").qualifiers.at(server_port.get_id()), env_errno_t::ENV_PORT_MISSING);
  }

  void AsyncCSPortWriteForwardRead() {}
  void AsyncCSPortSenderQueueOverflow() {}
  void AsyncCSPortReaderWriter() {}
  void AsyncCSPortConnectionExists() {}
  void AsyncCSPortMaximumConnections() {}
  void AsyncCSPortConnectionWithSameNames() {}
  void AsyncCSPortTestWithComplexDataType() {}
  void AsyncCSPortTestConnectWriteForwardReadDisconnectWithSeveralPorts() {}
  void AsyncCSPortTestHooks() {}
};

static env env_;

TEST(AsyncCSPort, PortConnecting) { env_.AsyncCSPortConnecting(); }
TEST(AsyncCSPort, PortWriteForwardRead) { env_.AsyncCSPortWriteForwardRead(); }
TEST(AsyncCSPort, PortSenderQueueOverflow) { env_.AsyncCSPortSenderQueueOverflow(); }
TEST(AsyncCSPort, PortReaderWriter) { env_.AsyncCSPortReaderWriter(); }
TEST(AsyncCSPort, PortConnectionExists) { env_.AsyncCSPortConnectionExists(); }
TEST(AsyncCSPort, PortMaximumConnections) { env_.AsyncCSPortMaximumConnections(); }
TEST(AsyncCSPort, PortConnectionWithSameNames) { env_.AsyncCSPortConnectionWithSameNames(); }
TEST(AsyncCSPort, PortTestWithComplexDataType) { env_.AsyncCSPortTestWithComplexDataType(); }

TEST(AsyncCSPort, PortTestConnectWriteForwardReadDisconnectWithSeveralPorts) {
  env_.AsyncCSPortTestConnectWriteForwardReadDisconnectWithSeveralPorts();
}

TEST(AsyncCSPort, PortTestHooks) { env_.AsyncCSPortTestHooks(); }
