#include "client_port.hpp"
#include "server_port.hpp"
#include "gtest/gtest.h"

class env {
public:
  void SyncCSPortConnecting() {
    sync_client_port_t<std::string(std::string *, std::string *)> client_port("Client");
    sync_server_port_t<std::string(std::string *, std::string *)> server_port("Server");

    sync_server_port_t<std::string(std::string *, std::string *)> api_server_port(
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
      auto request_result = client_port.request("ServerAPI", "Call1", &str1, &str2);

      EXPECT_EQ(request_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(request_result.id, client_port.get_id());

      auto forward_result = client_port.forward();

      EXPECT_EQ(forward_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(forward_result.id, client_port.get_id());

      auto response_result = client_port.response();

      EXPECT_EQ(response_result.status.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(response_result.status.id, client_port.get_id());

      std::string result = response_result.data.payload;

      EXPECT_EQ(result, "Hello World!");
      result.clear();
    }

    {
      auto request_result = client_port.request("Server", "Call", &str1, &str2);

      EXPECT_EQ(request_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(request_result.id, client_port.get_id());

      auto forward_result = client_port.forward();

      EXPECT_EQ(forward_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_CALL_MISSING);
      EXPECT_EQ(forward_result.id, client_port.get_id());
      EXPECT_EQ(server_error_id, env_errno_t::ENV_CALL_MISSING);

      auto response_result = client_port.response();

      EXPECT_EQ(response_result.status.qualifiers.at(client_port.get_id()), env_errno_t::ENV_BUFFER_EMPTY);
      EXPECT_EQ(response_result.status.id, client_port.get_id());

      std::string result = response_result.data.payload;

      EXPECT_EQ(result.empty(), true);
      result.clear();

      auto forward_again_result = client_port.forward();

      EXPECT_EQ(forward_again_result.qualifiers.at(client_port.get_id()), env_errno_t::ENV_NO_TRANSACTIONS);
      EXPECT_EQ(forward_again_result.id, client_port.get_id());
    }

    EXPECT_EQ(client_port.disconnect_port("Server").qualifiers.at(client_port.get_id()), env_errno_t::ENV_CLEAR);
    EXPECT_EQ(server_port.disconnect_port("Client").qualifiers.at(server_port.get_id()), env_errno_t::ENV_PORT_MISSING);
  }

  void SyncCSPortWriteForwardRead() {
    sync_client_port_t<unified_t(unified_t, unified_t)> client_port("Client");
    sync_server_port_t<unified_t(unified_t, unified_t)> server_port("Server");

    client_port.connect_port(&server_port);
    server_port.register_call("Call", [](int a, int b) -> int { return a + b; });

    {
      client_port.request("Server", "Call", 10, 10);
      client_port.forward();
      int c = client_port.response().data.payload;
      EXPECT_EQ(c, 10 + 10);
    }

    {
      int a = 15, b = 15;
      client_port.request("Server", "Call", a, b);
      client_port.forward();
      int c = client_port.response().data.payload;
      c = client_port.response().data.payload;
      EXPECT_EQ(c, 0);
    }
  }

  void SyncCSPortSenderQueueOverflow() {}

  void SyncCSPortReaderWriter() {}

  void SyncCSPortConnectionExists() {}

  void SyncCSPortMaximumConnections() {}

  void SyncCSPortConnectionWithSameNames() {}

  void SyncCSPortTestWithComplexDataType() {}

  void SyncCSPortTestConnectWriteForwardReadDisconnectWithSeveralPorts() {}

  void SyncCSPortTestHooks() {}
};

static env env_;

TEST(SyncCSPort, PortConnecting) { env_.SyncCSPortConnecting(); }
TEST(SyncCSPort, PortWriteForwardRead) { env_.SyncCSPortWriteForwardRead(); }
TEST(SyncCSPort, PortSenderQueueOverflow) { env_.SyncCSPortSenderQueueOverflow(); }
TEST(SyncCSPort, PortReaderWriter) { env_.SyncCSPortReaderWriter(); }
TEST(SyncCSPort, PortConnectionExists) { env_.SyncCSPortConnectionExists(); }
TEST(SyncCSPort, PortMaximumConnections) { env_.SyncCSPortMaximumConnections(); }
TEST(SyncCSPort, PortConnectionWithSameNames) { env_.SyncCSPortConnectionWithSameNames(); }
TEST(SyncCSPort, PortTestWithComplexDataType) { env_.SyncCSPortTestWithComplexDataType(); }
TEST(SyncCSPort, PortTestConnectWriteForwardReadDisconnectWithSeveralPorts) {
  env_.SyncCSPortTestConnectWriteForwardReadDisconnectWithSeveralPorts();
}

TEST(SyncCSPort, PortTestHooks) { env_.SyncCSPortTestHooks(); }
