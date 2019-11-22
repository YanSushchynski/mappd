#include "receiver_port.hpp"
#include "sender_port.hpp"
#include "gtest/gtest.h"

struct env {
  void SyncSRPortConnecting() {
    std::string a;
    sync_receiver_port_t<std::string> receiver_port("Receiver");
    sync_sender_port_t<std::string> sender_port("Sender");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender_port
                  .set_error_handler([&sender_error_id](const uint64_t &, const uint32_t &id,
                                                        const uint32_t &) -> void { sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Sender")),
              env_errno_t::ENV_CLEAR);

    {
      EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_port.read(a).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_BUFFER_EMPTY);
      EXPECT_EQ(a.empty(), true);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_BUFFER_EMPTY);
      EXPECT_EQ(sender_error_id, env_errno_t::ENV_CLEAR);
    }
  }

  void SyncSRPortWriteForwardRead() {
    std::string a;
    sync_receiver_port_t<std::string> receiver_port("Receiver");
    sync_sender_port_t<std::string> sender_port("Sender");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender_port
                  .set_error_handler([&sender_error_id](const uint64_t &, const uint32_t &id,
                                                        const uint32_t &) -> void { sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Sender")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    {
      EXPECT_EQ(sender_port.write("Tested!").qualifiers.at(std::hash<std::string>()("Sender")), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(sender_port.forward().qualifiers.at(std::hash<std::string>()("Sender")), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_port.read(a).qualifiers.at(std::hash<std::string>()("Receiver")), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(a, "Tested!");
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);
      EXPECT_EQ(sender_error_id, env_errno_t::ENV_CLEAR);
    }
  }

  void SyncSRPortSenderQueueOverflow() {
    std::string a;
    sync_receiver_port_t<std::string> receiver_port("Receiver");
    sync_sender_port_t<std::string> sender_port("Sender");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender_port
                  .set_error_handler([&sender_error_id](const uint64_t &, const uint32_t &id,
                                                        const uint32_t &) -> void { sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Sender")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    {
      for (uint64_t i = 0; i < sender_port.max_buffer_size(); i++) {

        EXPECT_EQ(sender_port.write("Tested!").qualifiers.at(std::hash<std::string>()("Sender")),
                  env_errno_t::ENV_CLEAR);
        EXPECT_EQ(sender_error_id, env_errno_t::ENV_CLEAR);
      }

      EXPECT_EQ(sender_port.write("Tested!").qualifiers.at(std::hash<std::string>()("Sender")),
                env_errno_t::ENV_BUFFER_OVERFLOW);
      EXPECT_EQ(sender_error_id, env_errno_t::ENV_BUFFER_OVERFLOW);
      EXPECT_EQ(sender_port.forward().qualifiers.at(std::hash<std::string>()("Sender")), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);

      for (uint64_t i = 0; i < receiver_port.max_buffer_size(); i++) {

        EXPECT_EQ(receiver_port.read(a).qualifiers.at(std::hash<std::string>()("Receiver")), env_errno_t::ENV_CLEAR);
        EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);
        EXPECT_EQ(a, "Tested!");
        a.clear();
      }

      EXPECT_EQ(receiver_port.read(a).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_BUFFER_EMPTY);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_BUFFER_EMPTY);
      EXPECT_EQ(a.empty(), true);
    }
  }

  void SyncSRPortReaderWriter() {
    std::string a;
    std::string testing_string;
    sync_receiver_port_t<std::string> receiver_port("Receiver");
    sync_sender_port_t<std::string> sender_port("Sender");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender_port
                  .set_error_handler([&sender_error_id](const uint64_t &, const uint32_t &id,
                                                        const uint32_t &) -> void { sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Sender")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    {
      EXPECT_EQ(receiver_port
                    .set_reader([&testing_string](std::string &&str) -> std::string {
                      testing_string = str;
                      return str;
                    })
                    .qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_CLEAR);

      EXPECT_EQ(sender_port
                    .set_writer([&testing_string](std::string &&str) -> std::string {
                      testing_string = str;
                      return str;
                    })
                    .qualifiers.at(std::hash<std::string>()("Sender")),
                env_errno_t::ENV_CLEAR);

      {
        EXPECT_EQ(sender_port.write("Tested!").qualifiers.at(std::hash<std::string>()("Sender")),
                  env_errno_t::ENV_CLEAR);
        EXPECT_EQ(testing_string, "Tested!");
        testing_string.clear();

        EXPECT_EQ(sender_port.forward().qualifiers.at(std::hash<std::string>()("Sender")), env_errno_t::ENV_CLEAR);

        EXPECT_EQ(receiver_port.read(a).qualifiers.at(std::hash<std::string>()("Receiver")), env_errno_t::ENV_CLEAR);
        EXPECT_EQ(a, "Tested!");
        EXPECT_EQ(testing_string, "Tested!");
        testing_string.clear();
      }
    }
  }

  void SyncSRPortConnectionExists() {
    sync_receiver_port_t<std::string> receiver_port("Receiver");
    sync_sender_port_t<std::string> sender_port("Sender");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender_port
                  .set_error_handler([&sender_error_id](const uint64_t &, const uint32_t &id,
                                                        const uint32_t &) -> void { sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Sender")),
              env_errno_t::ENV_CLEAR);

    {
      EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_PORT_EXISTS);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_PORT_EXISTS);
    }
  }

  void SyncSRPortMaximumConnections() {
    sync_receiver_port_t<std::string> receiver_port("Receiver");

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("Receiver")),
              env_errno_t::ENV_CLEAR);

    {
      std::vector<sync_sender_port_t<std::string> *> senders;

      for (unsigned int i = 0; i < receiver_port.get_max_connections_(); i++) {

        sync_sender_port_t<std::string> *p_sender_port =
            new sync_sender_port_t<std::string>("Sender" + std::to_string(i));
        senders.push_back(p_sender_port);
        EXPECT_EQ(receiver_port.connect_port(p_sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
                  env_errno_t::ENV_CLEAR);
        EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);
      }

      sync_sender_port_t<std::string> *p_sender_port =
          new sync_sender_port_t<std::string>("Sender" + std::to_string(receiver_port.get_max_connections_()));
      senders.push_back(p_sender_port);
      EXPECT_EQ(receiver_port.connect_port(p_sender_port).qualifiers.at(std::hash<std::string>()("Receiver")),
                env_errno_t::ENV_MAX_PORTS_CONNECTED);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_MAX_PORTS_CONNECTED);

      for (auto p_sender : senders)
        delete p_sender;

      senders.clear();
    }
  }

  void SyncSRPortConnectionWithSameNames() {
    std::string receiver_port_name = "ReceiverPort";
    std::string sender_port_name = "SenderPort";

    sync_receiver_port_t<std::string> receiver_port(receiver_port_name);
    sync_sender_port_t<std::string> first_sender_port(sender_port_name);
    sync_sender_port_t<std::string> second_sender_port(sender_port_name);

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;
    uint32_t first_sender_error_id = env_errno_t::ENV_CLEAR;
    uint32_t second_sender_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(first_sender_port
                  .set_error_handler([&first_sender_error_id](const uint64_t &, const uint32_t &id,
                                                              const uint32_t &) -> void { first_sender_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("SenderPort")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(
        second_sender_port
            .set_error_handler([&second_sender_error_id](const uint64_t &, const uint32_t &id,
                                                         const uint32_t &) -> void { second_sender_error_id = id; })
            .qualifiers.at(std::hash<std::string>()("SenderPort")),
        env_errno_t::ENV_CLEAR);

    {
      EXPECT_EQ(receiver_port.connect_port(&first_sender_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
                env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);

      EXPECT_EQ(receiver_port.connect_port(&second_sender_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
                env_errno_t::ENV_PORT_EXISTS);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_PORT_EXISTS);
    }
  }

  void SyncSRPortTestWithComplexDataType() {
    struct Test {
      int property = 10;
    };

    std::string receiver_port_name = "ReceiverPort";
    std::string sender_port_name = "SenderPort";

    sync_receiver_port_t<Test> receiver_port(receiver_port_name);
    sync_sender_port_t<Test> sender_port(sender_port_name);

    uint32_t receiver_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(receiver_port
                  .set_error_handler([&receiver_error_id](const uint64_t &, const uint32_t &id,
                                                          const uint32_t &) -> void { receiver_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);

    {
      Test test_obj;
      Test test_obj2;

      test_obj.property = 100;
      EXPECT_EQ(sender_port.write(test_obj).qualifiers.at(std::hash<std::string>()("SenderPort")),
                env_errno_t::ENV_CLEAR);
      EXPECT_EQ(sender_port.forward().qualifiers.at(std::hash<std::string>()("SenderPort")), env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_port.read(test_obj2).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
                env_errno_t::ENV_CLEAR);
      EXPECT_EQ(receiver_error_id, env_errno_t::ENV_CLEAR);
      EXPECT_EQ(test_obj2.property, 100);
    }
  }

  void SyncSRPortTestConnectWriteForwardReadDisconnectWithSeveralPorts() {
    struct Test {
      bool property = false;
    };

    std::vector<sync_sender_port_t<Test> *> senders;

    std::string recevier_port_name = "ReceiverPort";
    std::string sender1_port_name = "SenderPort1";
    std::string sender2_port_name = "SenderPort2";
    std::string sender3_port_name = "SenderPort3";
    std::string sender4_port_name = "SenderPort4";

    sync_receiver_port_t<Test> receiver_port(recevier_port_name);
    sync_sender_port_t<Test> sender1_port(sender1_port_name);
    sync_sender_port_t<Test> sender2_port(sender2_port_name);
    sync_sender_port_t<Test> sender3_port(sender3_port_name);
    sync_sender_port_t<Test> sender4_port(sender4_port_name);

    senders.push_back(&sender1_port);
    senders.push_back(&sender2_port);
    senders.push_back(&sender3_port);
    senders.push_back(&sender4_port);

    uint32_t receiver_port_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender1_port_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender2_port_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender3_port_error_id = env_errno_t::ENV_CLEAR;
    uint32_t sender4_port_error_id = env_errno_t::ENV_CLEAR;

    EXPECT_EQ(
        receiver_port
            .set_error_handler([&receiver_port_error_id](const uint64_t &, const uint32_t &id,
                                                         const uint32_t &) -> void { receiver_port_error_id = id; })
            .qualifiers.at(std::hash<std::string>()("ReceiverPort")),
        env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender1_port
                  .set_error_handler([&sender1_port_error_id](const uint64_t &, const uint32_t &id,
                                                              const uint32_t &) -> void { sender1_port_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("SenderPort1")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender2_port
                  .set_error_handler([&sender2_port_error_id](const uint64_t &, const uint32_t &id,
                                                              const uint32_t &) -> void { sender2_port_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("SenderPort2")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender3_port
                  .set_error_handler([&sender3_port_error_id](const uint64_t &, const uint32_t &id,
                                                              const uint32_t &) -> void { sender3_port_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("SenderPort3")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(sender4_port
                  .set_error_handler([&sender4_port_error_id](const uint64_t &, const uint32_t &id,
                                                              const uint32_t &) -> void { sender4_port_error_id = id; })
                  .qualifiers.at(std::hash<std::string>()("SenderPort4")),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.connect_port(&sender1_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.connect_port(&sender2_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.connect_port(&sender3_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.connect_port(&sender4_port).qualifiers.at(std::hash<std::string>()("ReceiverPort")),
              env_errno_t::ENV_CLEAR);

    {
      Test test_obj;
      Test test_obj2;

      test_obj.property = true;

      for (sync_sender_port_t<Test> *sender : senders) {

        EXPECT_EQ(sender->write(test_obj).qualifiers.at(sender->get_id()), env_errno_t::ENV_CLEAR);
        EXPECT_EQ(sender->forward().qualifiers.at(sender->get_id()), env_errno_t::ENV_CLEAR);
        EXPECT_EQ(receiver_port.read(test_obj2).qualifiers.at(receiver_port.get_id()), env_errno_t::ENV_CLEAR);
        EXPECT_EQ(receiver_port_error_id, env_errno_t::ENV_CLEAR);
        EXPECT_EQ(test_obj2.property, true);
        test_obj2.property = false;
      }
    }

    EXPECT_EQ(receiver_port.disconnect_port("SenderPort1").qualifiers.at(receiver_port.get_id()),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.disconnect_port("SenderPort2").qualifiers.at(receiver_port.get_id()),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.disconnect_port("SenderPort3").qualifiers.at(receiver_port.get_id()),
              env_errno_t::ENV_CLEAR);
    EXPECT_EQ(receiver_port.disconnect_port("SenderPort4").qualifiers.at(receiver_port.get_id()),
              env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port_error_id, env_errno_t::ENV_CLEAR);
    EXPECT_EQ(sender1_port_error_id, env_errno_t::ENV_CLEAR);
    EXPECT_EQ(sender2_port_error_id, env_errno_t::ENV_CLEAR);
    EXPECT_EQ(sender3_port_error_id, env_errno_t::ENV_CLEAR);
    EXPECT_EQ(sender4_port_error_id, env_errno_t::ENV_CLEAR);
  }

  void SyncSRPortTestHooks() {
    std::vector<std::string> testing_strings;

    sync_receiver_port_t<int> receiver_port("Receiver");
    sync_sender_port_t<int> sender_port("Sender");

    EXPECT_EQ(receiver_port.connect_port(&sender_port).qualifiers.at(receiver_port.get_id()), env_errno_t::ENV_CLEAR);

    EXPECT_EQ(receiver_port.on_receive()
                  .add("OnReceiveFirst",
                       [&testing_strings](int &data) -> void {
                         testing_strings.push_back("TestedByFirstHookOnReceive! Data = " + std::to_string(data));
                       })
                  .status.qualifiers.at(receiver_port.on_receive().get_id()),
              hook_errno_t::HOOK_CLEAR);

    EXPECT_EQ(receiver_port.on_receive()
                  .add("OnReceiveSecond",
                       [&testing_strings](int &data) -> void {
                         testing_strings.push_back("TestedBySecondHookOnReceive! Data = " + std::to_string(data));
                       })
                  .status.qualifiers.at(receiver_port.on_receive().get_id()),
              hook_errno_t::HOOK_CLEAR);

    EXPECT_EQ(receiver_port.on_receive()
                  .add("OnReceiveThird",
                       [&testing_strings](int &data) -> void {
                         testing_strings.push_back("TestedByThirdHookOnReceive! Data = " + std::to_string(data));
                       })
                  .status.qualifiers.at(receiver_port.on_receive().get_id()),
              hook_errno_t::HOOK_CLEAR);

    EXPECT_EQ(receiver_port.on_receive()
                  .add("OnReceiveForth",
                       [&testing_strings](int &data) -> void {
                         testing_strings.push_back("TestedByForthHookOnReceive! Data = " + std::to_string(data));
                       })
                  .status.qualifiers.at(receiver_port.on_receive().get_id()),
              hook_errno_t::HOOK_CLEAR);

    size_t read_count = receiver_port.read_count();
    size_t write_count = sender_port.write_count();

    EXPECT_EQ(sender_port.write(100).qualifiers.at(sender_port.get_id()), env_errno_t::ENV_CLEAR);
    EXPECT_EQ(testing_strings.size(), 0);
    EXPECT_EQ(sender_port.write_count(), write_count + 1);
    EXPECT_EQ(receiver_port.read_count(), read_count);
    EXPECT_EQ(sender_port.forward().qualifiers.at(sender_port.get_id()), env_errno_t::ENV_CLEAR);

    write_count = sender_port.write_count();
    read_count = receiver_port.read_count();

    ASSERT_EQ(testing_strings.size(), 4);
    EXPECT_EQ(testing_strings.at(0), "TestedByFirstHookOnReceive! Data = 100");
    EXPECT_EQ(testing_strings.at(1), "TestedBySecondHookOnReceive! Data = 100");
    EXPECT_EQ(testing_strings.at(2), "TestedByThirdHookOnReceive! Data = 100");
    EXPECT_EQ(testing_strings.at(3), "TestedByForthHookOnReceive! Data = 100");

    int result;
    EXPECT_EQ(receiver_port.read(result).qualifiers.at(receiver_port.get_id()), env_errno_t::ENV_CLEAR);
    EXPECT_EQ(sender_port.write_count(), write_count);
    EXPECT_EQ(receiver_port.read_count(), read_count + 1);
    EXPECT_EQ(result, 100);
  }
};

static env env_;

TEST(SyncSRPort, PortConnecting) { env_.SyncSRPortConnecting(); }
TEST(SyncSRPort, PortWriteForwardRead) { env_.SyncSRPortWriteForwardRead(); }
TEST(SyncSRPort, PortSenderQueueOverflow) { env_.SyncSRPortSenderQueueOverflow(); }
TEST(SyncSRPort, PortReaderWriter) { env_.SyncSRPortReaderWriter(); }
TEST(SyncSRPort, PortConnectionExists) { env_.SyncSRPortConnectionExists(); }
TEST(SyncSRPort, PortMaximumConnections) { env_.SyncSRPortMaximumConnections(); }
TEST(SyncSRPort, PortConnectionWithSameNames) { env_.SyncSRPortConnectionWithSameNames(); }
TEST(SyncSRPort, PortTestWithComplexDataType) { env_.SyncSRPortTestWithComplexDataType(); }

TEST(SyncSRPort, PortTestConnectWriteForwardReadDisconnectWithSeveralPorts) {
  env_.SyncSRPortTestConnectWriteForwardReadDisconnectWithSeveralPorts();
}

TEST(SyncSRPort, PortTestHooks) { env_.SyncSRPortTestHooks(); }
