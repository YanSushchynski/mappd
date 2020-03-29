#include "client_port.hpp"
#include "env_static.hpp"
#include "receiver_port.hpp"
#include "sender_port.hpp"
#include "server_port.hpp"
#include "gtest/gtest.h"

void foo0(int argc, char *argv[], const struct env_base_s *const env_) { std::printf("Foo0\r\n"); }
void foo1(int argc, char *argv[], const struct env_base_s *const env_) { std::printf("Foo1\r\n"); }
void foo2(int argc, char *argv[], const struct env_base_s *const env_) { std::printf("Foo2\r\n"); }

auto &env = env_static_sglt_gen_s<env_networking_type_e::IPV4>::env_static_inst(
    "TestEnv",
    cmps_list_static_s(
        cmps_static_s(

            "Composition0",

            port_list_static_s<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                               sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1", "Server1"),

            cmp_list_static_s(

                cmp_static_s(

                    "Component0",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5",
                                                               "Port6", "Port7", "Port8", "Port9", "Port10", "Port11"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component1",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                        "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component2",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component3",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}}))),
            cmps_list_static_s(
                cmps_static_s(

                    "Composition0",

                    port_list_static_s<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                                       sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1",
                                                                  "Server1"),

                    cmp_list_static_s(

                        cmp_static_s(

                            "Component0",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4",
                                                                       "Port5", "Port6", "Port7", "Port8", "Port9",
                                                                       "Port10", "Port11"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component1",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                                "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component2",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component3",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})))),

                cmps_static_s(

                    "Composition1",

                    port_list_static_s<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                                       sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1",
                                                                  "Server1"),

                    cmp_list_static_s(

                        cmp_static_s(

                            "Component0",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4",
                                                                       "Port5", "Port6", "Port7", "Port8", "Port9",
                                                                       "Port10", "Port11"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component1",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                                "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component2",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        cmp_static_s(

                            "Component3",

                            port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_s(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})))))),
        cmps_static_s(

            "Composition1",

            port_list_static_s<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                               sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1", "Server1"),

            cmp_list_static_s(

                cmp_static_s(

                    "Component0",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5",
                                                               "Port6", "Port7", "Port8", "Port9", "Port10", "Port11"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component1",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                        "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component2",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_s({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                cmp_static_s(

                    "Component3",

                    port_list_static_s<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_s(
                        {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}}))))));

TEST(EnvTesting, SimpleTestCase) { env.nm().run(); }
