#include "client_port.hpp"
#include "env_static.hpp"
#include "receiver_port.hpp"
#include "sender_port.hpp"
#include "server_port.hpp"
#include "gtest/gtest.h"

void foo0(int argc, char *argv[], const struct env_base_t *const env_) { std::printf("Foo0\r\n"); }
void foo1(int argc, char *argv[], const struct env_base_t *const env_) { std::printf("Foo1\r\n"); }
void foo2(int argc, char *argv[], const struct env_base_t *const env_) { std::printf("Foo2\r\n"); }

env_static_t env(
    "TestEnv",
    composition_list_static_t(
        composition_static_t(

            "Composition0",

            port_list_static_t<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                               sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1", "Server1"),

            component_list_static_t(

                component_static_t(

                    "Component0",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5",
                                                               "Port6", "Port7", "Port8", "Port9", "Port10", "Port11"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component1",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                        "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component2",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component3",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}}))),
            composition_list_static_t(
                composition_static_t(

                    "Composition0",

                    port_list_static_t<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                                       sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1",
                                                                  "Server1"),

                    component_list_static_t(

                        component_static_t(

                            "Component0",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4",
                                                                       "Port5", "Port6", "Port7", "Port8", "Port9",
                                                                       "Port10", "Port11"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component1",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                                "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component2",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component3",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})))),

                composition_static_t(

                    "Composition1",

                    port_list_static_t<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                                       sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1",
                                                                  "Server1"),

                    component_list_static_t(

                        component_static_t(

                            "Component0",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                               sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4",
                                                                       "Port5", "Port6", "Port7", "Port8", "Port9",
                                                                       "Port10", "Port11"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component1",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                                "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component2",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                        component_static_t(

                            "Component3",

                            port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                               assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                            runtime_list_static_t(
                                {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})))))),
        composition_static_t(

            "Composition1",

            port_list_static_t<recp_t<int>, recp_t<int>, recp_t<int>, sclip_t<int(int, int)>,
                               sserp_t<int(int, double)>>("Receiver1", "Receiver2", "Receiver3", "Client1", "Server1"),

            component_list_static_t(

                component_static_t(

                    "Component0",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>, sclip_t<int(int, int)>, sclip_t<int(int, int)>,
                                       sclip_t<int(int, int)>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5",
                                                               "Port6", "Port7", "Port8", "Port9", "Port10", "Port11"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component1",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>, sclip_t<int(int, int)>, asclip_t<int(int, int)>>(
                        "Port0", "Port1", "Port2", "Port3", "Port4", "Port5", "Port6", "Port7"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component2",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_t({{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}})),

                component_static_t(

                    "Component3",

                    port_list_static_t<recp_t<int>, ssenp_t<int>, assenp_t<int>, recp_t<int>, ssenp_t<int>,
                                       assenp_t<int>>("Port0", "Port1", "Port2", "Port3", "Port4", "Port5"),

                    runtime_list_static_t(
                        {{"foo0", "sig0", foo0}, {"foo1", "sig1", foo1}, {"foo2", "sig2", foo2}}))))));

TEST(EnvTesting, SimpleTestCase) { env.network_manager().run(); }
