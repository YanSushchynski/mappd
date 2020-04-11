package(default_visibility = ["//visibility:public"])

load("//tests/src:tests.bzl", "test_gen")


cc_library(
    name = "main_test",
    srcs = ["main.cpp"] ,
    copts = [
        "-Iexternal/gtest/include",
            ] ,
    deps = [
        "@gtest//googletest-1.10.x:gtest",
	    "@protobuf//:protobuf_lite",
    ]
)

config_setting(
    name = "arm_build",
    values = {"cpu": "arm"},
)

config_setting(
    name = "x86_debug_build",
    values = {
        "cpu": "x86",
        "compilation_mode": "dbg",
    },
)
