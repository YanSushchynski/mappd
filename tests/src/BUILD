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
    ]
)
