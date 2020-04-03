load(":cc_toolchain_config.bzl", "cc_toolchain_config")

cc_toolchain_config(name = "clang++_toolchain_config")

cc_toolchain_suite(
    name = "gcc",
    toolchains = {
        "k8": ":k8_toolchain",
    },
)

filegroup(name = "empty")

filegroup(
    name = "all",
    srcs = [
        "clang++.sh",
        "@arm-linux//:all",
    ],
)


cc_toolchain(
    name = "k8_toolchain",
    toolchain_identifier = "clang++-toolchain",
    toolchain_config = ":clang++_toolchain_config",
    all_files = ":all",
    compiler_files = ":all",
    dwp_files = ":empty",
    linker_files = ":all",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)