package(default_visibility = ["//visibility:public"])

load(":cc_arm64_toolchain_config.bzl", "cc_arm64_toolchain_config")

cc_arm64_toolchain_config(
    name = "arm64_toolchain_config"
)

cc_toolchain_suite(
    name = "arm64",
    toolchains = {
        "k8": ":k8_linux_arm_toolchain",
    },
)

filegroup(name = "empty")

filegroup(
    name = "all",
    srcs = [
        "arm64.sh",
        "@arm-linux//:all",
    ],
)


cc_toolchain(
    name = "k8_linux_arm_toolchain",
    toolchain_identifier = "arm64-toolchain",
    toolchain_config = ":arm64_toolchain_config",
    all_files = ":all",
    compiler_files = ":all",
    dwp_files = ":empty",
    linker_files = ":all",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)