package(default_visibility = ["//visibility:public"])

load(
    ":cc_arm64_toolchain_config.bzl", 
    "cc_arm64_toolchain_config", 
    "cc_x64_windows_arm64_toolchain_config",
    "cc_x86_64_unknown_linux_gnu_toolchain_config"
)

# load( ":cc_x86_64_unknown_linux_gnu_toolchain_config.bzl", "cc_x86_64_unknown_linux_gnu_toolchain_config") 

cc_arm64_toolchain_config(
    name = "arm64_toolchain_config"
)


cc_x64_windows_arm64_toolchain_config(
    name = "x64_windows_arm64_toolchain_config"
)

cc_x86_64_unknown_linux_gnu_toolchain_config(
    name = "x86_64_unknown_linux_gnu_toolchain_config"
)

cc_toolchain_suite(
    name = "arm64",
    toolchains = {
        "k8": ":k8_linux_arm_toolchain",
        "x64_windows": ":x64_windows_linux_arm_toolchain",
    },
)

cc_toolchain_suite(
    name = "x64_linux",
    toolchains = {
        "x64_windows": ":x86_64_unknown_linux_gnu",
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

filegroup(
    name = "windows-arm-toolchain-files",
    srcs = [
        "@windows_arm_linux//:all",
        "win_arm.bat"
    ]
)


filegroup(
    name = "x86_64-unknown-linux-gnu-toolchain-files",
    srcs = [
        "@win_lin//:all",
        "win_linux.bat"
    ]
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

cc_toolchain(
    name = "x64_windows_linux_arm_toolchain",
    toolchain_identifier = "x64_windows-arm64-toolchain",
    toolchain_config = ":x64_windows_arm64_toolchain_config",
    all_files = ":windows-arm-toolchain-files",
    compiler_files = ":windows-arm-toolchain-files",
    dwp_files = ":empty",
    linker_files = ":windows-arm-toolchain-files",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)


cc_toolchain(
    name = "x86_64_unknown_linux_gnu",
    toolchain_identifier = "x86_64-unknown-linux-gnu-toolchain",
    toolchain_config = ":x86_64_unknown_linux_gnu_toolchain_config",
    all_files = ":x86_64-unknown-linux-gnu-toolchain-files",
    compiler_files = ":x86_64-unknown-linux-gnu-toolchain-files",
    dwp_files = ":empty",
    linker_files = ":x86_64-unknown-linux-gnu-toolchain-files",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)