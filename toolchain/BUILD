load(":cc_toolchain_config.bzl", "cc_toolchain_config")

cc_toolchain_config(name = "clang++_toolchain_config")

cc_toolchain_suite(
    name = "clang++",
    toolchains = {
        "clang++": ":clang++_toolchain",
    },
)

filegroup(name = "empty")

cc_toolchain(
    name = "clang++_toolchain",
    toolchain_identifier = "clang++-toolchain",
    toolchain_config = ":clang++_toolchain_config",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)