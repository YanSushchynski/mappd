load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "feature",
    "flag_group",
    "flag_set",
    "tool_path",
)


ARM_LINUX_PATH_FMT = "external/arm-linux/%s"

def _impl(ctx):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = "clang++.sh",
        ),
        tool_path(
            name = "ld",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-ld",
        ),
        tool_path(
            name = "ar",
            # path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-ar",
            path = "/usr/bin/ar",
        ),
        tool_path(
            name = "cpp",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-cpp",
        ),
        tool_path(
            name = "gcov",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-gcov",
        ),
        tool_path(
            name = "nm",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-nm",
        ),
        tool_path(
            name = "objdump",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-objdump",
        ),
        tool_path(
            name = "strip",
            path = ARM_LINUX_PATH_FMT % "bin/armv8l-linux-gnueabihf-strip",
        ),
    ]
    toolchain_include_directories_feature = feature(
        name = "toolchain_include_directories",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.linkstamp_compile,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.lto_backend,
                    ACTION_NAMES.clif_match,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/include/c++/7.5.0/",
                            "-isystem",
                            "external/arm-linux/lib/gcc/armv8l-linux-gnueabihf/7.5.0/include/",
                            "-isystem",
                            "external/arm-linux/lib/gcc/armv8l-linux-gnueabihf/7.5.0/include-fixed/",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/libc",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/libc/usr/include/",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/include/",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/include/c++/7.5.0",
                            # "-isystem",
                            # "external/arm-linux/armv8l-linux-gnueabihf/include/c++/7.5.0/armv8l-linux-gnueabihf/bits/",
                            "-isystem",
                            "external/arm-linux/armv8l-linux-gnueabihf/include/c++/7.5.0/armv8l-linux-gnueabihf",
                        ],
                    ),
                ],
            ),
        ],
    )
    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        toolchain_identifier = "k8-toolchain",
        host_system_name = "i686-unknown-linux-gnu",
        target_system_name = "i686-unknown-linux-gnu",
        target_cpu = "k8",
        target_libc = "unknown",
        compiler = "gcc",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = tool_paths,
        features = [toolchain_include_directories_feature],
    )


cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)