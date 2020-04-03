load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "feature",
    "flag_group",
    "flag_set",
    "tool_path",
)


ARM_LINUX_FMT = "external/arm-linux/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/%"

def _impl(ctx):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = "clang++.sh",
        ),
        tool_path(
            name = "ld",
            path = "emcc.sh",
        ),
        tool_path(
            name = "ar",
            path = "/bin/false",
        ),
        tool_path(
            name = "clang++",
            path = "/usr/bin/gcc"
        ),
        tool_path(
            name = "cpp",
            path = "/bin/false",
        ),
        tool_path(
            name = "gcov",
            path = "/bin/false",
        ),
        tool_path(
            name = "nm",
            path = "/bin/false",
        ),
        tool_path(
            name = "objdump",
            path = "/bin/false",
        ),
        tool_path(
            name = "strip",
            path = "/bin/false",
        ),
    ]
    # toolchain_include_directories_feature = feature(
    #     name = "toolchain_include_directories",
    #     enabled = True,
    #     flag_sets = [
    #         flag_set(
    #             actions = [
    #                 ACTION_NAMES.assemble,
    #                 ACTION_NAMES.preprocess_assemble,
    #                 ACTION_NAMES.linkstamp_compile,
    #                 ACTION_NAMES.c_compile,
    #                 ACTION_NAMES.cpp_compile,
    #                 ACTION_NAMES.cpp_header_parsing,
    #                 ACTION_NAMES.cpp_module_compile,
    #                 ACTION_NAMES.cpp_module_codegen,
    #                 ACTION_NAMES.lto_backend,
    #                 ACTION_NAMES.clif_match,
    #             ],
    #             flag_groups = [
    #                 flag_group(
    #                     flags = [
    #                         "-isystem",
    #                         "/usr/include",
    #                         "-isystem",
    #                         "external/emscripten_toolchain/system/include/libc",
    #                     ],
    #                 ),
    #             ],
    #         ),
    #     ],
    # )
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
        # features = [toolchain_include_directories_feature],
    )


cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)