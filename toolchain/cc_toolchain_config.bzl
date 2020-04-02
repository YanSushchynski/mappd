load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "tool_path")

def _impl(ctx):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = "emcc.sh",
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
            path = "/usr/bin/clang++"
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
    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        toolchain_identifier = "clang++-toolchain",
        host_system_name = "i686-unknown-linux-gnu",
        target_system_name = "i686-unknown-linux-gnu",
        target_cpu = "clang++",
        target_libc = "unknown",
        compiler = "clang++",
        abi_version = "unknown",
        abi_libc_version = "unknown",
    )


cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)