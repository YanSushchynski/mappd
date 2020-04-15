load("toolchain.bzl", "custom_toolchain")

toolchain_type(name = "toolchain_type")


custom_toolchain(
    name = "cc_linux",
    arch_flags = [
        "--arch=Linux",
        "--debug_everything",
    ],
    compiler_path = "/usr/bin/gcc",
    system_lib = "/usr/lib/libbarc.so",
)


toolchain(
    name = "win_linux_toolchain",
    exec_compatible_with = [
        "@platforms//os:windows",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        ":k8",
    ],
    toolchain = "//toolchain:k8_linux_arm_toolchain",
    toolchain_type = ":toolchain_type",
)


constraint_setting(name = "glibc_version")

constraint_value(
    name = "glibc_2_25",
    constraint_setting = ":glibc_version",
)

constraint_value(
    name = "glibc_2_26",
    constraint_setting = ":glibc_version",
)


constraint_setting(name = "cpu")

constraint_value(
    name = "k8",
    constraint_setting = ":cpu",
)


constraint_setting(name = "cc_compiler")

constraint_value(
    name = "clang",
    constraint_setting = ":cc_compiler",
)

platform(
    name = "linux_x86",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
        "@bazel_tools//tools/cpp:clang"
        # ":k8"
        # ":glibc_2_25"
    ],
    remote_execution_properties = """
        properties: {
          name: "container-image"
          value:"docker://gcr.io/gcp-runtimes/ubuntu_16_0_4@sha256:096632d8fb3e78fbd58ae6a2b25ed46020dc70e65d89bca774af6f7b2de6898c"
        }
        properties {
           name: "OSFamily"
           value:  "Linux"
        }
        """,

)


platform(
    name = "windows",
    constraint_values = [
        "@platforms//os:windows",
        "@platforms//cpu:x86_64"

        # ":glibc_2_25"
    ],

)
