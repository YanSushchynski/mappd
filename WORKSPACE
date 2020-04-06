load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "gtest",
    urls = ["https://github.com/google/googletest/archive/v1.10.x.tar.gz"],
)


http_archive(
    name = "arm-linux",
    urls = ["https://releases.linaro.org/components/toolchain/binaries/latest-7/armv8l-linux-gnueabihf/gcc-linaro-7.5.0-2019.12-x86_64_armv8l-linux-gnueabihf.tar.xz"],
    build_file = "//:arm-linux.BUILD",
    strip_prefix = "gcc-linaro-7.5.0-2019.12-x86_64_armv8l-linux-gnueabihf"
)

git_repository(
    name = "protobuf",
    commit = "243558921f9b257d8137cfb4436e9ee774ce95f9",
    remote = "https://github.com/protocolbuffers/protobuf.git",
)

new_local_repository(
    name = "boost",
    path = "/usr/lib/x86_64-linux-gnu/",
    build_file = "boost.BUILD",
)

new_local_repository(
    name = "openssl",
    path = "/usr/include/",
    build_file = "openssl.BUILD",
)
