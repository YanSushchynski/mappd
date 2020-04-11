load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "bazel_skylib",
    commit = "9935e0f820692f5f38e3b00c64ccbbff30cebe11",
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

# https://github.com/bazelbuild/rules_python
git_repository(
    name = "rules_python",
    commit = "94677401bc56ed5d756f50b441a6a5c7f735a6d4",
    remote = "https://github.com/bazelbuild/rules_python",
    shallow_since = "1573842889 -0500",
    #    sha256 = "aa96a691d3a8177f3215b14b0edc9641787abaaa30363a080165d06ab65e1161",
)

bazel_skylib_workspace()

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

# git_repository(
#     name = "protobuf",
#     commit = "ab968155e5f7da774bca1e40d90210401299f77d",
#     remote = "https://github.com/protocolbuffers/protobuf.git",
# )

http_archive(
    name = "protobuf",
    urls = ["https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-all-3.11.2.tar.gz"],
    # build_file = "//:arm-linux.BUILD",
    strip_prefix = "protobuf-3.11.2"
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


new_local_repository(
    name = "windows_arm_linux",
    path = "C:/Users/Yan_Sushchynski/work/toolchain/gcc-linaro-7.5.0-2019.12-i686-mingw32_armv8l-linux-gnueabihf",
    build_file = "arm-linux.BUILD",
)
