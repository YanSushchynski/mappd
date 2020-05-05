load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# protobuf repo requires bazel_skylib to be installed
git_repository(
    name = "bazel_skylib",
    commit = "9935e0f820692f5f38e3b00c64ccbbff30cebe11",
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

git_repository(
    name = "rules_python",
    commit = "94677401bc56ed5d756f50b441a6a5c7f735a6d4",
    remote = "https://github.com/bazelbuild/rules_python",
    shallow_since = "1573842889 -0500",
)

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

http_archive(
    name = "windows_arm_linux",
    urls = [ "https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/gcc-linaro-7.5.0-2019.12-i686-mingw32_aarch64-linux-gnu.tar.xz"],
    build_file = "//:arm-linux.BUILD",
    strip_prefix = "gcc-linaro-7.5.0-2019.12-i686-mingw32_aarch64-linux-gnu"
)

http_archive(
    name = "protobuf",
    urls = ["https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-all-3.11.2.tar.gz"],
    strip_prefix = "protobuf-3.11.2"
)

git_repository(
    name = "com_google_protobuf",
    commit = "09745575a923640154bcf307fba8aedff47f240a",
    remote = "https://github.com/protocolbuffers/protobuf",
    shallow_since = "1558721209 -0700",
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "libconfig",
    urls = ["https://github.com/hyperrealm/libconfig/archive/v1.7.2.tar.gz"],
    strip_prefix = "libconfig-1.7.2",
    build_file = "//:libconfig.BUILD",
)

http_archive(
    name = "zlib",
    build_file = "@com_google_protobuf//:third_party/zlib.BUILD",
    sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://mirror.bazel.build/zlib.net/zlib-1.2.11.tar.gz",
        "https://zlib.net/zlib-1.2.11.tar.gz",
    ],
)

# from official Bazel docs
git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "9f9fb8b2f0213989247c9d5c0e814a8451d18d7f",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1570056263 -0700",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
