load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "gtest",
    urls = ["https://github.com/google/googletest/archive/v1.10.x.tar.gz"],
)

http_archive(
    name = "protobuf",
    urls = ["https://github.com/google/googletest/archive/v1.10.x.tar.gz"],
)


git_repository(
    name = "protobuf",
    commit = "243558921f9b257d8137cfb4436e9ee774ce95f9",
    remote = "https://github.com/protocolbuffers/protobuf.git",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "9f9fb8b2f0213989247c9d5c0e814a8451d18d7f",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1570056263 -0700",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()