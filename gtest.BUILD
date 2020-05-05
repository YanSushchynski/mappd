cc_library(
    name = "main",
    srcs = glob(
        ["googletest-release-1.7.x/src/*.cc"],
        exclude = ["googletest-release-1.7.x/src/gtest-all.cc"]
    ),
    hdrs = glob([
        "googletest-release-1.7.x/include/**/*.h",
        "googletest-release-1.7.x/src/*.h"
    ]),
    copts = [
        "-Iexternal/gtest/googletest-release-1.7.x/include",
        "-Iexternal/gtest/googletest-release-1.7.x"
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)