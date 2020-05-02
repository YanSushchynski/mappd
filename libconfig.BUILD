cc_library(
    name = "lib",
    srcs= glob([
        "lib/*.c",
        "lib/*.c++",
        "lib/*.h",
        "lib/*.h++"
    ]),
    hdrs = ["lib/libconfig.h++"],
    visibility = ["//visibility:public"],
    copts = [
        "-Ilib"
    ],
)