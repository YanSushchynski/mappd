package(default_visibility = ["//visibility:public"])

cc_library(
    name = "env",
    hdrs = glob(["inc/*"]),
    includes = ["inc"],
    copts = [
            "-Iinc/",
            "-lssl"
    ],
    deps = [
        "//utils/env:env",
        "@boost//:format",
    ],
)

