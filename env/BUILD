package(default_visibility = ["//visibility:public"])

cc_library(
    name = "env",
    hdrs = glob(["inc/*"]),
    includes = ["inc"],
    copts = [
            "-Iinc/",
    ],
    deps = [
        "//utils/env:env"
    ],
)

