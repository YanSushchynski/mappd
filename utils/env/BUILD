package(default_visibility = ["//visibility:public"])


cc_library(
    name = "env",
    srcs = glob(["src/*"]),
    hdrs = glob(["inc/*"]),
    includes = [
        "inc"
    ],
    deps = [
        "//env/traits:traits",
        "//utils/libcidr:libcidr",
        "//utils/hash/sha256:sha256",
        "//utils/function:function"
    ]
)
