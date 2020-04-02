cc_library(
    name = "env",
    srcs = glob(["src/*"]),
    hdrs = glob(["inc/*"]),
    visibility = [
        "//utils/jit:__pkg__",
        "//env:__pkg__",
    ],
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
