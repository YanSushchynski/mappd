cc_library(
    name = "jit",
    srcs = glob(["src/*"]),
    hdrs = glob(["inc/*"]),
    visibility = ["//utils/dl:__pkg__"],
    includes = ["inc"],
    deps = [
        "//utils/env:env", 
        "//utils/hash/sha256:sha256", 
        "//utils/hash/fnv1a:fnv1a", 
        "//utils/function:function", 
        "//utils/tuple:tuple", 
    ],
)
