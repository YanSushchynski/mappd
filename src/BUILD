cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "//utils/dl:dl", 
        "//env/traits:traits", 
        "//env:env", 
        "@gtest//googletest-1.10.x:gtest",
    ],
    copts = [
        "-std=gnu++17",
        "-g",
        "-ldl"
    ]
)
