load("//tests/src:tests.bzl", "test_gen")


[ 
    test_gen(
        name = filename.replace("/", "_").replace(".cpp", ""),
        srcs =  [filename],
        copts = [
            "-Iexternal/gtest/include",
            "-pthread",
            "-std=gnu++17",
        ],
        deps = [
            "@gtest//googletest-1.10.x:gtest",
            "//env:env",
            "//tests/src:main_test",
            "//utils/tuple:tuple",
        ],
    )    
    for filename in glob(["*"], exclude=["main.cpp", "BUILD"])
]