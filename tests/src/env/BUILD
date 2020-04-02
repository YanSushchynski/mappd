load("//tests/src:tests.bzl", "test_gen")


[ 
    test_gen(
        name = i.replace("/", "_").replace(".cpp", ""),
        srcs =  [i],
        copts = [
            "-Iexternal/gtest/include",
            "-pthread",
        ],
        deps = [
            "@gtest//googletest-1.10.x:gtest",
            "@protobuf//:protobuf",
            "//env:env",
            "//tests/src:main_test",
            "//utils/tuple:tuple",
        ],
    )    
    for i in glob(["*"], exclude=["main.cpp", "BUILD"])
]