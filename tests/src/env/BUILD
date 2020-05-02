load("//tests/src:tests.bzl", "test_gen")


[ 
    test_gen(
        name = filename.replace("/", "_").replace(".cpp", ""),
        srcs =  [filename],
        copts = [
            "-Iexternal/gtest/include",
            "-Iexternal/libconfig/lib",
            "-pthread",
            "-lcrypto",
            "-std=gnu++17",
            "-lgdi32",
            "-lssl" ,
        ],
        deps = [
            "@gtest//googletest-1.10.x:gtest",
            "@protobuf//:protobuf",
            "@libconfig//:lib",
            "//env:env",
            "//tests/src:main_test",
            "//utils/tuple:tuple",
        ],
    )    
    for filename in glob(["*"], exclude=["main.cpp", "BUILD"])
]