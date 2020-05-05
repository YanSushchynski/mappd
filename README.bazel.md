# 

This project requires opens-ssl to be preinstalled. 
```
apt-get install libssl-dev
```
There is a problem with openssl/configuration.h.in which is generated into openssl/configuration.h during apt-install.

# Testing

Tests work under Linux or WSL2.

```
# run all tests
bazel test tests/src/... -k

# run tests of a submodule 
bazel test tests/src/hooks:all -k

# run a single tests
bazel test  tests/src/hooks:test_hook_replace_pure_lambdas 
```

Use the "-k" flag to keep going testing if any test fails.

# Cross-Compilation.

If you want cross compile the project for the ARM-platform, simply add "--config=arm64" to a bazel command. It works both on Windows 10 and Linux resolving a necessary toolchain automatically.
For details about toolchains' configurations see the toolchain folder.

```
bazel build tests/src:main_test --config=arm64 
```
