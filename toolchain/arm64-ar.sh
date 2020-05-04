#!/bin/bash

set -euo pipefail

C_INCLUDE_PATH=external/arm-linux

external/arm-linux/bin/armv8l-linux-gnueabihf-gcc-7.5.0  "$@"

 # Remove the first line of .d file
find . -name "*.d" -exec sed -i '2d' {} \;