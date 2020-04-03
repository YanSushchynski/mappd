#!/bin/bash

set -euo pipefail

echo `ls`

external/arm-linux/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++  -ld "$@"

 # Remove the first line of .d file
#  find . -name "*.d" -exec sed -i '2d' {} \;