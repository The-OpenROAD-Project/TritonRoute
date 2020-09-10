#!/bin/bash
set -x
set -e
cmake --build build --target test
cd ./test || exit 1
./unit_test.sh ../build/TritonRoute
