#!/bin/bash
set -x
set -e
export CC=/opt/rh/llvm-toolset-7.0/root/usr/bin/clang
export CXX=/opt/rh/llvm-toolset-7.0/root/usr/bin/clang++
scl enable llvm-toolset-7.0 ./jenkins/test.sh
