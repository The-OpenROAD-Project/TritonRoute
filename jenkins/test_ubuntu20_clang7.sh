#!/bin/bash
set -x
set -e
export CC=/usr/bin/clang-7
export CXX=/usr/bin/clang++-7
./jenkins/test.sh
