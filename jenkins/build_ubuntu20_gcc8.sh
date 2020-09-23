#!/bin/bash
set -x
set -e
export CC=/usr/bin/gcc-8
export CXX=/usr/bin/g++-8
./jenkins/build.sh
