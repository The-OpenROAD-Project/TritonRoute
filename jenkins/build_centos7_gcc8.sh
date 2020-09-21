#!/bin/bash
set -x
set -e
export CC=/opt/rh/devtoolset-8/root/usr/bin/gcc
export CXX=/opt/rh/devtoolset-8/root/usr/bin/g++
scl enable devtoolset-8 ./jenkins/build.sh
