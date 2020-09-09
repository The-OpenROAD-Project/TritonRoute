#!/bin/bash
set -x
set -e

if [[ $# -ne 2 ]]; then
    echo "usage: $0 TARGET_OS TARGET_COMPILER"
    exit 1
fi

TARGET_OS="$1"
TARGET_COMPILER="$2"

DOCKER_TAG="openroad/tritonroute_${TARGET_OS}_${TARGET_COMPILER}"

BUILD_TEST="cmake --build build --target test"
RUN_UNITTEST="cd /TritonRoute/test && ./unit_test.sh /TritonRoute/build/TritonRoute"

docker run --rm "$DOCKER_TAG" bash -c "$BUILD_TEST && $RUN_UNITTEST"
