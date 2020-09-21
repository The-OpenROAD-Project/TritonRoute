#!/bin/bash
set -x
set -e
TARGET_OS=${1:-centos7}
TARGET_COMPILER=${2:-gcc8}
docker run --rm "openroad/tritonroute_${TARGET_OS}_${TARGET_COMPILER}" bash -c "./jenkins/test_${TARGET_OS}_${TARGET_COMPILER}.sh"
