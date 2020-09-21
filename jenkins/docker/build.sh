#!/bin/bash
set -x
set -e
TARGET_OS=${1:-centos7}
TARGET_COMPILER=${2:-gcc8}
docker build --tag "openroad/tritonroute_${TARGET_OS}_${TARGET_COMPILER}"  -f "./jenkins/docker/Dockerfile.${TARGET_OS}"  --build-arg compiler="${TARGET_COMPILER}" .
