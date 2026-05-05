#!/bin/bash

# This file is used to launch interactive
# docker container with opentitan
# https://github.com/lowRISC/opentitan/tree/master/util/container

docker run --rm -t -i \
  -v $(pwd):/home/dev/src \
  -v /home/marc-andre/tools/opentitan:/home/dev/opentitan \
  --env DEV_UID=$(id -u) --env DEV_GID=$(id -g) \
  --env OPENTITAN="/home/dev/opentitan" \
  opentitan:latest \
  bash
