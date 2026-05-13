#!/bin/bash

# spawn an opentitan docker
# and run the generate.sh inside there

docker run --rm -t -i \
  -v $(pwd):/home/dev/src \
  -v /home/marc-andre/tools/opentitan:/home/dev/opentitan \
  --env DEV_UID=$(id -u) --env DEV_GID=$(id -g) \
  --env OPENTITAN="/home/dev/opentitan" \
  opentitan:latest \
  'bash /home/dev/src/rtl/adc_acquisition/reg/generate.sh'