#!/bin/bash

PORT=${1:-2809}

mkdir -p /tmp/omniNames-logdir-$$

trap 'rm -fr /tmp/omniNames-logdir-$$' HUP INT QUIT TERM
omniNames -start $PORT -logdir /tmp/omniNames-logdir-$$ || echo -e "\e[31momniNames is already running at $PORT\e[m"

