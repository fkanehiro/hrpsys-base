#!/bin/bash

openhrp-controller-bridge --server-name HRP-4C \
    --periodic-rate HRP-4C\(Robot\)0:1.0 \
    --periodic-rate HGcontroller0:1.0 \
    --in-port qRef:JOINT_VALUE \
    --in-port dqRef:JOINT_VELOCITY \
    --in-port ddqRef:JOINT_ACCELERATION \
    --out-port q:JOINT_VALUE \
    --connection qRef:HGcontroller0:qOut \
    --connection dqRef:HGcontroller0:dqOut \
    --connection ddqRef:HGcontroller0:ddqOut



