#!/bin/bash

openhrp-controller-bridge --server-name HRP-4C --in-port qRef:JOINT_VALUE --in-port dqRef:JOINT_VELOCITY --in-port ddqRef:JOINT_ACCELERATION --out-port q:JOINT_VALUE


