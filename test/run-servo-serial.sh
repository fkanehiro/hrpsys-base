#!/bin/sh
# Standalone Linux test; never accesses a robot or physical serial port.
set -eu
test_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
output=${1:?usage: sh test/run-servo-serial.sh NEW_OUTPUT_DIRECTORY}
mkdir "$output"
output=$(CDPATH= cd -- "$output" && pwd)
for logging in off on; do
    flag=
    if [ "$logging" = on ]; then flag=-DSERVO_SERIAL_DEBUG; fi
    "${CXX:-g++}" -std=gnu++98 -pthread $flag \
        -I"$test_root/../rtc/ServoController" "$test_root/test-servo-serial.cpp" \
        -lutil -o "$output/test-$logging" > "$output/build-$logging.log" 2>&1
    cut=0
    while [ "$cut" -le 26 ]; do
        timeout 3s "$output/test-$logging" fragment "$cut" > "$output/$logging-fragment-$cut.log" 2>&1
        cut=$((cut + 1))
    done
    for mode in stale coalesced missing-echo missing-return checksum; do
        timeout 3s "$output/test-$logging" "$mode" > "$output/$logging-$mode.log" 2>&1
    done
    if [ "$logging" = on ]; then
        grep -q 'sending :' "$output/$logging-coalesced.log"
    else
        if grep -q 'sending :' "$output/$logging-coalesced.log"; then exit 1; fi
    fi
done
echo 'PASS: fragmented/coalesced packets, stale different-ID frames and failed reads; logging OFF/ON.'
