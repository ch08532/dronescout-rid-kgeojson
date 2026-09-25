#!/bin/sh
# Build odid_sim against opendroneid-core-c at the commit the DroneScout firmware uses,
# then generate the default scenario (Oslo Gardermoen, 4 drones, 600 s).
# Extra arguments are passed to odid_sim, e.g.: sh build.sh --seconds 1200
set -e
C=4785de4570e2ecd418543d130d16147108181d0e
U=https://raw.githubusercontent.com/opendroneid/opendroneid-core-c/$C/libopendroneid
curl -sfLO $U/opendroneid.h
curl -sfLO $U/opendroneid.c
gcc -O1 -o odid_sim odid_sim.c opendroneid.c -lm
./odid_sim "$@" > sim.jsonl 2> layout.txt
tail -1 layout.txt
echo "wrote sim.jsonl ($(wc -l < sim.jsonl) samples) and layout.txt"
