#!/bin/sh
# Builds the track simulator against the real control code and runs every test.
#
# The simulator compiles track.c, racing_line.c and driver.c unmodified - the same
# files that go on the car - and drives a bicycle-model car round synthetic circuits,
# rendering a Pixy2-like 79x52 frame at every step. It needs any host C compiler.
#
#   ./build_and_run.sh              all tests
#   ./build_and_run.sh -line        the racing line through one corner
#   ./build_and_run.sh -chicane     small chicane handling
#   ./build_and_run.sh -isec        intersection detection and crossing
#   ./build_and_run.sh -tracks      12 layouts x 5 starting offsets, and a
#                                   four road crossing in every part of each
#   ./build_and_run.sh -fault       camera failure behaviour
#   ./build_and_run.sh -sweep       robustness over camera mountings
#   ./build_and_run.sh -v           lap trace
set -e
CC=${CC:-gcc}
HERE=$(dirname "$0")
OUT="$HERE/track_sim"
$CC -O1 -std=gnu99 -o "$OUT" \
    "$HERE/track_sim.c" \
    "$HERE/../source/track.c" \
    "$HERE/../source/racing_line.c" \
    "$HERE/../source/intersection.c" \
    "$HERE/../source/driver.c" \
    "$HERE/../source/speed_ctl.c" \
    -I"$HERE/../include" -DRACE_BENCH_MODE=0 -lm
if [ $# -eq 0 ]; then
    "$OUT"; echo; "$OUT" -line; echo; "$OUT" -chicane; echo; "$OUT" -isec; echo; "$OUT" -tracks; echo; "$OUT" -fault; echo; "$OUT" -sweep
else
    "$OUT" "$@"
fi
