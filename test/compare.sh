#!/bin/sh
# Builds the simulator with whatever -D flags are given and prints one line per
# circuit, so two configurations can be diffed directly.
#
#   ./compare.sh                       current race_config.h
#   ./compare.sh -DSPEED_MAX=90.0f     with an override
set -e
HERE=$(dirname "$0")
OUT="$HERE/track_sim_cmp"
${CC:-gcc} -O1 -std=gnu99 -o "$OUT" \
    "$HERE/track_sim.c" "$HERE/../source/track.c" \
    "$HERE/../source/racing_line.c" "$HERE/../source/intersection.c" "$HERE/../source/driver.c" \
    "$HERE/../source/speed_ctl.c" \
    -I"$HERE/../include" -DRACE_BENCH_MODE=0 "$@" -lm
"$OUT" | awk '
  function num(s, key,   p) {
      p = index(s, key)
      if (p == 0) return 0
      return substr(s, p + length(key)) + 0
  }
  /FINISHED|DNF/ {
      name = $0; sub(/ +(FINISHED|\*DNF\*).*/, "", name)
      lt = num($0, "laptime="); av = num($0, "avg="); mx = num($0, "max=")
      fin = ($0 ~ /FINISHED/); next
  }
  /minClearance/ {
      mc = num($0, "minClearance="); ex = num($0, "excursions=")
      printf "%-32s %7.2fs avg=%6.1f max=%6.1f clr=%+5.2f exc=%d%s\n",
             name, lt, av, mx, mc, ex, (fin ? "" : "  DNF")
      tot += lt; texc += ex; if (!fin) dnf++
  }
  END {
      printf "%-32s %7.2fs   total    excursions=%d%s\n",
             "== TOTAL ==", tot, texc, (dnf ? "   " dnf " DNF" : "")
  }'
