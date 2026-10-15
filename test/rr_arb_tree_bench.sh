#!/usr/bin/env bash
# Copyright 2026 KU Leuven.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

# Compares the structural rr_arb_tree with the fast_sim model (Bender target `fast_sim`,
# src/fast_sim/rr_arb_tree.sv) in VCS with test/rr_arb_tree_bench_tb.sv:
#   * the two runs must print the same CHK lines (a hash of every output of every instance, every
#     1000 cycles) and the same BENCH hash, and no unknown output, grant error or X-check error;
#   * the wall and CPU time of each, on 1 core and, with --threads, with VCS fine-grained
#     parallelism (the simv is built with -fgp; simv -fgp=num_threads:N uses N + 1 cores).
#
#   test/rr_arb_tree_bench.sh [--cycles N] [--threads "2 4"] [--reps R] [--jobs J] [--no-build]
#                             [--equiv-only]
#
# Needs bender and VCS on PATH. Builds go to build/rr_arb_tree_bench/<impl>/.

set -euo pipefail

CYCLES=20000
THREADS=""
REPS=3
JOBS=4
BUILD=1
SPEED=1
while [ $# -gt 0 ]; do
  case "$1" in
    --cycles)   CYCLES="$2";  shift 2 ;;
    --threads)  THREADS="$2"; shift 2 ;;
    --reps)     REPS="$2";    shift 2 ;;
    --jobs)     JOBS="$2";    shift 2 ;;
    --no-build) BUILD=0;      shift ;;
    --equiv-only) SPEED=0;    shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build/rr_arb_tree_bench"
IMPLS="structural fast_sim"

command -v bender >/dev/null || { echo "bender not found" >&2; exit 1; }
command -v vcs    >/dev/null || { echo "vcs not found" >&2; exit 1; }

build() {
  local impl="$1" target=""
  [ "$impl" = fast_sim ] && target="-t fast_sim"
  mkdir -p "$OUT/$impl"
  (
    cd "$ROOT"
    bender script vcs $target --vlog-arg="-timescale=1ns/1ps" > "$OUT/$impl/compile.sh"
  )
  (
    cd "$OUT/$impl"
    bash compile.sh > vlogan.log 2>&1
    vlogan -sverilog -full64 -timescale=1ns/1ps "$ROOT/test/rr_arb_tree_bench_tb.sv" >> vlogan.log 2>&1
    vcs -full64 -O4 -j"$JOBS" -fgp -assert disable_cover -timescale=1ns/1ps \
        -Mdir=csrc -o simv rr_arb_tree_bench_tb > vcs.log 2>&1
  )
}

if [ "$BUILD" = 1 ]; then
  for impl in $IMPLS; do
    echo "== build $impl"
    build "$impl"
  done
  # the structural build must not contain the model and the other way round
  grep -q 'fast_sim/rr_arb_tree.sv' "$OUT/fast_sim/compile.sh"   || { echo "fast_sim build lacks the model" >&2; exit 1; }
  ! grep -q 'fast_sim/rr_arb_tree.sv' "$OUT/structural/compile.sh" || { echo "structural build has the model" >&2; exit 1; }
  [ "$(grep -c 'src/rr_arb_tree.sv' "$OUT/fast_sim/compile.sh")" = 0 ] || { echo "fast_sim build has two rr_arb_tree" >&2; exit 1; }
fi

# run <impl> <threads|0> <rep>: prints "<wall s> <cpu s> <ok|WRONG|NOCORES>". An FGP run needs
# threads + 1 free cores; when VCS cannot lock them (other simulations hold the machine) it is
# retried, then given up. Every run must print the CHK and BENCH lines of the structural 1-core run.
run() {
  local impl="$1" threads="$2" rep="$3" flags="" t0 t1 cpu log attempt
  log="$OUT/$impl/run_t${threads}_${rep}.log"
  [ "$threads" != 0 ] && flags="-fgp=num_threads:$threads"
  for attempt in 1 2 3 4; do
    t0=$(date +%s.%N)
    ( cd "$OUT/$impl" && ./simv $flags +cycles="$CYCLES" -l "$(basename "$log")" > /dev/null 2>&1 ) || true
    t1=$(date +%s.%N)
    grep -q FGP_AFFINITY_FAILED "$log" || break
    sleep 20
  done
  if grep -q FGP_AFFINITY_FAILED "$log"; then echo "nan nan NOCORES"; return; fi
  cpu=$(grep -m1 'CPU Time' "$log" | sed -E 's/.*CPU Time: *([0-9.]+).*/\1/' || true)
  if ! diff -q <(grep '^CHK\|^BENCH' "$log") "$OUT/structural.chk" > /dev/null; then
    printf '%.2f %s WRONG\n' "$(echo "$t1 - $t0" | bc -l)" "${cpu:-nan}"; return
  fi
  printf '%.2f %s ok\n' "$(echo "$t1 - $t0" | bc -l)" "${cpu:-nan}"
}

status=0
printf 'host %s, load %s, %s cycles\n' "$(hostname)" "$(cut -d' ' -f1-3 /proc/loadavg)" "$CYCLES"

# equivalence: single core run of each
for impl in $IMPLS; do
  ( cd "$OUT/$impl" && ./simv +cycles="$CYCLES" -l equiv.log > /dev/null 2>&1 )
done
grep '^CHK\|^BENCH' "$OUT/structural/equiv.log" > "$OUT/structural.chk"
grep '^CHK\|^BENCH' "$OUT/fast_sim/equiv.log"   > "$OUT/fast_sim.chk"
if diff -q "$OUT/structural.chk" "$OUT/fast_sim.chk" > /dev/null; then
  echo "EQUIVALENT: $(wc -l < "$OUT/fast_sim.chk") lines identical"
  tail -1 "$OUT/fast_sim.chk"
else
  echo "MISMATCH, first difference:"
  diff "$OUT/structural.chk" "$OUT/fast_sim.chk" | head -4
  status=1
fi
for impl in $IMPLS; do
  tail -1 "$OUT/$impl.chk" | grep -q 'unknown_cycles=0 grant_errors=0 x_check_errors=0' \
    || { echo "$impl: unknown output, grant error or X-check error:"; tail -1 "$OUT/$impl.chk"; status=1; }
done

# speed: interleaved repetitions, best of the correct ones
[ "$SPEED" = 1 ] || exit $status
echo
printf '%-9s %-12s %10s %10s\n' cores impl wall_s cpu_s
for threads in 0 $THREADS; do
  declare -A best_wall=() best_cpu=()
  for rep in $(seq 1 "$REPS"); do
    for impl in $IMPLS; do
      read -r wall cpu verdict < <(run "$impl" "$threads" "$rep") || true
      case "$verdict" in
        ok) if [ -z "${best_wall[$impl]:-}" ] || (( $(echo "$wall < ${best_wall[$impl]}" | bc -l) )); then
              best_wall[$impl]=$wall; best_cpu[$impl]=$cpu
            fi ;;
        WRONG) echo "$impl, $((threads + 1)) cores: the run printed other CHK/BENCH lines than the reference" >&2
               status=1 ;;
      esac
    done
  done
  label=1; [ "$threads" != 0 ] && label=$((threads + 1))
  for impl in $IMPLS; do
    printf '%-9s %-12s %10s %10s\n' "${label}-core" "$impl" "${best_wall[$impl]:-n/a}" "${best_cpu[$impl]:-n/a}"
  done
  if [ -n "${best_wall[structural]:-}" ] && [ -n "${best_wall[fast_sim]:-}" ]; then
    printf '%-9s %-12s %9.2fx\n' "${label}-core" speedup "$(echo "${best_wall[structural]} / ${best_wall[fast_sim]}" | bc -l)"
  fi
  unset best_wall best_cpu
done

exit $status
