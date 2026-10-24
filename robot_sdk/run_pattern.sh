#!/usr/bin/env bash
# Execute a precomputed canvas drawing in the already-running official simulator.
set -euo pipefail
SDK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname -- "$SDK_DIR")"
LIBRARY="${ASTRIBOT_PATTERN_LIBRARY:-$REPO_DIR/pattern_library/v1}"
CONTAINER="${ASTRIBOT_SIM_CONTAINER:-astribot-simulation}"
PYTHON_BIN="${PYTHON:-/usr/bin/python3.10}"
INDEX=""; PLAN_ONLY=0; OUTPUT=""
usage() {
    cat <<'USAGE'
Usage: bash robot_sdk/run_pattern.sh [0-57] [--plan-only] [--output DIRECTORY]
       bash robot_sdk/run_pattern.sh --list

No number: prompt for a number. --list shows the exact stable mapping.
The official simulation must already be running. Reuses cached coupled drawing;
plans a new checked approach from current feedback to the fixed canvas start.
125 Hz, joint ceiling 1 rad/s, tip ceiling 0.2 m/s; stop at drawing endpoint.
USAGE
}
while (($#)); do
    case "$1" in
        --list) "$PYTHON_BIN" "$SDK_DIR/tools/pattern_selection.py" "$LIBRARY" --list; exit 0 ;;
        --plan-only) PLAN_ONLY=1; shift ;;
        --output) (($#>=2)) || { usage; exit 2; }; OUTPUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) [[ -z "$INDEX" && "$1" =~ ^[0-9]{1,2}$ ]] || { usage; exit 2; }; INDEX="$1"; shift ;;
    esac
done
if [[ -z "$INDEX" ]]; then read -r -p 'Pattern number [0-57]: ' INDEX; fi
[[ "$INDEX" =~ ^[0-9]{1,2}$ ]] || { echo 'Expected a number in 0..57' >&2; exit 2; }
((10#$INDEX <= 57)) || { echo 'Expected a number in 0..57' >&2; exit 2; }
selection="$("$PYTHON_BIN" "$SDK_DIR/tools/pattern_selection.py" "$LIBRARY" "$INDEX")"
mapfile -t fields <<< "$selection"
identity="${fields[0]}"
printf 'Pattern %s: %s | %s\n' "$INDEX" "$identity" "${fields[1]}"
[[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" == true ]] || { echo 'Simulation is not running' >&2; exit 2; }
[[ "$(docker inspect -f '{{index .Config.Labels "io.astribot.simulation-only"}}' "$CONTAINER")" == true ]] || { echo 'Container is not marked simulation-only' >&2; exit 2; }
OUTPUT="${OUTPUT:-$REPO_DIR/output/pattern-$INDEX-$(date +%Y%m%d-%H%M%S)}"
mkdir -p -- "$OUTPUT"; OUTPUT="$(cd -- "$OUTPUT" && pwd)"
STAGE="$(docker exec "$CONTAINER" mktemp -d /tmp/astribot-pattern.XXXXXXXX)"
docker exec "$CONTAINER" mkdir -p "$STAGE/robot_sdk" "$STAGE/svg_planner" "$STAGE/output" "$STAGE/pattern_library/v1/items/$identity"
docker cp "$SDK_DIR/." "$CONTAINER:$STAGE/robot_sdk/"
for source in "$REPO_DIR"/svg_planner/*.py; do docker cp "$source" "$CONTAINER:$STAGE/svg_planner/"; done
docker cp "$REPO_DIR/svg_planner/t1-canvas" "$CONTAINER:$STAGE/svg_planner/t1-canvas"
docker cp "$LIBRARY/pattern_lookup.json" "$CONTAINER:$STAGE/pattern_library/v1/pattern_lookup.json"
docker cp "${fields[2]}" "$CONTAINER:$STAGE/pattern_library/v1/items/$identity/source.svg"
docker cp "${fields[3]}" "$CONTAINER:$STAGE/pattern_library/v1/items/$identity/sdk-plan.npz"
docker cp "${fields[4]}" "$CONTAINER:$STAGE/pattern_library/v1/items/$identity/sdk-plan.validation.json"
args=(--library "$STAGE/pattern_library/v1" --index "$INDEX" --output "$STAGE/output")
((PLAN_ONLY==0)) || args+=(--plan-only)
revision="$(docker inspect -f '{{index .Config.Labels "com.astribot.descriptions.revision"}}' "$CONTAINER")"
cancel_owner() {
    docker exec "$CONTAINER" sh -c '
      test -f "$1/owner.pid" || exit 0
      pid=$(cat "$1/owner.pid")
      case "$pid" in ""|*[!0-9]*) exit 1;; esac
      test -r "/proc/$pid/cmdline" || exit 0
      tr "\000" " " < "/proc/$pid/cmdline" | grep -q astribot_draw_sdk.pattern_runner || exit 1
      kill -INT "$pid"
    ' sh "$STAGE" || true
}
collect() {
    result=$?; trap - EXIT INT TERM
    docker cp "$CONTAINER:$STAGE/output/." "$OUTPUT/" || result=1
    printf 'Reports: %s\nContainer diagnostics: %s:%s\n' "$OUTPUT" "$CONTAINER" "$STAGE" >&2
    exit "$result"
}
trap collect EXIT
trap cancel_owner INT TERM
docker exec -e "ASTRIBOT_DRAW_MODEL_REVISION=$revision" "$CONTAINER" bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  export ASTRIBOT_COMMU_BACKEND=ros
  source /opt/astribot_simulation/env.sh --robot t1
  export PYTHONPATH="$1/robot_sdk/src${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
  echo $$ > "$1/owner.pid"
  shift
  exec /usr/bin/python3.10 -m astribot_draw_sdk.pattern_runner "$@"
' bash "$STAGE" "${args[@]}" &
client=$!; result=0
wait "$client" || result=$?
if kill -0 "$client" 2>/dev/null; then wait "$client" || result=$?; fi
exit "$result"
