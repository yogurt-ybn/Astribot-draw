#!/usr/bin/env bash
# Internal adapter: run the SDK inside the existing simulation-deploy container.
set -euo pipefail
SDK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=$1
MODE=$2
OUTPUT=$3
PLAN=$4
PYTHON_BIN=$5
shift 5

[[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" == true ]] || {
    echo "Simulation container is not running: $CONTAINER" >&2; exit 2;
}
[[ "$(docker inspect -f '{{index .Config.Labels "io.astribot.simulation-only"}}' "$CONTAINER")" == true ]] || {
    echo 'Container is not marked as the simulation-deploy simulator' >&2; exit 2;
}
[[ "$MODE" == probe || "$MODE" == demo || "$MODE" == run || "$MODE" == draw ]] || exit 2
mkdir -p -- "$OUTPUT"
OUTPUT="$(cd -- "$OUTPUT" && pwd)"
STAGE="$(docker exec "$CONTAINER" mktemp -d /tmp/astribot-draw-sdk.XXXXXXXX)"
docker exec "$CONTAINER" mkdir -p "$STAGE/robot_sdk" "$STAGE/output" "$STAGE/input"
docker cp "$SDK_DIR/." "$CONTAINER:$STAGE/robot_sdk/"
args=("$MODE")
if [[ "$MODE" == run || "$MODE" == draw ]]; then
    if [[ "$MODE" == draw ]]; then
        REPO_DIR="$(dirname -- "$SDK_DIR")"
        [[ -f "$REPO_DIR/svg_planner/svg_plan.py" ]] || { echo 'Update GitHub to include svg_planner first' >&2; exit 2; }
        docker exec "$CONTAINER" mkdir -p "$STAGE/svg_planner"
        docker cp "$REPO_DIR/svg_planner/." "$CONTAINER:$STAGE/svg_planner/"
        docker cp "$PLAN" "$CONTAINER:$STAGE/input/drawing.svg"
        args+=("$STAGE/input/drawing.svg")
    else
        docker cp "$PLAN" "$CONTAINER:$STAGE/input/plan.npz"
        validation="${PLAN%.*}.validation.json"
        [[ ! -f "$validation" ]] || docker cp "$validation" "$CONTAINER:$STAGE/input/plan.validation.json"
        args+=("$STAGE/input/plan.npz")
    fi
fi
args+=(--backend sdk-sim --transport ros --output "$STAGE/output")
args+=("$@")
model_revision="$(docker inspect -f '{{index .Config.Labels "com.astribot.descriptions.revision"}}' "$CONTAINER")"

cancel_owner() {
    # Docker exec does not forward Ctrl+C to the container process by itself.
    docker exec "$CONTAINER" sh -c '
        test -f "$1/owner.pid" || exit 0
        pid=$(cat "$1/owner.pid")
        case "$pid" in ""|*[!0-9]*) exit 1;; esac
        test -r "/proc/$pid/cmdline" || exit 0
        tr "\000" " " < "/proc/$pid/cmdline" | grep -q astribot_draw_sdk || exit 1
        kill -INT "$pid"
    ' sh "$STAGE" || true
}
collect_logs() {
    status=$?
    trap - EXIT INT TERM
    docker cp "$CONTAINER:$STAGE/output/." "$OUTPUT/" || {
        echo "Could not collect reports; retained in $CONTAINER:$STAGE/output" >&2
        status=1
    }
    printf 'Host reports: %s\nContainer diagnostics: %s:%s\n' "$OUTPUT" "$CONTAINER" "$STAGE" >&2
    exit "$status"
}
trap collect_logs EXIT
trap 'cancel_owner' INT TERM
printf 'Official simulation container: %s | Mode: %s\n' "$CONTAINER" "$MODE" >&2
# Keep the container's ROS domain, RMW, bootstrap and installed dependency paths.
docker exec -e "ASTRIBOT_DRAW_MODEL_REVISION=$model_revision" "$CONTAINER" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    export ASTRIBOT_COMMU_BACKEND=ros
    source /opt/astribot_simulation/env.sh --robot t1
    export PYTHONPATH="$1/robot_sdk/src${PYTHONPATH:+:$PYTHONPATH}"
    export PYTHONDONTWRITEBYTECODE=1
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
    echo $$ > "$1/owner.pid"
    py=$2
    shift 2
    exec "$py" -m astribot_draw_sdk "$@"
' bash "$STAGE" "$PYTHON_BIN" "${args[@]}" &
client=$!
status=0
wait "$client" || status=$?
# A signal can interrupt wait before the container owner has completed cleanup.
if kill -0 "$client" 2>/dev/null; then
    wait "$client" || status=$?
fi
exit "$status"
