#!/usr/bin/env bash
# Run against an already-started Astribot SDK-compatible simulator.
set -euo pipefail

SDK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname -- "$SDK_DIR")"
PYTHON_BIN="${PYTHON:-/usr/bin/python3.10}"
SIM_DIR="${ASTRIBOT_SIMU_ROOT:-$REPO_DIR/../astribot_simulation}"
TRANSPORT=ros
RUNTIME=auto
CONTAINER="${ASTRIBOT_SIM_CONTAINER:-astribot-simulation}"
LAUNCHER="${ASTRIBOT_SIM_LAUNCHER:-$HOME/code/astribot-agent-meta/aos/astribot-e2e/simulation-deploy/start-simulation.sh}"
PEERS="${ASTRIBOT_SIM_PEERS:-tcp/127.0.0.1:7447}"
BACKEND=sdk-sim
MODE=""
PLAN=""
OUTPUT=""
PLAN_ONLY=0
SEEDS=12
SPEED_SCALE=1

usage() {
    cat <<'USAGE'
Usage: bash robot_sdk/run_sim.sh [start|probe|demo|draw INPUT.svg|run PLAN.npz] [OPTIONS]

  start               Use existing simulation-deploy/start-simulation.sh when present
  demo                Visible 10-degree double elbow motion, twice in 12s (not drawing)
  probe               Read state only; no control request or motion
  run PLAN.npz        Execute an exported, validated T1 command plan
  draw INPUT.svg      Plan from current pose, execute SVG, stop at endpoint, audit
  --plan-only         Validate/plan SVG without motion
  --seeds NUMBER      Deterministic SVG IK seed budget (default 12)
  --speed-scale RATE  SVG speed multiplier in (0,1] (default 1)
  --sim-dir PATH      Official astribot_simulation checkout
  --runtime MODE      auto (default), docker, or local
  --container NAME    Existing simulation container (default astribot-simulation)
  --launcher PATH     Existing aos/astribot-e2e/simulation-deploy/start-simulation.sh
  --transport MODE    ros (default, official simulator) or link
  --peers ENDPOINT    Simulator address when --transport link is selected
  --python PATH       Python with the vendor SDK (default /usr/bin/python3.10)
  --output DIRECTORY  Directory for execution reports and JSONL logs
  --mock              Offline protocol test; no vendor SDK or network
  -h, --help          Show this help

Start the official simulator first (start command, in a separate terminal).
No command: show help. draw accepts the upstream two-arm T1 SVG format.
Both terminals must use the same transport and ROS_DOMAIN_ID (default 25).
The script does not install packages; Link endpoints must route only to simulation.
Relative plan/output paths are resolved from your current directory.
USAGE
}

fail() { printf 'Error: %s\n' "$*" >&2; exit 2; }

(($#)) || { usage; exit 0; }

if (($#)) && [[ "$1" == start || "$1" == demo || "$1" == probe || "$1" == run || "$1" == draw ]]; then
    MODE="$1"
    shift
    if [[ "$MODE" == run || "$MODE" == draw ]]; then
        (($#)) || fail "$MODE requires an input file"
        PLAN="$1"
        shift
        [[ -f "$PLAN" ]] || fail "Plan file not found: $PLAN"
    fi
fi

while (($#)); do
    case "$1" in
        --peers|--python|--output|--sim-dir|--transport|--runtime|--container|--launcher|--seeds|--speed-scale)
            (($# >= 2)) && [[ -n "$2" ]] || fail "$1 requires a value"
            case "$1" in
                --peers) PEERS="$2" ;;
                --python) PYTHON_BIN="$2" ;;
                --output) OUTPUT="$2" ;;
                --sim-dir) SIM_DIR="$2" ;;
                --transport) TRANSPORT="$2" ;;
                --runtime) RUNTIME="$2" ;;
                --container) CONTAINER="$2" ;;
                --launcher) LAUNCHER="$2" ;;
                --seeds) SEEDS="$2" ;;
                --speed-scale) SPEED_SCALE="$2" ;;
            esac
            shift 2 ;;
        --mock) BACKEND=mock; shift ;;
        --plan-only) PLAN_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "Unknown argument: $1 (use --help)" ;;
    esac
done

[[ -n "$MODE" ]] || fail 'Specify start, probe, demo, draw INPUT.svg, or run PLAN.npz (use --help)'
draw_options=()
if [[ "$MODE" == draw ]]; then
    draw_options=(--seeds "$SEEDS" --speed-scale "$SPEED_SCALE")
    ((PLAN_ONLY == 0)) || draw_options+=(--plan-only)
fi
[[ "$TRANSPORT" == ros || "$TRANSPORT" == link ]] || fail '--transport must be ros or link'
[[ "$RUNTIME" == auto || "$RUNTIME" == docker || "$RUNTIME" == local ]] || fail '--runtime must be auto, docker or local'
if [[ "$MODE" == start && -f "$LAUNCHER" && "$RUNTIME" != local ]]; then
    exec bash "$LAUNCHER"
fi
if [[ "$RUNTIME" == auto ]]; then
    RUNTIME=local
    if [[ "$BACKEND" == sdk-sim && "$MODE" != start ]] && command -v docker >/dev/null 2>&1 && \
            [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" == true ]]; then
        RUNTIME=docker
    fi
fi
if [[ "$BACKEND" == sdk-sim && "$RUNTIME" == docker ]]; then
    [[ "$MODE" != start ]] || fail 'Use --launcher PATH to start the deployed simulator'
    [[ "$TRANSPORT" == ros ]] || fail 'The deployed-container adapter requires ROS transport'
    exec bash "$SDK_DIR/run_sim_docker.sh" "$CONTAINER" "$MODE" \
        "${OUTPUT:-$REPO_DIR/output/sdk-sim-$MODE}" "$PLAN" "$PYTHON_BIN" "${draw_options[@]}"
fi
if [[ "$MODE" == start || ("$BACKEND" == sdk-sim && "$TRANSPORT" == ros) ]]; then
    [[ -f "$SIM_DIR/env.sh" ]] || fail "Official simulator env.sh not found: $SIM_DIR (use --sim-dir)"
    [[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS2 Humble setup.bash not found'
    # Official environment scripts and conda hooks are not nounset-compatible.
    set +u
    if [[ "$MODE" == start ]]; then
        [[ "$BACKEND" != mock ]] || fail 'start does not accept --mock'
        CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
        [[ -f "$CONDA_SETUP" ]] || fail "Conda setup not found: $CONDA_SETUP"
        source "$CONDA_SETUP"
        conda activate "${ASTRIBOT_SIM_ENV:-astribot_simulation}"
    fi
    source /opt/ros/humble/setup.bash
    export ASTRIBOT_COMMU_BACKEND="$TRANSPORT"
    source "$SIM_DIR/env.sh" --robot t1
    set -u
    if [[ "$MODE" == start ]]; then
        export ASTRIBOT_EGL_CONTEXT_HANDOFF=1
        cd -- "$SIM_DIR"
        exec python3 astribot_simulation.py astribot_t1 --sensors custom
    fi
    # SDK is installed in the system Python user site on the current PC;
    # simulator conda env intentionally masks that site and has no SDK installed.
    [[ "$PYTHON_BIN" != /usr/bin/python3.10 ]] || export PYTHONNOUSERSITE=0
fi
command -v "$PYTHON_BIN" >/dev/null 2>&1 || fail "Python not found: $PYTHON_BIN (use --python PATH)"
export PYTHONPATH="$SDK_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
OUTPUT="${OUTPUT:-$REPO_DIR/output/$BACKEND-$MODE}"
args=("$MODE")
[[ "$MODE" != run && "$MODE" != draw ]] || args+=("$PLAN")
args+=(--backend "$BACKEND" --transport "$TRANSPORT" --output "$OUTPUT")
args+=("${draw_options[@]}")
[[ "$BACKEND" == mock || "$TRANSPORT" == ros ]] || args+=(--peers "$PEERS")
printf 'Mode: %s | Backend: %s | Reports: %s\n' "$MODE" "$BACKEND" "$OUTPUT" >&2
# exec preserves Ctrl+C delivery and the Python executor's cleanup/exit status.
exec "$PYTHON_BIN" -m astribot_draw_sdk "${args[@]}"
