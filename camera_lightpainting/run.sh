#!/usr/bin/env bash
set -euo pipefail
module_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${LIGHTPAINT_PYTHON:-}" ]]; then
    task_python="$LIGHTPAINT_PYTHON"
elif [[ -x "$module_dir/.venv/bin/python" ]]; then
    task_python="$module_dir/.venv/bin/python"
elif [[ -x "$module_dir/../.venv/bin/python" ]]; then
    task_python="$module_dir/../.venv/bin/python"
else
    task_python="python3"
fi
"$task_python" -c 'import cv2, numpy, PIL, flask, waitress'
exec "$task_python" "$module_dir/web_app.py" "$@"
