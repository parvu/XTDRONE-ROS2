#!/usr/bin/env bash
# Build the XTDrone ROS 2 / Gazebo Harmonic manual (MkDocs) into docs/manual/site.
#   scripts/build_manual.sh          # build static HTML, open docs/manual/site/index.html
#   scripts/build_manual.sh serve    # live preview on http://127.0.0.1:8000
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${XTDRONE_CACHE:-${HOME}/.cache/xtdrone}/mkdocs-venv"
if [[ ! -x "${VENV}/bin/mkdocs" ]]; then
  /usr/bin/python3 -m venv "${VENV}"
  "${VENV}/bin/pip" install -q -r "${ROOT_DIR}/docs/manual/requirements.txt"
fi
unset PYTHONPATH  # keep ROS 2 Python packages out of the docs venv
cd "${ROOT_DIR}/docs/manual"
if [[ "${1:-build}" == "serve" ]]; then
  exec "${VENV}/bin/mkdocs" serve
fi
"${VENV}/bin/mkdocs" build --strict
echo "Manual built: ${ROOT_DIR}/docs/manual/site/index.html"
