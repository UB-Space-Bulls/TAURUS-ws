#!/usr/bin/env bash
# =============================================================================
# view_sim.sh — run nav2tag with fake tags and watch it in RViz, in a browser.
#
# Run on your HOST (Mac/Linux/WSL), not inside a container:
#   rover_ws/src/rover_nav2tag/scripts/view_sim.sh          # target tag 5
#   rover_ws/src/rover_nav2tag/scripts/view_sim.sh 3        # target tag 3
#   rover_ws/src/rover_nav2tag/scripts/view_sim.sh stop     # tear it down
#
# Then open http://localhost:6080/vnc.html?autoconnect=1&resize=scale
#
# How it works: Docker containers can't open GUI windows on a Mac, so the
# container gets a virtual screen (Xvfb), RViz draws onto it, and noVNC streams
# that screen to your browser. Those display tools live in a small derived image
# (rover-dev-viz, built FROM rover-dev) so dockerfile.dev itself stays untouched.
#
# Needs: Docker running, and the rover-dev image built once from the repo root:
#   docker build -f docker/dockerfile.dev -t rover-dev .
#
# Limits: no Nav2 is running, so the rover never moves — you see the fake rover,
# the decoy + target tags, and nav2tag's computed goal (red arrow).
# =============================================================================
set -euo pipefail

CONTAINER=rover-nav2tag-sim
VIZ_IMAGE=rover-dev-viz
PORT=6080
URL="http://localhost:${PORT}/vnc.html?autoconnect=1&resize=scale"

# Repo root = four levels up from this script (rover_ws/src/rover_nav2tag/scripts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [[ "${1:-}" == "stop" ]]; then
    docker rm -f "${CONTAINER}" >/dev/null 2>&1 && echo "Stopped ${CONTAINER}." || echo "${CONTAINER} wasn't running."
    exit 0
fi

TARGET_TAG="${1:-5}"
if ! [[ "${TARGET_TAG}" =~ ^-?[0-9]+$ ]]; then
    echo "Usage: $0 [target_tag_id | stop]" >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Docker isn't running. Start Docker Desktop (Mac: open -a Docker) and retry." >&2
    exit 1
fi

if ! docker image inspect rover-dev >/dev/null 2>&1; then
    echo "No rover-dev image yet. Build it once from the repo root:" >&2
    echo "  cd ${REPO_ROOT} && docker build -f docker/dockerfile.dev -t rover-dev ." >&2
    exit 1
fi

# Derived image with the virtual-display + browser-viewer tools. Docker caches
# this, so it only takes real time the first run (or after rover-dev changes).
echo "==> Preparing ${VIZ_IMAGE} image (cached after the first run)..."
docker build -q -t "${VIZ_IMAGE}" - >/dev/null <<'EOF'
FROM rover-dev
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        xvfb x11vnc novnc python3-websockify fluxbox libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
EOF

echo "==> Starting container ${CONTAINER}..."
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
if ! docker run -d --name "${CONTAINER}" -p "${PORT}:6080" \
        -v "${REPO_ROOT}/rover_ws:/workspaces/rover_ws" \
        "${VIZ_IMAGE}" sleep infinity >/dev/null; then
    echo "Couldn't start the container — is something else using port ${PORT}?" >&2
    echo "  (check with: docker ps)" >&2
    exit 1
fi

# Build as the host user so build/ install/ log/ aren't root-owned on Linux hosts
echo "==> Building rover_nav2tag..."
docker exec --user "$(id -u):$(id -g)" -e HOME=/tmp "${CONTAINER}" bash -lc \
    'source /opt/ros/jazzy/setup.bash && cd /workspaces/rover_ws && colcon build --packages-select rover_nav2tag --symlink-install' \
    | tail -3

echo "==> Starting virtual screen, browser viewer, nav2tag and RViz..."
docker exec -d "${CONTAINER}" bash -c 'Xvfb :1 -screen 0 1280x800x24 >/tmp/xvfb.log 2>&1'
sleep 2
docker exec -d "${CONTAINER}" bash -c 'DISPLAY=:1 fluxbox >/tmp/fluxbox.log 2>&1'
docker exec -d "${CONTAINER}" bash -c 'x11vnc -display :1 -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1'
docker exec -d "${CONTAINER}" bash -c 'websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1'
docker exec -d "${CONTAINER}" bash -lc "source /workspaces/rover_ws/install/setup.bash && \
    ros2 launch rover_nav2tag nav2tag.launch.py launch_fake_tags:=true target_tag_id:=${TARGET_TAG} >/tmp/nav2tag.log 2>&1"
docker exec -d "${CONTAINER}" bash -lc 'source /workspaces/rover_ws/install/setup.bash && \
    DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /workspaces/rover_ws/src/rover_nav2tag/rviz/nav2tag.rviz >/tmp/rviz.log 2>&1'

for _ in $(seq 1 30); do
    curl -fs -o /dev/null "http://localhost:${PORT}/vnc.html" && break
    sleep 1
done

cat <<MSG

Ready! Open:  ${URL}
  (RViz takes a few seconds to appear after the page connects.)

Targeting tag ${TARGET_TAG}. Switch tags live (arrow moves within ~5 s):
  docker exec ${CONTAINER} bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /target_tag_id std_msgs/msg/Int32 "{data: 3}"'

Node logs:  docker exec ${CONTAINER} tail -f /tmp/nav2tag.log
Stop:       $0 stop
MSG

if [[ -n "${NO_BROWSER:-}" ]]; then
    :                      # NO_BROWSER=1 skips auto-opening the page
elif command -v open >/dev/null 2>&1; then
    open "${URL}"          # macOS
elif command -v xdg-open >/dev/null 2>&1; then
    xdg-open "${URL}" >/dev/null 2>&1 || true   # Linux desktop
fi
