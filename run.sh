#!/bin/bash
#
# Run hpp-exec Docker container.
#
# robotpkg provides base HPP C++ deps. hpp-manipulation and hpp-python are
# built from source (devel branch) inside a persistent volume.
#
# First run inside container:
#   cd ~/devel/src && make all
#
# Usage:
#   ./run.sh                              # interactive bash
#   ./run.sh python3 script.py            # run a command
#   ./run.sh --domain-id 7                # set ROS_DOMAIN_ID
#   ./run.sh --rebuild                    # force rebuild image
#
# To open another terminal in the same container:
#   docker exec -it hpp-exec bash
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER_NAME="hpp-exec"
DOMAIN_ID=0
REBUILD=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --rebuild) REBUILD=true; shift ;;
        --) shift; break ;;
        *) break ;;
    esac
done

# Persistent volume for hpp-manipulation/hpp-python builds and ros2_ws
DEVEL_DIR="$SCRIPT_DIR/docker/devel"
mkdir -p "$DEVEL_DIR/src" "$DEVEL_DIR/install"

# Copy config.sh and Makefile (always update)
cp "$SCRIPT_DIR/docker/config.sh" "$DEVEL_DIR/config.sh"
cp "$SCRIPT_DIR/docker/Makefile" "$DEVEL_DIR/src/Makefile"

# Set up ROS2 workspace
mkdir -p "$DEVEL_DIR/ros2_ws/src"

# Check if hpp-python is built
if [ ! -d "$DEVEL_DIR/install/lib/python3.12/site-packages/pyhpp" ]; then
    echo "============================================"
    echo "First run: hpp-manipulation + hpp-python need to be built."
    echo "Inside the container, run:"
    echo "  cd ~/devel/src && make all"
    echo "============================================"
    echo ""
fi

# Build image if not exists or --rebuild
if $REBUILD || ! docker image inspect hpp-exec >/dev/null 2>&1; then
    echo "Building hpp-exec Docker image..."
    docker build \
        --build-arg DOCKER_USER=$(id -u) \
        --build-arg DOCKER_GROUP=$(id -g) \
        -t hpp-exec "$SCRIPT_DIR"
fi

# If container already running, exec into it
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Attaching to running container..."
    docker exec -it "$CONTAINER_NAME" "${@:-bash}"
    exit 0
fi

# Remove stopped container with same name if exists
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# GPU support
DOCKER_GPU_ARGS=""
if command -v nvidia-smi &>/dev/null && docker info 2>/dev/null | grep -q nvidia; then
    DOCKER_GPU_ARGS="--gpus all"
fi

# X11 forwarding
DOCKER_DISPLAY_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

# Use -it only when stdin is a terminal
TTY_FLAG=""
if [ -t 0 ]; then
    TTY_FLAG="-it"
fi

docker run $TTY_FLAG --rm --net host \
    --name "$CONTAINER_NAME" \
    --privileged \
    $DOCKER_GPU_ARGS \
    $DOCKER_DISPLAY_ARGS \
    -v "$DEVEL_DIR:/home/user/devel" \
    -v "$SCRIPT_DIR:/home/user/devel/hpp-exec" \
    -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
    -e "DEVEL_HPP_DIR=/home/user/devel" \
    hpp-exec \
    "${@:-bash}"
