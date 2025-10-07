# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Docker-based environment for running NVIDIA Isaac Sim with ROS 2 integration. The setup uses Docker Compose to orchestrate two services:
- **isaac-sim**: NVIDIA Isaac Sim 5.0.0 running in headless mode with WebRTC streaming and ROS 2 bridge extension enabled
- **ros2**: ROS 2 Humble (ros-base) container for interacting with Isaac Sim via ROS 2 topics/services

## Architecture

The system uses a multi-container setup with host networking:

1. **Isaac Sim Container**: Runs headless with the ROS 2 bridge extension (`isaacsim.ros2.bridge`) enabled at startup. It uses NVIDIA runtime for GPU access and WebRTC for remote visualization.

2. **ROS 2 Container**: A separate container running ROS 2 Humble that communicates with Isaac Sim via the ROS_DOMAIN_ID (default: 0).

3. **Volume Mounts**: Isaac Sim uses extensive volume mounts for caching and persistence:
   - Kit, Omniverse, pip, and GPU shader caches under `${HOST_BASE}/cache/`
   - Logs under `${HOST_BASE}/logs/`
   - User data/documents under `${HOST_BASE}/data/` and `${HOST_BASE}/documents/`
   - X11 socket for local GUI (if not using WebRTC)

## Common Commands

### Start the environment
```bash
docker-compose up -d
```

### Stop the environment
```bash
docker-compose down
```

### Access Isaac Sim container
```bash
docker exec -it isaac-sim bash
```

### Access ROS 2 container
```bash
docker exec -it ros2 bash
```

### Run ROS 2 demo script
Inside the ros2 container:
```bash
bash /root/ros2_demo.sh
```

This script:
- Sources ROS 2 Humble environment
- Starts a demo talker node
- Lists available ROS 2 topics

### View ROS 2 topics
Inside the ros2 container:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Configuration

All configuration is in `.env`:
- `ISAAC_SIM_IMAGE`: Isaac Sim Docker image version (default: 5.0.0)
- `ROS2_IMAGE`: ROS 2 Docker image (default: humble-ros-base)
- `ROS_DOMAIN_ID`: ROS 2 domain ID for discovery (must match between containers)
- `DISPLAY`: X11 display for local GUI
- `HOST_BASE`: Local directory for persistent volumes (default: `./_work`)

## Important Notes

- **GPU Access**: Isaac Sim requires NVIDIA runtime and GPU capabilities configured in Docker
- **ROS 2 Bridge**: The bridge extension is enabled via the startup flag `--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge`
- **Headless Mode**: Isaac Sim runs with `runheadless.sh -v` for WebRTC streaming. To use local X11 GUI, switch to `runapp.sh` in `scripts/run_isaac.sh:14`
- **Network Mode**: Both containers use `network_mode: host` for ROS 2 discovery to work properly
- **Shell Scripts**: The bash scripts use `set -euo pipefail` for safety, but `ros2_demo.sh` omits `-u` because `setup.bash` references undefined variables
