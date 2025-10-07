#!/usr/bin/env bash
set -euo pipefail

# ワークスペースディレクトリを作成
# Docker が root で自動作成するのを防ぐため、事前にユーザー権限で作成

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

# .env ファイルから設定を読み込む
if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
fi

# HOST_BASE のデフォルト値
HOST_BASE="${HOST_BASE:-./_work}"
ROS2_WORKSPACE="${ROS2_WORKSPACE:-./_ros2_ws}"

echo "Creating workspace directories..."

# _work ディレクトリ構造を作成
mkdir -p "${HOST_BASE}/cache/kit"
mkdir -p "${HOST_BASE}/cache/ov"
mkdir -p "${HOST_BASE}/cache/pip"
mkdir -p "${HOST_BASE}/cache/glcache"
mkdir -p "${HOST_BASE}/cache/computecache"
mkdir -p "${HOST_BASE}/logs"
mkdir -p "${HOST_BASE}/data"
mkdir -p "${HOST_BASE}/documents"

# ROS 2 ワークスペースを作成
mkdir -p "${ROS2_WORKSPACE}/src"

echo "Workspace directories created successfully:"
echo "  - ${HOST_BASE}"
echo "  - ${ROS2_WORKSPACE}"
echo ""
echo "Owner: $(whoami)"
echo "Permissions: $(stat -c '%a' ${HOST_BASE})"
