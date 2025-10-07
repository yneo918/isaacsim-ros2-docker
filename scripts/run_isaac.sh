#!/usr/bin/env bash
set -euo pipefail

# X11 を使うローカル Linux の場合のみ解放
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root || true
fi

cd /isaac-sim

# --- ここ最重要 ---
# ブリッジ拡張を起動フラグで有効化しつつ、ヘッドレス(-v)で WebRTC ストリーミングを有効化。
# GUI が必要なら runapp.sh に置き換え可（ローカル X11 が安定しない場合は Streaming Client 推奨）
./runheadless.sh \
  -v \
  --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
# -------------------

