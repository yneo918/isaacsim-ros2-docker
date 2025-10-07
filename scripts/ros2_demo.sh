#!/usr/bin/env bash
# ここでは -u を使わない（setup.bash が未定義変数を参照するため）
set -eo pipefail

# あるいは、どうしても -u を使いたい場合は以下のように一時解除
# set -euo pipefail
# set +u

# ROS 2 環境を読み込み
source /opt/ros/humble/setup.bash

# set -u へ戻したい場合はここで再度有効化
# set -u

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS 2 nodes / topics をウォームアップします…"

# デモ: talker をバックグラウンド起動してトピックを確認
( ros2 run demo_nodes_cpp talker & ) >/dev/null 2>&1
sleep 2
ros2 topic list
sleep 8

