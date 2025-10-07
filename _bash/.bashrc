# ROS 2 環境設定
source /opt/ros/humble/setup.bash

# ROS_DOMAIN_ID
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ワークスペースが存在する場合は自動的にソース
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# エイリアス
alias cb='cd /root/ros2_ws && colcon build && source install/setup.bash'
alias ws='cd /root/ros2_ws'

# プロンプト設定
export PS1='\[\033[01;32m\][ROS2]\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\]\$ '

echo "ROS 2 Humble environment loaded (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
