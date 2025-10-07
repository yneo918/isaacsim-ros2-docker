# isaacsim-ros2-docker

NVIDIA Isaac Sim と ROS 2 を Docker で統合する環境です。

## 概要

このリポジトリは、NVIDIA Isaac Sim 5.0.0 と ROS 2 Humble を Docker Compose で連携させるセットアップを提供します。

### 構成

- **isaac-sim コンテナ**: Isaac Sim をヘッドレスモード（WebRTC ストリーミング）で実行し、ROS 2 ブリッジ拡張を有効化
- **ros2 コンテナ**: ROS 2 Humble 環境で Isaac Sim と通信

## 必要な環境

- Docker と Docker Compose
- NVIDIA GPU と NVIDIA Container Toolkit（nvidia-docker2）
- （オプション）ローカル GUI の場合は X11

## セットアップ

1. `.env` ファイルで設定を確認・編集:
   ```bash
   # Isaac Sim と ROS 2 のバージョン
   ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.0.0
   ROS2_IMAGE=ros:humble-ros-base

   # ROS_DOMAIN_ID（両コンテナで一致させる）
   ROS_DOMAIN_ID=0

   # 永続ボリュームのベースディレクトリ
   HOST_BASE=./_work

   # ROS 2 ワークスペース
   ROS2_WORKSPACE=./_ros2_ws
   ```

2. ワークスペースディレクトリを作成:
   ```bash
   bash scripts/setup_workspace.sh
   ```

   このスクリプトは `_work` と `_ros2_ws` ディレクトリを現在のユーザー権限で事前作成します。これにより Docker が root 権限で作成するのを防ぎます。

3. 環境を起動:
   ```bash
   docker compose up -d
   ```

## 使い方

### コンテナへのアクセス

Isaac Sim コンテナ:
```bash
docker exec -it isaac-sim bash
```

ROS 2 コンテナ:
```bash
docker exec -it ros2 bash
```

### ROS 2 デモの実行

ROS 2 コンテナ内で:
```bash
bash /root/ros2_demo.sh
```

このスクリプトは demo talker ノードを起動し、トピックリストを表示します。

### ROS 2 トピックの確認

ROS 2 コンテナ内で:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /chatter
```

### ROS 2 ワークスペースでの開発

ホスト側の `./_ros2_ws` ディレクトリがコンテナ内の `/root/ros2_ws` にマウントされます。

ROS 2 コンテナ内でパッケージをビルド:
```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Isaac Sim の GUI アクセス

デフォルトではヘッドレスモード（WebRTC ストリーミング）で動作します。

#### WebRTC ストリーミングクライアント

1. **クライアントのダウンロード**:
   - [Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/manual_livestream_clients.html) をダウンロード
   - Linux: Ubuntu 22.04+ では `libfuse2` が必要
     ```bash
     sudo apt install libfuse2
     ```

2. **接続方法**:
   - Isaac Sim コンテナが起動し、"Isaac Sim Full Streaming App is loaded" メッセージを確認
   - Streaming Client を起動
   - サーバーアドレスに `127.0.0.1` を入力（localhost の場合）
   - "Connect" をクリック

3. **ネットワーク設定**:
   - UDP ポート 47998 と TCP ポート 49100 を開放
   - リモート接続の場合はホストの IP アドレスを指定

#### ローカル X11 GUI

`scripts/run_isaac.sh:14` の `runheadless.sh` を `runapp.sh` に変更してコンテナを再起動

## 環境の停止

```bash
docker-compose down
```

## トラブルシューティング

### GPU が認識されない
NVIDIA Container Toolkit が正しくインストールされているか確認:
```bash
docker run --rm --runtime=nvidia nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### ROS 2 ノードが見えない
`ROS_DOMAIN_ID` が両コンテナで一致しているか `.env` を確認してください。

## 参考資料

このセットアップは以下の公式ドキュメントを参考に作成されています:
- [Isaac Sim Container Installation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html)
- [Isaac Sim LiveStreaming Clients](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/manual_livestream_clients.html)

## ライセンス

このプロジェクトは MIT ライセンスの下で提供されています。詳細は LICENSE ファイルをご覧ください。
