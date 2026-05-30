# phytium_vision clean v2

## 目标架构

```text
飞腾派 D435i RGB
  -> rgb_sender 低分辨率 JPEG
  -> RTX3050/4060Ti yolo_bbox_server 推理
  -> 返回 bbox/class/conf
  -> 飞腾派 target_depth_follower 用本地高分辨率 aligned depth 算距离
  -> 发布 /perception/target 和 /cmd_vel_follow
```

云端不再处理深度图。深度留在飞腾派本地，避免 RGB 低分辨率和高分辨率 depth 混用时距离错位。

## 飞腾派安装

把整个 `phytium_vision` 文件夹放到：

```bash
~/phytium_ws/src/phytium_vision
```

编译：

```bash
cd ~/phytium_ws
colcon build --packages-select phytium_vision
source /opt/ros/foxy/setup.bash
source ~/phytium_ws/install/setup.bash
```

## RTX3050 端启动

```bash
pip install -r scripts_rtx3050/requirements_rtx3050.txt
python3 scripts_rtx3050/yolo_bbox_server.py \
  --model yolov8.engine \
  --rgb-port 9999 \
  --flyt-pi-host <飞腾派IP> \
  --bbox-port 9997 \
  --target-class person \
  --conf 0.45
```

先不用 TensorRT 时可以用：

```bash
python3 scripts_rtx3050/yolo_bbox_server.py --model yolov8n.pt --flyt-pi-host <飞腾派IP>
```

## 飞腾派启动视觉客户端

```bash
ros2 launch phytium_vision vision_client.launch.py inference_host:=<RTX3050_IP>
```

查看目标：

```bash
ros2 topic echo /perception/target
ros2 topic echo /cmd_vel_follow
```

## 接入底盘

默认只发布 `/cmd_vel_follow`，不直接控制 `/cmd_vel`。
确认目标检测、距离和速度都正常后，再启动速度仲裁：

```bash
ros2 launch phytium_vision cmd_vel_mux.launch.py mode:=follow
```

## 安全默认值

```text
max_linear  = 0.12 m/s
max_reverse = 0.05 m/s
max_angular = 0.35 rad/s
target_distance = 1.0 m
lost_timeout_sec = 0.6 s
```
