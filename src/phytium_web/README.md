# phytium_web clean v2

最终 Web 总控台干净版：飞腾派性能、RTX3050 推理状态、机器人状态、视觉目标、Web 手动控制、急停。

## 替换
```bash
cd ~/phytium_ws/src
mv phytium_web phytium_web_backup_old
unzip phytium_web_clean_v2.zip -d ~/phytium_ws/src
cd ~/phytium_ws
colcon build --packages-select phytium_web
source /opt/ros/foxy/setup.bash
source ~/phytium_ws/install/setup.bash
```

## 启动
```bash
ros2 launch phytium_web bringup_web.launch.py yolo_receiver_url:=http://192.168.43.163:8080
```
浏览器访问：
```text
http://192.168.43.41:5000
```

## 控制说明
Web 手动控制默认发布到 `/cmd_vel_web`，不会直接抢 `/cmd_vel`。后续用速度仲裁节点决定是否转发。
