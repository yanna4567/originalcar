# Origincar 使用说明

## 编译

```bash
# 默认工作空间 /userdata/dev_ws/
cd /userdata/dev_ws/
colcon build --symlink-install
```

## 底盘和控制

启动底盘
```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

启动键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
以下按键进行控制
```
   u    i    o
   j    k    l
   m    ,    .
```
键盘节点会发布 速度消息 到 /cmd_vel 话题，底盘节点订阅速度消息实现键盘控制

## USB 相机驱动与图像可视化
```bash
ros2 launch origincar_bringup usb_websocket_display.launch.py
```
运行成功后，在同一网络的PC端，打开浏览器，输入 http://IP:8000，选择“web展示端”，即可查看图像和算法效果，IP为OriginBot的IP地址。

## 深度相机驱动与图像可视化
启动相机

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py

```
启动rqt_image_view
```
ros2 run rqt_image_view rqt_image_view
```

小车ip：192.168.127.10
联网ip：192.168.228.248
上位机连接ip：ws://192.168.228.248:9090
#安装rosbridge
sudo apt install ros-foxy-rosbridge-suite
#启动rosbridge端⼝
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

检查内存命令： df -h
编译命令：
cd /userdata/dev_ws/ 
colcon build
扩容命令：
sudo apt update
sudo apt install cloud-guest-utils
sudo growpart /dev/mmcblk1 2
sudo resize2fs /dev/mmcblk1p2

（Windows）按键wasd：
cd D:\ZJU\car_comp\digital\src
python bridge_client.py 192.168.228.248 

底盘：
ros2 launch origincar_base origincar_bringup.launch.py 
按键（uiojkl……）：
ros2 run teleop_twist_keyboard teleop_twist_keyboard
相机：
ros2 launch origincar_bringup usb_websocket_display.launch.py 
功能全开（可以识别二维码，有意义的命令）：
ros2 launch origincar_competition start.launch.py
二维码（只识别内容，不翻译为命令）：
ros2 run qr_decoder qr_decoder