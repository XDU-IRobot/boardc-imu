# boardc-imu/ros2_driver

## 使用方法

添加 udev 规则

```bash
sudo ./scripts/setup_udev_rule.sh
```

如果电脑上现在已经插着 C 板，则在添加规则之后需要插拔一次

编译运行

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run boardc_imu_node boardc_imu_node
```

## topic list

| topic | 消息类型                        | 备注                     |
| ----- | ------------------------------- | ------------------------ |
| /imu  | sensor_msgs/Imu                 | 加速度计、陀螺仪原始数据 |
| /quat | geometry_msgs/QuaternionStamped | Mahony 解算出的姿态      |
