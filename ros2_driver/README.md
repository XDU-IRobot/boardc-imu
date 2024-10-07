# boardc-imu/ros2_driver

<table>
<tr>
<td>foxy</td>
<td rowspan=2>
    <a href="https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_ros2_eol.yml">
        <img src="https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_ros2_eol.yml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>galactic</td>
</tr>
<tr>
<td>humble</td>
<td rowspan=4>
    <a href="https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_ros2.yml">
        <img src="https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_ros2.yml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>iron</td>
</tr>
<tr>
<td>jazzy</td>
</tr>
<tr>
<td>rolling</td>
</tr>
</table>

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

| topic       | 消息类型                        | 备注                |
| ----------- | ------------------------------- | ------------------- |
| /quaternion | geometry_msgs/QuaternionStamped | Mahony 解算出的姿态 |
