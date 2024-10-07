# boardc-imu/stm32_firmware

[![build_stm32](https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_stm32.yml/badge.svg)](https://github.com/IRobot-EC-2024/boardc-imu/actions/workflows/build_stm32.yml)

数据会通过 USBCDC 虚拟串口和两条 CAN 总线发送出来

为了减小仓库体积，没有包含 CubeMX 生成的工程文件和库文件，编译之前请重新生成一次

## USBCDC 数据包结构

```
byte1       SOF(0xA5)
byte2       SEQ
byte3       CRC8(SOF+SEQ)
byte4~5     quat_w      [-1, 1] -> [0~65535]
byte6~7     quat_x      [-1, 1] -> [0~65535]
byte8~9     quat_y      [-1, 1] -> [0~65535]
byte10~11   quat_z      [-1, 1] -> [0~65535]
byte12~13   CRC16(整个数据包)
```

## CAN 数据包结构

**STDID: 0xAAA**

**DLC: 8**

```
byte1~2     quat_w      [-1, 1] -> [0~65535]
byte3~4     quat_x      [-1, 1] -> [0~65535]
byte5~6     quat_y      [-1, 1] -> [0~65535]
byte7~8     quat_z      [-1, 1] -> [0~65535]
```

## 虚拟串口参数

- 任意波特率（建议 921600）
- 8N1
- 无流控
