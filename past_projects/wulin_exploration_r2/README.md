# 武林探秘 R2

本目录将 R2 机器人的 STM32 下位机与 ROS 2 视觉上位机集中存放，便于联调、归档和移交。

## 目录

- `26_RC_02/`：STM32H723 + FreeRTOS 下位机，包含底盘、机械臂、攀爬机构、USB/USART 通信及 PC 串口调试工具。
- `vo2/`：ROS 2 上位机，包含目标检测、PnP 定位、双向 STM32 通信、Nav2 任务协调及整车控制接口。
- [上下位机功能核查.md](上下位机功能核查.md)：依据当前源码整理的功能清单、已接通流程与真机联调边界。
- `26_RC_02_项目解析.md`：下位机历史解析，部分坐标描述已过时，当前采用下文 FLU 约定。
- `vo_上位机项目解析.md`：另一个双相机排球 `vo` 项目的历史资料，不代表当前 `vo2` 的功能。

## 联调约定

两端统一采用右手 FLU 坐标系：

- X 轴向前
- Y 轴向左
- Z 轴向上
- 从上方观察，yaw 逆时针为正

`vo2` 中的位置单位为米；`robot_serial_bridge` 发送 `ARM_SET_TARGET` 前转换为毫米。USB 帧格式为：

```text
A5 5A LEN CMD DATA CRC_H CRC_L FF
```

协议、字段和安全限制分别见：

- `26_RC_02/ROBOT_USB_CONTROL_PROTOCOL.md`
- `26_RC_02/COORDINATE_SYSTEM.md`
- `vo2/COORDINATE_SYSTEM.md`

## 下位机

使用 Keil MDK 打开 `26_RC_02/MDK-ARM/26_RC_02.uvprojx`，或使用 STM32CubeMX 打开 `26_RC_02/26_RC_02.ioc`。完整固件编译需要 STM32H7 工具链和对应软件包。

主机侧一致性检查：

```bash
cd 26_RC_02
python3 tools/verify_control_plan.py
gcc -std=c11 -Wall -Wextra -Werror \
  -IComponents/Algorithm/Inc \
  tools/test_coordinate_math.c \
  Components/Algorithm/Src/ins_nav_math.c \
  Components/Algorithm/Src/mecanum_classic.c \
  -lm -o /tmp/26_RC_02_coordinate_math_test
/tmp/26_RC_02_coordinate_math_test
PYTHONPATH=PC_USB_Serial_Tool \
  python3 -m unittest discover -s PC_USB_Serial_Tool/tests -v
```

## 上位机

支持 Ubuntu、ROS 2 Humble 和 Python 3.10。构建方式：

```bash
cd vo2
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

相机内参、相机相对机械臂基座的位置和目标实际尺寸必须完成标定后才能用于真机。串口输出默认关闭；确认定位坐标后，通过 `serial_enabled:=true` 显式开启。机械臂、吸盘夹爪、底盘和上下台阶的 ROS 2 调用方式见 [控制接口说明](vo2/src/robot_serial_bridge/CONTROL.md)，已有 SLAM/Nav2 时的导航、越障和抓取任务见 [任务接口说明](vo2/src/r2_mission/MISSION.md)。

## 打包说明

归档保留源码、项目配置、模型、文档以及 `vo2` 的 Git 历史。下位机的 Keil/CMSIS 编译中间目录、PC 工具虚拟环境与 Python 缓存未复制；上位机的 `build/`、`install/` 和 `log/` 可按上述命令重新生成。
