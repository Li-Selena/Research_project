# 排球机器人

本目录整合 STM32 下位机与视觉上位机：

```text
排球机器人/
├── 下位机/                 STM32F407、Keil MDK 工程
├── 上位机/                 Python 视觉与 Modbus 主站
├── docs/modbus_registers.md
└── README.md
```

上位机源码来自 Gitee 仓库 `wyzyoi/vo` 的提交 `4191b2f`，复制时没有保留嵌套 `.git`。本版本修复了路径、相机标定目录、检测参数、PID 历史误差/积分清零和单轴底盘发送等问题，并将原 `A5/5A` 裸字节通信替换为线程安全的 Modbus RTU。

## 已实现功能

- CAN1 的 1/2/3 号 3508 驱动三轮底盘，支持本体坐标定速和世界坐标绝对定位；定位原点上电建立，只在明确执行“里程计清零”时改变。
- CAN1 的 4/5/6 号 3508 驱动 Delta。三个主动臂在水平位置上电，首次有效反馈作为 0°；关节正方向为从水平面逆时针。默认保持 `(0, 0, 161.116)` mm，击向默认 `(0, 0, 400)` mm 后自动回原位。
- CyberGear 上电反馈位置定义为甩杆 0°，随后自动到 `+150°` 预备位；可回 0°、击向 `−40°` 并自动回预备位。
- Delta 与甩杆动作互斥；击球开始时强制停止底盘。默认视觉击球机构为 Delta，可在配置中改成 `rod`。
- 上位机以 50 Hz 刷新底盘命令，以 10 Hz 刷新心跳并读取状态。心跳丢失 500 ms 后下位机执行安全停车和机构回收。
- USART1 Modbus RTU 支持 `0x03/0x04/0x06/0x10`、CRC、异常响应、分包与连续请求。

完整寄存器定义见 [docs/modbus_registers.md](docs/modbus_registers.md)。

## 硬件连接

| STM32F407 | 外设 | 用途 |
|---|---|---|
| PA9 / USART1_TX | USB 转 TTL RX | 上位机 Modbus |
| PA10 / USART1_RX | USB 转 TTL TX | 上位机 Modbus |
| GND | USB 转 TTL GND | 共地 |
| USART2 | HWT605 | 姿态传感器，保持原接线 |
| CAN1 | 3508 ID 1/2/3 | 底盘 |
| CAN1 | 3508 ID 4/5/6 | Delta |
| CAN2 | CyberGear ID 2 | 甩杆 |

USART1 配置为 115200、8N1、TTL。UART4 遥控器接收已经停用并从初始化和中断配置中移除。

## 下位机编译

1. 使用 Keil MDK 打开 `下位机/MDK-ARM/F407_whole_car.uvprojx`。
2. 选择工程现有 STM32F407 target，重新 Build。
3. 烧录重新生成的 HEX。

工程已加入 `modbus_rtu.c`、`modbus_crc.c` 和 `robot_control.c`。当前开发环境没有 Keil/ArmClang，因此仓库不附带由本次源码生成的新 HEX；交付前必须在安装了器件包的 Keil 中完整编译一次。

Delta 尺寸和控制参数集中在 `下位机/applications/Inc/control_config.h`。当前几何参数为底座半径 98.57 mm、动平台半径 110 mm、主动臂 192 mm、从动臂 242 mm；速度 PID 为 `Kp=25`、`Ki=0.05`、`Kd=0.2`，输出上限 12000，积分上限 3000。

## 上位机安装与运行

建议使用 Python 3.10 或更新版本：

```bash
cd 上位机
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

编辑 `上位机/config.json`：

- `serial_port` 建议明确填写，如 Linux 的 `/dev/ttyUSB0` 或 Windows 的 `COM5`；保留 `null` 时仅在找到唯一 USB/TTL 串口时自动选择。
- 主相机默认为编号 0、`model/camera6`；Delta 相机默认为编号 2、`model/camera3_135`。
- `default_striker` 只能使用 `delta` 或 `rod`。

运行视觉控制：

```bash
./start.sh
```

Windows 可执行 `start.bat`。按 `q` 退出；关闭时客户端会发送底盘停止。

## 无相机调试

所有命令从 `上位机` 目录执行：

```bash
python3 robot_cli.py status
python3 robot_cli.py speed 300 0 0 --seconds 2
python3 robot_cli.py position 1000 0 90 --seconds 8
python3 robot_cli.py stop
python3 robot_cli.py reset-odometry
python3 robot_cli.py delta-point 0 0 400 --timeout-ms 300
python3 robot_cli.py delta-strike
python3 robot_cli.py delta-home
python3 robot_cli.py rod-prepare
python3 robot_cli.py rod-home
python3 robot_cli.py rod-strike
python3 robot_cli.py emergency-stop
```

Python 代码可直接使用 `RobotClient` 的 `set_chassis_speed`、`move_chassis_to`、`stop_chassis`、`reset_odometry`、`set_delta_strike_point`、`strike_delta`、`home_delta`、`prepare_rod`、`home_rod`、`strike_rod`、`emergency_stop` 和 `read_status` 接口。

## 首次上电调试

1. 架空底盘并卸除或隔离击球负载，确认急停可用。
2. 人工将 Delta 三个主动臂放在水平 0°，将甩杆放在定义的默认 0°。系统没有额外限位开关，上电姿态就是机械基准。
3. 上电后不要触碰机构。等待 CAN 反馈；甩杆会自动转到 `+150°` 预备位，Delta 会保持原位。
4. 执行 `robot_cli.py status`，确认状态位 bit2、bit3、bit4 均在线，Delta 状态为 1，甩杆状态为 3。
5. 先以低速、短时间验证底盘三个方向。若坐标或电机方向相反，在 `control_config.h` 的统一方向参数或底盘矩阵中修改，不要交换协议字段。
6. 单独测试 Delta 回原位，再用较低 Z 击球点逐步增加到 400 mm；确认三个臂没有机械干涉。
7. 单独测试甩杆回原位、预备位和击球。若“向上”实际为负方向，将 `ROD_DIRECTION` 改为 `-1.0f`。
8. 最后启用视觉程序。一次只选择一个击球机构；状态机也会拒绝并发动作。

紧急停止会使 Delta 和甩杆进入停止状态。排除故障、人工恢复安全姿态后重新上电，重新建立机械零位。

## 验证

不连接相机或硬件即可执行：

```bash
cd 上位机
python3 -m unittest discover -s tests -v
python3 -m py_compile *.py tests/*.py
```

下位机包含运动学/底盘矩阵测试和 CRC 测试源码。项目已用主机 GCC 完成 C99 语法和严格警告检查；实机验收仍需验证电机方向、机械限位、CAN ID、底盘尺寸换算和绝对定位精度。

```bash
cd 下位机
./tests/run_host_tests.sh
```

该脚本执行同一份生产代码的 CRC、Modbus 解析/异常响应、Delta 到位/超时/不可达/反馈丢失状态机、Delta/甩杆互斥与通信看门狗，以及运动学/底盘矩阵测试。
