# 可用命令清单

坐标统一为 X 前、Y 左、Z 上，yaw 从上方观察逆时针为正。完整约定见 `COORDINATE_SYSTEM.md`，帧与状态字段见 `ROBOT_USB_CONTROL_PROTOCOL.md`。

USB 帧为 `A5 5A LEN CMD DATA... CRC_H CRC_L FF`。CRC 使用 CRC16/Modbus，初值 `0xFFFF`、多项式 `0xA001`，范围为头、长度、命令和数据。带参数命令的 DATA 是 4 个小端 `float32`，共 16 字节；无参数命令使用 `LEN=0`。

## 系统与底盘

| CMD | 名称 | DATA / 作用 |
|---:|---|---|
| `00` | `SYS_DISABLE` | 全系统失能 |
| `01` | `SYS_ENABLE` | 底盘、机械臂、工具使能 |
| `02` | `SYS_SWITCH_SOURCE` | `f0=0` USART，`f0=1` USB |
| `05` | `SYS_STOP` | 停止 USB 控制动作 |
| `06` | `SYS_GET_STATUS` | 查询系统状态 |
| `10/11` | `CHS_DISABLE/ENABLE` | 底盘失能/使能 |
| `12` | `CHS_SET_MODE` | `f0=mode`，范围 `0..7` |
| `13` | `CHS_SET_VEL` | `f0=vx` 前，`f1=vy` 左，`f2=yaw_data` CCW |
| `14` | `CHS_SET_POS` | `f0=dx` 前，`f1=dy` 左，`f2=yaw_data` CCW |
| `15/16` | `CHS_STOP/GET_STATUS` | 停止/查询底盘 |

模式 `0..3` 是速度模式，`4..7` 是位置模式；偶数模式锁定 yaw，奇数模式允许 yaw 运动；`0,1,4,5` 使用机器人系，`2,3,6,7` 使用世界系。无 yaw 模式下 `yaw_data` 是锁定角，单位度；其他速度模式下为 `vw` rad/s，其他位置模式下为 `dyaw` rad。速度命令应以小于 100 ms 的间隔持续发送。

## 机械臂和工具

| CMD | 名称 | DATA / 作用 |
|---:|---|---|
| `20/21` | `ARM_DISABLE/ENABLE` | 机械臂失能/使能 |
| `23` | `ARM_SET_TARGET` | 机械臂基座系：`f0=x` 前、`f1=y` 左、`f2=z` 上，单位 mm |
| `25/26` | `ARM_STOP/GET_STATUS` | 保持/查询机械臂 |
| `30/31` | `TOOL_DISABLE/ENABLE` | 工具失能/使能 |
| `32` | `TOOL_SET_MODE` | `f0=0` 夹爪位，`f0=1` 吸盘位 |
| `33` | `TOOL_ACTION` | 兼容命令：`f0=0` 关，`f0=1` 开 |
| `34` | `TOOL_SET_STATE` | `f0=0/1` 夹爪/吸盘，`f1=0/1` 关/开 |
| `35/36` | `TOOL_STOP/GET_STATUS` | 停止/查询工具 |

机械臂输入外框为 X、Y `[-500,500] mm`，Z `[50,500] mm`；可达性和安全区仍由 IK 做第二层检查。

## 整车状态和 yaw 整定

| CMD | 名称 | 作用 |
|---:|---|---|
| `46` | `ROBOT_GET_STATUS` | 查询整车综合状态；协议版本 4 表示统一 FLU 坐标 |
| `47` | `YAW_TUNE_START` | 可选 `f0=pass_count`，启动 yaw 整定 |
| `48` | `YAW_TUNE_STOP` | 停止整定 |
| `49` | `YAW_TUNE_GET_STATUS` | 查询整定状态 |

## 上下台阶

| CMD | 名称 | 作用 |
|---:|---|---|
| `50/51` | `CLIMB_DISABLE/ENABLE` | 机构失能/使能 |
| `52` | `CLIMB_SET_CTRL` | `f0=enable, f1=step, f2=auto`，兼容 USART 控制位 |
| `53/54` | `CLIMB_UP_STEP/AUTO` | 上台阶单步/自动 |
| `55/56` | `CLIMB_STOP/GET_STATUS` | 停止/查询 |
| `57` | `CLIMB_TEST_ACTION` | `f0=action_id`，执行单项测试 |
| `58/59` | `CLIMB_DOWN_STEP/AUTO` | 下台阶单步/自动 |
| `5A/5B` | `CLIMB_UP_GATE/DOWN_GATE` | 只执行对应激光门控和接近动作 |
| `5C/5D` | `CLIMB_UP_AUTO_PAUSE/DOWN_AUTO_PAUSE` | 自动运行并在设定中断点暂停 |
| `5E` | `CLIMB_AUTO_RESUME` | 从中断点继续 |

自动上台阶以 `+X=0.08 m/s` 前进，等待 `x_pos` 通道进入 `0..34 mm`；随后四杆到 `-30 mm`，底盘再沿 `+X` 前进 30 mm。自动下台阶也沿 `+X=0.08 m/s` 前进，等待高度通道超过 65 mm；触发后再沿 `+X` 前进 5 mm，四杆到 `-30 mm`。激光连续无效 1000 ms 或 8000 ms 未触发会进入错误状态。

正式流程分别以 `CLIMB_ACTION_SEQUENCE.yaml` 和 `DOWNSTAIRS_ACTION_SEQUENCE.yaml` 为准。

测试动作 ID 为 `1..38`，名称由上位机 `serial_tool/status.py` 中的 `CLIMB_TEST_ACTIONS` 定义。常用动作包括：

- `1 ALL_LEGS_220`、`16 ALL_LEGS_ZERO`
- `23 FRONT_220`、`24 FRONT_MINUS_30`
- `25 REAR_220`、`26 REAR_MINUS_30`
- `27..32` 前部小驱动轮前进/后退测试
- `33..38` 全部小驱动轮前进/后退测试

上位机 shell 会自动生成 `climb_<action_name_lowercase>` 快捷命令，例如 `climb_front_minus_30`。完整工具用法见 `PC_USB_Serial_Tool/README.md`。
