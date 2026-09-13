# 机器人 USB 控制协议

本文描述当前固件和 `PC_USB_Serial_Tool` 共用的协议。综合状态的协议版本为 `4`，表示底盘、导航、机械臂和上位机字段统一采用 X 前、Y 左、Z 上、yaw 逆时针为正的右手坐标系。详细坐标定义见 `COORDINATE_SYSTEM.md`。

## 1. USB 帧

```text
A5 5A LEN CMD DATA[LEN] CRC_H CRC_L FF
```

- `LEN` 是 DATA 字节数，不含头、CMD、CRC 和尾。
- CRC16/Modbus 初值 `0xFFFF`、多项式 `0xA001`，计算范围从 `A5` 到 DATA 末尾，发送时高字节在前。
- 参数命令使用 16 字节 DATA：4 个小端 IEEE-754 `float32`。
- 无参数命令和状态请求使用 `LEN=0`。
- 接收端会检查命令所需长度，短帧不会读取上一帧残留数据。

## 2. 坐标和底盘模式

`vx/dx/x` 的正方向是向前，`vy/dy/y` 的正方向是向左，`vw/dyaw/yaw` 的正方向是从上方观察逆时针。机器人模式中的轴随车体旋转；世界模式中的轴固定，并在 yaw 为零时与机器人轴对齐。

| mode | 名称 | 参考系 | 控制 |
|---:|---|---|---|
| 0 | `ROBOT_NO_YAW_VEL` | 机器人 | 速度，锁定 yaw |
| 1 | `ROBOT_VEL` | 机器人 | 速度，允许 yaw 速度 |
| 2 | `WORLD_NO_YAW_VEL` | 世界 | 速度，锁定 yaw |
| 3 | `WORLD_VEL` | 世界 | 速度，允许 yaw 速度 |
| 4 | `ROBOT_NO_YAW_POS` | 机器人 | 位移，锁定 yaw |
| 5 | `ROBOT_POS` | 机器人 | 位移，允许 yaw 位移 |
| 6 | `WORLD_NO_YAW_POS` | 世界 | 位移，锁定 yaw |
| 7 | `WORLD_POS` | 世界 | 位移，允许 yaw 位移 |

`CHS_SET_VEL` 的 `f0/f1` 是 `vx/vy`，`CHS_SET_POS` 的 `f0/f1` 是 `dx/dy`。无 yaw 模式中 `f2` 是锁定角，单位度；其他速度模式中 `f2=vw`，单位 rad/s；其他位置模式中 `f2=dyaw`，单位 rad。速度控制有 100 ms 看门狗。

## 3. 命令

| CMD | 名称 | DATA |
|---:|---|---|
| `00` | `SYS_DISABLE` | 无 |
| `01` | `SYS_ENABLE` | 无 |
| `02` | `SYS_SWITCH_SOURCE` | `f0=0` USART，`f0=1` USB |
| `05` | `SYS_STOP` | 无 |
| `06` | `SYS_GET_STATUS` | 无 |
| `10/11` | `CHS_DISABLE/ENABLE` | 无 |
| `12` | `CHS_SET_MODE` | `f0=mode` |
| `13` | `CHS_SET_VEL` | `f0=vx, f1=vy, f2=yaw_data, f3=0` |
| `14` | `CHS_SET_POS` | `f0=dx, f1=dy, f2=yaw_data, f3=0` |
| `15/16` | `CHS_STOP/GET_STATUS` | 无 |
| `20/21` | `ARM_DISABLE/ENABLE` | 无 |
| `23` | `ARM_SET_TARGET` | `f0=x, f1=y, f2=z`，机械臂基座系，mm |
| `25/26` | `ARM_STOP/GET_STATUS` | 无 |
| `30/31` | `TOOL_DISABLE/ENABLE` | 无 |
| `32` | `TOOL_SET_MODE` | `f0=0` 夹爪位，`f0=1` 吸盘位 |
| `33` | `TOOL_ACTION` | 兼容接口：`f0=0/1` 关/开 |
| `34` | `TOOL_SET_STATE` | `f0=0/1` 夹爪/吸盘，`f1=0/1` 关/开 |
| `35/36` | `TOOL_STOP/GET_STATUS` | 无 |
| `46` | `ROBOT_GET_STATUS` | 无 |
| `47` | `YAW_TUNE_START` | 可选 `f0=pass_count` |
| `48/49` | `YAW_TUNE_STOP/GET_STATUS` | 无 |
| `50/51` | `CLIMB_DISABLE/ENABLE` | 无 |
| `52` | `CLIMB_SET_CTRL` | `f0=enable, f1=step, f2=auto` |
| `53/54` | `CLIMB_UP_STEP/AUTO` | 无 |
| `55/56` | `CLIMB_STOP/GET_STATUS` | 无 |
| `57` | `CLIMB_TEST_ACTION` | `f0=action_id` |
| `58/59` | `CLIMB_DOWN_STEP/AUTO` | 无 |
| `5A/5B` | `CLIMB_UP_GATE/DOWN_GATE` | 无，只执行激光门控 |
| `5C/5D` | `CLIMB_UP_AUTO_PAUSE/DOWN_AUTO_PAUSE` | 无，运行到中断点暂停 |
| `5E` | `CLIMB_AUTO_RESUME` | 无 |
| `90` | `ARM_IK_RESULT` | 固件异步回包 |

名称、别名和快捷命令以 `PC_USB_Serial_Tool/serial_tool/commands.py` 为准。

## 4. 上下台阶状态机

四根立杆顺序为前右、后右、后左、前左。位置零点是上电接地点；正值向下伸出，负值向上收回，允许范围 `-30..260 mm`。前部小驱动轮在 FDCAN1，后部小驱动轮在 FDCAN2；正驱动位移使整车向前。

公共状态：`0=IDLE`、`22=DONE`、`23=ERROR`、`24=PREPARE_ALL_LEGS_MINUS_30`、`25=UP_PREPARE_CHASSIS_FORWARD_30`、`26=UP_LASER_APPROACH_X_LT_35`、`27=DOWN_LASER_APPROACH_H_GT_65`、`28=DOWN_PREPARE_CHASSIS_FORWARD_5`。

自动上台阶先沿 `+X` 以 0.08 m/s 前进，等待 `x_pos` 通道满足 `0 <= value < 35 mm`，再将四杆移到 -30 mm，并沿 `+X` 前进 30 mm。自动下台阶先沿 `+X` 以 0.08 m/s 前进，等待高度通道超过 65 mm，再沿 `+X` 前进 5 mm，之后将四杆移到 -30 mm。两种门控都在连续无效 1000 ms 或总等待超过 8000 ms 时进入错误状态。

正式上台阶有 14 步，下台阶有 15 步。具体目标、顺序和中断点分别见 `CLIMB_ACTION_SEQUENCE.yaml`、`DOWNSTAIRS_ACTION_SEQUENCE.yaml`；这两个文件与 `Applications/R2_user/Src/R2_climb.c` 同步。

## 5. 状态回包

状态回包继续使用相同 USB 帧格式，CMD 与请求命令相同：

| CMD | DATA 长度 | 内容 |
|---:|---:|---|
| `06` | 8 | 系统使能、任务、底盘电机在线、USB 超时 |
| `16` | 112 | 底盘模式、速度、里程计、限制、位置误差、INS、目标值 |
| `26` | 64 | IK、目标/实际关节、请求坐标、安全状态 |
| `36` | 32 | 工具选择、目标/实际角度、执行与错误状态 |
| `46` | 240 | 整车综合状态；`DATA[0]=4` |
| `49` | 64 | yaw 整定状态、阶段、轨迹和评分 |
| `56` | 68 | 台阶状态、四杆、前后驱动轮、到位掩码 |

所有多字节整数和浮点数为小端。精确偏移由固件 `Applications/Task/Src/PC_TX_Task.c` 定义，上位机 `PC_USB_Serial_Tool/serial_tool/status.py` 按同一布局解码。协议 v4 中所有综合状态的 `nav.x/vx`、`chassis.odom_x/target_vx/target_dx` 都指向前轴；对应 Y 字段指向左轴。激光 `x_pos/y_pos` 保留通道名，不是有符号坐标。

综合状态首部的主要字段为：

```text
[0] protocol_version = 4
[1] active_source: 0 USART, 1 USB, 2 NONE
[2] enable_flags
[3] executing_flags
[4] error_flags
[5] online_flags
```

旧上位机若只按长度和固定偏移解析，数据布局保持不变，但必须按 v4 的新坐标语义解释 X/Y。支持 v4 的上位机会在解析结果中输出 `coordinate_frame.name = "FLU"`。

## 6. USART 遥控帧

```text
A5 DATA[40] CHECKSUM 5A
```

校验和为 40 字节 DATA 的 8 位累加和。`DATA[0..7]` 是模式 one-hot，`[8..15]` 是机构控制位，`[16..27]` 是底盘三个小端 float，`[28..39]` 是机械臂三个小端 float。底盘三个值按当前模式解释为 `vx,vy,vw` 或 `dx,dy,dyaw`，轴方向与 USB 完全一致；机械臂是基座系 `x,y,z`。

## 7. 兼容迁移

旧底盘数据若采用 X 右、Y 前，转换后再发送：

```text
new_x = old_y
new_y = -old_x
new_yaw = old_yaw
```

综合状态协议版本从 3 升为 4，用于阻止上位机静默误解相同偏移处的 X/Y 字段。机械臂有效控制接口此前已经是 X 前、Y 左、Z 上，不对机械臂目标重复旋转。
