# Robot USB Serial Tool

这是一个独立的上位机串口工具工程，只新增在 `PC_USB_Serial_Tool` 文件夹内，不修改现有 STM32 工程。

功能：

- 读取 USB CDC / 串口数据，并按当前固件协议流式解帧。
- 校验并解析下位机状态回包：System、Chassis、Arm、Tool、Robot、Climb、Arm IK result。
- 按命令名或原始 hex 发送数据给下位机。
- 支持交互式 shell，边读边发，适合 VSCode 调试。
- 可生成 USART 遥控帧：`A5 + 40 DATA + checksum + 5A`。

所有底盘和机械臂命令采用 X 前、Y 左、Z 上、yaw 逆时针为正的统一坐标系。`vel/pos` 的第一个参数是前向 X，第二个参数是左向 Y；综合状态协议版本为 4。

## 快速开始

在 VSCode 中打开本文件夹：

```powershell
cd F:\spareE\Vinci_Robocon_2026\26_RC_Projects\26_RC_02\PC_USB_Serial_Tool
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

列出串口：

```powershell
.\.venv\Scripts\python -m serial_tool ports
```

进入交互 shell：

```powershell
.\.venv\Scripts\python -m serial_tool shell --port COM7 --baud 115200
```

USB CDC 虚拟串口通常不真正依赖波特率，但 Windows/pyserial 仍然需要传一个值，默认 `115200` 即可。

## 常用 shell 命令

进入 `shell` 后可直接输入：

```text
usb                         # 切换到 USB 控制源
usart                       # 切换到 USART 控制源
enable                      # 系统使能
disable                     # 系统失能
stop                        # 系统停止
status                      # 查询整车综合状态
tune_start 1                # 切到 USB、使能并开始 yaw 自动整定
tune_status                 # 查询 yaw 自动整定状态
tune_stop                   # 停止 yaw 自动整定并查询状态
climb_up_auto               # 切到 USB、使能上台阶并自动运行
climb_step                  # 切到 USB、使能上台阶并步进一步
climb_down_auto             # 切到 USB、使能下台阶并自动运行
climb_downstairs_step       # 切到 USB、使能下台阶并步进一步
climb_wait 20               # 轮询 CLIMB_GET_STATUS，直到就绪或完成
climb_step_wait 40          # 等当前上台阶动作完成，再步进并等待这一步完成
climb_up_auto_wait 180      # 发送自动上台阶，并等待最终 DONE
climb_downstairs_step_wait 40 # 等当前下台阶动作完成，再步进并等待这一步完成
climb_down_auto_wait 180    # 发送自动下台阶，并等待最终 DONE
climb_wait_then CLIMB_UP_STEP # 等当前上/下台阶动作完成，再发送一条 USB 命令
climb_tests                 # 列出上台阶独立调试动作 ID
climb_test CHASSIS_FORWARD_100      # 单独发送底盘麦轮前进 100mm，不等待完成
climb_test_wait FRONT_UP_10 10      # 发送前两根立杆上升 10mm，并等待完成，超时 10s
flow_start upstairs_v1      # 开始记录一套上台阶调试流程
flow_confirm lift front     # 询问是否保存刚才发送的 climb_test/climb_test_wait 动作
flow_save lift front        # 不询问，直接保存刚才发送的动作
flow_show                   # 显示已记录的流程步骤
flow_export upstairs_v1.json # 导出 JSON，后续用于生成状态机
flow_recover                # 从 .climb_flow_autosave.json 恢复误触 Ctrl+C 前的流程
send CHS_SET_MODE 3
send CHS_SET_VEL 0.4 0 0 0
send YAW_TUNE_START 1
query YAW_TUNE
send ARM_SET_TARGET 200 0 180 0
send TOOL_SET_MODE 0
send TOOL_SET_STATE 0 1
query CLIMB_GET_STATUS
raw A5 5A 00 56 C5 82 FF
commands
exit
```

`COM26` can run directly like this:

```powershell
.\.venv\Scripts\python usb_cmd_tool.py shell --port COM26 --baud 115200
```

More USB command examples are in `USB_COMMAND_TOOL.md`.
All mechanism shortcut commands are listed in `ALL_MECHANISM_COMMANDS.md`.

## 上台阶流程积木命令

这些命令不会改原来的完整上台阶状态机，只是单独执行一个 `CLIMB_TEST_ACTION`，方便你调出参数后用 `flow_confirm` 确认保存为流程步骤。
带 `timeout_s` 时会等待 `CLIMB_GET_STATUS.state_done == 1`；不带参数时只发送动作并查询一次状态。

```text
climb_all_legs_220 [timeout_s]        # 四根立柱统一到 220mm
climb_all_legs_up_10 [timeout_s]      # 四根立柱基于当前位置抬升底盘 10mm
climb_all_legs_down_10 [timeout_s]    # 四根立柱基于当前位置下降底盘 10mm

climb_drive_forward_30 [timeout_s]    # 后驱动轮前进 30mm
climb_drive_forward_10 [timeout_s]    # 后驱动轮前进 10mm
climb_drive_backward_10 [timeout_s]   # 后驱动轮后退 10mm
climb_drive_backward_30 [timeout_s]   # 后驱动轮后退 30mm
climb_drive_forward_500 [timeout_s]   # 后驱动轮前进 500mm
climb_drive_backward_500 [timeout_s]  # 后驱动轮后退 500mm

climb_front_zero [timeout_s]          # 前两根立杆回到 0 位
climb_front_220 [timeout_s]           # 前两根立杆目标到 220mm
climb_front_minus_30 [timeout_s]      # 前两根立杆目标到 -30mm
climb_front_up_10 [timeout_s]         # 前两根立杆基于当前位置上升 10mm
climb_front_down_10 [timeout_s]       # 前两根立杆基于当前位置下降 10mm

climb_chassis_forward_100 [timeout_s] # 底盘麦轮前进 100mm
climb_chassis_backward_100 [timeout_s] # 底盘麦轮后退 100mm
climb_chassis_forward_50 [timeout_s]  # 底盘麦轮前进 50mm
climb_chassis_backward_50 [timeout_s] # 底盘麦轮后退 50mm
climb_chassis_forward_300 [timeout_s] # 底盘麦轮前进 300mm
climb_chassis_backward_300 [timeout_s] # 底盘麦轮后退 300mm

climb_rear_zero [timeout_s]           # 后两根立杆回到 0 位
climb_rear_220 [timeout_s]            # 后两根立杆目标到 220mm
climb_rear_minus_30 [timeout_s]       # 后两根立杆目标到 -30mm
climb_rear_up_10 [timeout_s]          # 后两根立杆基于当前位置上升 10mm
climb_rear_down_10 [timeout_s]        # 后两根立杆基于当前位置下降 10mm

climb_all_legs_zero [timeout_s]       # 四根立柱统一回到 0 位
```

也可以用动作名或编号：

```text
climb_test_wait ALL_LEGS_220 10
climb_test_wait 16 10
```

调试并记录流程的典型用法：

```text
flow_start upstairs_v1
climb_all_legs_220 10
flow_confirm 四根立柱到 220
climb_drive_forward_30 10
flow_confirm 后驱动轮前进 30
flow_show
flow_export upstairs_v1.json
```

每次 `climb_test*` 生成候选动作、`flow_confirm`/`flow_save` 保存动作、`flow_add` 手动补动作后，工具都会自动写入 `.climb_flow_autosave.json`。如果误触 `Ctrl+C` 退出，重新进入 shell 后执行：

```text
flow_recover
flow_show
flow_export downstairs_v1.json
```

收到下位机回包后，工具会打印 JSON 形式的解析结果和原始帧 hex。

## 命令行发送

只打包不发送：

```powershell
.\.venv\Scripts\python -m serial_tool pack SYS_ENABLE
.\.venv\Scripts\python -m serial_tool pack CHS_SET_VEL 0.4 0 0 0
```

发送单条命令并等待 0.5 秒回包：

```powershell
.\.venv\Scripts\python -m serial_tool send --port COM7 ROBOT_GET_STATUS --wait 0.5
```

持续监听：

```powershell
.\.venv\Scripts\python -m serial_tool monitor --port COM7
```

周期轮询状态：

```powershell
.\.venv\Scripts\python -m serial_tool poll --port COM7 ROBOT_GET_STATUS --rate 20
```

生成 USART 遥控帧：

```powershell
.\.venv\Scripts\python -m serial_tool remote-pack --mode 3 --source-usb --chassis 0.2 0 0 --arm-target 0 0 180
```

USART 遥控帧工具字段为 `byte10..12`：`tool_flag / clampuse_flag / chuckuse_flag`，分别控制旋转位置、夹爪开闭、吸盘开闭。上台阶字段顺延到 `byte13..15`：`climb_enable / climb_step / climb_auto`。步进和自动都按 `0->1` 上升沿触发；自动启动后仍要持续发送 `--source-usart --climb-enable` 保活帧，否则固件 `300ms` USART 看门狗会停止 USART 侧控制器。

## 协议来源

本工具按仓库内当前实现对齐：

- USB 帧格式：`BSP/Src/bsp_usb.c`
- USB 命令路由：`Components/Algorithm/Src/Data_Analysis.c`
- 状态回包：`Applications/Task/Src/PC_TX_Task.c`
- USART 遥控帧：`Components/Algorithm/Src/CRC.c`

特别注意：当前固件的底盘状态回包 `CHS_GET_STATUS` 实际为 `LEN=88`，工具按源码中的真实长度解析。
