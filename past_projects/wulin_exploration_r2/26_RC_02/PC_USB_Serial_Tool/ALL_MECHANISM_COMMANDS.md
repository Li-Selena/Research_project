# 全机构 USB 指令清单

运行交互工具：

```powershell
cd F:\spareE\Vinci_Robocon_2026\26_RC_Projects\26_RC_02\PC_USB_Serial_Tool # 进入 USB 串口工具目录
.\.venv\Scripts\python usb_cmd_tool.py shell --port COM26 --baud 115200     # 打开 COM26 交互式串口命令行
```

进入后建议先执行：

```text
usb             # 切换到 USB 控制源
enable          # 使能 USB 控制的各机构
status          # 查询整车综合状态
```

## 系统

```text
usb             # 切换到 USB 控制源
usart           # 切回 USART 控制源
enable          # 系统使能，等价 SYS_ENABLE
disable         # 系统失能，等价 SYS_DISABLE
stop            # 系统停止，等价 SYS_STOP
status          # 查询整车综合状态，等价 ROBOT_GET_STATUS
query SYS       # 查询系统状态，等价 SYS_GET_STATUS
```

## 底盘

统一坐标：`+X` 向前，`+Y` 向左，`+Z` 向上，`+yaw` 逆时针。

```text
chs_enable      # 底盘使能
chs_disable     # 底盘失能
chs_mode 3      # 设置底盘模式，3=WORLD_VEL
vel 0.2 0 0 0   # 发送速度指令 vx=0.2, vy=0, vw=0, 锁 yaw=0
pos 0.5 0 0     # 发送位置指令 dx=0.5, dy=0, dyaw=0
chs_stop        # 底盘停止
chs_status      # 查询底盘状态
```

底盘模式：

```text
0 ROBOT_NO_YAW_VEL   # 机器人坐标系速度模式，不给 yaw 速度
1 ROBOT_VEL          # 机器人坐标系速度模式，带 yaw 速度
2 WORLD_NO_YAW_VEL   # 世界坐标系速度模式，不给 yaw 速度
3 WORLD_VEL          # 世界坐标系速度模式，带 yaw 速度
4 ROBOT_NO_YAW_POS   # 机器人坐标系位置模式，不给 yaw 目标
5 ROBOT_POS          # 机器人坐标系位置模式，带 yaw 目标
6 WORLD_NO_YAW_POS   # 世界坐标系位置模式，不给 yaw 目标
7 WORLD_POS          # 世界坐标系位置模式，带 yaw 目标
```

底盘速度命令：

```text
vel VX VY YAW_DATA           # ROBOT_NO_YAW: robot yaw deg；WORLD_NO_YAW: world yaw deg；其它速度模式: vw_rad_s
```

底盘位置命令：

```text
pos DX DY YAW_DATA          # ROBOT_NO_YAW: robot yaw deg；WORLD_NO_YAW: world yaw deg；其它位置模式: dyaw_rad
```

## 机械臂

```text
arm_enable          # 机械臂使能
arm_target 200 0 180  # 设置机械臂目标点 x=200, y=0, z=180
arm_status          # 查询机械臂状态
arm_stop            # 机械臂停止并保持
arm_disable         # 机械臂失能
```

机械臂目标点单位是 mm：

```text
arm_target X Y Z    # X/Y/Z 单位 mm
```

当前固件限制：

```text
X: -500..500        # X 方向目标范围，单位 mm
Y: -500..500        # Y 方向目标范围，单位 mm
Z: 50..500          # Z 方向目标范围，单位 mm
```

## 工具机构

```text
tool_enable         # 工具机构使能
tool_chuck          # 旋转到吸盘位/Chuck position
tool_clamp          # 旋转到夹爪位/Clamp position
tool_open           # 旧兼容命令：当前位置执行器打开
tool_close          # 旧兼容命令：当前位置执行器闭合
tool_state 0 1      # 夹爪打开，不改变旋转位置
tool_state 0 0      # 夹爪关闭，不改变旋转位置
tool_state 1 1      # 吸盘打开，不改变旋转位置
tool_state 1 0      # 吸盘关闭，不改变旋转位置
tool_status         # 查询工具机构状态
tool_stop           # 工具机构停止
tool_disable        # 工具机构失能
```

也可以用组合快捷命令：

```text
chuck_open          # 吸盘打开，不改变旋转位置
chuck_close         # 吸盘关闭，不改变旋转位置
clamp_open          # 夹爪打开，不改变旋转位置
clamp_close         # 夹爪关闭，不改变旋转位置
```

## 上/下台阶机构

```text
climb_enable                    # 上/下台阶机构使能
climb_step                      # 手动推进上台阶状态机一步
climb_step_wait 40              # 等当前动作完成，发送上台阶 step，再等待本步完成，超时 40s
climb_up_auto                   # 启动/继续上台阶自动流程
climb_up_auto_wait 180          # 启动上台阶自动流程并等待最终 DONE，超时 180s
climb_upstairs_step             # 上台阶 step 的明确别名
climb_upstairs_step_wait 40     # 上台阶等待后步进
climb_upstairs_auto             # 上台阶 auto 的明确别名
climb_upstairs_auto_wait 180    # 上台阶自动并等待 DONE
climb_downstairs_step           # 手动推进下台阶状态机一步
climb_downstairs_step_wait 40   # 等当前动作完成，发送下台阶 step，再等待本步完成，超时 40s
climb_down_auto                 # 启动/继续下台阶自动流程
climb_down_auto_wait 180        # 启动下台阶自动流程并等待最终 DONE，超时 180s
# Auto laser gate: upstairs auto starts forward immediately; firmware drives forward until valid X reaches 0 <= x < 35mm. Transient X=-1 is tolerated; continuous invalid X for 1000ms times out.
# Auto laser gate: downstairs auto starts forward along +X; h > 65mm detects the drop jump, then chassis moves forward another 5mm before the normal flow. H=-1 is tolerated briefly; 1000ms invalid or 8000ms without trigger times out.
climb_wait 40                   # 仅等待当前 climb 动作完成，超时 40s
climb_wait_then CLIMB_UP_STEP   # 等当前动作完成后发送指定 USB 命令
climb_status                    # 查询上/下台阶状态，返回 flow=UPSTAIRS/DOWNSTAIRS
climb_stop                      # 停止上/下台阶机构
climb_disable                   # 上/下台阶机构失能
climb_tests                     # 列出所有 climb 测试动作编号
climb_test ACTION               # 发送指定测试动作，不等待完成
climb_test_wait ACTION [timeout_s]  # 发送指定测试动作并等待完成
climb_all_legs_220 [timeout_s]  # 四条腿目标到 220mm；带超时时间则等待完成
climb_all_legs_up_10 [timeout_s] # 四条腿当前位置上升 10mm；带超时时间则等待完成
climb_all_legs_down_10 [timeout_s] # 四条腿当前位置下降 10mm；带超时时间则等待完成
climb_drive_forward_30 [timeout_s] # 后驱动轮前进 30mm；带超时时间则等待完成
climb_drive_forward_10 [timeout_s] # 后驱动轮前进 10mm；带超时时间则等待完成
climb_drive_backward_10 [timeout_s] # 后驱动轮后退 10mm；带超时时间则等待完成
climb_drive_backward_30 [timeout_s] # 后驱动轮后退 30mm；带超时时间则等待完成
climb_drive_forward_500 [timeout_s] # 后驱动轮前进 500mm；带超时时间则等待完成
climb_drive_backward_500 [timeout_s] # 后驱动轮后退 500mm；带超时时间则等待完成
climb_front_zero [timeout_s]    # 前腿回零；带超时时间则等待完成
climb_front_220 [timeout_s]     # 前两根立杆目标到 220mm；带超时时间则等待完成
climb_front_minus_30 [timeout_s] # 前两根立杆目标到 -30mm；带超时时间则等待完成
climb_front_up_10 [timeout_s]   # 前腿上升 10mm；带超时时间则等待完成
climb_front_down_10 [timeout_s] # 前腿下降 10mm；带超时时间则等待完成
climb_chassis_forward_100 [timeout_s] # 底盘前进 100mm；带超时时间则等待完成
climb_chassis_backward_100 [timeout_s] # 底盘后退 100mm；带超时时间则等待完成
climb_chassis_forward_50 [timeout_s] # 底盘前进 50mm；带超时时间则等待完成
climb_chassis_backward_50 [timeout_s] # 底盘后退 50mm；带超时时间则等待完成
climb_chassis_forward_300 [timeout_s] # 底盘前进 300mm；带超时时间则等待完成
climb_chassis_backward_300 [timeout_s] # 底盘后退 300mm；带超时时间则等待完成
climb_rear_zero [timeout_s]     # 后腿回零；带超时时间则等待完成
climb_rear_220 [timeout_s]      # 后两根立杆目标到 220mm；带超时时间则等待完成
climb_rear_minus_30 [timeout_s] # 后两根立杆目标到 -30mm；带超时时间则等待完成
climb_rear_up_10 [timeout_s]    # 后腿上升 10mm；带超时时间则等待完成
climb_rear_down_10 [timeout_s]  # 后腿下降 10mm；带超时时间则等待完成
climb_all_legs_zero [timeout_s] # 四条腿统一回到 0mm；带超时时间则等待完成
```

模拟旧 USART 三个控制字节：

```text
climb_ctrl ENABLE STEP AUTO     # 模拟旧 USART 的 enable/step/auto 三个控制位
```

例如：

```text
climb_ctrl 1 0 0                # 使能 climb，不触发 step/auto
climb_ctrl 1 1 0                # 使能 climb，并触发一次 step
climb_ctrl 1 0 1                # 使能 climb，并触发/继续 auto
```

## 自动调参

```text
tune_start 1        # 启动 yaw 自动调参，执行 1 轮
tune_status         # 查询 yaw 自动调参状态
tune_stop           # 停止 yaw 自动调参
```

底层命令等价写法：

```text
send YAW_TUNE_START 1   # 原生命令：启动 yaw 自动调参
query YAW_TUNE          # 原生命令：查询 yaw 自动调参状态
send YAW_TUNE_STOP      # 原生命令：停止 yaw 自动调参
```

## 原生命令名

如果不想用快捷命令，可以直接用 `send`：

```text
send SYS_ENABLE             # 系统使能
send CHS_SET_MODE 3         # 设置底盘模式为 WORLD_VEL
send CHS_SET_VEL 0.2 0 0 0  # 发送底盘速度指令
send ARM_SET_TARGET 200 0 180  # 设置机械臂目标点
send TOOL_SET_MODE 0        # 旋转到夹爪位/Clamp position
send TOOL_SET_STATE 0 1     # 夹爪打开，不改变旋转位置
send CLIMB_UP_STEP          # 上台阶手动推进一步
send CLIMB_UP_AUTO          # 上台阶自动执行/继续
send CLIMB_DOWN_STEP        # 下台阶手动推进一步
send CLIMB_DOWN_AUTO        # 下台阶自动执行/继续
send CLIMB_TEST_ACTION 10   # 执行 climb 测试动作 10：底盘前进 100mm
send YAW_TUNE_START 1       # 启动 yaw 自动调参
send YAW_TUNE_GET_STATUS    # 查询 yaw 自动调参状态
send YAW_TUNE_STOP          # 停止 yaw 自动调参
```

查看完整底层 USB 命令号和打包结果：

```text
commands                    # 查看工具内置的完整 USB 命令表
pack YAW_TUNE_GET_STATUS    # 打印该命令的完整 USB 帧
pack CLIMB_DOWN_STEP        # 打印下台阶 step 的完整 USB 帧
```
