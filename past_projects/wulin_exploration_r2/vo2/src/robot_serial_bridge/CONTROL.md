# R2 上下位机控制接口

`robot_serial_bridge` 通过 USB CDC 协议 v4 与 STM32 双向通信。节点读取并校验
CRC 回包，发布整车状态和本地里程计，并在下位机反馈到位、停止或故障后发布
命令结果。

## 启动

单独启动串口桥：

```bash
source install/setup.bash
ros2 run robot_serial_bridge robot_serial_bridge --ros-args \
  -p send_enabled:=true \
  -p port:=/dev/ttyACM0 \
  -p claim_usb_source:=true \
  -p vision_control_enabled:=false
```

与视觉节点一起启动：

```bash
ros2 launch pnp_solve vo2.launch.py \
  calibration_file:=$PWD/src/pnp_solve/config/calibration.yaml \
  pnp_config_file:=$PWD/src/pnp_solve/config/pnp.yaml \
  serial_enabled:=true \
  serial_port:=/dev/ttyACM0 \
  claim_usb_source:=true
```

联合启动时，`vision_control_enabled` 默认为 `true`，有效的
`arm_base_link` 目标会控制机械臂。只做手动机构联调时应传入
`vision_control_enabled:=false`。

## ROS 接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/robot/command` | `msg_interface/srv/RobotCommand` | 提交离散或限时动作 |
| `/robot/command_result` | `msg_interface/msg/RobotCommandResult` | 动作运行、完成、失败、取消或暂停事件 |
| `/robot/status` | `std_msgs/msg/String` | JSON 格式整车与各机构反馈、通信新鲜度和活动请求 |
| `/robot/odom` | `nav_msgs/msg/Odometry` | 编码器平移和 IMU 朝向形成的本地里程计 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 可选的连续底盘速度输入 |
| `/pnp_result` | `msg_interface/msg/PnpResult` | 可选的视觉机械臂目标输入 |

服务返回 `accepted=true` 只表示请求通过上位机校验并已排队。动作结果必须以
`/robot/command_result` 为准：

```bash
ros2 topic echo /robot/command_result
ros2 topic echo /robot/status
ros2 topic echo /robot/odom
```

## 命令

坐标统一采用 X 前、Y 左、Z 上，yaw 从上方看逆时针为正。机械臂服务参数使用
毫米；底盘位置、线速度、角度和角速度分别使用米、米每秒、弧度和弧度每秒。

| `command` | `values` | 行为与完成条件 |
|---|---|---|
| `claim` | `[]` | 切换到 USB 控制，并等待控制源回包确认 |
| `release` | `[]` | 停止 USB 动作并切回 USART 控制 |
| `arm_move` | `[x_mm, y_mm, z_mm]` | 机械臂运动；IK 安全且编码器最大关节误差不超过 2°时完成 |
| `tool_select` | `[0]` 或 `[1]` | 旋转到夹爪或吸盘位置，并等待到位 |
| `gripper` | `[0]` 或 `[1]` | 夹爪闭合或张开，确认下位机输出状态 |
| `suction` | `[0]` 或 `[1]` | 吸盘关闭或开启，确认下位机输出状态 |
| `velocity` | `[vx, vy, wz]` | 按 `duration_s` 定速运动，结束后主动停止并等待确认 |
| `move_relative` | `[dx, dy, dyaw]` | 相对当前机器人坐标系运动，并等待里程计到位 |
| `move_to` | `[x, y, yaw]` | 移动到当前本地 `odom` 坐标系中的目标位姿 |
| `climb_up` / `climb_down` | `[]` | 执行完整上台阶或下台阶自动序列 |
| `climb_step_up` / `climb_step_down` | `[]` | 执行一个上行或下行步骤 |
| `climb_up_pause` / `climb_down_pause` | `[]` | 运行到预设暂停点 |
| `climb_gate_up` / `climb_gate_down` | `[]` | 执行门控步骤 |
| `climb_resume` | `[]` | 从预设暂停点继续 |
| `climb_test` | `[1..38]` | 执行一个机构测试动作 |
| `stop` | `[]` | 停止整车 USB 动作，机械臂进入当前位置保持 |
| `arm_stop` / `tool_stop` | `[]` | 停止对应机构 |
| `chassis_stop` / `climb_stop` | `[]` | 同时停止爬阶机构和底盘，解除二者占用 |

`gripper_open`、`gripper_close`、`suction_on`、`suction_off`、
`select_gripper`、`select_suction` 是不带 `values` 的便捷别名。

## 调用示例

先取得 USB 控制权：

```bash
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: claim, values: [], duration_s: 0.0, timeout_s: 5.0}"
```

机械臂移动并开启吸盘：

```bash
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: arm_move, values: [350.0, 0.0, 400.0], duration_s: 0.0, timeout_s: 15.0}"
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: select_suction, values: [], duration_s: 0.0, timeout_s: 5.0}"
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: suction_on, values: [], duration_s: 0.0, timeout_s: 5.0}"
```

以 0.3 m/s 前进 2 秒：

```bash
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: velocity, values: [0.3, 0.0, 0.0], duration_s: 2.0, timeout_s: 6.0}"
```

移动到本地里程计坐标 `(1.0, 0.5, 0.0)`：

```bash
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: move_to, values: [1.0, 0.5, 0.0], duration_s: 0.0, timeout_s: 30.0}"
```

执行上台阶：

```bash
ros2 service call /robot/command msg_interface/srv/RobotCommand \
  "{command: climb_up, values: [], duration_s: 0.0, timeout_s: 180.0}"
```

## 连续速度控制

设置 `cmd_vel_enabled:=true` 后，节点接收 `/cmd_vel`。每条消息续租 0.2 秒，
输入中断后节点会发送明确的底盘停止命令。发布频率应至少为 10 Hz；桥内部按
40 ms 间隔续发，满足下位机 100 ms 速度看门狗。

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"
```

## 控制边界

- `move_to` 和 `/robot/odom` 使用下位机编码器加 IMU 的本地参考系，会累计漂移；
  它们不等同于地图定位、SLAM 或 Nav2 全局定位。
- 节点默认同时发布 `odom → base_link` TF，并为平面位姿和速度填写协方差；已有
  其他节点发布该 TF 时应设置 `publish_odom_tf:=false`，避免重复发布。
- 速度限制为 `|vx|, |vy| <= 2 m/s`、`|wz| <= 0.6283185 rad/s`。
- 机械臂目标先经上位机有限值和范围检查，再由下位机做真实工作空间与安全区
  判断。Z 不高于 250 mm 时，下位机要求 `|Y| <= 5 mm`。
- 爬阶动作执行时独占底盘；底盘动作未停止时也不能启动爬阶。
- `arm_stop` 和通信超时会让机械臂闭环保持当前位置，并不切断电机电源。
- 工具反馈能确认选择位置和输出状态，但当前硬件接口不能判断物体是否抓牢或
  吸牢。
