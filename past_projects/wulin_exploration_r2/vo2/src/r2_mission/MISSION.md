# 导航、越障与抓取任务

`r2_mission` 提供 `TraverseAndPick` 动作，把已有 Nav2、下位机上下台阶流程和
视觉抓取连接起来。任务阶段为：

```text
导航到台阶入口（可选）
→ 上/下台阶（可选）
→ 导航到抓取区（可选）
→ 等待速度租约结束并确认底盘停止
→ 等待稳定的实测目标
→ 选择并复位夹爪/吸盘
→ 预抓取
→ 接近目标
→ 夹取或吸取
→ 撤回机械臂
```

## 前置条件

- Nav2 已加载地图、定位正常，并提供 `/navigate_to_pose` 动作服务。
- TF 链包含 `map → odom → base_link`。串口桥默认发布
  `odom → base_link`；定位系统通常发布 `map → odom`。
- Nav2 控制器向 `/cmd_vel` 输出速度，串口桥使用
  `cmd_vel_enabled:=true`。
- 串口桥取得 USB 控制权，且 `vision_control_enabled:=false`，由任务节点独占
  机械臂动作。
- 相机内参、目标尺寸和 `camera → arm_base_link` 外参已经标定。
- 地图目标点应让机器人对准台阶入口，并让抓取目标落入机械臂真实工作空间。

## 启动

```bash
source install/setup.bash
ros2 run r2_mission mission_server
```

任务节点会等待 Nav2 和 `/robot/command`，启动顺序没有要求。

## 动作目标示例

下面示例先导航到地图中的上台阶入口，执行完整上台阶流程，再导航到抓取区，
用吸盘抓取目标：

```bash
ros2 action send_goal --feedback \
  /traverse_and_pick msg_interface/action/TraverseAndPick \
  "{navigate_to_terrain: true,
    terrain_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}},
    terrain_action: up,
    navigate_to_pickup: true,
    pickup_pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}, orientation: {w: 1.0}}},
    tool: suction,
    navigation_timeout_s: 180.0,
    target_timeout_s: 15.0,
    stable_samples: 5,
    stable_tolerance_m: 0.02,
    pregrasp_backoff_m: 0.08,
    pregrasp_lift_m: 0.05,
    grasp_forward_offset_m: 0.0,
    retreat_backoff_m: 0.10,
    retreat_lift_m: 0.08}"
```

`terrain_action` 可取 `none`、`up` 或 `down`，`tool` 可取 `gripper` 或
`suction`。地图中没有越障点时可将 `navigate_to_terrain` 设为 `false`，并将
`terrain_action` 设为 `none`。

超时、Nav2 失败、下位机拒绝、IK 不可达或客户端取消任务时，任务节点会取消
当前导航并请求整车停止。动作成功表示机械臂到位并且夹爪/吸盘输出状态得到
下位机确认；现有硬件没有抓取检测传感器，因此不能确认物体仍被夹住或吸住。
每次 Nav2 导航结束后，任务节点还会等待最后的 `/cmd_vel` 租约结束，并通过
下位机反馈确认底盘停止，避免速度流与爬阶状态机争用底盘。

当前越障入口由地图任务点指定。下位机在越障流程内部使用高度和距离激光完成
接近及固定机构序列，但现有传感信息不足以可靠识别任意未知地形类型。
