from pathlib import Path
import math
import re


ROOT = Path(__file__).resolve().parents[1]


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8", errors="ignore")


def require(cond, message):
    if not cond:
        raise AssertionError(message)


def function_body(source, name):
    match = re.search(
        r"\b(?:void|static\s+(?:void|uint8_t))\s+" + re.escape(name) +
        r"\s*\([^)]*\)\s*\{",
        source,
    )
    require(match is not None, f"missing function {name}")
    start = match.end()
    depth = 1
    i = start
    while i < len(source) and depth:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
        i += 1
    require(depth == 0, f"could not parse function {name}")
    return source[start : i - 1]


def check_r2_mode_math():
    world_modes = {2, 3, 6, 7}
    for mode in range(8):
        is_vel = mode <= 3
        is_pos = mode >= 4
        is_world = mode in world_modes
        is_no_yaw = (mode & 1) == 0

        require(is_vel != is_pos, f"mode {mode}: vel/pos overlap")
        require(is_no_yaw == (mode in {0, 2, 4, 6}), f"mode {mode}: no-yaw classification")
        require(is_world == (mode in world_modes), f"mode {mode}: world classification")

    yaw = math.pi / 2.0
    wx, wy = 1.0, 0.0
    rx = wx * math.cos(yaw) + wy * math.sin(yaw)
    ry = -wx * math.sin(yaw) + wy * math.cos(yaw)
    require(abs(rx) < 1e-6 and abs(ry + 1.0) < 1e-6, "world_to_robot 90deg transform")


def check_coordinate_contract():
    frame = read("Components/Algorithm/Inc/robot_frame.h")
    require("+X forward, +Y left, +Z up" in frame, "missing canonical FLU declaration")
    require("ROBOT_STATUS_PROTOCOL_VERSION 4U" in frame, "status protocol must declare FLU as v4")
    require("*rx = result_x" in frame and "float result_x = sy" in frame,
            "IMU X must map from sensor Y")
    require("float result_y = -sx" in frame, "IMU Y must map from negative sensor X")

    mecanum = read("Components/Algorithm/Src/mecanum_classic.c")
    for formula in [
        "chassis->vx - chassis->vy - k",
        "chassis->vx + chassis->vy + k",
        "chassis->vx + chassis->vy - k",
        "chassis->vx - chassis->vy + k",
    ]:
        require(formula in mecanum, f"missing FLU mecanum term: {formula}")

    climb = read("Applications/R2_user/Src/R2_climb.c")
    require("R2_Move_SetVel(move_ctrl,\n                               R2_CLIMB_UP_LASER_APPROACH_SPEED_MPS,\n                               0.0f" in climb,
            "upstairs approach must command +X")
    require("R2_Move_SetDist(move_ctrl, chassis_delta_m, 0.0f, 0.0f)" in climb,
            "climb chassis distance must use X")

    pc_tx = read("Applications/Task/Src/PC_TX_Task.c")
    require("buf[0] = ROBOT_STATUS_PROTOCOL_VERSION" in pc_tx,
            "robot status must publish shared protocol version")

    status = read("PC_USB_Serial_Tool/serial_tool/status.py")
    require('"name": "FLU"' in status and "protocol_version >= ROBOT_STATUS_PROTOCOL_VERSION" in status,
            "host decoder must expose protocol-v4 coordinates")


def check_climb_sequence_docs():
    source = read("Applications/R2_user/Src/R2_climb.c")

    def c_states(array_name):
        match = re.search(
            r"static const R2_ClimbMainStep_t\s+" + re.escape(array_name) +
            r"\[[^]]+\]\s*=\s*\{(.*?)\n\};",
            source,
            re.S,
        )
        require(match is not None, f"missing climb array {array_name}")
        return re.findall(r'\{"([A-Z0-9_]+)"', match.group(1))

    def yaml_states(path):
        return re.findall(
            r"^\s+- \{index: \d+, state: ([A-Z0-9_]+),",
            read(path),
            re.M,
        )

    require(c_states("s_main_steps") == yaml_states("CLIMB_ACTION_SEQUENCE.yaml"),
            "upstairs YAML is not synchronized with R2_climb.c")
    require(c_states("s_downstairs_steps") == yaml_states("DOWNSTAIRS_ACTION_SEQUENCE.yaml"),
            "downstairs YAML is not synchronized with R2_climb.c")


def check_static_invariants():
    control = read("Applications/Task/Src/Control_Task.c")
    isr_body = function_body(control, "control_tim1mscallback")
    for forbidden in ["R2_Move_Update", "R2_Move_UpdateOdom", "Mecanum_Calc", "cosf", "sinf"]:
        require(forbidden not in isr_body, f"TIM3 ISR still contains heavy call: {forbidden}")
    require("vTaskNotifyGiveFromISR" in isr_body, "TIM3 ISR must notify control task")

    step_body = function_body(control, "R2_Control_1msStep")
    yaw_call = step_body.find("R2_Move_UpdateYaw(&")
    odom_call = step_body.find("R2_Move_UpdateOdom(&")
    require(yaw_call >= 0 and odom_call >= 0 and yaw_call < odom_call,
            "R2 yaw must be updated before odom integration")

    data = read("Components/Algorithm/Src/Data_Analysis.c")
    data_body = function_body(data, "Data_Analysis")
    require("USB_Read4Floats(d, f)" not in data_body, "Data_Analysis must not read payload directly")
    require(data_body.count("USB_Read4FloatsChecked") >= 5, "expected guarded USB payload reads")
    checked_body = function_body(data, "USB_Read4FloatsChecked")
    require(checked_body.count("isfinite(") == 4,
            "all four USB float parameters must reject NaN/Inf")
    require("Control_SetSource" in data_body, "USB source switch must use unified source setter")

    pc_rx = read("Applications/Task/Src/PC_RX_Task.c")
    require("static uint8_t cdc_read_buf[USB_FRAME_BUF_SIZE]" in pc_rx,
            "CDC reads need storage separate from the streaming frame parser")
    require("CDC_App_Read(cdc_read_buf" in pc_rx and
            "Receive(cdc_read_buf[i])" in pc_rx,
            "CDC chunks must be forwarded from the independent read buffer")

    fdcan = read("BSP/Src/bsp_fdcan.c")
    for name in ["FDCAN1_Filter_Init", "FDCAN2_Filter_Init", "FDCAN3_Filter_Init"]:
        require("FDCAN_Start" not in function_body(fdcan, name), f"{name} should only configure filters")
    require("FDCAN_Motor_Start_All" in fdcan, "missing unified FDCAN start function")

    can = read("Applications/Task/Src/CAN_Task.c")
    require("pid_call_3( tool_target, 4)" in can, "FDCAN3 motor4 must use active tool target")
    require("osDelay(5000)" in can, "CAN startup delay must yield to scheduler")

    tools = read("Components/Device/Src/arm_tools.c")
    for symbol in ["clamp_usart", "clamp_usb", "chuck_usart", "chuck_usb"]:
        require(symbol in tools, f"missing independent tool object {symbol}")


def main():
    check_r2_mode_math()
    check_coordinate_contract()
    check_climb_sequence_docs()
    check_static_invariants()
    print("control plan verification passed")


if __name__ == "__main__":
    main()
