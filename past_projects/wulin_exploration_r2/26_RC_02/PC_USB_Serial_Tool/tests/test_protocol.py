import struct
import unittest

from serial_tool.commands import build_climb_test_shortcut_sequence, build_usb_command
from serial_tool.protocol import UsbStreamParser, bytes_to_hex, crc16_modbus, pack_usb_frame
from serial_tool.status import CLIMB_TEST_ACTIONS, decode_usb_frame
from serial_tool.usart_remote import build_remote_frame


class ProtocolTests(unittest.TestCase):
    def test_known_empty_frames(self):
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_DISABLE")), "A5 5A 00 00 FB 02 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_ENABLE")), "A5 5A 00 01 3B C3 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_STOP")), "A5 5A 00 05 F8 C2 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_GET_STATUS")), "A5 5A 00 06 F9 82 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("CLIMB_GET_STATUS")), "A5 5A 00 56 C5 82 FF")

    def test_known_4float_frame(self):
        frame = build_usb_command("SYS_SWITCH_SOURCE", [1.0])
        self.assertEqual(
            bytes_to_hex(frame),
            "A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF",
        )

    def test_stream_parser_split_frame(self):
        frame = pack_usb_frame(0x46)
        parser = UsbStreamParser()
        out = parser.feed(b"\x00\x11" + frame[:3])
        self.assertEqual(out, [])
        out = parser.feed(frame[3:] + b"\x99")
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].cmd, 0x46)
        self.assertEqual(out[0].payload, b"")

    def test_decode_robot_status_payload(self):
        payload = bytearray(96)
        payload[0] = 1
        payload[1] = 1
        payload[2] = 0b00000111
        payload[5] = 0b00011101
        struct.pack_into("<I", payload, 8, 12)
        struct.pack_into("<f", payload, 32, 1.25)
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["active_source_name"], "USB")
        self.assertTrue(decoded["payload"]["enable_flags"]["chassis"])
        self.assertEqual(decoded["payload"]["usb_rx"]["count"], 12)
        self.assertAlmostEqual(decoded["payload"]["nav"]["x_m"], 1.25)

    def test_decode_robot_status_v2_extension(self):
        payload = bytearray(160)
        payload[0] = 2
        payload[1] = 1
        payload[2] = 0b00001111
        payload[3] = 0b10110000
        payload[5] = 0b01000000
        payload[96] = 1
        payload[97] = 5
        payload[98] = 1
        payload[100] = 1
        payload[105] = 6
        payload[106] = 10
        payload[107] = 0b11
        struct.pack_into("<I", payload, 108, 2345)
        payload[116] = 0b111
        payload[117] = 0b101
        payload[119] = 1
        struct.pack_into("<I", payload, 124, 9876)
        struct.pack_into("<i", payload, 128, 123)
        struct.pack_into("<i", payload, 132, -1)
        struct.pack_into("<i", payload, 136, 456)
        payload[140] = 1
        payload[142] = 2
        payload[143] = 3
        struct.pack_into("<I", payload, 144, 4321)
        struct.pack_into("<f", payload, 152, -2.5)
        struct.pack_into("<f", payload, 156, 8.75)
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["protocol_version"], 2)
        self.assertTrue(decoded["enable_flags"]["climb"])
        self.assertTrue(decoded["executing_flags"]["climb_motor_active"])
        self.assertTrue(decoded["executing_flags"]["yaw_tune_running"])
        self.assertTrue(decoded["executing_flags"]["any"])
        self.assertEqual(decoded["climb"]["state_name"], "STEP_05_FRONT_MINUS_30")
        self.assertEqual(decoded["climb"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertTrue(decoded["climb"]["test_chassis_active"])
        self.assertEqual(decoded["laser"]["distance_mm"]["y_pos"], -1)
        self.assertEqual(decoded["laser"]["update_tick"], 9876)
        self.assertEqual(decoded["yaw_tune"]["state_name"], "RUNNING")
        self.assertEqual(decoded["yaw_tune"]["active_mode_name"], "WORLD_VEL")
        self.assertAlmostEqual(decoded["yaw_tune"]["score"], 8.75)

    def test_decode_robot_status_downstairs_flow(self):
        payload = bytearray(160)
        payload[0] = 2
        payload[96] = 1
        payload[97] = 4
        payload[107] = 0b100
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["climb"]["flow_name"], "DOWNSTAIRS")
        self.assertEqual(decoded["climb"]["state_name"], "DOWN_04_ALL_LEGS_UP_10")

    def test_decode_yaw_tune_status_payload(self):
        payload = bytearray(64)
        payload[0] = 1
        payload[1] = 2
        payload[2] = 9
        payload[3] = 0
        struct.pack_into("<I", payload, 4, 1234)
        struct.pack_into("<I", payload, 8, 456)
        struct.pack_into("<f", payload, 12, -1.5)
        struct.pack_into("<f", payload, 28, 3.25)
        struct.pack_into("<f", payload, 32, 1.1)
        struct.pack_into("<f", payload, 40, 0.8)
        payload[60] = 0
        payload[61] = 1
        payload[62] = 3
        payload[63] = 2
        frame = pack_usb_frame(0x49, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["cmd_name"], "YAW_TUNE_GET_STATUS")
        self.assertEqual(decoded["payload"]["state_name"], "RUNNING")
        self.assertEqual(decoded["payload"]["segment_index"], 2)
        self.assertAlmostEqual(decoded["payload"]["yaw_error_deg"], -1.5)
        self.assertAlmostEqual(decoded["payload"]["score"], 3.25)
        self.assertEqual(decoded["payload"]["active_mode_name"], "WORLD_VEL")
        self.assertEqual(decoded["payload"]["phase_name"], "RUN")

    def test_decode_extended_mechanism_status_payloads(self):
        chassis = bytearray(112)
        chassis[0] = 3
        chassis[1] = 1
        chassis[84] = 1
        chassis[86] = 0b11010011
        chassis[87] = 0b10000
        struct.pack_into("<f", chassis, 88, 0.4)
        struct.pack_into("<f", chassis, 100, 1.25)
        frame = pack_usb_frame(0x16, bytes(chassis))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["mode_name"], "WORLD_VEL")
        self.assertEqual(decoded["coordinate_frame"]["name"], "FLU")
        self.assertTrue(decoded["status_flags"]["moving"])
        self.assertTrue(decoded["error_flags"]["partial_motor_offline"])
        self.assertAlmostEqual(decoded["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["target_pos"]["dx"], 1.25)

        arm = bytearray(64)
        arm[0] = 1
        arm[1] = 2
        arm[2] = 1
        arm[52] = 1
        arm[53] = 0
        arm[54] = 3
        arm[55] = 1
        arm[56] = 3
        arm[57] = 0b01011111
        arm[58] = 0b00000101
        struct.pack_into("<f", arm, 16, 10.0)
        struct.pack_into("<f", arm, 28, 8.0)
        struct.pack_into("<f", arm, 40, 200.0)
        struct.pack_into("<f", arm, 60, 2.0)
        frame = pack_usb_frame(0x26, bytes(arm))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["ik_status_name"], "UNSAFE")
        self.assertEqual(decoded["coordinate_frame"]["x_positive"], "forward")
        self.assertEqual(decoded["unsafe_reason_name"], "WORKSPACE_MARGIN")
        self.assertTrue(decoded["error_flags"]["unsafe"])
        self.assertAlmostEqual(decoded["requested_xyz_mm"][0], 200.0)
        self.assertAlmostEqual(decoded["max_abs_err_deg"], 2.0)

        tool = bytearray(32)
        tool[0] = 1
        tool[2] = 1
        tool[9] = 2
        tool[11] = 1
        tool[24] = 0b01001101
        tool[25] = 0b00000110
        tool[26] = 1
        tool[27] = 0
        struct.pack_into("<f", tool, 16, 180.0)
        struct.pack_into("<f", tool, 20, 330.0)
        frame = pack_usb_frame(0x36, bytes(tool))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["selected_tool_name"], "CHUCK")
        self.assertTrue(decoded["status_flags"]["selected_moving"])
        self.assertTrue(decoded["error_flags"]["chuck_error"])
        self.assertAlmostEqual(decoded["clamp"]["target_angle"], 180.0)
        self.assertAlmostEqual(decoded["chuck"]["target_angle"], 330.0)

    def test_decode_robot_status_v3_targets(self):
        payload = bytearray(240)
        payload[0] = 3
        payload[1] = 1
        struct.pack_into("<f", payload, 160, 0.4)
        struct.pack_into("<f", payload, 172, 1.25)
        struct.pack_into("<f", payload, 184, 200.0)
        struct.pack_into("<f", payload, 196, 10.0)
        struct.pack_into("<f", payload, 208, 8.0)
        struct.pack_into("<f", payload, 220, 180.0)
        struct.pack_into("<f", payload, 224, 175.0)
        payload[236] = 3
        payload[237] = 1
        payload[238] = 0x08
        payload[239] = 1
        frame = pack_usb_frame(0x46, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["protocol_version"], 3)
        self.assertAlmostEqual(decoded["chassis"]["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["chassis"]["target_pos"]["dx"], 1.25)
        self.assertAlmostEqual(decoded["arm"]["target_xyz_mm"][0], 200.0)
        self.assertAlmostEqual(decoded["arm"]["actual_joint_deg"][0], 8.0)
        self.assertAlmostEqual(decoded["tool"]["clamp_real_angle"], 175.0)
        self.assertTrue(decoded["tool"]["error"])
        self.assertTrue(decoded["climb_summary_error_flags"]["flow_switch"])
        self.assertTrue(decoded["active_source_stale"])

    def test_decode_robot_status_v4_coordinate_frame(self):
        payload = bytearray(240)
        payload[0] = 4
        struct.pack_into("<f", payload, 160, 0.4)
        struct.pack_into("<f", payload, 164, -0.2)
        frame = pack_usb_frame(0x46, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["coordinate_frame"]["name"], "FLU")
        self.assertEqual(decoded["coordinate_frame"]["x_positive"], "forward")
        self.assertEqual(decoded["coordinate_frame"]["y_positive"], "left")
        self.assertAlmostEqual(decoded["chassis"]["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["chassis"]["target_vel"]["vy"], -0.2)

    def test_decode_climb_status_payload(self):
        payload = bytearray(64)
        payload[0] = 5
        payload[1] = 1
        payload[2] = 0
        payload[3] = 1
        payload[4] = 0b100
        payload[5] = 1
        payload[6] = 6
        payload[7] = 10
        struct.pack_into("<I", payload, 8, 2345)
        struct.pack_into("<f", payload, 16, 12.5)
        struct.pack_into("<f", payload, 32, 220.0)
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["cmd_name"], "CLIMB_GET_STATUS")
        self.assertEqual(decoded["payload"]["state_name"], "STEP_05_FRONT_MINUS_30")
        self.assertTrue(decoded["payload"]["state_done"])
        self.assertTrue(decoded["payload"]["error_flags"]["test_action"])
        self.assertEqual(decoded["payload"]["active_source_name"], "USB")
        self.assertEqual(decoded["payload"]["fdcan2_motor_online_count"], 6)
        self.assertEqual(decoded["payload"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertEqual(decoded["payload"]["elapsed_ms"], 2345)
        self.assertAlmostEqual(decoded["payload"]["leg_pos_mm"][0], 12.5)
        self.assertAlmostEqual(decoded["payload"]["leg_target_mm"][0], 220.0)

    def test_decode_climb_prepare_status_payload(self):
        payload = bytearray(64)
        payload[0] = 24
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "PREPARE_ALL_LEGS_MINUS_30")

        payload[0] = 25
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "UP_PREPARE_CHASSIS_FORWARD_30")

        payload[0] = 26
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "UP_LASER_APPROACH_X_LT_35")

        payload[0] = 27
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "DOWN_LASER_APPROACH_H_GT_65")

        payload[0] = 28
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "DOWN_PREPARE_CHASSIS_FORWARD_5")

    def test_climb_test_action_frame(self):
        frame = build_usb_command("CLIMB_TEST_ACTION", [16.0])
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(parsed.cmd, 0x57)
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 16.0)

        frame = build_usb_command("CLIMB_TEST_ACTION", [23.0])
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(CLIMB_TEST_ACTIONS[23], "FRONT_220")
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 23.0)

        self.assertEqual(CLIMB_TEST_ACTIONS[4], "REAR_DRIVE_FORWARD_30")
        self.assertEqual(CLIMB_TEST_ACTIONS[27], "FRONT_DRIVE_FORWARD_30")
        self.assertEqual(CLIMB_TEST_ACTIONS[33], "ALL_DRIVE_FORWARD_30")

        frame = build_usb_command("climb_all_drive_forward_30")
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(parsed.cmd, 0x57)
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 33.0)

        frames = build_climb_test_shortcut_sequence("climb_front_drive_forward_30")
        self.assertIsNotNone(frames)
        parsed = [UsbStreamParser().feed(frame)[0] for frame in frames]
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x57])
        self.assertAlmostEqual(struct.unpack_from("<f", parsed[2].payload, 0)[0], 27.0)

    def test_climb_downstairs_command_frames(self):
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x53)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x53)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x54)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x54)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x58)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x58)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_RUN"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5A)
        parsed = UsbStreamParser().feed(build_usb_command("UP_LASER_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5A)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5B)
        parsed = UsbStreamParser().feed(build_usb_command("DOWN_LASER_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5B)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5C)
        parsed = UsbStreamParser().feed(build_usb_command("UP_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5C)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5D)
        parsed = UsbStreamParser().feed(build_usb_command("DOWN_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5D)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_AUTO_RESUME"))[0]
        self.assertEqual(parsed.cmd, 0x5E)
        parsed = UsbStreamParser().feed(build_usb_command("AUTO_RESUME"))[0]
        self.assertEqual(parsed.cmd, 0x5E)

    def test_tool_set_state_frames(self):
        self.assertEqual(
            bytes_to_hex(build_usb_command("TOOL_SET_STATE", [0.0, 0.0])),
            "A5 5A 10 34 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 29 68 FF",
        )
        self.assertEqual(
            bytes_to_hex(build_usb_command("TOOL_SET_STATE", [0.0, 1.0])),
            "A5 5A 10 34 00 00 00 00 00 00 80 3F 00 00 00 00 00 00 00 00 30 7A FF",
        )
        self.assertEqual(
            bytes_to_hex(build_usb_command("TOOL_SET_STATE", [1.0, 0.0])),
            "A5 5A 10 34 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 96 BC FF",
        )
        self.assertEqual(
            bytes_to_hex(build_usb_command("TOOL_SET_STATE", [1.0, 1.0])),
            "A5 5A 10 34 00 00 80 3F 00 00 80 3F 00 00 00 00 00 00 00 00 8F AE FF",
        )

    def test_decode_climb_downstairs_status_payload(self):
        payload = bytearray(68)
        payload[0] = 4
        payload[1] = 1
        payload[3] = 1
        payload[5] = 1
        payload[6] = 6
        payload[64] = 1
        payload[65] = 0x81
        payload[66] = 0x0F
        payload[67] = 0x03
        struct.pack_into("<I", payload, 8, 2345)
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["flow_name"], "DOWNSTAIRS")
        self.assertEqual(decoded["state_name"], "DOWN_04_ALL_LEGS_UP_10")
        self.assertTrue(decoded["status_flags"]["motor_output_active"])
        self.assertTrue(decoded["status_flags"]["ready_for_next"])
        self.assertEqual(decoded["leg_reached"], [True, True, True, True])
        self.assertEqual(decoded["drive_reached"], [True, True])

    def test_usart_remote_frame(self):
        frame = build_remote_frame(
            mode=3,
            source_usb=1,
            tool=1,
            clamp_action=1,
            chuck_action=1,
            chassis=(0.2, 0.0, 0.0),
            arm_target=(0.0, 0.0, 180.0),
        )
        self.assertEqual(frame[0], 0xA5)
        self.assertEqual(frame[-1], 0x5A)
        self.assertEqual(len(frame), 43)
        self.assertEqual(frame[1 + 3], 1)
        self.assertEqual(frame[1 + 9], 1)
        self.assertEqual(frame[1 + 10], 1)
        self.assertEqual(frame[1 + 11], 1)
        self.assertEqual(frame[1 + 12], 1)
        self.assertEqual(frame[-2], sum(frame[1:-2]) & 0xFF)

    def test_usart_remote_climb_offsets(self):
        frame = build_remote_frame(
            mode=0,
            climb_enable=1,
            climb_step=1,
            climb_auto=0,
            chassis=(1.0, 2.0, 3.0),
            arm_target=(4.0, 5.0, 6.0),
        )
        data = frame[1:41]
        self.assertEqual(len(data), 40)
        self.assertEqual(data[13], 1)
        self.assertEqual(data[14], 1)
        self.assertEqual(data[15], 0)
        self.assertEqual(struct.unpack("<6f", data[16:40]), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))


class CrcTests(unittest.TestCase):
    def test_crc16_modbus_order(self):
        body = bytes.fromhex("A5 5A 00 06")
        self.assertEqual(crc16_modbus(body), 0xF982)

    def test_yaw_tune_command_pack(self):
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_GET_STATUS")), "A5 5A 00 49 0D C3 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_START", [1.0])), "A5 5A 10 47 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 BD 69 FF")

    def test_direct_command_names_are_case_insensitive(self):
        self.assertEqual(
            bytes_to_hex(build_usb_command("tool_set_mode", [0.0])),
            "A5 5A 10 32 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 88 8B FF",
        )
        self.assertEqual(
            bytes_to_hex(build_usb_command("ARM_SET_TARGET", [200.0, 0.0, 180.0])),
            "A5 5A 10 23 00 00 48 43 00 00 00 00 00 00 34 43 00 00 00 00 2D 25 FF",
        )


if __name__ == "__main__":
    unittest.main()
