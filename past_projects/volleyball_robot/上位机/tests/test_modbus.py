import struct
import unittest

from modbus_client import (CMD_DELTA_STRIKE, FC_READ_INPUT,
                           FC_WRITE_MULTIPLE, FC_WRITE_SINGLE,
                           ModbusError, ModbusRTUClient, RobotClient, crc16,
                           floats_to_registers, registers_to_floats, with_crc)


class FakeSerial:
    timeout = 0.01

    def __init__(self, corrupt_first=False):
        self.response = bytearray()
        self.requests = []
        self.corrupt_first = corrupt_first

    def reset_input_buffer(self):
        self.response.clear()

    def close(self):
        pass

    def write(self, request):
        self.requests.append(request)
        if struct.unpack("<H", request[-2:])[0] != crc16(request[:-2]):
            return len(request)
        slave, function = request[:2]
        if function in (0x03, 0x04):
            _, _, address, count = struct.unpack(">BBHH", request[:-2])
            values = [(address + i) & 0xFFFF for i in range(count)]
            payload = bytes((slave, function, count * 2)) + struct.pack(f">{count}H", *values)
        elif function == FC_WRITE_SINGLE:
            payload = request[:-2]
        elif function == FC_WRITE_MULTIPLE:
            address, count = struct.unpack(">HH", request[2:6])
            payload = struct.pack(">BBHH", slave, function, address, count)
        else:
            payload = bytes((slave, function | 0x80, 1))
        response = bytearray(with_crc(payload))
        if self.corrupt_first:
            response[-1] ^= 0xFF
            self.corrupt_first = False
        self.response.extend(response)
        return len(request)

    def read(self, size):
        data = bytes(self.response[:size])
        del self.response[:size]
        return data


class ModbusTests(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(with_crc(bytes.fromhex("01030000000a")).hex(), "01030000000ac5cd")

    def test_float_word_order_round_trip(self):
        values = [0.0, -12.5, 400.0]
        self.assertEqual(registers_to_floats(floats_to_registers(values)), values)
        self.assertEqual(floats_to_registers([1.0]), [0x3F80, 0x0000])

    def test_read_and_write_frames(self):
        fake = FakeSerial()
        client = ModbusRTUClient(serial_instance=fake)
        self.assertEqual(client.read_input(0x10, 3), [0x10, 0x11, 0x12])
        client.write_single(1, CMD_DELTA_STRIKE)
        client.write_multiple(0x20, floats_to_registers((0.0, 0.0, 400.0)) + [300])
        self.assertEqual(fake.requests[0][1], FC_READ_INPUT)
        self.assertEqual(fake.requests[1][1], FC_WRITE_SINGLE)
        self.assertEqual(fake.requests[2][1], FC_WRITE_MULTIPLE)

    def test_crc_error_is_retried(self):
        fake = FakeSerial(corrupt_first=True)
        client = ModbusRTUClient(serial_instance=fake, retries=1)
        self.assertEqual(client.read_input(0, 1), [0])
        self.assertEqual(len(fake.requests), 2)

    def test_invalid_delta_timeout_rejected_locally(self):
        robot = RobotClient(modbus=ModbusRTUClient(serial_instance=FakeSerial()))
        with self.assertRaises(ValueError):
            robot.set_delta_strike_point(0, 0, 400, 99)

    def test_robot_command_refreshes_heartbeat_first(self):
        fake = FakeSerial()
        robot = RobotClient(modbus=ModbusRTUClient(serial_instance=fake))
        robot.strike_delta()
        self.assertEqual([request[3] for request in fake.requests], [0x00, 0x01])
        self.assertEqual([request[1] for request in fake.requests],
                         [FC_WRITE_SINGLE, FC_WRITE_SINGLE])

    def test_timeout_without_response(self):
        class SilentSerial(FakeSerial):
            def write(self, request):
                self.requests.append(request)
                return len(request)
        client = ModbusRTUClient(serial_instance=SilentSerial(), retries=0)
        with self.assertRaises(ModbusError):
            client.read_input(0, 1)


if __name__ == "__main__":
    unittest.main()
