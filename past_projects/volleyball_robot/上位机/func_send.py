"""Compatibility exports for code that previously imported func_send."""

from modbus_client import ModbusError, ModbusRTUClient, RobotClient, RobotStatus

__all__ = ["ModbusError", "ModbusRTUClient", "RobotClient", "RobotStatus"]
