#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

cc_common=(-std=c99 -Wall -Wextra -Wpedantic -Werror)

gcc "${cc_common[@]}" -Iapplications/Inc \
  tests/test_modbus_crc.c applications/Src/modbus_crc.c \
  -o /tmp/volleyball_modbus_crc_test
/tmp/volleyball_modbus_crc_test

gcc "${cc_common[@]}" -Itests/host_stub -Iapplications/Inc \
  tests/test_modbus_parser.c applications/Src/modbus_rtu.c applications/Src/modbus_crc.c \
  -o /tmp/volleyball_modbus_parser_test
/tmp/volleyball_modbus_parser_test

gcc "${cc_common[@]}" -Itests/delta_stub -Iapplications/Inc \
  tests/test_delta_state.c applications/Src/delta_clac.c applications/Src/delta_kinematics.c \
  -lm -o /tmp/volleyball_delta_state_test
/tmp/volleyball_delta_state_test

gcc "${cc_common[@]}" -Itests/robot_stub -Iapplications/Inc \
  tests/test_robot_state.c applications/Src/robot_control.c \
  -lm -o /tmp/volleyball_robot_state_test
/tmp/volleyball_robot_state_test

gcc "${cc_common[@]}" -ffunction-sections -fdata-sections -Wno-int-to-pointer-cast -DSTM32F407xx \
  -Iapplications/Inc -Iapplications/classic/Inc -Ibsp/boards/Inc -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include \
  tests/test_control_math.c applications/Src/delta_kinematics.c applications/Src/velocity_calc.c \
  -Wl,--gc-sections -lm -o /tmp/volleyball_control_math_test
/tmp/volleyball_control_math_test
