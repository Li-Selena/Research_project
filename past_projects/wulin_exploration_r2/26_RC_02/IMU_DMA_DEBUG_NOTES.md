# IMU DMA Debug Notes

> Historical DMA investigation from 2026-06-13. It records the debugging
> evidence. Current coordinate semantics are defined by `COORDINATE_SYSTEM.md`
> and current runtime behavior by the source code.

Date: 2026-06-13

## Current User Request

Goal: fix IMU UART7 DMA receive path so `imu_data` and `g_imu_debug` update correctly through DMA, not single-byte interrupt receive.

## Known Working Baseline

- Single-byte UART interrupt receive previously worked.
- In interrupt mode, these values were observed increasing normally:
  - `reg_update_count`
  - `acc_update_count`
  - `gyro_update_count`
  - `angle_update_count`
  - `imu_data.online = 1`
  - `last_error_code = 0`

## Current DMA Build Observation

Latest Keil Debug screenshots after switching back to DMA:

`imu_data`

- Address: `0x20005930`
- All decoded values are zero:
  - `acc_x/y/z = 0`
  - `gyro_x/y/z = 0`
  - `roll/pitch/yaw = 0`
  - `last_update_tick = 0`
  - `online = 0`
  - `update_flag = 0`

`g_imu_debug`

- Address: `0x24004D0C`
- `dma_start_count = 212671`
- `dma_rx_event_count = 4845`
- `dma_rx_byte_count = 1240832`
- `dma_restart_count = 212833`
- `reg_update_count = 0`
- `acc_update_count = 0`
- `gyro_update_count = 0`
- `angle_update_count = 0`
- `last_update_tick = 0`
- `last_reg = 0`
- `last_reg_num = 0`
- `last_dma_size = 256`
- `last_dma_read_pos = 0`
- `dma_started = 1`
- `initialized = 1`
- `last_rx_byte = 0`
- `last_error_code = 16`
- `dma_buf_addr = 536871136`
- `dma_ndtr = 256`
- `dma_event_type = 2`
- `dma_restart_fail_count = 0`
- `dma_rx_overrun_count = 0`
- `rx_mode = 2`

Important conversion:

- `dma_buf_addr = 536871136 = 0x200000E0`

This is the key clue: the DMA receive buffer is still linked in DTCM (`0x20000000` range), not AXI SRAM (`0x24000000` range). On STM32H7, DMA cannot access DTCM, which matches:

- `last_error_code = 16` (`HAL_UART_ERROR_DMA`)
- huge `dma_restart_count`
- `reg_update_count = 0`
- decoded `imu_data` all zero

## Current Code State

Files touched for DMA path:

- `Components/Device/Src/imu.c`
- `Components/Device/Inc/imu.h`
- `BSP/Src/bsp_uart.c`
- `Core/Src/usart.c`
- `Core/Src/stm32h7xx_it.c`
- `Core/Src/dma.c`
- `MDK-ARM/26_RC_02.uvprojx`
- `MDK-ARM/26_RC_02/26_RC_02.sct`

Current intended path:

- `IMU_Init()` calls `IMU_StartDmaReceive()`
- `IMU_StartDmaReceive()` calls `HAL_UARTEx_ReceiveToIdle_DMA(&huart7, imu_rx_dma_buf, 256)`
- UART7 DMA callback:
  - `HAL_UARTEx_RxEventCallback()`
  - `IMU_RxDmaEventCallback(Size)`
  - `IMU_ProcessDmaRange()`
  - `WitSerialDataIn(byte)`
- UART7 single-byte interrupt receive path has been removed from main callback path.

Current intended Keil scatter config:

- `MDK-ARM/26_RC_02.uvprojx`
  - `<useFile>1</useFile>`
  - `<ScatterFile>.\26_RC_02\26_RC_02.sct</ScatterFile>`
- `MDK-ARM/26_RC_02/26_RC_02.sct`
  - `RW_IRAM2 0x24000000 ...`
  - `*(.imu_dma)`

## Next Debug Step

The next fix focused on why `imu_rx_dma_buf` was still at `0x200000E0`.

Likely causes to check:

1. Keil is still not actually using `MDK-ARM/26_RC_02/26_RC_02.sct`.
2. The section pattern `*(.imu_dma)` does not match the emitted section name under the current compiler.
3. Keil regenerated or overwrote the scatter/project file before build.
4. Object files were not cleaned; old `imu.o` or old map is still being used.

Immediate verification after any fix:

- Clean Targets
- Rebuild
- Check map:
  - `imu_rx_dma_buf` must be `0x2400xxxx`
- Check debug:
  - `g_imu_debug.dma_buf_addr` must be `0x2400xxxx`
  - `last_error_code` should stop being `16`
  - `dma_restart_count` should stop increasing rapidly
  - `reg_update_count` should start increasing

## 2026-06-13 DMA Fix Update

The DMA buffer placement was changed to avoid relying only on linker allocation.

Current intended runtime DMA buffer address:

- `IMU_DMA_AXI_BASE = 0x2401FF00`
- `g_imu_debug.dma_buf_addr` must become `0x2401FF00`

Why this address:

- It is in AXI SRAM (`0x24000000` range), accessible by DMA.
- It is 32-byte aligned.
- It reserves exactly 256 bytes at the end of configured IRAM2.
- It avoids the previous DTCM address `0x200000E0`.

Current `imu.c` approach:

- `imu_rx_dma_buf` is a fixed pointer:
  - `(uint8_t *)0x2401FF00`
- `imu_rx_dma_reserve[256]` remains in `.imu_dma` only to reserve/link a matching section when scatter is active.
- DMA start clears the buffer with `memset`, then calls `SCB_CleanInvalidateDCache_by_Addr`.
- DMA Rx callback calls `SCB_InvalidateDCache_by_Addr` before reading bytes.

Current scatter reservation:

- `RW_IRAM2 0x24000000 0x0001FF00`
- `RW_IMU_DMA 0x2401FF00 0x00000100`

Important next validation:

- If `g_imu_debug.dma_buf_addr` is not `0x2401FF00` after download, the board is not running the rebuilt firmware.
- If `dma_buf_addr = 0x2401FF00` but `last_error_code = 16`, inspect DMA stream flags/config next.
- If `last_error_code = 0`, `dma_rx_event_count` grows, but `reg_update_count = 0`, inspect actual bytes/protocol parsing next.
