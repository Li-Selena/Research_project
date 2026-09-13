# HWT605 INS Prompt

> 历史设计提示。当前实现以 `INS_Task.c`、`robot_frame.h` 和
> `COORDINATE_SYSTEM.md` 为准；下文示例不覆盖已经确定的安装矩阵。

请基于已有的 HWT605 / WIT IMU 接入代码，实现或完善一个惯性姿态数据模块，用于替代 BMI088 + QuaternionEKF 的 INS 姿态解算。

## 核心要求

不要重新实现 BMI088 驱动。
不要实现 BMI088 温控。
不要实现四元数 EKF。
不要自己从原始加速度和陀螺仪融合姿态。

HWT605 / WIT 模块内部已经完成姿态融合，主控只负责接收、解析、坐标整理、连续 yaw 和输出。

## 已有文件

- `Components/Device/Src/imu.c`
- `Components/Device/Inc/imu.h`
- `Components/Device/Src/wit_c_sdk.c`
- `Components/Device/Inc/wit_c_sdk.h`
- `Components/Device/Inc/REG.h`
- `Applications/Task/Src/INS_Task.c`
- `Applications/Task/Inc/INS_Task.h`
- `Components/Algorithm/Src/`
- `Components/Algorithm/Inc/`

请保持 `wit_c_sdk.c`、`wit_c_sdk.h`、`REG.h` 的接口基本不变，优先在 `imu.c`、`imu.h`、`INS_Task.c`、`INS_Task.h` 中完成适配。

## 文件归属与分层

- `Components/Device/Src/imu.c` / `Components/Device/Inc/imu.h` 只负责 HWT605 的 UART DMA 接收、WIT SDK 适配、寄存器回调解析和 `imu_data` 更新。
- `Applications/Task/Src/INS_Task.c` / `Applications/Task/Inc/INS_Task.h` 负责惯性导航 FreeRTOS 任务实现、任务内周期调度、状态聚合和对外发布接口。
- 如果需要补充可复用算法，统一放入 `Components/Algorithm/Src/` 和 `Components/Algorithm/Inc/`，例如角度归一化、连续 yaw 展开、坐标系转换、里程计积分、滤波或姿态/位姿辅助计算。
- `INS_Task.c` 中不要堆叠复杂算法实现，只调用 `Components/Algorithm` 中的算法函数，并维护任务状态与线程安全的数据发布。
- `Components/Algorithm` 中的算法代码应尽量保持纯 C 计算逻辑，不直接依赖 UART、DMA、HAL 句柄或 FreeRTOS 阻塞 API，方便底盘、调试和其他任务复用。
- 底盘运动控制需要读取当前位置和自身状态时，应通过 `INS_Task.h` 暴露的状态读取接口获取，不建议在多个任务里直接读取或重复加工 HWT605 原始数据。

## 当前硬件与通信方式

- IMU 使用 `UART7`。
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收。
- DMA 接收缓冲区长度为 `256` 字节。
- DMA 缓冲区放在 `.imu_dma` section，并按 32 字节对齐：

```c
static uint8_t imu_rx_dma_buf[256] __attribute__((section(".imu_dma"), aligned(32)));
```

- 禁用 DMA 半传输中断，只使用 ReceiveToIdle 事件处理新数据。
- WIT 协议使用 `WIT_PROTOCOL_NORMAL`。
- 设备地址使用 `0xFF`。
- 输出频率设置为 `RRATE_200HZ`。
- 输出内容设置为：

```c
RSW_ACC | RSW_GYRO | RSW_ANGLE
```

## 已有 IMU 数据结构

沿用以下结构体，不要随意改字段名。若需要在线检测，可扩展 `last_update_tick` 或 `online` 字段。

```c
typedef struct {
    float acc_x;   /* g */
    float acc_y;   /* g */
    float acc_z;   /* g */
    float gyro_x;  /* deg/s */
    float gyro_y;  /* deg/s */
    float gyro_z;  /* deg/s */
    float roll;    /* deg */
    float pitch;   /* deg */
    float yaw;     /* deg */

    uint8_t update_flag;
} IMU_Data_t;

extern IMU_Data_t imu_data;
```

## IMU 初始化逻辑

实现或保持 `IMU_Init()`：

1. 注册 WIT SDK 串口发送函数：

```c
WitSerialWriteRegister(IMU_SerialWrite);
```

2. 注册 WIT SDK 寄存器更新回调：

```c
WitRegisterCallBack(IMU_RegUpdateCallback);
```

3. 注册 WIT SDK 延时函数：

```c
WitDelayMsRegister((DelaymsCb)IMU_DelayMs);
```

4. 初始化 WIT 协议：

```c
WitInit(WIT_PROTOCOL_NORMAL, 0xFF);
```

5. 启动 UART7 DMA ReceiveToIdle 接收：

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart7, imu_rx_dma_buf, 256);
```

6. 若 `huart7.hdmarx != NULL`，关闭 DMA 半传输中断：

```c
__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
```

7. 配置 HWT605 输出频率：

```c
WitSetOutputRate(RRATE_200HZ);
```

8. 配置输出内容：

```c
WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
```

9. 任意关键步骤失败时调用 `Error_Handler()`。

## DMA 接收处理逻辑

实现或保持 `IMU_RxDmaEventCallback(uint16_t size)`：

- `size` 是 `HAL_UARTEx_RxEventCallback()` 传入的当前 DMA 写入位置。
- 保存一个静态或全局 `read_pos`。
- 若 `write_pos == read_pos`，直接返回。
- 若 `write_pos > read_pos`，处理 `[read_pos, write_pos)`。
- 若 `write_pos < read_pos`，说明 DMA 环形缓冲回绕：
  - 先处理 `[read_pos, 256)`。
  - 再处理 `[0, write_pos)`。
- 每处理一个字节，都调用：

```c
WitSerialDataIn(byte);
```

- 最后更新 `read_pos = write_pos`。
- 若 `read_pos >= 256`，则置 0。

实现或保持 `IMU_RestartDmaReceive()`：

- 清除 `dma_started` 标志。
- 调用 `HAL_UART_AbortReceive(&huart7)`。
- 重新调用 `IMU_StartDmaReceive()`。

## WIT SDK 解析逻辑

沿用 `wit_c_sdk.c` 中的 `WitSerialDataIn()`。

`WIT_PROTOCOL_NORMAL` 帧格式：

- 11 字节一帧。
- `Byte0 = 0x55`。
- `Byte1` 是数据类型：
  - `0x51`：加速度，`WIT_ACC`。
  - `0x52`：角速度，`WIT_GYRO`。
  - `0x53`：姿态角，`WIT_ANGLE`。
- `Byte2~Byte9` 为 4 个 `int16_t` 数据。
- `Byte10` 为前 10 字节累加和低 8 位。
- 校验失败时滑窗丢弃 1 字节继续找帧。
- 校验成功后解析为 `sReg[]` 寄存器，并触发：

```c
IMU_RegUpdateCallback(uiReg, uiLen);
```

## REG.h 关键寄存器

```c
#define AX     0x34
#define AY     0x35
#define AZ     0x36
#define GX     0x37
#define GY     0x38
#define GZ     0x39
#define Roll   0x3d
#define Pitch  0x3e
#define Yaw    0x3f
```

## 寄存器回调逻辑

实现或保持 `IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiLen)`。

- 可以忽略 `uiLen`。
- 为避免多任务读写竞争，更新 `imu_data` 时可以短暂使用 `__disable_irq()` / `__enable_irq()`。

当 `uiReg == AX`：

```c
imu_data.acc_x = (float)sReg[AX] / 32768.0f * 16.0f;
imu_data.acc_y = (float)sReg[AY] / 32768.0f * 16.0f;
imu_data.acc_z = (float)sReg[AZ] / 32768.0f * 16.0f;
imu_data.update_flag = 1U;
```

当 `uiReg == GX`：

```c
imu_data.gyro_x = (float)sReg[GX] / 32768.0f * 2000.0f;
imu_data.gyro_y = (float)sReg[GY] / 32768.0f * 2000.0f;
imu_data.gyro_z = (float)sReg[GZ] / 32768.0f * 2000.0f;
imu_data.update_flag = 1U;
```

当 `uiReg == Roll`：

```c
imu_data.roll  = (float)sReg[Roll]  / 32768.0f * 180.0f;
imu_data.pitch = (float)sReg[Pitch] / 32768.0f * 180.0f;
imu_data.yaw   = (float)sReg[Yaw]   / 32768.0f * 180.0f;
imu_data.update_flag = 1U;
```

## 单位要求

- `acc_x / acc_y / acc_z` 单位为 `g`。
- `gyro_x / gyro_y / gyro_z` 单位为 `deg/s`。
- `roll / pitch / yaw` 单位为 `deg`。
- 不要再乘 `RadiansToDegrees`，因为 HWT605 输出已经是角度制。

## 兼容原 INS_Info

若需要兼容原 BMI088 INS 输出，定义或更新：

```c
typedef struct
{
    float Pitch_Angle;
    float Yaw_Angle;
    float Yaw_TolAngle;
    float Roll_Angle;

    float Pitch_Gyro;
    float Yaw_Gyro;
    float Roll_Gyro;

    float Angle[3];
    float Gyro[3];
    float Accel[3];

    float Last_Yaw_Angle;
    int16_t YawRoundCount;
} INS_Info_Typedef;
```

建议实现以 IMU 快照为输入的 `INS_Update_From_IMU()`，避免一边更新一边读取：

```c
void INS_Update_From_IMU(const IMU_Data_t *imu)
{
    if (imu == NULL) return;

    INS_Info.Roll_Angle  = imu->roll;
    INS_Info.Pitch_Angle = imu->pitch;
    INS_Info.Yaw_Angle   = imu->yaw;

    INS_Info.Roll_Gyro  = imu->gyro_x;
    INS_Info.Pitch_Gyro = imu->gyro_y;
    INS_Info.Yaw_Gyro   = imu->gyro_z;

    INS_Info.Accel[0] = imu->acc_x;
    INS_Info.Accel[1] = imu->acc_y;
    INS_Info.Accel[2] = imu->acc_z;

    INS_Info.Gyro[0] = imu->gyro_x;
    INS_Info.Gyro[1] = imu->gyro_y;
    INS_Info.Gyro[2] = imu->gyro_z;

    INS_Info.Angle[0] = imu->yaw;
    INS_Info.Angle[1] = imu->roll;
    INS_Info.Angle[2] = imu->pitch;
}
```

如果为了兼容已有代码保留无参 `INS_Update_From_IMU(void)`，也应在函数内部先复制 `imu_data` 快照，再用快照更新 `INS_Info`。

## 惯性导航状态输出

`INS_Task.c` 不只是姿态搬运函数，应作为 FreeRTOS 中的惯性导航任务，统一维护机器人当前位姿、速度和 IMU 在线状态，方便底盘运动控制读取。

注意：HWT605 只提供加速度、角速度和融合后的姿态角，不直接提供底盘平面位置。`x / y` 位置不要通过 IMU 加速度二次积分硬算，优先由底盘编码器、麦克纳姆正向运动学或其他定位来源提供，再结合 HWT605 yaw 做 robot frame 到 world frame 的转换与发布。

建议在 `INS_Task.h` 中提供类似状态结构体和读取接口：

```c
typedef struct
{
    float x_m;             /* world frame position x, m */
    float y_m;             /* world frame position y, m */
    float yaw_rad;         /* wrapped yaw, rad */
    float yaw_total_rad;   /* continuous yaw, rad */

    float vx_mps;          /* robot/world frame velocity, according to project convention */
    float vy_mps;
    float wz_radps;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_total_deg;

    uint8_t imu_online;
    uint32_t update_tick;
} INS_NavState_t;

void INS_GetState(INS_NavState_t *out);
```

`INS_GetState()` 内部应使用短临界区或任务安全的复制方式，把当前 `INS_NavState_t` 拷贝给调用者，避免底盘任务读取到一半更新的数据。

如果需要新增里程计算法，建议放入：

```text
Components/Algorithm/Inc/ins_odometry.h
Components/Algorithm/Src/ins_odometry.c
```

`INS_Task.c` 只负责调用算法、更新 `INS_Info` / `INS_NavState_t`，并把最新状态发布给底盘。

## 连续 yaw 角处理

HWT605 的 yaw 通常在 `-180` 到 `180` 度之间。实现：

```c
void INS_Update_Yaw_TotalAngle(void)
{
    if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.0f)
    {
        INS_Info.YawRoundCount++;
    }
    else if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.0f)
    {
        INS_Info.YawRoundCount--;
    }

    INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
    INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.0f;
}
```

## 坐标安装方向

当前安装关系已经确定：传感器原始轴为 X 右、Y 前、Z 上，项目统一轴为 X 前、Y 左、Z 上。因此向量转换为 `robot_x=sensor_y`、`robot_y=-sensor_x`、`robot_z=sensor_z`；Euler 角通过完整旋转矩阵换基，不直接交换角度字段。转换只在 `INS_Task` 入口执行一次。

不要把安装方向转换写死在 WIT 协议解析函数里。

如果机器人坐标系和传感器坐标系不一致，请单独写：

```c
IMU_To_Robot_Frame();
```

或：

```c
INS_ApplyAxisMapping();
```

该函数只负责轴交换和符号翻转。

以下仅为通用示例：

```c
robot_roll  = imu_roll;
robot_pitch = imu_pitch;
robot_yaw   = imu_yaw;
```

或根据安装方向改成：

```c
robot_pitch = -imu_roll;
robot_roll  = imu_pitch;
robot_yaw   = imu_yaw;
```

若坐标转换包含多个安装姿态、角度归一化或 robot/world 坐标变换，请把核心函数放到 `Components/Algorithm`，例如 `ins_frame.c` / `ins_frame.h`；`INS_Task.c` 只选择当前配置并调用。

## 任务结构

- `IMU_Init()` 应在系统外设初始化后执行。
- 惯性导航任务的实现放在 `Applications/Task/Src/INS_Task.c`，声明放在 `Applications/Task/Inc/INS_Task.h`。
- `Core/Src/freertos.c` 中只保留任务创建和弱函数兜底，不要把业务实现写进 CubeMX 生成文件。
- `INS_Task.c` 应实现 `void INS_Task(void const *argument)`，它是 FreeRTOS 任务之一，用于持续维护底盘可读取的当前位置与自身状态。
- `IMU_Init()` 建议由 `INS_Task.c` 在任务启动后调用一次；如果当前工程已在其他任务中初始化 IMU，需要迁移或保证不会重复初始化 UART DMA。
- 串口数据由 UART DMA + IDLE 回调喂入 `WitSerialDataIn()`。
- 主循环或任务中不要阻塞等待 IMU 串口数据。
- 任务周期可以是 `1 ms`。
- HWT605 当前输出率是 `200 Hz`，因此不是每个 `1 ms` 都一定有新数据。

任务中建议逻辑：

```c
void INS_Task(void const *argument)
{
    IMU_Data_t imu_snapshot;

    IMU_Init();

    for (;;)
    {
        if (imu_data.update_flag == 1U)
        {
            __disable_irq();
            imu_snapshot = imu_data;
            imu_data.update_flag = 0U;
            __enable_irq();

            INS_Update_From_IMU(&imu_snapshot);
            INS_Update_Yaw_TotalAngle();
            INS_Update_NavState();
        }

        osDelay(1);
    }
}
```

如果 `INS_Update_Yaw_TotalAngle()`、`INS_Update_NavState()` 或里程计积分逻辑较复杂，应把核心计算拆到 `Components/Algorithm`，`INS_Task.c` 只负责调度、快照和发布。

如需在线检测，可扩展 `imu_data.last_update_tick`，每次回调更新 tick，超过 `100 ms` 未更新则认为 IMU 离线。

## HAL 回调接入

在 `HAL_UARTEx_RxEventCallback()` 中加入：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart7)
    {
        IMU_RxDmaEventCallback(Size);
    }
}
```

如果工程中 DMA ReceiveToIdle 事件后不会自动继续接收，则需要在回调中或 `IMU_RxDmaEventCallback()` 后重新启动：

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart7, imu_rx_dma_buf, IMU_RX_DMA_BUF_LEN);
__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
```

但如果当前 `IMU_StartDmaReceive()` 已经以循环 DMA 正常工作，不要重复启动导致接收状态混乱。

## 代码风格要求

- 使用 C 语言。
- 适配 STM32 HAL。
- 保持现有 `wit_c_sdk.c` / `wit_c_sdk.h` / `REG.h` 接口不变。
- 不要把 WIT SDK 改成手写解析器，除非明确要求移除 SDK。
- 设备接入相关代码放在 `Components/Device` 的 `imu.c` / `imu.h`。
- 惯性导航任务、状态聚合和对外接口放在 `Applications/Task` 的 `INS_Task.c` / `INS_Task.h`。
- 可复用算法统一放在 `Components/Algorithm/Src` / `Components/Algorithm/Inc`，不要把算法散落到 `Core` 或设备驱动里。
- `INS_Task.c` 可以包含 `cmsis_os.h` / FreeRTOS 相关调度逻辑；`Components/Algorithm` 中的算法文件尽量不要依赖 RTOS。
- 保留 `imu_rx_byte`，因为当前代码中它用于旧外部引用兼容。
- 如果 `imu.h` 中声明了 `IMU_ParseData()` 但实际不需要，可以删除声明或实现为空包装，避免链接或维护混乱。

## 最终输出

请给出需要修改或新增的 C 代码，并说明：

1. UART7 DMA 回调如何接入。
2. 如何从 `imu_data` 映射到 `INS_Info`。
3. 如何处理连续 yaw。
4. 是否需要在线检测。
5. HWT605 方案与 BMI088 + EKF 方案的区别。
6. 哪些新增算法放入 `Components/Algorithm`，对应的 `.c` / `.h` 文件名和接口。
7. `Applications/Task/Src/INS_Task.c` 中 FreeRTOS 惯性导航任务的完整实现方式。
8. 底盘运动控制如何通过 `INS_Task.h` 获取当前位置、速度、yaw 和 IMU 在线状态。

核心区别：

HWT605 输出已融合姿态，主控只做解析、坐标转换、连续 yaw 和在线检测；BMI088 方案则需要主控读取原始加速度和陀螺仪，并自行通过 Quaternion EKF 做姿态融合。
