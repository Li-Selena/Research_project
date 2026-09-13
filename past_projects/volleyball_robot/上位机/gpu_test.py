import numpy as np
import torch
import time


def benchmark_gemm(dtype=torch.float32, size=8192, trials=10, device='cuda'):
    assert dtype in [torch.float32, torch.float64, torch.float16], "Only float32 and float64 supported."
    dtype_name = dtype

    print(f"\nTesting GEMM ({dtype_name}) on {device.upper()}... Matrix size: {size}x{size}")

    # 生成测试矩阵
    a = torch.randn((size, size), dtype=dtype, device=device)
    b = torch.randn((size, size), dtype=dtype, device=device)
    print(1)
    # 预热 CUDA
    if device != 'cpu':
        torch.cuda.synchronize()
    for _ in range(3):
        print(1.5)
        _ = torch.matmul(a, b)
    if device != 'cpu':
        torch.cuda.synchronize()
    print(2)
    # 正式计时
    times = []
    for _ in range(trials):
        start = time.time()
        _ = torch.matmul(a, b)
        if device != 'cpu':
            torch.cuda.synchronize()
        end = time.time()
        times.append(end - start)

    avg_time = sum(times) / trials
    gflops = 2 * size ** 3 / avg_time / 1e9

    print(f"Average Time: {avg_time:.6f} s")
    print(f"Estimated Performance: {gflops:.2f} GFLOPs ({dtype_name})")


# 必须使用 CUDA
if torch.cuda.is_available():
    torch.set_printoptions(precision=4, sci_mode=False)
    print(f"Using GPU: {torch.cuda.get_device_name(0)}")
    benchmark_gemm(dtype=torch.float16)
    benchmark_gemm(dtype=torch.float32)
    benchmark_gemm(dtype=torch.float64)
else:
    print("CUDA is not available.")
