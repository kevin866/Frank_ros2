#!/usr/bin/env python3
"""
benchmark_yunet.py
------------------
Benchmarks YuNet face detection in two modes:
  1. ONNX via onnxruntime (CPU) — current production path
  2. TensorRT FP16 engine (GPU) — converted from ONNX

Prints a side-by-side comparison table and saves results to
benchmark_results.json for portfolio documentation.

Usage:
    python3 benchmark_yunet.py
"""

import time
import json
import numpy as np
import onnxruntime as ort
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # noqa: F401

# ── paths ─────────────────────────────────────────────────────────────────────
ONNX_PATH = "/home/frank/frank_ws/src/frank_vision/models/face_detection_yunet_2023mar.onnx"
TRT_PATH  = "/home/frank/frank_ws/src/frank_vision/models/face_detection_yunet_2023mar.trt"

# ── benchmark config ──────────────────────────────────────────────────────────
INPUT_H  = 640
INPUT_W  = 640
N_WARMUP = 20
N_RUNS   = 200


# ═══════════════════════════════════════════════════════════════════════════════
# ONNX baseline — onnxruntime CPU
# ═══════════════════════════════════════════════════════════════════════════════
def benchmark_onnx(frame: np.ndarray) -> dict:
    sess = ort.InferenceSession(ONNX_PATH, providers=["CPUExecutionProvider"])
    inp  = frame.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0

    print(f"  Warming up ONNX ({N_WARMUP} runs)...", flush=True)
    for _ in range(N_WARMUP):
        sess.run(None, {"input": inp})

    print(f"  Benchmarking ONNX ({N_RUNS} runs)...", flush=True)
    times = []
    for _ in range(N_RUNS):
        t0 = time.perf_counter()
        sess.run(None, {"input": inp})
        times.append((time.perf_counter() - t0) * 1000)

    return _stats(times, "ONNX (onnxruntime / CPU)")


# ═══════════════════════════════════════════════════════════════════════════════
# TensorRT FP16 engine
# ═══════════════════════════════════════════════════════════════════════════════
def benchmark_trt(frame: np.ndarray) -> dict:
    logger  = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)

    with open(TRT_PATH, "rb") as f:
        engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()

    # Input buffer
    inp_host = np.ascontiguousarray(
        frame.transpose(2, 0, 1)[np.newaxis].astype(np.float16) / 255.0
    )
    inp_dev = cuda.mem_alloc(inp_host.nbytes)

    # Output buffers
    tensor_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
    out_bufs_host, out_bufs_dev = [], []
    for name in tensor_names[1:]:
        shape = tuple(engine.get_tensor_shape(name))
        buf   = np.empty(shape, dtype=np.float16)
        out_bufs_host.append(buf)
        out_bufs_dev.append(cuda.mem_alloc(buf.nbytes))

    # Bind tensor addresses
    context.set_tensor_address(tensor_names[0], int(inp_dev))
    for i, name in enumerate(tensor_names[1:]):
        context.set_tensor_address(name, int(out_bufs_dev[i]))

    stream = cuda.Stream()

    def infer():
        cuda.memcpy_htod_async(inp_dev, inp_host, stream)
        context.execute_async_v3(stream_handle=stream.handle)
        for h, d in zip(out_bufs_host, out_bufs_dev):
            cuda.memcpy_dtoh_async(h, d, stream)
        stream.synchronize()

    print(f"  Warming up TensorRT ({N_WARMUP} runs)...", flush=True)
    for _ in range(N_WARMUP):
        infer()

    print(f"  Benchmarking TensorRT ({N_RUNS} runs)...", flush=True)
    times = []
    for _ in range(N_RUNS):
        t0 = time.perf_counter()
        infer()
        times.append((time.perf_counter() - t0) * 1000)

    return _stats(times, "TensorRT FP16 (GPU)")


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════
def _stats(times: list, label: str) -> dict:
    arr = np.array(times)
    return {
        "label":     label,
        "n_runs":    len(times),
        "mean_ms":   round(float(arr.mean()), 3),
        "median_ms": round(float(np.median(arr)), 3),
        "min_ms":    round(float(arr.min()), 3),
        "max_ms":    round(float(arr.max()), 3),
        "p95_ms":    round(float(np.percentile(arr, 95)), 3),
        "fps":       round(1000.0 / float(arr.mean()), 1),
    }


def print_table(onnx: dict, trt_res: dict):
    speedup  = onnx["mean_ms"] / trt_res["mean_ms"]
    fps_gain = trt_res["fps"] - onnx["fps"]

    print("\n" + "═" * 62)
    print("  YuNet Face Detector — ONNX (CPU) vs TensorRT FP16 (GPU)")
    print("  Jetson Orin Nano  |  Input: 640×640  |  200 runs")
    print("═" * 62)
    print(f"  {'Metric':<18} {'ONNX CPU':>14} {'TensorRT FP16':>16}")
    print("─" * 62)
    for key, label in [
        ("mean_ms",   "Mean latency"),
        ("median_ms", "Median latency"),
        ("min_ms",    "Min latency"),
        ("p95_ms",    "p95 latency"),
        ("fps",       "Throughput"),
    ]:
        unit = " FPS" if key == "fps" else " ms"
        print(f"  {label:<18} {str(onnx[key]) + unit:>14} {str(trt_res[key]) + unit:>16}")
    print("─" * 62)
    print(f"  {'Speedup':<18} {'':>14} {speedup:.2f}×  faster")
    print(f"  {'FPS gain':<18} {'':>14} +{fps_gain:.1f} FPS")
    print("═" * 62 + "\n")


# ═══════════════════════════════════════════════════════════════════════════════
def main():
    frame = np.random.randint(0, 255, (INPUT_H, INPUT_W, 3), dtype=np.uint8)

    print("\n── ONNX baseline (CPU) ──────────────────────────────────")
    onnx_res = benchmark_onnx(frame)

    print("\n── TensorRT FP16 (GPU) ──────────────────────────────────")
    trt_res = benchmark_trt(frame)

    print_table(onnx_res, trt_res)

    results = {
        "device":    "Jetson Orin Nano",
        "input":     f"{INPUT_W}x{INPUT_H}",
        "model":     "YuNet face_detection_yunet_2023mar",
        "onnx":      onnx_res,
        "tensorrt":  trt_res,
        "speedup_x": round(onnx_res["mean_ms"] / trt_res["mean_ms"], 2),
    }

    out_path = "/home/frank/frank_ws/src/frank_vision/benchmark_results.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Results saved to {out_path}")


if __name__ == "__main__":
    main()