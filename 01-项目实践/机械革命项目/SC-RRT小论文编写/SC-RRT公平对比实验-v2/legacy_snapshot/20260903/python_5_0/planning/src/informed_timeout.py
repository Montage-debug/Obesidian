"""Informed-RRT* 墙钟超时：子进程执行 + 强制终止。"""

import multiprocessing as mp
import sys
import time

from baselines import InformedRRTStar


def _mp_context():
    # Linux 批量实验用 fork，避免 spawn 对 __main__ 路径的依赖
    if sys.platform == "linux":
        return mp.get_context("fork")
    return mp.get_context("spawn")


def _informed_plan_worker(env: dict, max_iterations: int, out_queue: mp.Queue):
    """子进程执行 Informed-RRT*，供墙钟超时强制终止。"""
    try:
        result = InformedRRTStar(env, max_iterations).plan()
        out_queue.put(("ok", result))
    except Exception as exc:
        out_queue.put(("error", str(exc)))


def run_informed_with_timeout(env: dict, max_iterations: int, timeout_s: float) -> dict:
    """Hard 案例：墙钟超时后 terminate 子进程，记为 planning failure。"""
    ctx = _mp_context()
    out_queue = ctx.Queue()
    proc = ctx.Process(
        target=_informed_plan_worker,
        args=(env, max_iterations, out_queue),
    )
    t0 = time.perf_counter()
    proc.start()
    proc.join(timeout=timeout_s)
    elapsed = time.perf_counter() - t0

    if proc.is_alive():
        proc.terminate()
        proc.join(timeout=5.0)
        if proc.is_alive():
            proc.kill()
            proc.join()
        return {
            "success": False,
            "path": [],
            "path_length": float("inf"),
            "planning_time": round(min(elapsed, timeout_s), 4),
            "tree_nodes": 0,
            "path_efficiency": 0.0,
            "algorithm": "Informed-RRT*",
            "failure_mode": "timeout",
            "timed_out": True,
        }

    if not out_queue.empty():
        status, payload = out_queue.get_nowait()
        if status == "ok":
            payload.setdefault(
                "failure_mode",
                "none" if payload.get("success") else "max_iterations",
            )
            payload["timed_out"] = False
            return payload
        return {
            "success": False,
            "path": [],
            "path_length": float("inf"),
            "planning_time": round(elapsed, 4),
            "tree_nodes": 0,
            "path_efficiency": 0.0,
            "algorithm": "Informed-RRT*",
            "failure_mode": "error",
            "timed_out": False,
            "error": payload,
        }

    return {
        "success": False,
        "path": [],
        "path_length": float("inf"),
        "planning_time": round(elapsed, 4),
        "tree_nodes": 0,
        "path_efficiency": 0.0,
        "algorithm": "Informed-RRT*",
        "failure_mode": "error",
        "timed_out": False,
        "error": "empty worker result",
    }
