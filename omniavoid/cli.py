from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import sys
import time
import cv2
import numpy as np
from .config import load_config, live_transmit_errors
from .dataset import generate_demo, read_dataset
from .runtime import geometric_coverage, process_images, record, replay, run_live
from .sectors import advisory_speed_mps
from .stereo import StereoPipeline


def main(argv=None):
    parser = argparse.ArgumentParser(description="Four-fisheye stereo research baseline; defaults to no flight-controller transmission")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Render a synthetic room and run the complete offline pipeline")
    demo.add_argument("--config", default="configs/rig.example.json")
    demo.add_argument("--output", default="examples/synthetic_demo")
    replay_cmd = commands.add_parser("replay", help="Offline images to JSONL, never transmits")
    replay_cmd.add_argument("--config", required=True)
    replay_cmd.add_argument("--manifest", required=True)
    replay_cmd.add_argument("--output", required=True)
    replay_cmd.add_argument("--limit", type=int, default=0)
    replay_cmd.add_argument("--html")
    live = commands.add_parser("run", help="Live Linux V4L2 processing")
    live.add_argument("--config", required=True)
    live.add_argument("--output", default="logs/live_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    live.add_argument("--duration", type=float, default=0)
    live.add_argument("--transmit", action="store_true", help="Enable MAVLink only after physical verification gates pass")
    rec = commands.add_parser("record", help="Record timestamp-matched frames without flight-controller output")
    rec.add_argument("--config", required=True)
    rec.add_argument("--output", required=True)
    rec.add_argument("--samples", type=int, default=100)
    rec.add_argument("--interval", type=float, default=0.3)
    rec.add_argument("--timeout", type=float, default=120)
    board = commands.add_parser("make-board", help="Create printable, uniquely identified ArUco grid")
    board.add_argument("--output", default="calibration_board")
    calibration = commands.add_parser("calibrate", help="Estimate full-angle KB4 intrinsics and rig extrinsics; outputs an unvalidated candidate")
    calibration.add_argument("--config", required=True)
    calibration.add_argument("--manifest", required=True)
    calibration.add_argument("--output", required=True)
    calibration.add_argument("--min-views", type=int, default=12)
    inspect = commands.add_parser("inspect", help="Validate config and show optical overlap/verification gates")
    inspect.add_argument("--config", required=True)
    bench = commands.add_parser("benchmark", help="Static-image processing microbenchmark on this machine")
    bench.add_argument("--config", required=True)
    bench.add_argument("--manifest", required=True)
    bench.add_argument("--iterations", type=int, default=20)
    bench.add_argument("--output", required=True)
    speed = commands.add_parser("speed-estimate", help="Advisory static-obstacle braking calculation, no control output")
    for name in ("distance", "delay", "deceleration", "margin"):
        speed.add_argument("--" + name, type=float, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "make-board":
            from .calibration import write_board
            print(json.dumps(write_board(args.output), indent=2))
            return 0
        if args.command == "speed-estimate":
            print(json.dumps({"advisory_upper_bound_mps": advisory_speed_mps(args.distance, args.delay, args.deceleration, args.margin),
                              "not_a_flight_speed_rating": True}))
            return 0
        cfg = load_config(args.config)
        if args.command == "demo":
            cfg, manifest = generate_demo(cfg, args.output)
            output = Path(args.output)
            result = replay(cfg, manifest, output / "results.jsonl", html_path=output / "report.html", cloud_path=output / "cloud.npz")
            pipeline = StereoPipeline(cfg)
            try:
                _, images = next(read_dataset(manifest))
                for i, pair in enumerate(pipeline.pairs):
                    for side, image in zip(("left", "right"), pair.rectify(images)):
                        cv2.imwrite(str(output / f"pair{i}_{side}.png"), image)
            finally:
                pipeline.close()
            print(json.dumps({"output": str(output), "valid_sectors": result["valid_sectors"], "processing_ms": result["processing_ms"]}, indent=2))
        elif args.command == "replay":
            result = replay(cfg, args.manifest, args.output, args.limit, args.html)
            print(json.dumps({"output": args.output, "first_frame_valid_sectors": result["valid_sectors"]}))
        elif args.command == "run":
            run_live(cfg, args.output, args.transmit, args.duration)
        elif args.command == "record":
            print(record(cfg, args.output, args.samples, args.interval, args.timeout))
        elif args.command == "calibrate":
            from .calibration import calibrate_dataset
            print(json.dumps(calibrate_dataset(cfg, args.manifest, args.output, args.min_views), indent=2))
        elif args.command == "inspect":
            print(json.dumps({"python": sys.version.split()[0], "opencv": cv2.__version__, "platform": platform.platform(),
                              "overlap": [geometric_coverage(cfg, d) for d in (0.6, 1.0, 3.0, 6.0)],
                              "transmit_blockers": live_transmit_errors(cfg, "v4l2"),
                              "capture_note": "Configure frame rate, exposure, trigger and ISP routing using the board BSP; capture.fps is metadata, not sensor programming."}, indent=2))
        elif args.command == "benchmark":
            if args.iterations < 2:
                raise ValueError("Use at least two benchmark iterations")
            _, images = next(read_dataset(args.manifest))
            pipeline = StereoPipeline(cfg)
            timings = []
            try:
                process_images(pipeline, cfg, images)
                for _ in range(args.iterations):
                    result, _, _ = process_images(pipeline, cfg, images)
                    timings.append(result["processing_ms"])
            finally:
                pipeline.close()
            stats = {"iterations": args.iterations, "median_ms": float(np.median(timings)),
                     "p95_ms": float(np.percentile(timings, 95)), "max_ms": max(timings),
                     "platform": platform.platform(), "machine": platform.machine(), "opencv": cv2.__version__,
                     "valid_sectors": result["valid_sectors"], "timings_ms": timings,
                     "note": "Static synthetic/replay image compute only; excludes capture, real scene changes, link, controller and braking. Not an RK3588 measurement unless run there."}
            Path(args.output).write_text(json.dumps(stats, indent=2), encoding="utf-8")
            print(json.dumps({k: v for k, v in stats.items() if k != "timings_ms"}, indent=2))
        return 0
    except KeyboardInterrupt:
        print("Stopped", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
