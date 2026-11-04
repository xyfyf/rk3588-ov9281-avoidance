"""Summarize measured processing/age/coverage from live or replay JSONL."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in Path(args.log).read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError("Log is empty")
    def stats(key):
        values = np.array([r[key] for r in rows if key in r], dtype=float)
        return None if len(values) == 0 else {"p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)), "p99": float(np.percentile(values, 99)), "max": float(values.max())}
    distances = np.array([r["distances_cm"] for r in rows])
    report = {"frames": len(rows), "processing_ms": stats("processing_ms"), "age_ms": stats("age_ms"),
              "skew_ms": stats("skew_ms"), "valid_sectors": stats("valid_sectors"),
              "unknown_fraction_by_sector": np.mean(distances == 65535, axis=0).tolist(),
              "health_counts": {h: sum(r.get("health") == h for r in rows) for h in sorted({r.get("health", "unspecified") for r in rows})},
              "note": "Coverage is availability of depth outputs, not probability of detecting an obstacle. No flight speed rating is inferred."}
    Path(args.output).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
