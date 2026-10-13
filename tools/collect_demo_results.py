#!/usr/bin/env python3
"""Collect finished AURA demo outputs and copy the two generated PNGs."""
from pathlib import Path
import argparse
import json
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("demo", choices=("ekf", "semantic"))
    args = parser.parse_args()
    root = Path.cwd()
    images = root/"media/images"
    images.mkdir(parents=True, exist_ok=True)
    if args.demo == "ekf":
        folder = root/"demo_results/ekf"
        files = ("comparison.png", "metrics.json", "raw_odometry.csv", "aligned_comparison.csv")
        missing = [name for name in files if not (folder/name).is_file()]
        if missing:
            raise SystemExit("Finish the EKF recorder first; missing: "+", ".join(missing))
        shutil.copy2(folder/"comparison.png", images/"ekf_comparison.png")
        metrics = json.loads((folder/"metrics.json").read_text())
        print("EKF image:", images/"ekf_comparison.png")
        print("Samples:", metrics["samples"], "duration_s:", metrics["duration_s"])
    else:
        candidates = list((root/"semantic_slam_results").glob("aura_*/scene_graph.json"))
        if not candidates:
            raise SystemExit("Save the semantic map service first; no scene_graph.json found")
        latest = max(candidates, key=lambda p: p.stat().st_mtime_ns).parent
        folder = root/"demo_results/semantic"
        folder.mkdir(parents=True, exist_ok=True)
        files = ("scene_graph.json", "scene_graph.png", "semantic_map.ply", "anchor_observations.npz")
        missing = [name for name in files if not (latest/name).is_file()]
        if missing:
            raise SystemExit("Semantic snapshot is incomplete: "+", ".join(missing))
        for name in files:
            shutil.copy2(latest/name, folder/name)
        shutil.copy2(latest/"scene_graph.png", images/"scene_graph.png")
        graph = json.loads((folder/"scene_graph.json").read_text())
        counts = {}
        for obj in graph["objects"]:
            counts[obj["class"]] = counts.get(obj["class"], 0)+1
        print("Copied snapshot:", latest)
        print("Detected class counts:", counts)
        print("Graph image:", images/"scene_graph.png")


if __name__ == "__main__":
    main()
