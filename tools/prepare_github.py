#!/usr/bin/env python3
"""Prepare a separate, clean AURA repository; leave the working workspace intact."""
from collections import Counter
from pathlib import Path
import argparse
import json
import re
import shutil


PACKAGES = {"description_new", "navigation", "aura_navigation", "aura_fusion", "aura_semantic_slam"}
IMAGES = ("maze_environment.png", "six_object_environment.png", "scene_graph.png", "ekf_comparison.png")
VIDEOS = ("maze_planning.mp4", "ekf_fusion.mp4", "semantic_slam.mp4")


def keep_source(relative):
    parts = relative.parts
    if "__pycache__" in parts or relative.suffix in (".pyc", ".pyo", ".bak", ".tmp"):
        return False
    package = parts[0]
    if package not in PACKAGES:
        return False
    if package == "aura_navigation" and relative.name in ("aura_wheel_math_todo", "aura_wheel_math_todo.cpp"):
        return False
    if package == "aura_fusion" and relative.name == "wheel_sensor_todo.cpp":
        return False
    if package == "description_new":
        if len(parts) > 1 and parts[1] == "worlds":
            return len(parts) > 2 and parts[2] in ("aura_maze.sdf", "aura_mapping_demo.sdf", "aura_mapping_demo_assets")
        if len(parts) > 1 and parts[1] == "launch":
            return relative.name in ("launch_sim.launch.py", "rsp.launch.py", "launch_mapping_demo.launch.py")
        if relative.name == "description.csv" or (len(parts) > 1 and parts[1] == "description"):
            return False
    if package == "navigation" and len(parts) > 1:
        allowed = {"launch": {"aura_maze_navigation.launch.py"},
                   "config": {"aura_maze_nav.yaml", "aura_maze_amcl.yaml"},
                   "maps": {"aura_maze.pgm", "aura_maze.yaml"},
                   "rviz": {"aura_maze.rviz"}}
        if parts[1] in allowed:
            return relative.name in allowed[parts[1]]
    return True


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", required=True)
    parser.add_argument("--owner", default="Harivathsha")
    parser.add_argument("--repo", default="AURA-ROS2")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.owner) or not re.fullmatch(r"[A-Za-z0-9_.-]+", args.repo):
        raise SystemExit("Use a plain GitHub owner and repository name")
    root, dest = Path.cwd(), Path(args.destination).expanduser().resolve()
    if not (root/"src/aura_semantic_slam/package.xml").is_file():
        raise SystemExit("Run this script from the c++_project workspace root")
    if dest == root.resolve() or root.resolve() in dest.parents:
        raise SystemExit("Choose a destination outside the working workspace")
    if dest.exists() and any(dest.iterdir()):
        raise SystemExit(f"Destination is not empty: {dest}. Choose a new empty directory.")
    required = [root/"media/images"/n for n in IMAGES]+[root/"media/videos"/n for n in VIDEOS]
    required += [root/"demo_results/ekf"/n for n in ("comparison.png", "metrics.json", "raw_odometry.csv", "aligned_comparison.csv")]
    required += [root/"demo_results/semantic"/n for n in ("scene_graph.json", "scene_graph.png", "semantic_map.ply", "anchor_observations.npz")]
    required += [root/"models/yolo11s-seg.pt"]
    missing = [str(p.relative_to(root)) for p in required if not p.is_file() or p.stat().st_size == 0]
    if missing:
        raise SystemExit("Add/collect these files first:\n"+"\n".join(missing))
    for name in IMAGES:
        p = root/"media/images"/name
        with p.open("rb") as handle:
            if handle.read(8) != b"\x89PNG\r\n\x1a\n":
                raise SystemExit(f"Image must be an actual PNG: {p}")
        if p.stat().st_size >= 100*1024**2:
            raise SystemExit(f"Image exceeds GitHub's normal Git file limit: {p}")
    for name in VIDEOS:
        p = root/"media/videos"/name
        if p.stat().st_size >= 2*1024**3:
            raise SystemExit(f"Export this video below 2 GiB for the release upload: {p}")
    # A cleaned clone must not require Git to retain empty source directories.
    for name in ("description_new", "navigation"):
        text = (root/f"src/{name}/CMakeLists.txt").read_text()
        if re.search(r"^\s*src\s*$", text, re.M):
            raise SystemExit("Apply the preparation bundle's CMake fixes before staging")
    launch = (root/"src/aura_semantic_slam/launch/semantic_slam.launch.py").read_text()
    if 'LaunchConfiguration("show_rviz")' not in launch or '"rviz": "false"' not in launch:
        raise SystemExit("Apply the preparation bundle's semantic RViz fix before staging")
    sim = (root/"src/description_new/launch/launch_sim.launch.py").read_text()
    if "'aura_library.sdf'" in sim:
        raise SystemExit("Apply the preparation bundle's maze-default launch before staging")
    dest.mkdir(parents=True, exist_ok=True)
    kept, skipped = [], []
    for p in sorted((root/"src").rglob("*")):
        if not p.is_file():
            continue
        relative = p.relative_to(root/"src")
        if keep_source(relative):
            copy_file(p, dest/"src"/relative)
            kept.append(str(relative))
        else:
            skipped.append(str(relative))
    for name in ("README.md", ".gitignore", "THIRD_PARTY_NOTICES.md"):
        copy_file(root/name, dest/name)
    for folder in ("docs", "tools"):
        for p in (root/folder).rglob("*"):
            if p.is_file() and "__pycache__" not in p.parts:
                copy_file(p, dest/p.relative_to(root))
    for p in required:
        target = dest/"results"/p.relative_to(root/"demo_results") if root/"demo_results" in p.parents else dest/p.relative_to(root)
        copy_file(p, target)
    for p in (dest/"docs").glob("*.md"):
        p.write_text(p.read_text().replace("__OWNER__", args.owner).replace("__REPO__", args.repo))
    # The release files exist locally but the repository ignores MP4 and PT files.
    readme = (dest/"README.md").read_text().replace("__OWNER__", args.owner).replace("__REPO__", args.repo)
    metrics = json.loads((dest/"results/ekf/metrics.json").read_text())
    graph = json.loads((dest/"results/semantic/scene_graph.json").read_text())
    counts = Counter(o["class"] for o in graph["objects"])
    result = ["Recorded EKF run: **"+f"{metrics['duration_s']:.2f} s, {metrics['samples']} aligned samples**.",
              "", "| Estimate | Position RMSE | Heading RMSE |", "|---|---:|---:|"]
    for key, label in (("wheel", "Wheel only"), ("fused", "Wheel + gyro EKF")):
        result.append(f"| {label} | {metrics[key]['position_rmse_m']:.4f} m | {metrics[key]['heading_rmse_deg']:.3f}° |")
    result += ["", "These values describe one recorded simulation run.", "",
               "| Class | Room count | Confirmed graph count |", "|---|---:|---:|"]
    for key, expected in (("sports ball", 3), ("potted plant", 2), ("chair", 1)):
        result.append(f"| {key} | {expected} | {counts[key]} |")
    result += ["", "Object counts are predictions, not forced to match the room. Read the known limitations below."]
    readme = re.sub(r"<!-- RESULTS_START -->.*?<!-- RESULTS_END -->",
                    "<!-- RESULTS_START -->\n"+"\n".join(result)+"\n<!-- RESULTS_END -->", readme, flags=re.S)
    (dest/"README.md").write_text(readme)
    (dest/"docs/STAGING_REPORT.md").write_text("# Staged source\n\n"+f"{len(kept)} source files copied.\n\n"+
             "## Omitted working files\n\n"+"\n".join("- "+p for p in skipped)+"\n")
    print("Prepared repository:", dest)
    print("Source files:", len(kept), "omitted working files:", len(skipped))
    print("Videos and model are staged for the GitHub Release, and excluded from Git commits.")
    print("The working workspace was not deleted or moved.")


if __name__ == "__main__":
    main()
