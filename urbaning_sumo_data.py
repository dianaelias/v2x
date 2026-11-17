"""Export UrbanING vehicle poses and Lanelet2 boundaries as readable files."""

from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from pyproj import Transformer

from urbaning.data.info import gps_origins


def load_vehicle_trajectory(
    root_folder: str | Path,
    sequence_name: str,
    vehicle_name: str,
) -> list[dict[str, float]]:
    """Read one vehicle's poses from its timestamped state JSON files."""
    if vehicle_name not in {"vehicle1", "vehicle2"}:
        raise ValueError("vehicle_name must be 'vehicle1' or 'vehicle2'")

    state_folder = Path(root_folder) / "dataset" / sequence_name / f"{vehicle_name}_state"
    state_files = sorted(state_folder.glob("*.json"), key=lambda path: int(path.stem))
    if not state_files:
        raise FileNotFoundError(f"No state JSON files found in {state_folder}")

    start_ms = int(state_files[0].stem)
    trajectory = []
    for state_file in state_files:
        timestamp_ms = int(state_file.stem)
        state = json.loads(state_file.read_text())
        transform = state["gTv"]
        if len(transform) != 4 or any(len(row) != 4 for row in transform):
            raise ValueError(f"Expected a 4x4 gTv matrix in {state_file}")

        # The upper-left 2x2 part of the rigid transform stores planar rotation.
        heading_rad = math.atan2(transform[1][0], transform[0][0])
        trajectory.append(
            {
                "timestamp_ms": timestamp_ms,
                # Use elapsed time so each exported sequence starts at t = 0.
                "time_s": (timestamp_ms - start_ms) / 1000.0,
                "x_m": float(transform[0][3]),
                "y_m": float(transform[1][3]),
                "z_m": float(transform[2][3]),
                "heading_rad": heading_rad,
                "sumo_angle_deg": (90.0 - math.degrees(heading_rad)) % 360.0,
            }
        )

    return trajectory


def write_trajectory_csv(
    trajectory: list[dict[str, float]],
    output_path: str | Path,
) -> Path:
    """Write trajectory rows to CSV and return the output path."""
    if not trajectory:
        raise ValueError("trajectory is empty")

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(trajectory[0]))
        writer.writeheader()
        writer.writerows(trajectory)
    return output_path


def load_lanelet_map(osm_path: str | Path) -> dict[str, Any]:
    """Read Lanelet2 lane boundaries as local UTM32N coordinates in metres."""
    origin_lat, origin_lon, _ = gps_origins["crossing1"]
    project = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    origin_x, origin_y = project.transform(origin_lon, origin_lat)

    osm_root = ET.parse(osm_path).getroot()
    nodes: dict[str, tuple[float, float]] = {}
    for node in osm_root.findall("node"):
        if "lat" not in node.attrib or "lon" not in node.attrib:
            continue
        # Put map and vehicle coordinates into the same local metre frame.
        x, y = project.transform(float(node.attrib["lon"]), float(node.attrib["lat"]))
        nodes[node.attrib["id"]] = (x - origin_x, y - origin_y)

    ways: dict[str, list[tuple[float, float]]] = {}
    for way in osm_root.findall("way"):
        ways[way.attrib["id"]] = [
            nodes[nd.attrib["ref"]]
            for nd in way.findall("nd")
            if nd.attrib["ref"] in nodes
        ]

    lanelets = []
    for relation in osm_root.findall("relation"):
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue

        # Lanelet relations reference their left/right boundary ways by ID.
        boundary_ids = {
            member.attrib["role"]: member.attrib["ref"]
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
            and member.attrib.get("role") in {"left", "right"}
        }
        if "left" not in boundary_ids or "right" not in boundary_ids:
            continue

        lanelets.append(
            {
                "id": relation.attrib["id"],
                "tags": tags,
                "left_boundary": ways.get(boundary_ids["left"], []),
                "right_boundary": ways.get(boundary_ids["right"], []),
            }
        )

    return {
        "coordinate_frame": "UTM zone 32N, metres relative to the crossing1 GPS origin",
        "lanelets": lanelets,
    }


def main() -> None:
    """Export both vehicle trajectories and the shared Lanelet2 map."""
    parser = argparse.ArgumentParser(
        description="Export UrbanIng-V2X vehicle trajectories and Lanelet2 map data."
    )
    parser.add_argument("--root", default="datasets/UrbanIng-V2X")
    parser.add_argument("--sequence", default="20241126_0017_crossing1_00")
    parser.add_argument("--output", default="sumo_input")
    args = parser.parse_args()

    root = Path(args.root)
    output = Path(args.output)
    for vehicle_name in ("vehicle1", "vehicle2"):
        rows = load_vehicle_trajectory(root, args.sequence, vehicle_name)
        path = write_trajectory_csv(
            rows,
            output / f"{args.sequence}_{vehicle_name}_trajectory.csv",
        )
        print(f"{vehicle_name}: exported {len(rows)} poses to {path}")

    lanelet_map = load_lanelet_map(root / "crossings_lanelet2map.osm")
    map_path = output / "lanelet_map.json"
    map_path.parent.mkdir(parents=True, exist_ok=True)
    map_path.write_text(json.dumps(lanelet_map, indent=2))
    print(f"map: exported {len(lanelet_map['lanelets'])} lanelets to {map_path}")


if __name__ == "__main__":
    main()