from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def rectangle_corners(
    center_x: float,
    center_y: float,
    length_m: float,
    width_m: float,
    heading_rad: float,
) -> list[tuple[float, float]]:
    """Return counter-clockwise vehicle footprint corners in world coordinates."""
    cosine = math.cos(heading_rad)
    sine = math.sin(heading_rad)
    corners = []
    for local_x, local_y in (
        (-length_m / 2.0, -width_m / 2.0),
        (length_m / 2.0, -width_m / 2.0),
        (length_m / 2.0, width_m / 2.0),
        (-length_m / 2.0, width_m / 2.0),
    ):
        corners.append(
            (
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
            )
        )
    return corners


def _subtract(
    first: tuple[float, float],
    second: tuple[float, float],
) -> tuple[float, float]:
    return first[0] - second[0], first[1] - second[1]


def _point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    segment = _subtract(end, start)
    length_squared = segment[0] ** 2 + segment[1] ** 2
    if length_squared == 0.0:
        return math.dist(point, start)
    offset = _subtract(point, start)
    fraction = (offset[0] * segment[0] + offset[1] * segment[1]) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    projection = (start[0] + fraction * segment[0], start[1] + fraction * segment[1])
    return math.dist(point, projection)


def _polygons_overlap(
    first: list[tuple[float, float]],
    second: list[tuple[float, float]],
) -> bool:
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            edge = _subtract(end, start)
            axis = (-edge[1], edge[0])
            first_projection = [point[0] * axis[0] + point[1] * axis[1] for point in first]
            second_projection = [point[0] * axis[0] + point[1] * axis[1] for point in second]
            if (
                max(first_projection) < min(second_projection) - 1e-12
                or max(second_projection) < min(first_projection) - 1e-12
            ):
                return False
    return True


def footprint_clearance(
    first: list[tuple[float, float]],
    second: list[tuple[float, float]],
) -> float:
    """Return the Euclidean gap between two rectangles, or zero if they overlap."""
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    if _polygons_overlap(first, second):
        return 0.0

    return min(
        [
            _point_segment_distance(point, start, end)
            for point in first
            for start, end in second_edges
        ]
        + [
            _point_segment_distance(point, start, end)
            for point in second
            for start, end in first_edges
        ]
    )


def _track_rows(
    ego_track: dict[str, Any],
    sender_track: dict[str, Any],
) -> list[dict[str, Any]]:
    ego_samples = {
        round(float(timestamp) * 1000.0): (position, float(heading))
        for timestamp, position, heading in zip(
            ego_track["timestamps"],
            ego_track["positions"],
            ego_track["orientations"],
        )
    }
    sender_samples = {
        round(float(timestamp) * 1000.0): (position, float(heading))
        for timestamp, position, heading in zip(
            sender_track["timestamps"],
            sender_track["positions"],
            sender_track["orientations"],
        )
    }
    common_timestamps = sorted(set(ego_samples) & set(sender_samples))
    if not common_timestamps:
        raise ValueError("Connected ego and sender tracks have no common timestamps")

    ego_length, ego_width = (float(value) for value in ego_track["dimensions"][0][:2])
    sender_length, sender_width = (float(value) for value in sender_track["dimensions"][0][:2])
    start_time_s = common_timestamps[0] / 1000.0
    rows = []
    for timestamp_ms in common_timestamps:
        ego_position, ego_heading = ego_samples[timestamp_ms]
        sender_position, sender_heading = sender_samples[timestamp_ms]
        ego_x, ego_y = float(ego_position[0]), float(ego_position[1])
        sender_x, sender_y = float(sender_position[0]), float(sender_position[1])
        ego_corners = rectangle_corners(ego_x, ego_y, ego_length, ego_width, ego_heading)
        sender_corners = rectangle_corners(
            sender_x,
            sender_y,
            sender_length,
            sender_width,
            sender_heading,
        )
        clearance = footprint_clearance(ego_corners, sender_corners)
        rows.append(
            {
                "time_s": timestamp_ms / 1000.0 - start_time_s,
                "timestamp_unix_s": timestamp_ms / 1000.0,
                "ego_x_m": ego_x,
                "ego_y_m": ego_y,
                "ego_heading_rad": ego_heading,
                "sender_x_m": sender_x,
                "sender_y_m": sender_y,
                "sender_heading_rad": sender_heading,
                "center_distance_m": math.dist((ego_x, ego_y), (sender_x, sender_y)),
                "footprint_clearance_m": clearance,
                "footprints_overlap": int(clearance == 0.0),
                "ego_corners": ego_corners,
                "sender_corners": sender_corners,
            }
        )
    return rows


def _plot_encounter(
    rows: list[dict[str, Any]],
    ego_track: dict[str, Any],
    sender_track: dict[str, Any],
    sequence_name: str,
    output_path: Path,
    caution_distance_m: float,
) -> None:
    minimum = min(rows, key=lambda row: row["footprint_clearance_m"])
    minimum_time = minimum["time_s"]
    fig, (map_axis, distance_axis) = plt.subplots(
        2,
        1,
        figsize=(11, 10),
        gridspec_kw={"height_ratios": [1.3, 1.0]},
        constrained_layout=True,
    )

    map_axis.plot(
        [row["ego_x_m"] for row in rows],
        [row["ego_y_m"] for row in rows],
        color="#087E8B",
        linewidth=2.3,
        label=f"vehicle1 / Ego, Track {ego_track['track_id']}",
    )
    map_axis.plot(
        [row["sender_x_m"] for row in rows],
        [row["sender_y_m"] for row in rows],
        color="#F28C28",
        linewidth=2.3,
        label=f"vehicle2 / Sender, Track {sender_track['track_id']}",
    )
    for row in (rows[0], minimum, rows[-1]):
        map_axis.add_patch(
            Polygon(
                row["ego_corners"],
                closed=True,
                facecolor="#087E8B",
                edgecolor="#07545c",
                alpha=0.35 if row is minimum else 0.12,
                linewidth=1.2,
            )
        )
        map_axis.add_patch(
            Polygon(
                row["sender_corners"],
                closed=True,
                facecolor="#F28C28",
                edgecolor="#a95405",
                alpha=0.35 if row is minimum else 0.12,
                linewidth=1.2,
            )
        )
    map_axis.scatter(
        [minimum["ego_x_m"], minimum["sender_x_m"]],
        [minimum["ego_y_m"], minimum["sender_y_m"]],
        color=["#087E8B", "#F28C28"],
        edgecolors="white",
        linewidths=0.8,
        zorder=4,
    )
    map_axis.set_title(
        f"{sequence_name}: vehicle footprints at minimum clearance "
        f"(t = {minimum_time:.1f} s)"
    )
    map_axis.set_xlabel("x [m]")
    map_axis.set_ylabel("y [m]")
    map_axis.set_aspect("equal", adjustable="datalim")
    map_axis.grid(True, alpha=0.25)
    map_axis.legend(loc="best")

    times = [row["time_s"] for row in rows]
    center_distances = [row["center_distance_m"] for row in rows]
    clearances = [row["footprint_clearance_m"] for row in rows]
    distance_axis.plot(times, center_distances, color="#8D99AE", linewidth=1.6, label="center-to-center")
    distance_axis.plot(times, clearances, color="#C1292E", linewidth=2.2, label="body footprint clearance")
    distance_axis.axhline(caution_distance_m, color="#F28C28", linestyle="--", linewidth=1.3, label=f"reference: {caution_distance_m:g} m")
    distance_axis.axvline(minimum_time, color="#22223B", linestyle=":", linewidth=1.2)
    distance_axis.scatter(
        [minimum_time],
        [minimum["footprint_clearance_m"]],
        color="#C1292E",
        edgecolors="white",
        zorder=5,
    )
    distance_axis.set_title(
        f"Minimum footprint clearance: {minimum['footprint_clearance_m']:.2f} m"
        + (" (overlap in 2D)" if minimum["footprints_overlap"] else "")
    )
    distance_axis.set_xlabel("time since sequence start [s]")
    distance_axis.set_ylabel("distance [m]")
    distance_axis.grid(True, alpha=0.25)
    distance_axis.legend(loc="best")

    fig.suptitle("UrbanING connected-vehicle encounter check", fontsize=15)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def analyze_encounter(
    root_folder: str | Path,
    sequence_name: str,
    output_folder: str | Path,
    caution_distance_m: float = 2.0,
) -> dict[str, Any]:
    root = Path(root_folder)
    output = Path(output_folder)
    labels = json.loads((root / "labels" / f"{sequence_name}.json").read_text())
    connected = json.loads((root / "labels_av_track_ids.json").read_text())
    if sequence_name not in connected:
        raise ValueError(f"No connected-vehicle mapping for {sequence_name}")

    tracks = {int(track["track_id"]): track for track in labels["tracks"]}
    ego_id = int(connected[sequence_name]["vehicle1"])
    sender_id = int(connected[sequence_name]["vehicle2"])
    ego_track = tracks[ego_id]
    sender_track = tracks[sender_id]
    rows = _track_rows(ego_track, sender_track)

    minimum = min(rows, key=lambda row: row["footprint_clearance_m"])
    overlap_rows = [row for row in rows if row["footprints_overlap"]]
    summary = {
        "sequence": sequence_name,
        "ego_track_id": ego_id,
        "sender_track_id": sender_id,
        "ticks": len(rows),
        "duration_s": rows[-1]["time_s"],
        "minimum_center_distance_m": minimum["center_distance_m"],
        "minimum_footprint_clearance_m": minimum["footprint_clearance_m"],
        "minimum_clearance_time_s": minimum["time_s"],
        "footprint_overlap_ticks": len(overlap_rows),
        "caution_distance_m": caution_distance_m,
        "ticks_below_caution_distance": sum(
            row["footprint_clearance_m"] < caution_distance_m for row in rows
        ),
        "geometry_assumption": (
            "2D rectangles centered at annotated positions, rotated by annotated yaw; "
            "height and suspension are ignored."
        ),
        "caution_distance_is_safety_claim": False,
    }

    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / f"{sequence_name}_footprint_clearance.csv"
    with csv_path.open("w", newline="") as csv_file:
        fields = [key for key in rows[0] if key not in {"ego_corners", "sender_corners"}]
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row[key] for key in fields} for row in rows)

    summary_path = output / f"{sequence_name}_encounter_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    plot_path = output / f"{sequence_name}_encounter.png"
    _plot_encounter(
        rows,
        ego_track,
        sender_track,
        sequence_name,
        plot_path,
        caution_distance_m,
    )
    print(f"Minimum center distance: {minimum['center_distance_m']:.2f} m")
    print(f"Minimum body footprint clearance: {minimum['footprint_clearance_m']:.2f} m")
    print(f"2D overlap ticks: {len(overlap_rows)} / {len(rows)}")
    print(f"Wrote {plot_path}, {csv_path}, and {summary_path}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize connected vehicle tracks and calculate 2D body clearance."
    )
    parser.add_argument("--root", default="datasets/UrbanIng-V2X")
    parser.add_argument("--sequence", default="20241126_0022_crossing1_09")
    parser.add_argument("--output", default="sumo_input/encounter")
    parser.add_argument("--caution-distance", type=float, default=2.0)
    args = parser.parse_args()
    analyze_encounter(
        args.root,
        args.sequence,
        args.output,
        caution_distance_m=args.caution_distance,
    )


if __name__ == "__main__":
    main()