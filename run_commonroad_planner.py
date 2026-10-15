from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
from pathlib import Path
from typing import Any

from pyproj import Transformer

from urbaning.data.info import gps_origins
from urbaning_cam import _track_kinematics, track_sample_to_cam


IMAGE_NAME = "urbaning-commonroad-planner:2025.1"


def _sample_at_or_before(
    samples: list[dict[str, float]],
    timestamp_s: float,
) -> dict[str, float]:
    candidates = [sample for sample in samples if sample["timestamp_unix_s"] <= timestamp_s + 1e-9]
    if not candidates:
        raise ValueError(f"No annotation sample exists at or before {timestamp_s}")
    return candidates[-1]


def _decode_cam_to_local_state(
    cam_record: dict[str, Any],
    map_origin_xy: tuple[float, float],
) -> dict[str, Any]:
    parameters = cam_record["cam"]["cam"]["camParameters"]
    reference = parameters["basicContainer"]["referencePosition"]
    high_frequency = parameters["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    east, north = to_utm.transform(reference["longitude_deg"], reference["latitude_deg"])
    x_front = east - map_origin_xy[0]
    y_front = north - map_origin_xy[1]
    heading = math.radians(90.0 - high_frequency["heading"]["headingValue_deg"])
    length_m = high_frequency["vehicleLength"]["vehicleLengthValue_m"]
    center_x = x_front - math.cos(heading) * length_m / 2.0
    center_y = y_front - math.sin(heading) * length_m / 2.0
    speed = high_frequency["speed"]["speedValue_mps"]
    return {
        "center_x_m": center_x,
        "center_y_m": center_y,
        "heading_rad": heading,
        "speed_mps": speed,
        "length_m": length_m,
        "width_m": high_frequency["vehicleWidth"]["vehicleWidthValue_m"],
        "source_track_id": cam_record["metadata"]["source_track_id"],
        "generation_time_unix_s": cam_record["metadata"]["timestamp_unix_s"],
    }


def prepare_case(
    root_folder: str | Path,
    sequence_name: str,
    planning_time_s: float,
    delay_ms: float,
    output_folder: str | Path,
) -> tuple[Path, Path]:
    root = Path(root_folder)
    output = Path(output_folder)
    output.mkdir(parents=True, exist_ok=True)
    labels = json.loads((root / "labels" / f"{sequence_name}.json").read_text())
    connected = json.loads((root / "labels_av_track_ids.json").read_text())[sequence_name]
    av_vehicle_data = json.loads((root / "av_vehicle_data.json").read_text())
    tracks = {int(track["track_id"]): track for track in labels["tracks"]}
    ego_track = tracks[int(connected["vehicle1"])]
    sender_track = tracks[int(connected["vehicle2"])]
    ego_samples = _track_kinematics(ego_track)
    sender_samples = _track_kinematics(sender_track)

    sequence_start_s = ego_samples[0]["timestamp_unix_s"]
    target_time_s = sequence_start_s + planning_time_s
    ego_sample = _sample_at_or_before(ego_samples, target_time_s)
    sender_sample = _sample_at_or_before(sender_samples, ego_sample["timestamp_unix_s"])
    delayed_sender_sample = _sample_at_or_before(
        sender_samples,
        ego_sample["timestamp_unix_s"] - delay_ms / 1000.0,
    )
    ego_time = ego_sample["timestamp_unix_s"]

    origin_latitude, origin_longitude, _ = gps_origins["crossing1"]
    project = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    origin_east, origin_north = project.transform(origin_longitude, origin_latitude)

    role_tracks = {
        "vehicle1": ego_track,
        "vehicle2": sender_track,
    }
    snapshots: dict[str, Any] = {}
    for variant, sample in (
        ("A_no_v2x", None),
        ("B_ideal", sender_sample),
        ("C_latency_test", delayed_sender_sample),
    ):
        if sample is None:
            snapshots[variant] = None
            continue
        record = track_sample_to_cam(sender_track, sequence_name, sample)
        decoded_state = _decode_cam_to_local_state(record, (origin_east, origin_north))
        decoded_state["received_age_s"] = max(0.0, ego_time - sample["timestamp_unix_s"])
        snapshots[variant] = {"cam": record["cam"], "decoded_state": decoded_state}

    reference = [
        {
            "time_from_start_s": sample["timestamp_unix_s"] - ego_time,
            "center_x_m": sample["x_center_m"],
            "center_y_m": sample["y_center_m"],
            "heading_rad": sample["heading_rad"],
        }
        for sample in ego_samples
        if sample["timestamp_unix_s"] >= ego_time - 1e-9
    ]
    truth_sender = [
        {
            "time_from_start_s": sample["timestamp_unix_s"] - ego_time,
            "center_x_m": sample["x_center_m"],
            "center_y_m": sample["y_center_m"],
            "heading_rad": sample["heading_rad"],
        }
        for sample in sender_samples
        if sample["timestamp_unix_s"] >= ego_time - 1e-9
    ]

    ego_parameters = av_vehicle_data["vehicle1"]
    sender_parameters = av_vehicle_data["vehicle2"]
    ego_state = {
        "center_x_m": ego_sample["x_center_m"],
        "center_y_m": ego_sample["y_center_m"],
        "heading_rad": ego_sample["heading_rad"],
        "speed_mps": ego_sample["speed_mps"],
        "acceleration_mps2": ego_sample["longitudinal_acceleration_mps2"],
        "yaw_rate_rad_s": ego_sample["yaw_rate_rad_s"],
    }
    planner_input = {
        "sequence": sequence_name,
        "ego_track_id": int(connected["vehicle1"]),
        "sender_track_id": int(connected["vehicle2"]),
        "plan_time_s": planning_time_s,
        "absolute_plan_time_unix_s": ego_time,
        "time_step_s": 0.1,
        "planning_horizon_s": 4.0,
        "ego_state": ego_state,
        "ego_dimensions": {
            "length_m": float(ego_parameters["size"][0]),
            "width_m": float(ego_parameters["size"][1]),
            "center_to_rear_axle_m": float(ego_parameters["dx_center_rearaxle"]),
            "front_to_rear_axle_m": float(ego_parameters["dx_frontaxle_rearaxle"]),
        },
        "sender_dimensions": {
            "length_m": float(sender_parameters["size"][0]),
            "width_m": float(sender_parameters["size"][1]),
        },
        "reference_route": reference,
        "received_cam_by_variant": snapshots,
        "model_assumptions": [
            "The recorded ego route is supplied as a fixed common reference route in A/B/C.",
            "The planner only receives the selected CAM snapshot in B/C; it never receives future sender ground truth.",
            "The sender prediction used for collision checking is constant-velocity from the decoded CAM snapshot.",
            "The lanelet is a synthetic corridor built around the recorded ego reference, not the UrbanING HD map.",
        ],
    }
    evaluator_truth = {
        "sender_ground_truth_future": truth_sender,
        "ego_ground_truth_future": reference,
    }

    input_path = output / "planner_input.json"
    truth_path = output / "evaluation_ground_truth.json"
    input_path.write_text(json.dumps(planner_input, indent=2))
    truth_path.write_text(json.dumps(evaluator_truth, indent=2))
    return input_path, truth_path


def run_container(
    project_root: Path,
    input_path: Path,
    truth_path: Path,
    output_folder: Path,
    build_image: bool,
) -> None:
    if shutil.which("docker") is None:
        raise RuntimeError("Docker is required; install/start Docker Desktop and retry")

    if build_image:
        subprocess.run(
            [
                "docker",
                "build",
                "--platform=linux/amd64",
                "-f",
                "Dockerfile.commonroad",
                "-t",
                IMAGE_NAME,
                ".",
            ],
            cwd=project_root,
            check=True,
        )

    project_mount = f"{project_root.resolve()}:/workspace"
    subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--platform=linux/amd64",
            "-v",
            project_mount,
            "-w",
            "/workspace",
            IMAGE_NAME,
            "--input",
            f"/workspace/{input_path.relative_to(project_root)}",
            "--ground-truth",
            f"/workspace/{truth_path.relative_to(project_root)}",
            "--output",
            f"/workspace/{output_folder.relative_to(project_root)}/planner_results.json",
        ],
        cwd=project_root,
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run one CommonRoad replanning step for A/B/C CAM inputs via Docker."
    )
    parser.add_argument("--root", default="datasets/UrbanIng-V2X")
    parser.add_argument("--sequence", default="20241126_0022_crossing1_09")
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--delay-ms", type=float, default=200.0)
    parser.add_argument("--output", default="sumo_input/commonroad_planner")
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent
    output = Path(args.output)
    if not output.is_absolute():
        output = project_root / output
    input_path, truth_path = prepare_case(
        args.root,
        args.sequence,
        args.planning_time,
        args.delay_ms,
        output,
    )
    run_container(
        project_root,
        input_path,
        truth_path,
        output,
        build_image=not args.skip_build,
    )


if __name__ == "__main__":
    main()