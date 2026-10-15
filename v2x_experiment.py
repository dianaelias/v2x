from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import random
from pathlib import Path
from typing import Any

from urbaning_cam import _track_kinematics, track_sample_to_cam


def _load_connected_tracks(
    root: Path,
    sequence_name: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, int]]:
    labels = json.loads((root / "labels" / f"{sequence_name}.json").read_text())
    connected_mapping = json.loads((root / "labels_av_track_ids.json").read_text())
    if sequence_name not in connected_mapping:
        raise ValueError(f"No connected-vehicle mapping for {sequence_name}")

    tracks_by_id = {int(track["track_id"]): track for track in labels["tracks"]}
    track_ids = {
        vehicle_name: int(track_id)
        for vehicle_name, track_id in connected_mapping[sequence_name].items()
    }
    missing = [track_id for track_id in track_ids.values() if track_id not in tracks_by_id]
    if missing:
        raise ValueError(f"Connected track IDs missing from annotations: {missing}")

    ego_track = tracks_by_id[track_ids["vehicle1"]]
    sender_track = tracks_by_id[track_ids["vehicle2"]]
    return ego_track, sender_track, track_ids


def _front_reference_positions(
    track: dict[str, Any],
) -> tuple[dict[int, tuple[float, float]], dict[int, dict[str, float]]]:
    length_m = float(track["dimensions"][0][0])
    samples = _track_kinematics(track)
    positions: dict[int, tuple[float, float]] = {}
    samples_by_timestamp: dict[int, dict[str, float]] = {}
    for sample in samples:
        timestamp_ms = round(sample["timestamp_unix_s"] * 1000.0)
        heading = sample["heading_rad"]
        positions[timestamp_ms] = (
            sample["x_center_m"] + math.cos(heading) * length_m / 2.0,
            sample["y_center_m"] + math.sin(heading) * length_m / 2.0,
        )
        samples_by_timestamp[timestamp_ms] = sample
    return positions, samples_by_timestamp


def _new_limited_random(seed: int, variant_name: str) -> random.Random:
    variant_offset = sum((index + 1) * ord(char) for index, char in enumerate(variant_name))
    return random.Random(seed + variant_offset)


def _run_variant(
    variant_name: str,
    sequence_name: str,
    ego_track: dict[str, Any],
    sender_track: dict[str, Any],
    ego_track_id: int,
    sender_track_id: int,
    delay_ms: float,
    packet_loss_probability: float,
    seed: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    is_baseline = variant_name == "A_no_v2x"
    is_ideal = variant_name == "B_ideal"
    effective_delay_s = 0.0 if is_baseline or is_ideal else delay_ms / 1000.0
    effective_loss_probability = 0.0 if is_baseline or is_ideal else packet_loss_probability

    ego_samples = _track_kinematics(ego_track)
    sender_reference_positions, sender_samples = _front_reference_positions(sender_track)
    ego_by_timestamp = {
        round(sample["timestamp_unix_s"] * 1000.0): sample
        for sample in ego_samples
    }
    common_timestamps = sorted(set(ego_by_timestamp) & set(sender_samples))
    if not common_timestamps:
        raise ValueError("Connected ego and sender tracks have no common timestamps")

    rng = _new_limited_random(seed, variant_name)
    delivery_queue: list[tuple[float, int, dict[str, Any]]] = []
    latest_ldm_entry: dict[str, Any] | None = None
    rows: list[dict[str, Any]] = []
    generated_count = 0
    delivered_count = 0
    dropped_count = 0

    start_time_s = common_timestamps[0] / 1000.0
    for sequence_number, timestamp_ms in enumerate(common_timestamps):
        timestamp_s = timestamp_ms / 1000.0
        simulation_time_s = timestamp_s - start_time_s
        ego_sample = ego_by_timestamp[timestamp_ms]

        if not is_baseline:
            sender_sample = sender_samples[timestamp_ms]
            sender_cam = track_sample_to_cam(sender_track, sequence_name, sender_sample)
            generated_count += 1
            is_dropped = rng.random() < effective_loss_probability
            if is_dropped:
                dropped_count += 1
            else:
                receive_time_s = simulation_time_s + effective_delay_s
                heapq.heappush(
                    delivery_queue,
                    (receive_time_s, sequence_number, sender_cam),
                )

        delivered_this_tick = 0
        while delivery_queue and delivery_queue[0][0] <= simulation_time_s + 1e-9:
            receive_time_s, _, received_cam = heapq.heappop(delivery_queue)
            latest_ldm_entry = {
                "cam": received_cam,
                "receive_time_s": receive_time_s,
            }
            delivered_count += 1
            delivered_this_tick += 1

        if latest_ldm_entry is None:
            message_age_s = None
            sender_position_error_m = None
            last_generated_time_s = None
            last_received_time_s = None
        else:
            cam_message = latest_ldm_entry["cam"]
            last_generated_time_s = cam_message["metadata"]["timestamp_unix_s"]
            last_received_time_s = latest_ldm_entry["receive_time_s"]
            message_age_s = max(0.0, timestamp_s - last_generated_time_s)
            generated_timestamp_ms = round(last_generated_time_s * 1000.0)
            transmitted_position = sender_reference_positions[generated_timestamp_ms]
            current_position = sender_reference_positions[timestamp_ms]
            sender_position_error_m = math.dist(transmitted_position, current_position)

        rows.append(
            {
                "variant": variant_name,
                "sequence": sequence_name,
                "time_s": round(simulation_time_s, 3),
                "ego_track_id": ego_track_id,
                "ego_x_m": ego_sample["x_center_m"],
                "ego_y_m": ego_sample["y_center_m"],
                "sender_track_id": sender_track_id,
                "cam_generated": int(not is_baseline),
                "cam_delivered_this_tick": delivered_this_tick,
                "ldm_has_sender": int(latest_ldm_entry is not None),
                "last_cam_generated_time_unix_s": last_generated_time_s,
                "last_cam_received_time_s": last_received_time_s,
                "age_of_information_s": message_age_s,
                "sender_reference_position_error_m": sender_position_error_m,
            }
        )

    aoi_values = [row["age_of_information_s"] for row in rows if row["age_of_information_s"] is not None]
    position_errors = [
        row["sender_reference_position_error_m"]
        for row in rows
        if row["sender_reference_position_error_m"] is not None
    ]
    tick_count = len(rows)
    summary = {
        "variant": variant_name,
        "sequence": sequence_name,
        "ego_track_id": ego_track_id,
        "sender_track_id": sender_track_id,
        "ticks": tick_count,
        "duration_s": round(rows[-1]["time_s"], 3) if rows else 0.0,
        "configured_delay_ms": effective_delay_s * 1000.0,
        "configured_packet_loss_probability": effective_loss_probability,
        "generated_cam_count": generated_count,
        "delivered_cam_count": delivered_count,
        "dropped_cam_count": dropped_count,
        "in_flight_at_end_count": len(delivery_queue),
        "in_window_delivery_ratio": delivered_count / generated_count if generated_count else None,
        "eventual_delivery_ratio": (
            (generated_count - dropped_count) / generated_count if generated_count else None
        ),
        "ldm_availability_ratio": sum(row["ldm_has_sender"] for row in rows) / tick_count,
        "mean_aoi_s": sum(aoi_values) / len(aoi_values) if aoi_values else None,
        "max_aoi_s": max(aoi_values) if aoi_values else None,
        "mean_sender_position_error_m": (
            sum(position_errors) / len(position_errors) if position_errors else None
        ),
        "max_sender_position_error_m": max(position_errors) if position_errors else None,
        "planner_metrics": None,
        "planner_note": "No trajectory planner is connected in this experiment scaffold.",
    }
    return rows, summary


def run_experiment(
    root_folder: str | Path,
    sequence_name: str,
    output_folder: str | Path,
    delay_ms: float = 200.0,
    packet_loss_probability: float = 0.0,
    seed: int = 7,
) -> dict[str, Any]:
    if delay_ms < 0:
        raise ValueError("delay_ms must be non-negative")
    if not 0.0 <= packet_loss_probability <= 1.0:
        raise ValueError("packet_loss_probability must be between 0 and 1")

    root = Path(root_folder)
    output = Path(output_folder)
    output.mkdir(parents=True, exist_ok=True)
    ego_track, sender_track, track_ids = _load_connected_tracks(root, sequence_name)

    variants = (
        "A_no_v2x",
        "B_ideal",
        "C_latency_test",
    )
    all_rows: list[dict[str, Any]] = []
    summaries = []
    for variant in variants:
        rows, summary = _run_variant(
            variant,
            sequence_name,
            ego_track,
            sender_track,
            track_ids["vehicle1"],
            track_ids["vehicle2"],
            delay_ms,
            packet_loss_probability,
            seed,
        )
        all_rows.extend(rows)
        summaries.append(summary)

    steps_path = output / "experiment_steps.csv"
    with steps_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)

    summary_path = output / "experiment_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "experiment": "UrbanING connected-vehicle CAM latency scaffold",
                "sequence": sequence_name,
                "sender": "vehicle2",
                "receiver_ego": "vehicle1",
                "sampling_hz": 10,
                "delay_test_ms": delay_ms,
                "packet_loss_probability": packet_loss_probability,
                "random_seed": seed,
                "interpretation": (
                    "Communication/LDM input metrics only. Planner stability, jerk, TTC, "
                    "and safety conclusions are not evaluated yet."
                ),
                "variants": summaries,
            },
            indent=2,
        )
    )
    print(f"Wrote {steps_path} and {summary_path}")
    return {"steps_path": steps_path, "summary_path": summary_path, "variants": summaries}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare no-V2X, ideal-CAM, and delayed-CAM LDM inputs."
    )
    parser.add_argument("--root", default="datasets/UrbanIng-V2X")
    parser.add_argument("--sequence", default="20241126_0017_crossing1_00")
    parser.add_argument("--output", default="sumo_input/experiment")
    parser.add_argument("--delay-ms", type=float, default=200.0)
    parser.add_argument("--packet-loss", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    run_experiment(
        args.root,
        args.sequence,
        args.output,
        delay_ms=args.delay_ms,
        packet_loss_probability=args.packet_loss,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()