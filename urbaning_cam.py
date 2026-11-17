"""Create readable synthetic CAM Release 2-shaped records from UrbanING tracks.

The output is a semantic JSON representation for experiments, not an ASN.1
UPER-encoded ETSI message.
"""

from __future__ import annotations

import argparse
import json
import math
import zlib
from pathlib import Path
from typing import Any

from pyproj import Transformer

from urbaning.data.info import gps_origins


ITS_EPOCH_UNIX_SECONDS = 1_073_291_200
CAM_PROTOCOL_VERSION = 2
CAM_MESSAGE_ID = 2

VEHICLE_STATION_TYPES = {
    "Car": {"value": 5, "name": "passengerCar"},
    "Van": {"value": 7, "name": "lightTruck"},
    "Truck": {"value": 8, "name": "heavyTruck"},
}


def _station_id(sequence_name: str, track_id: int) -> int:
    """Create a stable nonzero identifier for one sequence/track pair."""
    identity = f"{sequence_name}:{track_id}".encode("utf-8")
    return zlib.crc32(identity) or 1


def _angle_difference(first: float, second: float) -> float:
    """Return the signed shortest rotation from the first angle to the second."""
    return math.atan2(math.sin(second - first), math.cos(second - first))


def _central_derivative(values: list[float], timestamps: list[float], index: int) -> float:
    """Estimate a time derivative, using one-sided differences at track ends."""
    if len(values) < 2:
        return 0.0
    # A centered interval reduces the bias of a forward difference at interior samples.
    left = max(0, index - 1)
    right = min(len(values) - 1, index + 1)
    elapsed = timestamps[right] - timestamps[left]
    if elapsed <= 0:
        raise ValueError("Track timestamps must be strictly increasing")
    return (values[right] - values[left]) / elapsed


def _track_kinematics(track: dict[str, Any]) -> list[dict[str, float]]:
    """Derive speed, acceleration, yaw rate, and curvature for each track sample."""
    timestamps = [float(value) for value in track["timestamps"]]
    positions = track["positions"]
    orientations = [float(value) for value in track["orientations"]]

    if not timestamps or len(timestamps) != len(positions) or len(timestamps) != len(orientations):
        raise ValueError(f"Track {track.get('track_id')} has mismatched or empty arrays")
    if any(later <= earlier for earlier, later in zip(timestamps, timestamps[1:])):
        raise ValueError(f"Track {track.get('track_id')} timestamps are not strictly increasing")

    x_positions = [float(position[0]) for position in positions]
    y_positions = [float(position[1]) for position in positions]
    velocity_x = [
        _central_derivative(x_positions, timestamps, index)
        for index in range(len(timestamps))
    ]
    velocity_y = [
        _central_derivative(y_positions, timestamps, index)
        for index in range(len(timestamps))
    ]
    speed = [math.hypot(vx, vy) for vx, vy in zip(velocity_x, velocity_y)]
    acceleration = [
        _central_derivative(speed, timestamps, index)
        for index in range(len(timestamps))
    ]
    # Unwrap angles before differentiating so crossing +/-pi does not look like
    # an artificial rotation of nearly one full turn.
    unwrapped_orientations = [orientations[0]]
    for previous, current in zip(orientations, orientations[1:]):
        unwrapped_orientations.append(
            unwrapped_orientations[-1] + _angle_difference(previous, current)
        )
    yaw_rate = [
        _central_derivative(
            unwrapped_orientations,
            timestamps,
            index,
        )
        for index in range(len(timestamps))
    ]

    result = []
    for index, position in enumerate(positions):
        heading = orientations[index]
        forward_speed = velocity_x[index] * math.cos(heading) + velocity_y[index] * math.sin(heading)
        drive_direction = "backward" if forward_speed < -0.1 else "forward"
        curvature = yaw_rate[index] / speed[index] if speed[index] > 0.1 else 0.0
        result.append(
            {
                "timestamp_unix_s": timestamps[index],
                "x_center_m": x_positions[index],
                "y_center_m": y_positions[index],
                "z_center_m": float(position[2]),
                "heading_rad": heading,
                "speed_mps": speed[index],
                "longitudinal_acceleration_mps2": acceleration[index],
                "yaw_rate_rad_s": yaw_rate[index],
                "curvature_per_m": curvature,
                "drive_direction": drive_direction,
            }
        )
    return result


def _wgs84_reference_position(
    x_m: float,
    y_m: float,
    z_m: float,
) -> dict[str, float]:
    """Convert local crossing1 metre coordinates back to WGS84 coordinates."""
    origin_latitude, origin_longitude, origin_altitude = gps_origins["crossing1"]
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    to_wgs84 = Transformer.from_crs("EPSG:32632", "EPSG:4326", always_xy=True)
    origin_easting, origin_northing = to_utm.transform(origin_longitude, origin_latitude)
    longitude, latitude = to_wgs84.transform(origin_easting + x_m, origin_northing + y_m)
    return {
        "latitude_deg": latitude,
        "longitude_deg": longitude,
        "altitude_m": origin_altitude + z_m,
    }


def track_sample_to_cam(
    track: dict[str, Any],
    sequence_name: str,
    sample: dict[str, float],
) -> dict[str, Any]:
    """Create a readable synthetic CAM Release 2-shaped record for one sample."""
    object_type = track["object_type"]
    if object_type not in VEHICLE_STATION_TYPES:
        raise ValueError(f"Unsupported CAM station type for object type {object_type!r}")

    length_m, width_m, height_m = (float(value) for value in track["dimensions"][0])
    heading_rad = sample["heading_rad"]

    # UrbanING labels place the origin at the bounding-box centre; CAM uses the
    # ground projection of the centre of the vehicle's front side.
    front_x = sample["x_center_m"] + math.cos(heading_rad) * length_m / 2.0
    front_y = sample["y_center_m"] + math.sin(heading_rad) * length_m / 2.0
    front_z = sample["z_center_m"] - height_m / 2.0
    reference_position = _wgs84_reference_position(front_x, front_y, front_z)

    heading_deg = (90.0 - math.degrees(heading_rad)) % 360.0
    heading_deg = 0.0 if math.isclose(heading_deg, 360.0) else heading_deg
    # CAM generationDeltaTime is relative to the ITS epoch and wraps at 16 bits.
    timestamp_its_ms = round(
        (sample["timestamp_unix_s"] - ITS_EPOCH_UNIX_SECONDS) * 1000.0
    )
    speed_mps = sample["speed_mps"]

    cam = {
        "header": {
            "protocolVersion": CAM_PROTOCOL_VERSION,
            "messageId": CAM_MESSAGE_ID,
            "stationId": _station_id(sequence_name, int(track["track_id"])),
        },
        "cam": {
            "generationDeltaTime": timestamp_its_ms % 65_536,
            "camParameters": {
                "basicContainer": {
                    "stationType": VEHICLE_STATION_TYPES[object_type]["value"],
                    "referencePosition": {
                        **reference_position,
                        "positionConfidenceEllipse": {
                            "semiMajorAxisLength_m": "unavailable",
                            "semiMinorAxisLength_m": "unavailable",
                            "semiMajorAxisOrientation_deg": "unavailable",
                        },
                        "altitude": {
                            "altitudeValue_m": reference_position["altitude_m"],
                            "altitudeConfidence": "unavailable",
                        },
                    },
                },
                "highFrequencyContainer": {
                    "basicVehicleContainerHighFrequency": {
                        "heading": {
                            "headingValue_deg": heading_deg,
                            "headingConfidence": "unavailable",
                        },
                        "speed": {
                            "speedValue_mps": speed_mps,
                            "speedConfidence": "unavailable",
                        },
                        "driveDirection": sample["drive_direction"],
                        "vehicleLength": {
                            "vehicleLengthValue_m": length_m,
                            "vehicleLengthConfidenceIndication": "unavailable",
                        },
                        "vehicleWidth": {"vehicleWidthValue_m": width_m},
                        "longitudinalAcceleration": {
                            "value_mps2": sample["longitudinal_acceleration_mps2"],
                            "confidence": "unavailable",
                        },
                        "curvature": {
                            "curvatureValue_per_m": sample["curvature_per_m"],
                            "curvatureConfidence": "unavailable",
                        },
                        "curvatureCalculationMode": "unavailable",
                        "yawRate": {
                            "value_deg_s": math.degrees(sample["yaw_rate_rad_s"]),
                            "confidence": "unavailable",
                        },
                    }
                },
            },
        },
    }

    return {
        "metadata": {
            "standard_profile": "ETSI TS 103 900 V2.2.1 CAM Release 2",
            "synthetic": True,
            "encoding": "readable semantic JSON; not ASN.1 UPER encoded",
            "source_sequence": sequence_name,
            "source_track_id": int(track["track_id"]),
            "source_object_type": object_type,
            "station_type_assumption": VEHICLE_STATION_TYPES[object_type]["name"],
            "timestamp_unix_s": sample["timestamp_unix_s"],
            "units": "SI values with unit suffixes; confidence marked unavailable",
            "kinematic_warning": (
                "derived_speed_above_55_mps" if speed_mps > 55.0 else None
            ),
        },
        "cam": cam,
    }


def iter_cam_records(
    labels_path: str | Path,
    sequence_name: str,
    sender_track_ids: set[int] | None = None,
):
    """Yield CAM-shaped records for supported tracks, optionally filtered by ID."""
    labels = json.loads(Path(labels_path).read_text())
    tracks = labels.get("tracks")
    if not isinstance(tracks, list):
        raise ValueError(f"Expected a tracks list in {labels_path}")

    for track in tracks:
        if sender_track_ids is not None and int(track["track_id"]) not in sender_track_ids:
            continue
        if track.get("object_type") not in VEHICLE_STATION_TYPES:
            continue
        for sample in _track_kinematics(track):
            yield track_sample_to_cam(track, sequence_name, sample)


def main() -> None:
    """Select the connected sender (or all supported tracks) and write JSONL."""
    parser = argparse.ArgumentParser(
        description="Create synthetic ETSI CAM Release 2-shaped JSONL from UrbanING labels."
    )
    parser.add_argument("--root", default="datasets/UrbanIng-V2X")
    parser.add_argument("--sequence", default="20241126_0017_crossing1_00")
    parser.add_argument("--output", default="sumo_input/synthetic_cam_messages.jsonl")
    parser.add_argument("--sender", choices=("vehicle1", "vehicle2"), default="vehicle2")
    parser.add_argument(
        "--all-vehicles",
        action="store_true",
        help="Generate CAM records for every supported annotated vehicle track.",
    )
    args = parser.parse_args()

    root = Path(args.root)
    labels_path = root / "labels" / f"{args.sequence}.json"
    sender_track_ids = None
    if not args.all_vehicles:
        connected_vehicles_path = root / "labels_av_track_ids.json"
        connected_vehicles = json.loads(connected_vehicles_path.read_text())
        if args.sequence not in connected_vehicles:
            raise ValueError(f"No connected-vehicle track mapping for {args.sequence}")
        sender_track_ids = {int(connected_vehicles[args.sequence][args.sender])}

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    station_ids: set[int] = set()
    with output_path.open("w") as output_file:
        for record in iter_cam_records(labels_path, args.sequence, sender_track_ids):
            station_id = record["cam"]["header"]["stationId"]
            station_ids.add(station_id)
            output_file.write(json.dumps(record, separators=(",", ":")) + "\n")
            count += 1

    if count == 0:
        raise ValueError(f"No supported vehicle tracks found in {labels_path}")
    if len(station_ids) == 0:
        raise ValueError("No synthetic station IDs were generated")
    print(
        f"Wrote {count} synthetic CAM records from {len(station_ids)} stations "
        f"to {output_path}"
    )


if __name__ == "__main__":
    main()