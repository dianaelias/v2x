"""Run one CommonRoad Reactive Planner plan for each prepared CAM variant."""

# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.ndimage import gaussian_filter1d

from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, KSState, STState
from commonroad.scenario.trajectory import Trajectory
from commonroad_rp.reactive_planner import ReactivePlanner, ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.utils_coordinate_system import create_coordinate_system

from visualize_encounter import footprint_clearance, rectangle_corners


def _rear_axle_reference(case: dict[str, Any]) -> np.ndarray:
    """Smooth and extend the recorded ego route for the planner's rear-axle frame."""
    route = case["reference_route"]
    ego = case["ego_state"]
    rear_offset = case["ego_dimensions"]["center_to_rear_axle_m"]
    reference = []
    for point in route:
        heading = point["heading_rad"]
        reference.append(
            [
                point["center_x_m"] - math.cos(heading) * rear_offset,
                point["center_y_m"] - math.sin(heading) * rear_offset,
            ]
        )
    reference_array = np.asarray(reference, dtype=float)
    if len(reference_array) < 8:
        raise ValueError("Reference route has too few future ego samples")

    # Smooth annotation noise before down-sampling the route for the planner.
    smoothed = gaussian_filter1d(reference_array, sigma=5.0, axis=0, mode="nearest")
    heading = ego["heading_rad"]
    initial_rear = np.array(
        [
            ego["center_x_m"] - math.cos(heading) * rear_offset,
            ego["center_y_m"] - math.sin(heading) * rear_offset,
        ]
    )
    smoothed[0] = initial_rear
    sampled = smoothed[::5]
    if np.linalg.norm(sampled[-1] - smoothed[-1]) > 1e-5:
        sampled = np.vstack((sampled, smoothed[-1]))
    keep = [0]
    for index in range(1, len(sampled)):
        if np.linalg.norm(sampled[index] - sampled[keep[-1]]) >= 0.15:
            keep.append(index)
    result = sampled[keep]
    if len(result) < 8:
        raise ValueError("Reference route degenerates after removing repeated positions")

    start_tangent = result[1] - result[0]
    start_tangent /= np.linalg.norm(start_tangent)
    end_tangent = result[-1] - result[-2]
    end_tangent /= np.linalg.norm(end_tangent)
    extension_distances = np.arange(30.0, 0.0, -1.0)
    before = result[0] - extension_distances[:, None] * start_tangent
    after = result[-1] + extension_distances[::-1, None] * end_tangent
    return np.vstack((before, result, after))


def _make_synthetic_corridor(reference: np.ndarray, width_m: float = 4.5) -> LaneletNetwork:
    """Build a straight-through lanelet corridor around the reference route."""
    tangents = np.gradient(reference, axis=0)
    norms = np.linalg.norm(tangents, axis=1)
    if np.any(norms < 1e-6):
        raise ValueError("Reference route contains duplicate or stationary samples")
    tangents = tangents / norms[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    half_width = width_m / 2.0
    lanelet = Lanelet(
        left_vertices=reference + normals * half_width,
        center_vertices=reference,
        right_vertices=reference - normals * half_width,
        lanelet_id=1,
        predecessor=[],
        successor=[],
    )
    return LaneletNetwork.create_from_lanelet_list([lanelet])


def _add_cam_obstacle(
    scenario: Scenario,
    decoded_state: dict[str, Any],
    length_m: float,
    width_m: float,
    planning_horizon_steps: int,
    dt: float,
) -> None:
    """Add a constant-velocity obstacle initialized from the received CAM state."""
    heading = float(decoded_state["heading_rad"])
    velocity = float(decoded_state["speed_mps"])
    position = np.array(
        [float(decoded_state["center_x_m"]), float(decoded_state["center_y_m"])],
        dtype=float,
    )
    shape = Rectangle(length=length_m, width=width_m)
    initial_state = InitialState(
        position=position,
        orientation=heading,
        velocity=velocity,
        time_step=0,
    )
    future_states = []
    for time_step in range(1, planning_horizon_steps + 1):
        future_seconds = time_step * dt
        future_states.append(
            KSState(
                position=position
                + velocity
                * future_seconds
                * np.array([math.cos(heading), math.sin(heading)]),
                steering_angle=0.0,
                velocity=velocity,
                orientation=heading,
                time_step=time_step,
            )
        )
    prediction = TrajectoryPrediction(Trajectory(1, future_states), shape)
    scenario.add_objects(
        DynamicObstacle(
            obstacle_id=2,
            obstacle_type=ObstacleType.CAR,
            obstacle_shape=shape,
            initial_state=initial_state,
            prediction=prediction,
        )
    )


def _make_planner(
    case: dict[str, Any],
    variant_name: str,
    reference: np.ndarray,
    plan_steps: int,
    dt: float,
) -> tuple[ReactivePlanner, dict[str, Any]]:
    """Construct a planner, planning problem, corridor, and optional CAM obstacle."""
    ego = case["ego_state"]
    ego_dimensions = case["ego_dimensions"]
    scenario = Scenario(dt=dt)
    scenario.add_objects(_make_synthetic_corridor(reference))

    goal_shape = Rectangle(length=2.0, width=2.0, center=reference[-1])
    goal_state = STState(
        position=goal_shape,
        velocity=Interval(0.0, 60.0),
        time_step=Interval(0, 1000),
    )
    goal_region = GoalRegion([goal_state])
    initial_state = InitialState(
        position=np.array([ego["center_x_m"], ego["center_y_m"]], dtype=float),
        orientation=float(ego["heading_rad"]),
        velocity=float(ego["speed_mps"]),
        acceleration=float(ego["acceleration_mps2"]),
        yaw_rate=float(ego["yaw_rate_rad_s"]),
        slip_angle=0.0,
        time_step=0,
    )
    planning_problem = PlanningProblem(1, initial_state, goal_region)

    config = ReactivePlannerConfiguration()
    config.planning.dt = dt
    config.planning.time_steps_computation = plan_steps
    config.planning.replanning_frequency = 1
    config.planning.safety_margin_dynamic_obstacles = 0.2
    config.sampling.num_sampling_levels = 3
    config.sampling.d_min = -0.5
    config.sampling.d_max = 0.5
    config.debug.multiproc = False
    config.debug.show_plots = False
    config.debug.save_plots = False
    config.debug.logging_level = "ERROR"
    config.vehicle.length = float(ego_dimensions["length_m"])
    config.vehicle.width = float(ego_dimensions["width_m"])
    config.vehicle.wb_rear_axle = float(ego_dimensions["center_to_rear_axle_m"])
    config.vehicle.wb_front_axle = float(
        ego_dimensions["front_to_rear_axle_m"]
        - ego_dimensions["center_to_rear_axle_m"]
    )
    config.vehicle._update_computed_params()
    config.update(scenario=scenario, planning_problem=planning_problem)

    received = case["received_cam_by_variant"][variant_name]
    if received is not None:
        _add_cam_obstacle(
            scenario,
            received["decoded_state"],
            float(case["sender_dimensions"]["length_m"]),
            float(case["sender_dimensions"]["width_m"]),
            plan_steps,
            dt,
        )
    planner = ReactivePlanner(config)
    planner.set_reference_path(coordinate_system=create_coordinate_system(reference))
    planner.set_desired_velocity(current_speed=float(ego["speed_mps"]))
    return planner, {"scenario": scenario, "planning_problem": planning_problem}


def _plan_variant(
    case: dict[str, Any],
    ground_truth: dict[str, Any],
    variant_name: str,
) -> dict[str, Any]:
    """Plan once and evaluate the path against both prediction and held-out truth."""
    dt = float(case["time_step_s"])
    plan_steps = round(float(case["planning_horizon_s"]) / dt)
    reference = _rear_axle_reference(case)
    planner, _ = _make_planner(case, variant_name, reference, plan_steps, dt)
    planned_speed = float(case["ego_state"]["speed_mps"])
    result = planner.plan()
    if result is None:
        return {
            "variant": variant_name,
            "status": "no_feasible_trajectory",
            "cam_age_s": (
                case["received_cam_by_variant"][variant_name]["decoded_state"]["received_age_s"]
                if case["received_cam_by_variant"][variant_name]
                else None
            ),
            "planning_time_s": planner.planning_times[-1] if planner.planning_times else None,
            "infeasible_collision_count": planner.infeasible_count_collision,
            "infeasible_kinematic_count": planner.infeasible_count_kinematics,
            "infeasible_reason_counts": planner.infeasible_reason_dict,
            "candidate_count": planner.total_count_samples,
            "trajectory": [],
        }

    trajectory, _, _ = result
    center_states = [
        ReactivePlannerState.shift_state_to_center(state, planner.vehicle_params.wb_rear_axle)
        for state in trajectory.state_list
    ]
    sender_length = float(case["sender_dimensions"]["length_m"])
    sender_width = float(case["sender_dimensions"]["width_m"])
    ego_length = float(case["ego_dimensions"]["length_m"])
    ego_width = float(case["ego_dimensions"]["width_m"])
    # Ground truth is indexed only after planner.plan(); it is evaluation-only.
    truth_by_step = {
        round(sample["time_from_start_s"] / dt): sample
        for sample in ground_truth["sender_ground_truth_future"]
    }
    ego_truth_by_step = {
        round(sample["time_from_start_s"] / dt): sample
        for sample in ground_truth["ego_ground_truth_future"]
    }
    planned_rows = []
    clearance_values = []
    cam_prediction_clearances = []
    route_deviations = []
    cam_entry = case["received_cam_by_variant"][variant_name]
    cam_state = cam_entry["decoded_state"] if cam_entry is not None else None
    for state in center_states:
        relative_step = state.time_step
        x, y = float(state.position[0]), float(state.position[1])
        yaw = float(state.orientation)
        actual_sender = truth_by_step.get(relative_step)
        actual_ego = ego_truth_by_step.get(relative_step)
        clearance = None
        if actual_sender is not None:
            ego_corners = rectangle_corners(x, y, ego_length, ego_width, yaw)
            sender_corners = rectangle_corners(
                actual_sender["center_x_m"],
                actual_sender["center_y_m"],
                sender_length,
                sender_width,
                actual_sender["heading_rad"],
            )
            clearance = footprint_clearance(ego_corners, sender_corners)
            clearance_values.append(clearance)
        cam_prediction_clearance = None
        if cam_state is not None:
            cam_yaw = float(cam_state["heading_rad"])
            cam_elapsed = relative_step * dt
            cam_x = float(cam_state["center_x_m"]) + (
                float(cam_state["speed_mps"]) * math.cos(cam_yaw) * cam_elapsed
            )
            cam_y = float(cam_state["center_y_m"]) + (
                float(cam_state["speed_mps"]) * math.sin(cam_yaw) * cam_elapsed
            )
            cam_corners = rectangle_corners(
                cam_x,
                cam_y,
                sender_length,
                sender_width,
                cam_yaw,
            )
            cam_prediction_clearance = footprint_clearance(
                rectangle_corners(x, y, ego_length, ego_width, yaw),
                cam_corners,
            )
            cam_prediction_clearances.append(cam_prediction_clearance)
        if actual_ego is not None:
            route_deviations.append(
                math.dist((x, y), (actual_ego["center_x_m"], actual_ego["center_y_m"]))
            )
        planned_rows.append(
            {
                "time_step": relative_step,
                "time_s": relative_step * dt,
                "x_m": x,
                "y_m": y,
                "heading_rad": yaw,
                "speed_mps": float(state.velocity),
                "acceleration_mps2": float(state.acceleration),
                "steering_angle_rad": float(state.steering_angle),
                "clearance_to_sender_ground_truth_m": clearance,
                "clearance_to_received_cam_prediction_m": cam_prediction_clearance,
            }
        )

    accelerations = [row["acceleration_mps2"] for row in planned_rows]
    # Acceleration differences approximate longitudinal jerk at the planner step.
    jerks = [
        (later - earlier) / dt
        for earlier, later in zip(accelerations, accelerations[1:])
    ]
    return {
        "variant": variant_name,
        "status": "planned",
        "cam_age_s": cam_entry["decoded_state"]["received_age_s"] if cam_entry else None,
        "cam_station_id": cam_entry["cam"]["header"]["stationId"] if cam_entry else None,
        "cam_decoded_center_at_plan_time_m": (
            [cam_state["center_x_m"], cam_state["center_y_m"]] if cam_state else None
        ),
        "scenario_dynamic_obstacle_count": len(planner.config.scenario.dynamic_obstacles),
        "minimum_planned_clearance_to_cam_prediction_m": (
            min(cam_prediction_clearances) if cam_prediction_clearances else None
        ),
        "dynamic_obstacle_safety_margin_m": planner.config.planning.safety_margin_dynamic_obstacles,
        "planning_time_s": planner.planning_times[-1] if planner.planning_times else None,
        "optimal_cost": planner.optimal_cost,
        "candidate_count": planner.total_count_samples,
        "infeasible_collision_count": planner.infeasible_count_collision,
        "infeasible_kinematic_count": planner.infeasible_count_kinematics,
        "infeasible_reason_counts": planner.infeasible_reason_dict,
        "mean_deviation_from_recorded_ego_route_m": (
            sum(route_deviations) / len(route_deviations) if route_deviations else None
        ),
        "minimum_clearance_to_sender_ground_truth_m": min(clearance_values) if clearance_values else None,
        "peak_abs_jerk_mps3": max((abs(value) for value in jerks), default=0.0),
        "trajectory": planned_rows,
    }


def main() -> None:
    """Read prepared JSON files, plan A/B/C, compare, and write result JSON."""
    parser = argparse.ArgumentParser(description="Run one CommonRoad replan per A/B/C CAM case.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    logging.basicConfig(level=logging.ERROR)
    case = json.loads(Path(args.input).read_text())
    ground_truth = json.loads(Path(args.ground_truth).read_text())
    results = [
        _plan_variant(case, ground_truth, variant)
        for variant in ("A_no_v2x", "B_ideal", "C_latency_test")
    ]

    planned = {item["variant"]: item for item in results if item["status"] == "planned"}
    pairwise = {}
    for first_name, second_name in (("A_no_v2x", "B_ideal"), ("B_ideal", "C_latency_test")):
        first = planned.get(first_name)
        second = planned.get(second_name)
        if first is None or second is None:
            pairwise[f"{first_name}_vs_{second_name}"] = None
            continue
        common_steps = min(len(first["trajectory"]), len(second["trajectory"]))
        differences = [
            math.dist(
                (first["trajectory"][index]["x_m"], first["trajectory"][index]["y_m"]),
                (second["trajectory"][index]["x_m"], second["trajectory"][index]["y_m"]),
            )
            for index in range(common_steps)
        ]
        pairwise[f"{first_name}_vs_{second_name}"] = {
            "mean_planned_position_difference_m": (
                sum(differences) / len(differences) if differences else None
            ),
            "max_planned_position_difference_m": max(differences) if differences else None,
        }

    output = {
        "sequence": case["sequence"],
        "planning_time_s": case["plan_time_s"],
        "sender_track_id": case["sender_track_id"],
        "ego_track_id": case["ego_track_id"],
        "planner": "CommonRoad Reactive Planner 2025.1",
        "planner_time_step_s": case["time_step_s"],
        "planning_horizon_s": case["planning_horizon_s"],
        "reference_route_source": "recorded ego ground-truth positions; identical in A/B/C",
        "sender_future_ground_truth_used_by_planner": False,
        "synthetic_corridor_width_m": 4.5,
        "sender_prediction": "constant velocity from received CAM snapshot",
        "variants": results,
        "cross_variant_comparison": pairwise,
        "interpretation_limit": (
            "One replan at one scenario time. This is a smoke-test integration, not a validated "
            "closed-loop stability or safety result."
        ),
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2))
    print(f"Wrote planner comparison to {output_path}")
    for item in results:
        print(
            item["variant"],
            item["status"],
            "min GT clearance=",
            item.get("minimum_clearance_to_sender_ground_truth_m"),
            "peak jerk=",
            item.get("peak_abs_jerk_mps3"),
        )


if __name__ == "__main__":
    main()