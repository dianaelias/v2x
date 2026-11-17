"""Run a closed-loop A/B/C visibility and CAM-network experiment on CommonRoad."""

from __future__ import annotations

import argparse
import copy
import heapq
import json
import math
import random
from pathlib import Path
from typing import Any

import numpy as np

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad_rp.reactive_planner import ReactivePlanner, ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.utils_coordinate_system import (
    create_coordinate_system,
    create_initial_ref_path,
)

from visualize_encounter import footprint_clearance, rectangle_corners


VARIANTS = ("A_no_v2x", "B_ideal", "C_latency_test")


def _states_by_step(obstacle: DynamicObstacle) -> dict[int, Any]:
    """Index the obstacle's initial and predicted states by simulation step."""
    states = [obstacle.initial_state]
    if obstacle.prediction is not None:
        states.extend(obstacle.prediction.trajectory.state_list)
    return {int(state.time_step): state for state in states}


def _conflict_point(
    ego_reference_path: np.ndarray,
    actor: DynamicObstacle,
) -> tuple[float, float]:
    """Find the first spatial crossing of the ego path and actor centerline."""
    actor_points = np.asarray(
        [state.position for state in _states_by_step(actor).values()],
        dtype=float,
    )
    for actor_start, actor_end in zip(actor_points[:-1], actor_points[1:]):
        actor_direction = actor_end - actor_start
        for ego_start, ego_end in zip(
            ego_reference_path[:-1], ego_reference_path[1:]
        ):
            ego_direction = ego_end - ego_start
            denominator = (
                ego_direction[0] * actor_direction[1]
                - ego_direction[1] * actor_direction[0]
            )
            if abs(denominator) < 1e-10:
                continue
            separation = actor_start - ego_start
            ego_fraction = (
                separation[0] * actor_direction[1]
                - separation[1] * actor_direction[0]
            ) / denominator
            actor_fraction = (
                separation[0] * ego_direction[1]
                - separation[1] * ego_direction[0]
            ) / denominator
            if 0.0 <= ego_fraction <= 1.0 and 0.0 <= actor_fraction <= 1.0:
                point = ego_start + ego_fraction * ego_direction
                return float(point[0]), float(point[1])
    raise ValueError(
        "Could not derive a conflict point: ego reference path and actor track "
        "do not intersect"
    )


def _cam_predicted_obstacle(
    source: DynamicObstacle,
    received_state: Any,
    received_world_step: int,
    current_step: int,
    horizon_steps: int,
    dt: float,
) -> DynamicObstacle:
    """Extrapolate an old received actor state with constant speed and heading."""
    heading = float(received_state.orientation)
    speed = float(received_state.velocity)
    direction = np.array([math.cos(heading), math.sin(heading)])
    received_position = np.asarray(received_state.position, dtype=float)

    def position_at(step: int) -> np.ndarray:
        return (
            received_position
            + speed * (step - received_world_step) * dt * direction
        )

    shape = copy.deepcopy(source.obstacle_shape)
    initial_state = InitialState(
        position=position_at(current_step),
        orientation=heading,
        velocity=speed,
        time_step=current_step,
    )
    future_states = [
        KSState(
            position=position_at(step),
            orientation=heading,
            velocity=speed,
            steering_angle=0.0,
            time_step=step,
        )
        for step in range(current_step + 1, current_step + horizon_steps + 1)
    ]
    prediction = TrajectoryPrediction(
        Trajectory(current_step + 1, future_states), shape
    )
    return DynamicObstacle(
        obstacle_id=source.obstacle_id,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=shape,
        initial_state=initial_state,
        prediction=prediction,
    )


def _scene_for_variant(
    base_scenario: Any,
    source_obstacle: DynamicObstacle,
    variant: str,
    received_cam: tuple[Any, int] | None,
    current_step: int,
    horizon_steps: int,
    dt: float,
) -> Any:
    """Include the selected actor only when the variant currently knows its state."""
    scenario = copy.deepcopy(base_scenario)
    scenario.remove_obstacle(source_obstacle)
    if received_cam is None:
        return scenario
    received_state, received_world_step = received_cam
    scenario.add_objects(
        _cam_predicted_obstacle(
            source_obstacle,
            received_state,
            received_world_step,
            current_step,
            horizon_steps,
            dt,
        )
    )
    return scenario


def _run_variant(
    config: ReactivePlannerConfiguration,
    source_obstacle: DynamicObstacle,
    variant: str,
    delay_steps: int,
    actor_time_offset_steps: int,
    simulation_steps: int,
    packet_loss_probability: float,
    random_seed: int,
    conflict_point: tuple[float, float],
    sight_distance_m: float,
) -> dict[str, Any]:
    """Execute one scenario while replanning at the configured fixed frequency."""
    config = copy.deepcopy(config)
    scenario = config.scenario
    planning_problem = config.planning_problem
    dt = float(config.planning.dt)
    horizon_steps = int(config.planning.time_steps_computation)
    replanning_frequency = int(config.planning.replanning_frequency)
    coordinate_system = create_coordinate_system(
        create_initial_ref_path(scenario.lanelet_network, planning_problem)
    )
    planner = ReactivePlanner(config)
    planner.set_reference_path(coordinate_system=coordinate_system)
    planner.record_state_and_input(planner.x_0)

    source_states = _states_by_step(source_obstacle)
    actor_length = float(source_obstacle.obstacle_shape.length)
    actor_width = float(source_obstacle.obstacle_shape.width)
    ego_length = float(config.vehicle.length)
    ego_width = float(config.vehicle.width)
    executed: list[dict[str, Any]] = []
    planning_times = []
    failure_step = None
    optimal = None
    pending_packets: list[tuple[int, int, Any, int]] = []
    latest_received_cam: tuple[Any, int] | None = None
    generated_cam_count = 0
    delivered_cam_count = 0
    dropped_cam_count = 0
    warmup_cam_count = 0
    warmup_dropped_cam_count = 0
    packet_sequence = 0
    random_generator = random.Random(random_seed)
    aoi_samples_ms: list[float] = []
    local_actor_visible = False
    local_visibility_start_time_s: float | None = None
    local_sensor_observation_count = 0

    def enqueue_cam(generated_world_step: int, is_warmup: bool = False) -> None:
        nonlocal generated_cam_count
        nonlocal dropped_cam_count
        nonlocal packet_sequence
        nonlocal warmup_cam_count
        nonlocal warmup_dropped_cam_count

        source_step = generated_world_step - actor_time_offset_steps
        sender_state = source_states.get(source_step)
        if sender_state is None:
            return

        generated_cam_count += 1
        if is_warmup:
            warmup_cam_count += 1
        if random_generator.random() < packet_loss_probability:
            dropped_cam_count += 1
            if is_warmup:
                warmup_dropped_cam_count += 1
            return
        heapq.heappush(
            pending_packets,
            (
                generated_world_step + delay_steps,
                packet_sequence,
                sender_state,
                generated_world_step,
            ),
        )
        packet_sequence += 1

    if variant == "C_latency_test":
        # Seed the receiver with packets already in flight before simulation t=0.
        for warmup_step in range(-delay_steps, 0):
            enqueue_cam(warmup_step, is_warmup=True)

    # The planner is called every replanning_frequency ticks; intervening ticks
    # execute the corresponding next state of the most recently selected plan.
    for current_step in range(simulation_steps):
        source_step = current_step - actor_time_offset_steps
        current_actor_state = source_states.get(source_step)
        if variant == "A_no_v2x" and current_actor_state is not None:
            distance_to_conflict = math.dist(
                tuple(float(value) for value in current_actor_state.position),
                conflict_point,
            )
            if not local_actor_visible and distance_to_conflict <= sight_distance_m:
                local_actor_visible = True
                local_visibility_start_time_s = current_step * dt
            if local_actor_visible:
                latest_received_cam = (current_actor_state, current_step)
                local_sensor_observation_count += 1

        if variant != "A_no_v2x":
            if variant == "B_ideal":
                if current_actor_state is not None:
                    generated_cam_count += 1
                    delivered_cam_count += 1
                    latest_received_cam = (current_actor_state, current_step)
            else:
                enqueue_cam(current_step)

        while pending_packets and pending_packets[0][0] <= current_step:
            _, _, received_state, generated_world_step = heapq.heappop(
                pending_packets
            )
            if (
                latest_received_cam is None
                or generated_world_step > latest_received_cam[1]
            ):
                latest_received_cam = (received_state, generated_world_step)
            delivered_cam_count += 1

        if latest_received_cam is not None:
            aoi_samples_ms.append(
                (current_step - latest_received_cam[1]) * dt * 1000.0
            )

        if current_step % replanning_frequency == 0:
            active_scenario = _scene_for_variant(
                scenario,
                source_obstacle,
                variant,
                latest_received_cam,
                current_step,
                horizon_steps,
                dt,
            )
            planner.set_collision_checker(active_scenario)
            planner.set_desired_velocity(current_speed=planner.x_0.velocity)
            optimal = planner.plan()
            if optimal is None:
                failure_step = current_step
                break
            planning_times.append(
                planner.planning_times[-1] if planner.planning_times else None
            )

        state_index = current_step % replanning_frequency + 1
        if optimal is None or state_index >= len(optimal[0].state_list):
            failure_step = current_step
            break
        state = optimal[0].state_list[state_index]
        center_state = ReactivePlannerState.shift_state_to_center(
            state, planner.vehicle_params.wb_rear_axle
        )
        source_step = current_step + 1 - actor_time_offset_steps
        actor_state = source_states.get(source_step)
        clearance = None
        overlap = None
        if actor_state is not None:
            ego_corners = rectangle_corners(
                float(center_state.position[0]),
                float(center_state.position[1]),
                ego_length,
                ego_width,
                float(center_state.orientation),
            )
            actor_corners = rectangle_corners(
                float(actor_state.position[0]),
                float(actor_state.position[1]),
                actor_length,
                actor_width,
                float(actor_state.orientation),
            )
            clearance = footprint_clearance(ego_corners, actor_corners)
            overlap = clearance == 0.0
        executed.append(
            {
                "time_step": int(state.time_step),
                "x_m": float(center_state.position[0]),
                "y_m": float(center_state.position[1]),
                "orientation_rad": float(center_state.orientation),
                "speed_mps": float(state.velocity),
                "ground_truth_actor_clearance_m": clearance,
                "ground_truth_actor_overlap": overlap,
            }
        )
        planner.record_state_and_input(state)
        planner.reset(
            initial_state_cart=planner.record_state_list[-1],
            initial_state_curv=(optimal[1][state_index], optimal[2][state_index]),
            collision_checker=planner.collision_checker,
            coordinate_system=planner.coordinate_system,
        )

    clearances = [
        row["ground_truth_actor_clearance_m"]
        for row in executed
        if row["ground_truth_actor_clearance_m"] is not None
    ]
    return {
        "variant": variant,
        "status": "planned" if failure_step is None else "planner_failed",
        "failure_time_s": failure_step * dt if failure_step is not None else None,
        "executed_steps": len(executed),
        "replan_count": len(planning_times),
        "mean_planning_time_s": (
            float(np.mean(planning_times)) if planning_times else None
        ),
        "generated_cam_count": generated_cam_count,
        "delivered_cam_count": delivered_cam_count,
        "dropped_cam_count": dropped_cam_count,
        "in_flight_cam_count": len(pending_packets),
        "warmup_cam_count": warmup_cam_count,
        "warmup_dropped_cam_count": warmup_dropped_cam_count,
        "local_sensor_visible_from_s": local_visibility_start_time_s,
        "local_sensor_observation_count": local_sensor_observation_count,
        "packet_loss_probability": (
            packet_loss_probability if variant == "C_latency_test" else 0.0
        ),
        "mean_aoi_ms": (
            float(np.mean(aoi_samples_ms)) if aoi_samples_ms else None
        ),
        "maximum_aoi_ms": max(aoi_samples_ms) if aoi_samples_ms else None,
        "minimum_ground_truth_actor_clearance_m": min(clearances) if clearances else None,
        "ground_truth_actor_overlap_steps": sum(
            row["ground_truth_actor_overlap"] is True for row in executed
        ),
        "trajectory": executed,
    }


def run_experiment(
    scenario_path: Path,
    configuration_path: Path,
    planning_problem_id: int,
    obstacle_id: int,
    delay_ms: float,
    simulation_steps: int,
    actor_time_offset_ms: float = 0.0,
    packet_loss_probability: float = 0.0,
    random_seed: int = 7,
    sight_distance_m: float = 15.0,
) -> dict[str, Any]:
    """Load one scenario, run A/B/C, and compare their executed trajectories."""
    config = ReactivePlannerConfiguration.load(
        str(configuration_path), scenario_path.name
    )
    config.general.path_scenario = str(scenario_path)
    config.debug.multiproc = False
    config.debug.show_plots = False
    config.debug.save_plots = False
    config.debug.logging_level = "ERROR"
    config.update()

    scenario = config.scenario
    # The agreed experiment cadence is one planner call per 0.1 s scenario step.
    config.planning.replanning_frequency = 1
    config.planning.time_steps_computation = 20
    config.update(scenario=scenario, planning_problem=config.planning_problem)
    problem = config.planning_problem
    if problem.planning_problem_id != planning_problem_id:
        raise ValueError(
            f"Loaded planning problem {problem.planning_problem_id}, expected {planning_problem_id}"
        )
    source_obstacle = next(
        (
            obstacle
            for obstacle in scenario.dynamic_obstacles
            if obstacle.obstacle_id == obstacle_id
        ),
        None,
    )
    if source_obstacle is None:
        raise ValueError(f"Dynamic obstacle {obstacle_id} is not in the scenario")
    dt = float(scenario.dt)
    if not math.isfinite(delay_ms) or delay_ms < 0:
        raise ValueError(
            "CAM delay must be a finite, non-negative number of milliseconds"
        )
    if not math.isfinite(packet_loss_probability) or not (
        0.0 <= packet_loss_probability <= 1.0
    ):
        raise ValueError("Packet loss probability must be between 0 and 1")
    if not math.isfinite(sight_distance_m) or sight_distance_m <= 0.0:
        raise ValueError("Sight distance must be a finite, positive number of meters")
    if not math.isfinite(actor_time_offset_ms):
        raise ValueError("Actor time offset must be a finite number of milliseconds")
    delay_steps = round(delay_ms / 1000.0 / dt)
    actor_time_offset_steps = round(actor_time_offset_ms / 1000.0 / dt)
    effective_actor_time_offset_ms = actor_time_offset_steps * dt * 1000.0
    if not math.isclose(
        actor_time_offset_ms, effective_actor_time_offset_ms, abs_tol=1e-6
    ):
        raise ValueError(
            f"Actor time offset {actor_time_offset_ms} ms is not aligned "
            f"to the scenario's {dt * 1000:g} ms time step"
        )
    ego_reference_path = np.asarray(
        create_initial_ref_path(scenario.lanelet_network, config.planning_problem),
        dtype=float,
    )
    conflict_point = _conflict_point(ego_reference_path, source_obstacle)
    results = [
        _run_variant(
            config,
            source_obstacle,
            variant,
            delay_steps,
            actor_time_offset_steps,
            simulation_steps,
            packet_loss_probability,
            random_seed,
            conflict_point,
            sight_distance_m,
        )
        for variant in VARIANTS
    ]

    by_name = {result["variant"]: result for result in results}
    differences = {}
    for first_name, second_name in ((VARIANTS[0], VARIANTS[1]), (VARIANTS[1], VARIANTS[2])):
        first = by_name[first_name]["trajectory"]
        second = by_name[second_name]["trajectory"]
        common = min(len(first), len(second))
        distances = [
            math.dist(
                (first[index]["x_m"], first[index]["y_m"]),
                (second[index]["x_m"], second[index]["y_m"]),
            )
            for index in range(common)
        ]
        differences[f"{first_name}_vs_{second_name}"] = {
            "mean_position_difference_m": (
                float(np.mean(distances)) if distances else None
            ),
            "max_position_difference_m": max(distances) if distances else None,
        }

    return {
        "scenario": scenario_path.name,
        "planning_problem_id": planning_problem_id,
        "cam_source_dynamic_obstacle_id": obstacle_id,
        "cam_delay_ms": delay_ms,
        "packet_loss_probability": packet_loss_probability,
        "random_seed": random_seed,
        "visibility_model": (
            "A reveals the actor when its center is within the configured distance "
            "of the ego-reference/actor-track intersection; visibility remains "
            "active after first detection."
        ),
        "visibility_conflict_point_m": list(conflict_point),
        "visibility_sight_distance_m": sight_distance_m,
        "actor_time_offset_steps": actor_time_offset_steps,
        "actor_time_offset_ms": effective_actor_time_offset_ms,
        "planner": "CommonRoad Reactive Planner 2025.1",
        "planner_time_step_s": float(scenario.dt),
        "replanning_frequency_steps": int(config.planning.replanning_frequency),
        "replanning_period_s": (
            float(scenario.dt) * int(config.planning.replanning_frequency)
        ),
        "simulation_duration_s": simulation_steps * float(scenario.dt),
        "prediction_model": "constant velocity from latest available scenario state",
        "variants": results,
        "cross_variant_comparison": differences,
        "interpretation_limit": (
            "A uses a distance-triggered local-visibility approximation without "
            "building occluder geometry; B/C use ideal/delayed synthetic CAM snapshots. "
            "A negative actor time offset advances the actor track relative to ego; "
            "a positive offset delays it. The offset changes only the test track timing. "
            "C holds the latest delivered CAM and extrapolates it with constant speed "
            "and heading between deliveries. This controlled prototype does not model "
            "onboard perception or physical occlusion."
        ),
    }


def main() -> None:
    """Parse scenario/run settings and persist the result as JSON."""
    parser = argparse.ArgumentParser(
        description="Closed-loop CAM latency test on a CommonRoad benchmark scenario."
    )
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--configuration", required=True)
    parser.add_argument("--planning-problem-id", type=int, default=60000)
    parser.add_argument("--obstacle-id", type=int, default=1)
    parser.add_argument("--delay-ms", type=float, default=200.0)
    parser.add_argument("--actor-time-offset-ms", type=float, default=0.0)
    parser.add_argument("--packet-loss", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--sight-distance-m", type=float, default=15.0)
    parser.add_argument("--simulation-steps", type=int, default=147)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    result = run_experiment(
        Path(args.scenario),
        Path(args.configuration),
        args.planning_problem_id,
        args.obstacle_id,
        args.delay_ms,
        args.simulation_steps,
        args.actor_time_offset_ms,
        args.packet_loss,
        args.seed,
        args.sight_distance_m,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(f"Wrote CommonRoad scenario comparison to {output}")
    for variant in result["variants"]:
        print(
            variant["variant"],
            variant["status"],
            "steps=", variant["executed_steps"],
            "min_gt_clearance_m=", variant["minimum_ground_truth_actor_clearance_m"],
        )


if __name__ == "__main__":
    main()