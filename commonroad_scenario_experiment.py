from __future__ import annotations

import argparse
import copy
import json
import math
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
    states = [obstacle.initial_state]
    if obstacle.prediction is not None:
        states.extend(obstacle.prediction.trajectory.state_list)
    return {int(state.time_step): state for state in states}


def _cam_predicted_obstacle(
    source: DynamicObstacle,
    received_step: int,
    current_step: int,
    horizon_steps: int,
    dt: float,
) -> DynamicObstacle:
    states = _states_by_step(source)
    received = states[received_step]
    heading = float(received.orientation)
    speed = float(received.velocity)
    direction = np.array([math.cos(heading), math.sin(heading)])
    received_position = np.asarray(received.position, dtype=float)

    def position_at(step: int) -> np.ndarray:
        return received_position + speed * (step - received_step) * dt * direction

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
    current_step: int,
    delay_steps: int,
    horizon_steps: int,
    dt: float,
) -> Any:
    scenario = copy.deepcopy(base_scenario)
    scenario.remove_obstacle(source_obstacle)
    if variant != "A_no_v2x":
        received_step = max(0, current_step - delay_steps)
        scenario.add_objects(
            _cam_predicted_obstacle(
                source_obstacle,
                received_step,
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
    simulation_steps: int,
) -> dict[str, Any]:
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

    for current_step in range(simulation_steps):
        if current_step % replanning_frequency == 0:
            active_scenario = _scene_for_variant(
                scenario,
                source_obstacle,
                variant,
                current_step,
                delay_steps,
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
        actor_state = source_states.get(current_step + 1)
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
) -> dict[str, Any]:
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
    config.planning.replanning_frequency = 3
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
    delay_steps = round(delay_ms / 1000.0 / float(scenario.dt))
    results = [
        _run_variant(
            config,
            source_obstacle,
            variant,
            delay_steps,
            simulation_steps,
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
        "planner": "CommonRoad Reactive Planner 2025.1",
        "planner_time_step_s": float(scenario.dt),
        "simulation_duration_s": simulation_steps * float(scenario.dt),
        "prediction_model": "constant velocity from latest available scenario state",
        "variants": results,
        "cross_variant_comparison": differences,
        "interpretation_limit": (
            "A omits this cooperative actor; B/C use ideal/delayed synthetic CAM snapshots. "
            "This controlled prototype does not model onboard perception or packet loss."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Closed-loop CAM latency test on a CommonRoad benchmark scenario."
    )
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--configuration", required=True)
    parser.add_argument("--planning-problem-id", type=int, default=60000)
    parser.add_argument("--obstacle-id", type=int, default=1)
    parser.add_argument("--delay-ms", type=float, default=200.0)
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