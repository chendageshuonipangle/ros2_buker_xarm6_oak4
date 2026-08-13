#!/usr/bin/env python3
"""Replay verified eye-in-hand poses through MoveIt and collect fresh samples.

The seed NPZ stores poses that were reached manually while the fixed AprilTag
board was visible.  Replaying those known-good endpoints is deliberately more
conservative than inventing new Cartesian targets from an unverified hand-eye
transform.  MoveIt plans every segment before it is executed; dry-run is the
default and never sends a trajectory to the xArm.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import depthai as dai
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from .collect_eye_in_hand import (
    DEFAULT_BOARD_SIZE_M,
    DEFAULT_TAG_SIZE_M,
    SampleCollector,
    build_mono_stream,
    camera_socket,
    dictionary_candidates,
)
from .geometry import rotation_to_quaternion
from .solve_eye_in_hand import load_samples


def transform_to_pose(transform: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(value) for value in transform[:3, 3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(value) for value in rotation_to_quaternion(transform[:3, :3])
    )
    return pose


def wait_for_future(node, future, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
    if not future.done():
        raise TimeoutError(f"ROS action timed out after {timeout_s:.1f} s")
    return future.result()


class MoveItPoseRunner:
    """Plan and, only when explicitly requested, execute a TCP pose target."""

    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.move_group = ActionClient(node, MoveGroup, "/move_action")
        self.execute_trajectory = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

    def wait_for_servers(self):
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveIt /move_action is not available")
        if not self.execute_trajectory.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveIt /execute_trajectory is not available")

    def _goal(self, target: np.ndarray) -> MoveGroup.Goal:
        pose = transform_to_pose(target)
        constraint = Constraints(name="xarm_oak_handeye_verified_tcp_pose")

        position = PositionConstraint()
        position.header.frame_id = self.args.base_frame
        position.link_name = self.args.tcp_frame
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance_m]
        volume = BoundingVolume()
        volume.primitives.append(sphere)
        centre = Pose()
        centre.position = pose.position
        centre.orientation.w = 1.0
        volume.primitive_poses.append(centre)
        position.constraint_region = volume
        position.weight = 1.0
        constraint.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.args.base_frame
        orientation.link_name = self.args.tcp_frame
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self.args.orientation_tolerance_rad
        orientation.absolute_y_axis_tolerance = self.args.orientation_tolerance_rad
        orientation.absolute_z_axis_tolerance = self.args.orientation_tolerance_rad
        orientation.weight = 1.0
        constraint.orientation_constraints.append(orientation)

        request = MotionPlanRequest()
        request.group_name = self.args.planning_group
        request.num_planning_attempts = self.args.planning_attempts
        request.allowed_planning_time = self.args.planning_time_s
        request.max_velocity_scaling_factor = self.args.velocity_scale
        request.max_acceleration_scaling_factor = self.args.acceleration_scale
        request.goal_constraints.append(constraint)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def plan(self, target: np.ndarray):
        handle = wait_for_future(
            self.node, self.move_group.send_goal_async(self._goal(target)), self.args.action_timeout_s
        )
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the planning goal")
        wrapped_result = wait_for_future(
            self.node, handle.get_result_async(), self.args.action_timeout_s + self.args.planning_time_s
        )
        result = wrapped_result.result
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or result.error_code.val != 1:
            raise RuntimeError(f"MoveIt planning failed (error code {result.error_code.val})")
        trajectory = result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty trajectory")
        return trajectory

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = wait_for_future(
            self.node, self.execute_trajectory.send_goal_async(goal), self.args.action_timeout_s
        )
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the execution request")
        wrapped_result = wait_for_future(
            self.node, handle.get_result_async(), self.args.execution_timeout_s
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or wrapped_result.result.error_code.val != 1:
            raise RuntimeError(f"trajectory execution failed (error code {wrapped_result.result.error_code.val})")


def add_board_keepout(node, base_to_board: np.ndarray, metadata: dict, args):
    """Add a conservative collision box for the fixed physical AprilTag board."""
    pitch = float(metadata.get("grid_pitch_m", 0.02343))
    centre = np.asarray(base_to_board, dtype=float).copy()
    centre[:3, 3] += centre[:3, :3] @ np.array([2.5 * pitch, 2.5 * pitch, 0.0])

    collision = CollisionObject()
    collision.header.frame_id = args.base_frame
    collision.id = "xarm_oak_handeye_apriltag_board_keepout"
    collision.operation = CollisionObject.ADD
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    edge = float(metadata.get("board_extent_m", DEFAULT_BOARD_SIZE_M)) + 2.0 * args.board_margin_m
    box.dimensions = [edge, edge, args.board_thickness_m]
    collision.primitives.append(box)
    collision.primitive_poses.append(transform_to_pose(centre))

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(collision)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("MoveIt /apply_planning_scene is not available")
    request = ApplyPlanningScene.Request()
    request.scene = scene
    response = wait_for_future(node, client.call_async(request), timeout_s=5.0)
    if not response.success:
        raise RuntimeError("MoveIt did not accept the AprilTag board collision object")
    return client


def add_camera_keepout(node, tcp_to_camera: np.ndarray, args):
    """Attach a conservative camera-body sphere to the real TCP in MoveIt."""
    collision = CollisionObject()
    collision.header.frame_id = args.tcp_frame
    collision.id = "xarm_oak_handeye_oak_d_sr_camera_keepout"
    collision.operation = CollisionObject.ADD
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [args.camera_safety_radius_m]
    collision.primitives.append(sphere)
    collision.primitive_poses.append(transform_to_pose(tcp_to_camera))

    attached = AttachedCollisionObject()
    attached.link_name = args.tcp_frame
    attached.object = collision
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects.append(attached)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("MoveIt /apply_planning_scene is not available")
    request = ApplyPlanningScene.Request()
    request.scene = scene
    response = wait_for_future(node, client.call_async(request), timeout_s=5.0)
    if not response.success:
        raise RuntimeError("MoveIt did not accept the OAK-D-SR camera collision envelope")
    return client


def choose_indices(count: int, requested: int) -> np.ndarray:
    if requested <= 0 or requested >= count:
        return np.arange(count, dtype=int)
    # Preserve capture order.  The manually recorded order is intentionally a
    # conservative motion path through poses that were already reached safely.
    return np.unique(np.linspace(0, count - 1, requested, dtype=int))


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-samples", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--seed-result",
        type=Path,
        default=None,
        help="seed eye_in_hand_result.json; used to protect the fixed board during planning",
    )
    parser.add_argument("--target-count", type=int, default=0, help="0 means replay every seed pose")
    parser.add_argument("--execute", action="store_true", help="send planned trajectories to the real robot")
    parser.add_argument(
        "--allow-preliminary-result",
        action="store_true",
        help="acknowledge that seed poses may originate from a result not yet accepted for perception",
    )
    parser.add_argument("--planning-group", default="xarm6")
    parser.add_argument("--base-frame", default="link_base")
    parser.add_argument("--tcp-frame", default="link_tcp")
    parser.add_argument("--camera-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--position-tolerance-m", type=float, default=0.003)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--velocity-scale", type=float, default=0.10)
    parser.add_argument("--acceleration-scale", type=float, default=0.08)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--planning-time-s", type=float, default=8.0)
    parser.add_argument("--action-timeout-s", type=float, default=15.0)
    parser.add_argument("--execution-timeout-s", type=float, default=90.0)
    parser.add_argument("--settle-seconds", type=float, default=1.5)
    parser.add_argument("--frames-per-sample", type=int, default=5)
    parser.add_argument("--capture-timeout", type=float, default=12.0)
    parser.add_argument("--camera-socket", default="CAM_B", type=camera_socket)
    parser.add_argument("--dictionary", default="APRILTAG_36h11")
    parser.add_argument("--tag-size-mm", type=float, default=DEFAULT_TAG_SIZE_M * 1000.0)
    parser.add_argument("--board-size-mm", type=float, default=DEFAULT_BOARD_SIZE_M * 1000.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--detection-upscale", type=int, default=2)
    # SampleCollector rejects frames whose TF history still shows motion, so
    # the automatic runner must supply the same thresholds as the manual tool.
    parser.add_argument(
        "--settle-window",
        type=float,
        default=0.25,
        help="seconds of TF history that must show a stationary TCP; 0 disables the check",
    )
    parser.add_argument("--settle-translation-mm", type=float, default=0.5)
    parser.add_argument("--settle-rotation-deg", type=float, default=0.1)
    parser.add_argument("--min-grid-points", type=int, default=30)
    parser.add_argument("--max-reprojection-error-px", type=float, default=0.6)
    parser.add_argument("--board-margin-m", type=float, default=0.020)
    parser.add_argument("--board-thickness-m", type=float, default=0.020)
    parser.add_argument(
        "--camera-safety-radius-m",
        type=float,
        default=0.080,
        help="attached OAK-D-SR camera keep-out sphere radius",
    )
    return parser


def cleanup_listener(listener, node):
    if hasattr(listener, "executor"):
        listener.executor.shutdown()
        listener.dedicated_listener_thread.join(timeout=1.0)
    listener.unregister()
    node.destroy_node()


def main(argv=None):
    args = make_parser().parse_args(argv)
    if args.execute and not args.allow_preliminary_result:
        raise RuntimeError(
            "real motion requires --allow-preliminary-result after you have reviewed the dry-run plans"
        )
    if not 0.0 < args.velocity_scale <= 1.0 or not 0.0 < args.acceleration_scale <= 1.0:
        raise ValueError("velocity and acceleration scales must be in (0, 1]")
    if args.frames_per_sample < 1:
        raise ValueError("--frames-per-sample must be at least one")

    seed_base_tcp, _, _, _, metadata = load_samples(args.seed_samples)
    seed_result_path = args.seed_result or args.seed_samples.with_name("eye_in_hand_result.json")
    if not seed_result_path.is_file():
        raise FileNotFoundError(
            "--seed-result is required so the fixed AprilTag board can be added to MoveIt's collision scene"
        )
    seed_result = json.loads(seed_result_path.read_text(encoding="ascii"))
    if seed_result.get("mode") != "eye_in_hand" or "base_to_board" not in seed_result:
        raise ValueError("--seed-result is not a valid eye-in-hand result with base_to_board")
    base_to_board = np.asarray(seed_result["base_to_board"]["matrix"], dtype=float)
    tcp_to_camera = np.asarray(seed_result["tcp_to_camera"]["matrix"], dtype=float)
    quality = seed_result.get("quality", {})
    preliminary_rms = float(quality.get("handeye_translation_residual_mm_rms", float("nan")))
    indices = choose_indices(len(seed_base_tcp), args.target_count)
    if len(indices) < 8:
        raise ValueError("at least eight replay poses are required")
    output_dir = args.output_dir or (
        Path.home() / "oak_handeye" / time.strftime("auto_session_%Y%m%d_%H%M%S")
    )

    rclpy.init(args=None)
    motion_node = rclpy.create_node("xarm_oak_handeye_auto_motion")
    tf_node = rclpy.create_node("xarm_oak_handeye_auto_tf_listener")
    listener = TransformListener(Buffer(), tf_node, spin_thread=True)
    tf_buffer = listener.buffer
    pipeline = None
    try:
        runner = MoveItPoseRunner(motion_node, args)
        runner.wait_for_servers()
        camera_keepout_client = add_camera_keepout(motion_node, tcp_to_camera, args)
        board_keepout_client = add_board_keepout(motion_node, base_to_board, metadata, args)
        print(f"loaded {len(seed_base_tcp)} verified seed poses; selected {len(indices)}")
        print(
            "mode: EXECUTE at low speed" if args.execute else
            "mode: DRY-RUN only; no trajectory will be sent to the xArm"
        )
        if np.isfinite(preliminary_rms):
            print(f"seed result translation RMS: {preliminary_rms:.3f} mm")
        print(f"attached OAK-D-SR keep-out radius: {args.camera_safety_radius_m * 1000.0:.0f} mm")

        if not args.execute:
            # Dry-run plans every target from the current real state.  This
            # validates reachability and collisions without any robot motion.
            for ordinal, index in enumerate(indices, start=1):
                target = seed_base_tcp[index]
                runner.plan(target)
                xyz_mm = target[:3, 3] * 1000.0
                print(
                    f"plan {ordinal}/{len(indices)} seed={index + 1}: reachable "
                    f"tcp_mm=[{xyz_mm[0]:.1f}, {xyz_mm[1]:.1f}, {xyz_mm[2]:.1f}]"
                )
            print("dry-run passed: all selected seed poses are reachable; no robot motion was commanded")
            return

        tag_size_m = args.tag_size_mm / 1000.0
        board_size_m = args.board_size_mm / 1000.0
        candidates = dictionary_candidates(args.dictionary, tag_size_m, board_size_m)
        pipeline = dai.Pipeline()
        queue = build_mono_stream(pipeline, args.camera_socket, args.width, args.height, args.fps)
        pipeline.start()
        calibration = pipeline.getDefaultDevice().readCalibration()
        camera_matrix = np.asarray(
            calibration.getCameraIntrinsics(args.camera_socket, args.width, args.height), dtype=float
        )
        distortion = np.asarray(calibration.getDistortionCoefficients(args.camera_socket), dtype=float).reshape(-1, 1)
        collector = SampleCollector(args, camera_matrix, distortion, queue, candidates, tf_buffer, tf_node)

        for ordinal, index in enumerate(indices, start=1):
            # Plan immediately before every motion so MoveIt uses the previous
            # measured endpoint as the start state, not the initial pose.
            trajectory = runner.plan(seed_base_tcp[index])
            print(f"moving {ordinal}/{len(indices)} to verified seed pose {index + 1}")
            runner.execute(trajectory)
            time.sleep(args.settle_seconds)
            accepted, detail = collector.capture()
            print(f"sample {ordinal}/{len(indices)} seed={index + 1}: {detail}")
            if not accepted:
                raise RuntimeError("automatic collection stopped because the fixed board was not accepted")

        metadata.update({
            "schema": "xarm_oak_handeye_auto_eye_in_hand_v1",
            "collection_mode": "automatic_replay_of_verified_seed_poses",
            "seed_samples": str(args.seed_samples),
            "seed_indices_zero_based": [int(index) for index in indices],
            "frames_per_sample": args.frames_per_sample,
            "settle_seconds": args.settle_seconds,
            "velocity_scale": args.velocity_scale,
            "acceleration_scale": args.acceleration_scale,
            "captured_at_unix_s": time.time(),
        })
        saved = collector.save(output_dir, metadata)
        print(f"saved {len(collector.base_to_tcp)} automatic samples: {saved}")
        del board_keepout_client
        del camera_keepout_client
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        cleanup_listener(listener, tf_node)
        motion_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
