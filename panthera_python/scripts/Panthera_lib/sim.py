import numpy as np

from .pipeline import cartesian_waypoints_to_dicts
from .types import JointTrajectory, ManipulationPlan, RobotState


class SimPanthera:
    """Small hardware-free backend for planner and policy integration tests."""

    def __init__(self, dof=6):
        self.motor_count = dof
        self.joint_position = np.zeros(dof)
        self.joint_velocity = np.zeros(dof)
        self.joint_torque = np.zeros(dof)
        self.gripper_position = 0.0
        self.gripper_velocity = 0.0
        self.gripper_torque = 0.0
        self.camera_to_robot = np.eye(4)

    def get_state(self):
        return RobotState(
            joint_position=self.joint_position.copy(),
            joint_velocity=self.joint_velocity.copy(),
            joint_torque=self.joint_torque.copy(),
            gripper_position=self.gripper_position,
            gripper_velocity=self.gripper_velocity,
            gripper_torque=self.gripper_torque,
        )

    def fk(self, joint_angles=None):
        q = self.joint_position if joint_angles is None else np.asarray(joint_angles, dtype=float)
        transform = np.eye(4)
        transform[0, 3] = float(np.sum(np.cos(q)) * 0.03)
        transform[1, 3] = float(np.sum(np.sin(q)) * 0.03)
        transform[2, 3] = 0.2
        return {
            "position": transform[:3, 3].copy(),
            "rotation": transform[:3, :3].copy(),
            "transform": transform,
            "joint_angles": q.copy(),
        }

    def ik(self, target_position, target_rotation=None, init_q=None, **kwargs):
        q = self.joint_position if init_q is None else np.asarray(init_q, dtype=float)
        return q.copy()

    def plan_cartesian(self, waypoints, duration=None, smooth=True):
        waypoints = cartesian_waypoints_to_dicts(waypoints)
        n = max(2, len(waypoints))
        timestamps = np.linspace(0.0, duration if duration is not None else float(n - 1), n)
        positions = np.tile(self.joint_position, (n, 1))
        velocities = np.zeros_like(positions)
        return ManipulationPlan(
            joint_trajectory=JointTrajectory(positions=positions, timestamps=timestamps, velocities=velocities),
            cartesian_fraction=1.0,
        )

    def execute_trajectory(self, trajectory, max_torque=None):
        if isinstance(trajectory, ManipulationPlan):
            trajectory = trajectory.joint_trajectory
        if isinstance(trajectory, dict):
            positions = np.asarray(trajectory["positions"], dtype=float)
            velocities = np.asarray(trajectory.get("velocities", np.zeros_like(positions)), dtype=float)
        else:
            positions = np.asarray(trajectory.positions, dtype=float)
            velocities = np.asarray(trajectory.velocities, dtype=float)
        if len(positions) > 0:
            self.joint_position = positions[-1].copy()
            self.joint_velocity = velocities[-1].copy()
        return True

    def open_gripper(self, pos=1.6, vel=0.5, max_torque=0.5):
        self.gripper_position = pos
        self.gripper_velocity = vel
        self.gripper_torque = max_torque
        return True

    def close_gripper(self, pos=0.0, vel=0.5, max_torque=0.5):
        self.gripper_position = pos
        self.gripper_velocity = vel
        self.gripper_torque = max_torque
        return True

    def set_camera_to_robot_transform(self, transform):
        transform = np.asarray(transform, dtype=float)
        if transform.shape != (4, 4):
            raise ValueError("camera_to_robot transform must be a 4x4 matrix")
        self.camera_to_robot = transform

    def camera_point_to_robot(self, point):
        point_h = np.ones(4)
        point_h[:3] = np.asarray(point, dtype=float)
        return (self.camera_to_robot @ point_h)[:3]
