from dataclasses import dataclass

import numpy as np


@dataclass
class RobotState:
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    joint_torque: np.ndarray
    gripper_position: float
    gripper_velocity: float
    gripper_torque: float


@dataclass
class JointTrajectory:
    positions: np.ndarray
    timestamps: np.ndarray
    velocities: np.ndarray


@dataclass
class ManipulationPlan:
    joint_trajectory: JointTrajectory
    cartesian_fraction: float


@dataclass
class ObjectPose:
    object_id: str
    position: np.ndarray
    rotation: np.ndarray
    confidence: float = 1.0


@dataclass
class GraspCandidate:
    object_id: str
    position: np.ndarray
    rotation: np.ndarray
    width: float
    score: float
    approach_distance: float = 0.08
    retreat_distance: float = 0.08


@dataclass
class CartesianWaypoint:
    position: np.ndarray
    rotation: np.ndarray

    def as_dict(self):
        return {
            "position": np.asarray(self.position, dtype=float),
            "rotation": np.asarray(self.rotation, dtype=float),
        }


def grasp_to_waypoints(grasp: GraspCandidate):
    rotation = np.asarray(grasp.rotation, dtype=float)
    position = np.asarray(grasp.position, dtype=float)
    approach_axis = rotation[:, 2]

    pregrasp = CartesianWaypoint(
        position=position - approach_axis * grasp.approach_distance,
        rotation=rotation,
    )
    grasp_pose = CartesianWaypoint(position=position, rotation=rotation)
    retreat = CartesianWaypoint(
        position=position - approach_axis * grasp.retreat_distance,
        rotation=rotation,
    )
    return [pregrasp, grasp_pose, retreat]
