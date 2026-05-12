import numpy as np

from .types import CartesianWaypoint, GraspCandidate, ObjectPose, grasp_to_waypoints


def object_pose_from_camera_detection(object_id, position_camera, rotation_camera, confidence, camera_to_robot):
    transform = np.asarray(camera_to_robot, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("camera_to_robot must be a 4x4 transform")

    point_h = np.ones(4)
    point_h[:3] = np.asarray(position_camera, dtype=float)
    position_robot = (transform @ point_h)[:3]
    rotation_robot = transform[:3, :3] @ np.asarray(rotation_camera, dtype=float)
    return ObjectPose(object_id=object_id, position=position_robot, rotation=rotation_robot, confidence=confidence)


def plan_grasp_waypoints(grasp: GraspCandidate):
    return [waypoint.as_dict() for waypoint in grasp_to_waypoints(grasp)]


def cartesian_waypoints_to_dicts(waypoints):
    result = []
    for waypoint in waypoints:
        if isinstance(waypoint, CartesianWaypoint):
            result.append(waypoint.as_dict())
        else:
            result.append(waypoint)
    return result
