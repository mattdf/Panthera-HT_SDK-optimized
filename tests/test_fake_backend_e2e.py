import numpy as np

from Panthera_lib import (
    CartesianWaypoint,
    GraspCandidate,
    JointTrajectory,
    ManipulationPlan,
    RobotState,
    SimPanthera,
    grasp_to_waypoints,
)
from Panthera_lib.pipeline import object_pose_from_camera_detection


def test_fake_backend_grasp_pipeline_executes_end_to_end():
    robot = SimPanthera(dof=6)

    camera_to_robot = np.eye(4)
    camera_to_robot[:3, 3] = [0.3, -0.1, 0.2]
    robot.set_camera_to_robot_transform(camera_to_robot)

    object_pose = object_pose_from_camera_detection(
        object_id="small_part",
        position_camera=[0.02, 0.03, 0.15],
        rotation_camera=np.eye(3),
        camera_to_robot=camera_to_robot,
        confidence=0.87,
    )
    grasp = GraspCandidate(
        object_id=object_pose.object_id,
        position=object_pose.position,
        rotation=object_pose.rotation,
        width=0.018,
        score=0.92,
    )

    waypoints = grasp_to_waypoints(grasp)
    plan = robot.plan_cartesian(waypoints, duration=1.2)

    assert len(waypoints) == 3
    assert isinstance(plan, ManipulationPlan)
    assert isinstance(plan.joint_trajectory, JointTrajectory)
    assert plan.joint_trajectory.positions.shape == (3, 6)
    assert plan.joint_trajectory.velocities.shape == (3, 6)
    assert np.allclose(plan.joint_trajectory.timestamps, [0.0, 0.6, 1.2])

    assert robot.execute_trajectory(plan)
    state = robot.get_state()

    assert isinstance(state, RobotState)
    assert state.joint_position.shape == (6,)
    assert np.allclose(state.joint_position, plan.joint_trajectory.positions[-1])

    assert robot.close_gripper(pos=0.1, vel=0.4, max_torque=0.6)
    assert robot.open_gripper(pos=1.2, vel=0.5, max_torque=0.4)
    state = robot.get_state()
    assert state.gripper_position == 1.2
    assert state.gripper_velocity == 0.5
    assert state.gripper_torque == 0.4


def test_fake_backend_accepts_cartesian_waypoint_dataclasses():
    robot = SimPanthera(dof=7)
    waypoints = [
        CartesianWaypoint(position=np.array([0.2, 0.0, 0.3]), rotation=np.eye(3)),
        CartesianWaypoint(position=np.array([0.25, 0.01, 0.32]), rotation=np.eye(3)),
    ]

    plan = robot.plan_cartesian(waypoints)

    assert plan.cartesian_fraction == 1.0
    assert plan.joint_trajectory.positions.shape == (2, 7)

    fk = robot.fk(plan.joint_trajectory.positions[-1])
    assert set(fk) == {"position", "rotation", "transform", "joint_angles"}
    assert fk["position"].shape == (3,)
    assert fk["rotation"].shape == (3, 3)
    assert fk["transform"].shape == (4, 4)

    ik = robot.ik(fk["position"], fk["rotation"])
    assert ik.shape == (7,)


def test_python_extension_and_motor_alias_import_from_built_package():
    import hightorque_motor
    import hightorque_robot

    assert hightorque_motor.Robot is hightorque_robot.Robot

    state = hightorque_robot.MotorState()
    assert hasattr(state, "position")
    assert hasattr(state, "velocity")
    assert hasattr(state, "torque")
