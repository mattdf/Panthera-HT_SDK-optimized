"""
Panthera-HT 机械臂控制库

提供高层次的机械臂控制接口，包括运动学、动力学、轨迹规划和主从遥操作功能��

使用示例:
    from Panthera_lib import Panthera, TrajectoryRecorder

    # 创建机器人实例
    robot = Panthera()

    # 获取当前关节角度
    joint_pos = robot.get_current_pos()

    # 单关节位置速度控制（每个关节独立设置）
    robot.Joint_Pos_Vel([0, 0.5, -0.5, 0, 0.5, 0],
                        [0.5]*6, [10, 10, 10, 5, 5, 5])

    # 多关节同步到达控制（所有关节同时到达）
    robot.Joints_Sync_Arrival([0, 0.5, -0.5, 0, 0.5, 0],
                              duration=2.0)
"""

__version__ = "1.0.0"
__author__ = "HighTorque Robotics"

from .sim import SimPanthera
from .types import (
    CartesianWaypoint,
    GraspCandidate,
    JointTrajectory,
    ManipulationPlan,
    ObjectPose,
    RobotState,
    grasp_to_waypoints,
)


def __getattr__(name):
    if name == "Panthera":
        from .Panthera import Panthera
        return Panthera
    if name == "TrajectoryRecorder":
        from .recorder import Recorder
        return Recorder
    raise AttributeError(name)

# 导出公共API
__all__ = [
    'Panthera',
    'JointTrajectory',
    'ManipulationPlan',
    'RobotState',
    'ObjectPose',
    'GraspCandidate',
    'CartesianWaypoint',
    'grasp_to_waypoints',
    'SimPanthera',
    'TrajectoryRecorder',
]
