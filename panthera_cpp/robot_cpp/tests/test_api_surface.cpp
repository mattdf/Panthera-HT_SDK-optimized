#include <panthera/Panthera.hpp>
#include <hightorque_logger.hpp>

#include <cassert>
#include <type_traits>

int main()
{
    static_assert(std::is_default_constructible<panthera::JointCommand>::value,
                  "JointCommand should be a simple value type");
    static_assert(std::is_default_constructible<panthera::JointMitCommand>::value,
                  "JointMitCommand should be a simple value type");
    static_assert(std::is_default_constructible<panthera::JointTrajectoryPoint>::value,
                  "JointTrajectoryPoint should be a simple value type");

    panthera::JointCommand command;
    command.position = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    command.velocity = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    command.max_torque = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};
    assert(command.position.size() == command.velocity.size());
    assert(command.position.size() == command.max_torque.size());

    panthera::JointMitCommand mit_command;
    mit_command.position = command.position;
    mit_command.velocity = command.velocity;
    mit_command.torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mit_command.kp = {10.0, 10.0, 10.0, 5.0, 5.0, 5.0};
    mit_command.kd = {0.5, 0.5, 0.5, 0.2, 0.2, 0.2};
    assert(mit_command.position.size() == mit_command.kp.size());

    panthera::JointTrajectoryPoint point;
    point.position = command.position;
    point.velocity = command.velocity;
    point.time_from_start = 0.25;
    assert(point.time_from_start > 0.0);

    panthera::CartesianPose pose;
    assert(pose.position.isApprox(Eigen::Vector3d::Zero()));
    assert(pose.rotation.isApprox(Eigen::Matrix3d::Identity()));

    hightorque_robot::set_log_level(hightorque_robot::LogLevel::Error);
    assert(hightorque_robot::get_log_level() == hightorque_robot::LogLevel::Error);

    return 0;
}
