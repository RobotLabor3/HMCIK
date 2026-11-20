#include <hmcik/left_arm_kine.h>
#include <hmcik/right_arm_kine.h>
#include <ros/ros.h>

#include <cmath>
#include <exception>
#include <iostream>
#include <vector>

namespace
{
template <typename Arm>
bool testArm(const char *name, double lateral_angle)
{
    Arm arm("config_double.txt", false, 42);
    const Eigen::Matrix4d target = arm.calculate_T14G(0.35, lateral_angle, 0.3, -0.9, 0.45, 0.25, 0.1);
    int iterations = 0;
    const int status = arm.GARR ? arm.solve(arm.max_count, arm.eps, target, iterations)
                                : arm.solveSelfRotation(arm.max_count, arm.eps, target, iterations);
    if (status != 1)
    {
        std::cerr << name << " solve status=" << status << '\n';
        return false;
    }

    std::vector<double> config;
    arm.getConfig(config);
    const Eigen::Matrix4d desired = target * arm.Tend_1;
    const Eigen::Matrix4d achieved = arm.calculate_T14G(
        config.at(7), config.at(8), config.at(9), config.at(10),
        config.at(11), config.at(12), config.at(13)) * arm.Tend_1;
    const double error = (achieved.block<3, 1>(0, 3) - desired.block<3, 1>(0, 3)).norm();
    std::cout << name << " iterations=" << iterations << ", position error=" << error << " mm\n";
    return std::isfinite(error) && error <= arm.eps + 1e-8;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmcik_test");
    try
    {
        const bool left_ok = testArm<LeftArmKine>("LeftArmKine", 0.6);
        const bool right_ok = testArm<RightArmKine>("RightArmKine", -0.6);
        return left_ok && right_ok ? 0 : 1;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
