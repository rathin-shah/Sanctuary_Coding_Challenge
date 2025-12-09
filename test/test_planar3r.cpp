#include <gtest/gtest.h>
#include "planar_kinematics/Planar3R.hpp"
#include <cmath>

static double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0*M_PI;
    while (d <= -M_PI) d += 2.0*M_PI;
    return d;
}

TEST(Planar3R, ForwardKinematicsCorrectness)
{
    Planar3R kin(0.3, 0.3, 0.1);

    struct Case {
        double th1, th2, th3;
        double x, y, phi;
    };

    // Expected values using analytic formulas
    std::vector<Case> tests = {
        {0.0, 0.0, 0.0,
         0.3 + 0.3 + 0.1,
         0.0,
         0.0},

        {M_PI/2, 0.0, 0.0,
         0.0,
         0.3 + 0.3 + 0.1,
         M_PI/2},

        {0.3, -0.2, 0.5,
         0.3*std::cos(0.3)
         +0.3*std::cos(0.3-0.2)
         +0.1*std::cos(0.3-0.2+0.5),
         0.3*std::sin(0.3)
         +0.3*std::sin(0.3-0.2)
         +0.1*std::sin(0.3-0.2+0.5),
         0.3 - 0.2 + 0.5},

        {-0.4, 0.7, -0.3,
         0.3*std::cos(-0.4)
         +0.3*std::cos(-0.4+0.7)
         +0.1*std::cos(-0.4+0.7-0.3),
         0.3*std::sin(-0.4)
         +0.3*std::sin(-0.4+0.7)
         +0.1*std::sin(-0.4+0.7-0.3),
         -0.4 + 0.7 - 0.3}
    };

    for (auto &c : tests) {
        auto p = kin.computeFK(c.th1, c.th2, c.th3);

        EXPECT_NEAR(p.x, c.x, 1e-9);
        EXPECT_NEAR(p.y, c.y, 1e-9);
        EXPECT_NEAR(p.phi, c.phi, 1e-9);
    }
}

TEST(Planar3R, KinematicsModelProperties)
{
    Planar3R kin(0.3, 0.3, 0.1);

    // 1. At zero angles, arm is stretched on +X
    auto p0 = kin.computeFK(0,0,0);
    EXPECT_NEAR(p0.x, 0.3 + 0.3 + 0.1, 1e-12);
    EXPECT_NEAR(p0.y, 0, 1e-12);
    EXPECT_NEAR(p0.phi, 0, 1e-12);

    // 2. Small angle increases change FK slightly
    auto p1 = kin.computeFK(0.01, 0, 0);
    EXPECT_GT(p1.y, 0.0);     // should move upward
    EXPECT_LT(p1.x, p0.x);    // tip moves slightly inward

    // 3. th3 affects only the last segment — finite difference check
    auto p2 = kin.computeFK(0.5, 0.5, 0.5);
    auto p3 = kin.computeFK(0.5, 0.5, 0.5 + 0.1);

    double dp_x = p3.x - p2.x;
    double dp_y = p3.y - p2.y;

    double t123 = 0.5 + 0.5 + 0.5;
    double L3 = 0.1;

    double expected_dx =
        L3 * (std::cos(t123 + 0.1) - std::cos(t123));
    double expected_dy =
        L3 * (std::sin(t123 + 0.1) - std::sin(t123));

    EXPECT_NEAR(dp_x, expected_dx, 1e-9);
    EXPECT_NEAR(dp_y, expected_dy, 1e-9);

    // 4. Increasing th1 should rotate entire arm CCW
    auto p4 = kin.computeFK(0.2, 0.0, 0.0);
    auto p5 = kin.computeFK(0.3, 0.0, 0.0);

    EXPECT_GT(p5.y, p4.y);
    EXPECT_LT(p5.x, p4.x);
}

// Test that IK solves the pose correctly (pose match, not angle match)
TEST(Planar3R, ForwardInverseConsistency)
{
    Planar3R kin(0.3, 0.3, 0.1);

    std::vector<std::array<double,3>> tests = {
        { 0.0,  0.0,  0.0},
        { 0.3, -0.4,  0.2},
        { 1.0, -0.5, -0.3},
        {-0.7,  0.6,  0.4},
        { 2.5, -1.0,  0.5}
    };

    for (auto &th : tests) {

        auto p = kin.computeFK(th[0], th[1], th[2]);
        bool solved = false;

        for (int branch : {+1, -1}) {
            auto sol = kin.computeIK(p.x, p.y, p.phi, branch);
            if (!sol.has_value()) continue;

            auto q = *sol;
            auto p2 = kin.computeFK(q[0], q[1], q[2]);

            bool match = true;
            match &= std::abs(p2.x - p.x) < 1e-6;
            match &= std::abs(p2.y - p.y) < 1e-6;
            match &= std::abs(angle_diff(p2.phi, p.phi)) < 1e-6;

            if (match) {
                solved = true;
                break;
            }
        }

        EXPECT_TRUE(solved)
            << "IK could not reproduce FK pose for joints: "
            << th[0] << ", " << th[1] << ", " << th[2];
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
