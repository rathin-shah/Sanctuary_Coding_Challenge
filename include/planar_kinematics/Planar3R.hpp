#pragma once
// Unified Planar 3R FK + IK class (header-only)

#include <array>
#include <optional>
#include <cmath>
#include <algorithm>

/**
 * @brief Analytic forward/inverse kinematics for a planar 3R manipulator.
 *
 * Link lengths are in meters and angular quantities are in radians.
 */
class Planar3R {
public:
    /**
     * @brief Construct a Planar3R solver with optional default link lengths.
     *
     * @param L1 Link 1 length in meters (default 0.3).
     * @param L2 Link 2 length in meters (default 0.3).
     * @param L3 Link 3 length in meters (default 0.1).
     */
    Planar3R(double L1 = 0.3, double L2 = 0.3, double L3 = 0.1)
        : L1_(L1), L2_(L2), L3_(L3) {}

    /**
     * @brief Set the link lengths at runtime.
     *
     * @param L1 Link 1 length in meters.
     * @param L2 Link 2 length in meters.
     * @param L3 Link 3 length in meters.
     */
    void setLinks(double L1, double L2, double L3) {
        L1_ = L1; L2_ = L2; L3_ = L3;
    }

    /**
     * @brief 2D pose container used for FK results.
     *
     * Members: `x` and `y` are in meters, `phi` is yaw in radians.
     */
    struct Pose2D {
        double x;
        double y;
        double phi;
    };

    /**
     * @brief Compute forward kinematics for the 3R chain.
     *
     * @param th1 Joint 1 angle (radians).
     * @param th2 Joint 2 angle (radians).
     * @param th3 Joint 3 angle (radians).
     * @return Pose2D End-effector pose (x, y, phi).
     */
    Pose2D computeFK(double th1, double th2, double th3) const {
        double t12 = th1 + th2;
        double t123 = t12 + th3;

        double c1 = std::cos(th1);
        double s1 = std::sin(th1);
        double c12 = std::cos(t12);
        double s12 = std::sin(t12);
        double c123 = std::cos(t123);
        double s123 = std::sin(t123);

        Pose2D p;
        p.x = L1_*c1 + L2_*c12 + L3_*c123;
        p.y = L1_*s1 + L2_*s12 + L3_*s123;
        p.phi = t123;
        return p;
    }

    /**
     * @brief Compute inverse kinematics for a desired end-effector pose.
     *
     * The method computes joint angles (th1, th2, th3) that achieve the
     * pose (x, y, phi). The `branch` parameter selects the elbow configuration
     * (+1 for one branch, -1 for the other). If the pose is unreachable the
     * function returns `std::nullopt`.
     *
     * @param x Desired X position (meters).
     * @param y Desired Y position (meters).
     * @param phi Desired end-effector yaw (radians).
     * @param branch Elbow branch selection (+1 or -1). Default +1.
     * @param tol Numerical tolerance (unused in current implementation).
     * @return std::optional<std::array<double,3>> Joint angles (rad) or nullopt if unreachable.
     */
    std::optional<std::array<double,3>> computeIK(
        double x, double y, double phi,
        int branch = +1,
        double tol = 1e-12
    ) const 
    {

    double xw = x - L3_ * std::cos(phi);
    double yw = y - L3_ * std::sin(phi);


    double r2 = xw*xw + yw*yw;

    double c2 = (r2 - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);

    const double eps = 1e-9;
    if (c2 > 1.0 + eps || c2 < -1.0 - eps) {
        // truly unreachable
        return std::nullopt;
    }
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;


    double s2 = branch * std::sqrt(std::max(0.0, 1.0 - c2*c2));
    double th2 = std::atan2(s2, c2);

    double k1 = L1_ + L2_ * c2;
    double k2 = L2_ * s2;

    double th1 = std::atan2(yw, xw) - std::atan2(k2, k1);

    double th3 = phi - th1 - th2;

    auto norm = [](double a){
        while (a >  M_PI) a -= 2*M_PI;
        while (a <= -M_PI) a += 2*M_PI;
        return a;
    };

    th1 = norm(th1);
    th2 = norm(th2);
    th3 = norm(th3);

    return std::array<double,3>{ th1, th2, th3 };
}

private:
    double L1_, L2_, L3_;

    static void normalize(double &a) {
        while (a > M_PI) a -= 2.0*M_PI;
        while (a <= -M_PI) a += 2.0*M_PI;
    }
};
