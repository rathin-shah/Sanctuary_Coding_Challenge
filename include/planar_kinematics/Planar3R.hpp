#pragma once
// Unified Planar 3R FK + IK class (header-only)

#include <array>
#include <optional>
#include <cmath>
#include <algorithm>

class Planar3R {
public:
    Planar3R(double L1 = 0.3, double L2 = 0.3, double L3 = 0.1)
        : L1_(L1), L2_(L2), L3_(L3) {}

    void setLinks(double L1, double L2, double L3) {
        L1_ = L1; L2_ = L2; L3_ = L3;
    }

    struct Pose2D {
        double x;
        double y;
        double phi;
    };

    // -------------------------
    // Forward Kinematics (analytic)
    // -------------------------
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

    // -------------------------
    // Analytic Inverse Kinematics (wrist reduction)
    // Returns std::nullopt if unreachable.
    // branch = +1 => one elbow branch (default)
    // branch = -1 => the other branch
    // -------------------------
    std::optional<std::array<double,3>> computeIK(
        double x, double y, double phi,
        int branch = +1,
        double tol = 1e-12
    ) const 
    {
    // Wrist position (subtract link 3)
    double xw = x - L3_ * std::cos(phi);
    double yw = y - L3_ * std::sin(phi);

    // Two-link IK
    double r2 = xw*xw + yw*yw;

    double c2 = (r2 - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);

    // ============================
    // CRITICAL: Clamp c2 to [-1,1]
    // ============================
    const double eps = 1e-9;
    if (c2 > 1.0 + eps || c2 < -1.0 - eps) {
        // truly unreachable
        return std::nullopt;
    }
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    // s2 robust
    double s2 = branch * std::sqrt(std::max(0.0, 1.0 - c2*c2));
    double th2 = std::atan2(s2, c2);

    // th1
    double k1 = L1_ + L2_ * c2;
    double k2 = L2_ * s2;

    double th1 = std::atan2(yw, xw) - std::atan2(k2, k1);

    // th3
    double th3 = phi - th1 - th2;

    // Normalize angles
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
