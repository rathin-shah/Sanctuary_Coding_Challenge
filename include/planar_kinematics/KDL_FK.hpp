#pragma once

#include <memory>
#include <string>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>

class KDL_FK
{
public:

    struct Pose2D { double x, y, phi; };

    explicit KDL_FK(const std::string& urdf_path)
    {
        // ---------------------------
        // Load URDF using ROS2 API
        // ---------------------------
        auto model = urdf::parseURDFFile(urdf_path);
        if (!model) {
            throw std::runtime_error("Failed to load URDF: " + urdf_path);
        }

        // --------------------------------
        // Convert URDF → KDL Tree
        // --------------------------------
        if (!kdl_parser::treeFromUrdfModel(*model, tree_)) {
            throw std::runtime_error("Failed to convert URDF to KDL Tree");
        }

        // --------------------------------
        // Extract KDL chain from URDF links
        // --------------------------------
        if (!tree_.getChain("base_link", "ee_link", chain_)) {
            throw std::runtime_error("Failed to get KDL chain (base_link → ee_link)");
        }
        for (unsigned int i=0; i<chain_.getNrOfSegments(); ++i) {
            auto seg = chain_.getSegment(i);
            std::cerr << "Segment " << i << " name: " 
                    << seg.getName() 
                    << " origin: " 
                    << seg.getFrameToTip().p.x() << ", "
                    << seg.getFrameToTip().p.y() << "\n";
        }

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    }

    Pose2D compute(double th1, double th2, double th3)
    {
        KDL::JntArray q(3);
        q(0) = th1;
        q(1) = th2;
        q(2) = th3;

        KDL::Frame out;
        fk_solver_->JntToCart(q, out);

        double x = out.p.x();
        double y = out.p.y();

        // Yaw angle from rotation matrix
        double phi = atan2(out.M.UnitY().x(), out.M.UnitX().x());

        return {x, y, phi};
    }

private:
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};
