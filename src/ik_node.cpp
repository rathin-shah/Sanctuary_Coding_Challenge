#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "planar_kinematics/Planar3R.hpp"

class IKNode : public rclcpp::Node {
public:
    IKNode() : Node("ik_node") {
        declare_parameter<double>("L1", 0.3);
        declare_parameter<double>("L2", 0.3);
        declare_parameter<double>("L3", 0.1);
        double L1 = get_parameter("L1").as_double();
        double L2 = get_parameter("L2").as_double();
        double L3 = get_parameter("L3").as_double();
        kin_.setLinks(L1, L2, L3);

        sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "ee_pose", 10,
            std::bind(&IKNode::poseCb, this, std::placeholders::_1)
        );
    }

private:
    void poseCb(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        double x = msg->x;
        double y = msg->y;
        double phi = msg->theta;

        // try default branch (+1); could try branch -1 if fail
        auto sol = kin_.computeIK(x, y, phi, +1);
        if (!sol) {
            RCLCPP_WARN(get_logger(), "IK unreachable for x=%.6f y=%.6f phi=%.6f; trying other branch", x, y, phi);
            sol = kin_.computeIK(x, y, phi, -1);
            if (!sol) {
                RCLCPP_ERROR(get_logger(), "IK unreachable on both branches");
                return;
            }
        }

        auto arr = *sol;
        RCLCPP_INFO(get_logger(), "IK solution: th1=%.6f th2=%.6f th3=%.6f", arr[0], arr[1], arr[2]);
    }

    Planar3R kin_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
