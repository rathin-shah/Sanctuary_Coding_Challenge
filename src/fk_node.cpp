#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "planar_kinematics/Planar3R.hpp"
#include "planar_kinematics/KDL_FK.hpp"

using namespace std::chrono_literals;

class FKNode : public rclcpp::Node
{
public:
    FKNode() : Node("fk_node")
    {

        declare_parameter<std::string>("fk_mode", "analytical");
        declare_parameter<std::string>("urdf_path", "");

        declare_parameter<double>("th1", 0.0);
        declare_parameter<double>("th2", 0.0);
        declare_parameter<double>("th3", 0.0);

        declare_parameter<double>("L1", 0.3);
        declare_parameter<double>("L2", 0.3);
        declare_parameter<double>("L3", 0.1);

        fk_mode_ = get_parameter("fk_mode").as_string();

        th1_ = get_parameter("th1").as_double();
        th2_ = get_parameter("th2").as_double();
        th3_ = get_parameter("th3").as_double();

        if (fk_mode_ == "analytical") {
            double L1 = get_parameter("L1").as_double();
            double L2 = get_parameter("L2").as_double();
            double L3 = get_parameter("L3").as_double();
            analytical_fk_ = std::make_unique<Planar3R>(L1, L2, L3);

            RCLCPP_INFO(get_logger(), "FK MODE: analytical");
        }
        else if (fk_mode_ == "kdl")
        {
            std::string urdf_path = get_parameter("urdf_path").as_string();
            if (urdf_path.empty()) {
                RCLCPP_FATAL(get_logger(),
                    "fk_mode is 'kdl' but urdf_path parameter is empty!");
                throw std::runtime_error("Missing URDF path for KDL FK");
            }

            kdl_fk_ = std::make_unique<KDL_FK>(urdf_path);
            RCLCPP_INFO(get_logger(), "FK MODE: KDL (URDF path: %s)",
                        urdf_path.c_str());
        }
        else {
            RCLCPP_FATAL(get_logger(),
                "Invalid fk_mode '%s'. Use 'analytical' or 'kdl'.",
                fk_mode_.c_str());
            throw std::runtime_error("Invalid fk_mode");
        }

        pub_ = create_publisher<geometry_msgs::msg::Pose2D>("ee_pose", 10);
        timer_ = create_wall_timer(100ms, std::bind(&FKNode::compute_and_publish, this));
    }

private:

    void compute_and_publish()
    {
        geometry_msgs::msg::Pose2D msg;

        if (fk_mode_ == "analytical") {

            auto p = analytical_fk_->computeFK(th1_, th2_, th3_);
            msg.x = p.x;
            msg.y = p.y;
            msg.theta = p.phi;
        }
        else {  // kdl mode

            auto p = kdl_fk_->compute(th1_, th2_, th3_);
            msg.x = p.x;
            msg.y = p.y;
            msg.theta = -1 * p.phi;

        }

        pub_->publish(msg);
    }

    
    std::string fk_mode_;
    double th1_, th2_, th3_;

    
    std::unique_ptr<Planar3R> analytical_fk_;
    std::unique_ptr<KDL_FK>    kdl_fk_;

    
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FKNode>());
    rclcpp::shutdown();
    return 0;
}
