#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"


using std::placeholders::_1;


float x = 0;
float y = 0;
float z = 0;
float vx = 0;
float vy = 0;
float vz = 0; 

class Subscription : public rclcpp::Node
{
public:
    Subscription() : Node("subscriber_node")
    {
        auto qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&Subscription::callback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscriber node create");
    }
private:
    void callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        x = msg -> x;
        y = msg -> y;
        z = msg -> z;
        vx = msg -> vx;
        vy = msg -> vy;
        vz = msg -> vz;

        RCLCPP_INFO(this->get_logger(),
            "\nLocal Position:\n"
            "Time: %lu\n"
            "Position: x = %.2f, y = %.2f, z = %.2f\n"
            "Velocity: vx = %.2f, vy = %.2f, vz = %.2f\n",
            msg -> timestamp, x, y, z, vx, vy, vz
        );
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // auto pub_node = std::make_shared<Publisher>();
    auto sub_node = std::make_shared<Subscription>();

    auto executors = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // executors->add_node(pub_node);
    executors->add_node(sub_node);

    executors->spin();
    rclcpp::shutdown();
    return 0;
}