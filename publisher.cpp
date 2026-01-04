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

class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("publisher_node")
    {
        auto qos = rclcpp::QoS(10);

        // publisher_ = this->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>(
        //     "/fmu/out/vehicle_local_positionsetpoint", qos
        // );

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos
        );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Publisher::publish_setpoint, this));
        RCLCPP_INFO(this->get_logger(), "Publisher node create");
    }
private:
    void publish_setpoint()
    {
        // px4_msgs::msg::VehicleLocalPositionSetpoint msg;
        px4_msgs::msg::VehicleLocalPosition msg;
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;

        msg.x = 100;
        msg.y = 100;
        msg.z = 100;
        // msg.yaw = 0.0;

        // RCLCPP_INFO(this -> get_logger(), "\nPosition_setpoint: "
        //     "x = %.2f, y = %.2f, z = %.2f",
        //     msg.x, msg.y, msg.z
        // );
        publisher_ -> publish(msg);

    }
    // rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto pub_node = std::make_shared<Publisher>();

    auto executors = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executors->add_node(pub_node);

    executors->spin();
    rclcpp::shutdown();
    return 0;
}