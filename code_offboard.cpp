#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <cmath> 
#include <array>
#include <chrono> 
#include <iostream>

#include "package_cpp/params_offboard.hpp"

using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


enum class MODE {
    IDE,
    TAKEOFF,
    HOVER,
    MOVE_X_1,
    HOVER_X_1,
    MOVE_Y_1,
    HOVER_Y_1,
    MOVE_3,
    HOVER_3,
    DONE,
    MOVE_CIRCLE
};
MODE mode = MODE::IDE;  

class Subscription : public rclcpp::Node
{
public:
    Subscription() : Node("Subscriber")
    {
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&Subscription::vehicle_local_position, this, _1));

        vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&Subscription::vehicle_attitude, this, _1));

        vehicle_angular_velocity_subscriber_ = this->create_subscription<VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos, std::bind(&Subscription::vehicle_angular_velocity, this, _1));

        input_rc_subscriber_ = this->create_subscription<InputRc>(
            "/fmu/out/input_rc", qos, std::bind(&Subscription::input_rc, this, _1));
    }

    static std::array<float,2> body_to_local(float x, float y, float psi){

        float cpsi = std::clamp((float)cos(psi), -1.0f, 1.0f);
        float spsi = std::clamp((float)sin(psi), -1.0f, 1.0f);

        float dx = x*cpsi - y*spsi;
        float dy = x*spsi + y*cpsi;

        return{dx, dy};
    }

    static std::array<float,2> local_to_body(float x, float y, float psi){

        float cpsi = std::clamp((float)cos(psi), -1.0f, 1.0f);
        float spsi = std::clamp((float)sin(psi), -1.0f, 1.0f);

        float dx = x*cpsi + y*spsi;
        float dy = -x*spsi + y*cpsi;

        return{dx, dy};
    }

private:

    void vehicle_local_position(const VehicleLocalPosition::UniquePtr msg){
        x_r_ = msg -> x;    
        y_r_ = msg -> y;    
        z_r_  = msg -> z;

        auto body = local_to_body(x_r_, y_r_, psi_r_);
        x_body_r_ = body[0];
        y_body_r_ = body[1];
        
        vx_r_ = msg -> vx;
        vy_r_ = msg -> vy;
        vz_r_ = msg -> vz;

        //RCLCPP_INFO(this->get_logger(),"x=%.3f, x_init_=%.3f, y=%.3f, y_init_=%.3f, z=%.3f, z_init_=%.3f, psi=%.3f, psi_init_=%.3f", x_r_, x_init_, y_r_, y_init_, z_r_, z_init_, psi_r_*180.0/M_PI, psi_init_*180.0/M_PI);
        RCLCPP_INFO(this->get_logger(),"xb=%.3f, xb_init_=%.3f, yb=%.3f, yb_init_=%.3f, zb=%.3f, zb_init_=%.3f, psi=%.3f, psi_init_=%.3f", x_body_r_, x_init_body_, y_body_r_, y_init_body_, z_r_, z_init_, psi_r_*180.0/M_PI, psi_init_*180.0/M_PI);
    }
    
    void vehicle_attitude(const VehicleAttitude::UniquePtr msg){
        double qw_r = msg->q[0];
        double qx_r = msg->q[1];
        double qy_r = msg->q[2];
        double qz_r = msg->q[3];
        phi_r_ = atan2(2.0 * (qw_r * qx_r + qy_r * qz_r), 1.0 - 2.0 * (qx_r * qx_r + qy_r * qy_r));
        theta_r_ = asin(std::clamp(2.0 * (qw_r * qy_r - qz_r * qx_r), -1.0, 1.0));
        psi_r_ = atan2(2.0 * (qw_r * qz_r + qx_r * qy_r), 1.0 - 2.0 * (qy_r * qy_r + qz_r * qz_r));  
        //RCLCPP_INFO(this->get_logger(),"phi=%.3f, theta=%.3f, psi=%.3f", phi_r, theta_r, psi_r);
    }

    void vehicle_angular_velocity(const VehicleAngularVelocity::UniquePtr msg){
        omg_phi_r_ = msg->xyz[0];
        omg_theta_r_ = msg->xyz[1];
        omg_psi_r_ = msg->xyz[2];
        //RCLCPP_INFO(this->get_logger(),"omg_phi_r_ =%.3f, omg_theta_r_ =%.3f, omg_psi_r_ =%.3f", omg_phi_r_, omg_theta_r__, omg_psi_r_);
    }

    void input_rc(const InputRc::UniquePtr msg){
        rc_offboard_sw6_ = msg -> values[5];
        // RCLCPP_INFO(this->get_logger(),"sw6 =%.3f", rc_offboard_sw6_);
    }

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_subscriber_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscriber_;

};

using Sub = Subscription;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("Offboard")
    {
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
		
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);
        
        vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", qos);
        
        timer_ = this->create_wall_timer(20ms, std::bind(&OffboardControl::callback, this));

        offboard_setpoint_counter = 0;
    }

private:

    uint64_t offboard_setpoint_counter;
    uint64_t mission_time;
    uint64_t time0;
    uint64_t time1;
    uint64_t time2;
    uint64_t time3;
    uint64_t time4;
    steady_clock::time_point time_start_;

    void callback();
    void offboard_condition();
    void mission();
    void mission_circle();

    void status(char mode){
        switch (mode)
        {
        case ARM:
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            break;
        case DISARM:
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            break;
        case LAND:
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            break;
        default:
            break;
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0){
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void publish_offboard_control_mode(){
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_attitude_setpoint(){
        VehicleAttitudeSetpoint msg{};
        msg.q_d = std::array<float, 4>{qw_, qx_, qy_, qz_};
        msg.thrust_body[2] = f_thrust_; 
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_attitude_setpoint_publisher_->publish(msg);
    }

    void pp_z_control(float z_des){
       
        float vz_sp = std::clamp(Kp2_z*(z_des - z_r_), -2.0f, 2.0f);

        float u_z = Kp1_z*(vz_sp - vz_r_);

        thrust_ = ((g - u_z)*m/(cos(phi_r_)*cos(theta_r_)));

        if(std::fabs(thrust_) < 1e-3f){
            thrust_ = (thrust_ >= 0.0f) ? 1e-3f : -1e-3f;
        } 

        f_thrust_ = -(std::sqrt(std::clamp(thrust_, 0.0f, 10000.0f) / (8 * b)) - min_motor_vel) / (max_motor_vel - min_motor_vel);
        //RCLCPP_INFO(this->get_logger(), "f_thrust_ = %.3f (z_r_ = %.2f, z_init_ = %.2f)", f_thrust_, z_r_, z_init_);
    }

    void pp_position_control(float x_des, float y_des, float z_des){
        pp_z_control(z_des);

        float psi_des = psi_init_;

        dx = x_des - x_r_;
        dy = y_des - y_r_;

        float alpha = atan2(dy, dx);
        vxy_max_ = 1.5f;

        float vx_max = (vxy_max_ * std::fabs(cos(alpha)));
        float vy_max = (vxy_max_ * std::fabs(sin(alpha)));

        float vx_sp = std::clamp(Kp2_x*dx, -vx_max, vx_max);
        float vy_sp = std::clamp(Kp2_y*dy, -vy_max, vy_max);

        float x_2dot = Kp1_x*(vx_sp - vx_r_);
        float y_2dot = Kp1_y*(vy_sp - vy_r_);

        float ux = x_2dot*m/thrust_;
        float uy = y_2dot*m/thrust_;

        float phi = asin(std::clamp((float)(-ux*sin(psi_des) + uy*cos(psi_des)), -1.0f, 1.0f));
        float theta = asin(std::clamp((float)(-(ux*cos(psi_des) + uy*sin(psi_des))/cos(phi_r_)), -1.0f, 1.0f));
        
        float angle = 35.0f*M_PI/180.0f;
        float phi_des = std::clamp(phi, -angle, angle);
        float theta_des = std::clamp(theta, -angle, angle);

        qw_ = cos(phi_des/2) * cos(theta_des/2) * cos(psi_des/2) + sin(phi_des/2) * sin(theta_des/2) * sin(psi_des/2);
        qx_ = sin(phi_des/2) * cos(theta_des/2) * cos(psi_des/2) - cos(phi_des/2) * sin(theta_des/2) * sin(psi_des/2);
        qy_ = cos(phi_des/2) * sin(theta_des/2) * cos(psi_des/2) + sin(phi_des/2) * cos(theta_des/2) * sin(psi_des/2);
        qz_ = cos(phi_des/2) * cos(theta_des/2) * sin(psi_des/2) - sin(phi_des/2) * sin(theta_des/2) * cos(psi_des/2);      
    }

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};


//#################################################################################################//
//====================================== Execution Function =======================================//
//#################################################################################################//
void OffboardControl::callback() {
    offboard_condition();
    if(!land_mode_){
        publish_attitude_setpoint();
    }
    if(offboard_mode_){
        mission();
        mission_time ++;
    }
}

void OffboardControl::offboard_condition(){
    publish_offboard_control_mode();
    // Position control mode
    if(rc_offboard_sw6_ < 1500){
        if (offboard_mode_) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);  // Vào Position
            offboard_mode_ = false;
        }
        initialized_ = false;
        offboard_setpoint_counter = 0;
        mission_time = 0;
        return;
    }

    // Offboard control mode
    if((!offboard_mode_) && (rc_offboard_sw6_ > 1500)){
        land_mode_ = false;
        if(initialized_ == false){
            psi_init_ = psi_r_; 

            x_init_ = x_r_;
            y_init_ = y_r_;
            z_init_ = z_r_;

            auto body_init = Sub::local_to_body(x_init_, y_init_, psi_init_);
            x_init_body_ = body_init[0];
            y_init_body_ = body_init[1];

            initialized_ = true;
        }

        pp_position_control(x_init_, y_init_, z_init_);
        if (offboard_setpoint_counter == 50) {
           
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Vào Offboard
            offboard_mode_ = true;
            mode = MODE::HOVER;
            time0 = mission_time; 
        }
        offboard_setpoint_counter++;
    }
}

//#################################################################################################//
//======================================= Mission Function ========================================//
//#################################################################################################//
void OffboardControl::mission(){
    switch (mode)
    {
    case MODE::HOVER:{
        pp_position_control(x_init_, y_init_, z_init_);

        if(mission_time == (time0 + 300)){
           //mode = MODE::MOVE_X_1;
        }
        break;
    }
    case MODE::MOVE_X_1:{
        auto denta = Sub::body_to_local(10.0, 0.0, psi_init_);
        pp_position_control(x_init_ + denta[0], y_init_ + denta[1], z_init_);
       
        break;
    }
    // case MODE::DONE:
    //     if (!land_mode_) {
    //         land_mode_ = true;
    //         status(LAND); // chỉ gọi 1 lần
    //     }
    //     break;

    }
}

// void OffboardControl::mission_circle(){
//     switch (mode)
//     {
//     case MODE::HOVER:
//         pp_position_control(x_init_body_, y_init_body_, z_init_body_);
//         if (!time_initialized_) {
//             time_start_ = steady_clock::now();
//             time_initialized_ = true;
//         }
//         if(mission_time == (time0 + 200)){
//             mode = MODE::MOVE_X_1;
//         }
//         break;
//     case MODE::MOVE_X_1:
//         pp_position_control(x_init_body_ + radius_, y_init_body_, z_init_body_);
//         if(abs(x_r_ - radius_) < 0.2){
//             time1 = mission_time; 
//             mode = MODE::HOVER_X_1;              
//         }
//         break;
//     case MODE::HOVER_X_1:
//         pp_position_control(x_init_body_ + radius_, y_init_body_, z_init_body_);
//         if(mission_time == (time1 + 200)){
//             mode = MODE::MOVE_CIRCLE;
//         }
//         break;
//     case MODE::MOVE_CIRCLE:
//         double t = duration<double>(steady_clock::now() - time_start_).count();
//         double T = 2*M_PI*radius_ / vel_xy_;
//         double omega = 2*M_PI / T;
//         float x_des;
//         float y_des;
//         if (t < radius_ / vel_xy_) {
//             x_des = x_init_body_ + radius_ + vel_xy_*t;
//         } else {
//             double t_circle = t - radius_ / vel_xy_;
//             x_des = x_init_body_ + radius_ + radius_ * std::cos(omega * t_circle);
//             y_des = y_init_body_ + radius_ * std::sin(omega * t_circle);
//         }
//         pp_position_control(x_des, y_des, z_init_body_);
//         break;
//     }
// }

//#################################################################################################//
//==================================== End Execution Function =====================================//
//#################################################################################################//

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto sub_node = std::make_shared<Subscription>();
    auto offboard_node = std::make_shared<OffboardControl>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sub_node);
    executor.add_node(offboard_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}