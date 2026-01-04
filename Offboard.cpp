/**
 * @brief Offboard control based on the cascade P-P for position
 * @file pid_offboard.cpp
 * @author Tran Nguyen Ngoc Thanh <thanh.tnngoc@gmail.com>
 * @see Pham Quoc Khanh <khanhpqspkt@gmail.com>
**/

// === Import the libraries ===
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

#include <chrono>
#include <cmath>
#include <array>
#include <stdint.h>
#include <iostream>


using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//======================================================================================
struct params{
    float e_x          = 0.8f;  //0.7
    float w_x          = 2.67f;  //0.33

    float e_y          = 0.8f;  //0.8
    float w_y          = 2.67f; //0.31

    float e_z          = 0.8f; //0.85
    float w_z          = 5.1f;  //4.5

    float e_yaw        = 0.8f;
    float w_yaw        = 0.5f;
};
params params_;

//======================================================================================
struct UAVParams{
    float mass = 2.72f;
    float gravity = 9.80665f;
    float motor_coeff = 7.2706e-06;
    float min_motor_vel = 178.0f;
    float max_motor_vel = 1199.0f;

    float Kp1_z = 2.0f * params_.e_z * params_.w_z;
    float Kp2_z = params_.w_z / (2.0f * params_.e_z);
    float Kff_z = 0.0f;
    
    float Kp1_x = 2.0f * params_.e_x * params_.w_x;
    float Kp2_x = params_.w_x / (2.0f * params_.e_x);
    float Kff_x = 0.0f;

    float Kp1_y = 2.0f * params_.e_y * params_.w_y;
    float Kp2_y = params_.w_y / (2.0f * params_.e_y);
    float Kff_y = 0.0f;

    float Kp1_yaw = 2.0f * params_.e_yaw * params_.w_yaw;
    float Kp2_yaw = params_.w_yaw / (2.0f * params_.e_yaw);
};
UAVParams uavprs;

//======================================================================================
struct TrajectoryParams{
    //============================================================
    // Circle
    float circle_vel_xy_ = 0.5;
    float circle_high_ = 3.0;
    float circle_vel_z_ = 0.3;
    float circle_radius_ = 5.0;
    float time_circle_high_ = circle_high_/circle_vel_z_;
    float time_radius_ = circle_radius_/circle_vel_xy_; 

    //============================================================
    // Trajectory
    Eigen::VectorXf x_waypoints;
    Eigen::VectorXf y_waypoints;
    Eigen::VectorXf z_waypoints;
    Eigen::VectorXf t_waypoints;

    Eigen::VectorXf vx_mis;
    Eigen::VectorXf vy_mis;
    Eigen::VectorXf vz_mis;

    TrajectoryParams(){

        const int Point_number = 7;
        
        x_waypoints = Eigen::VectorXf(Point_number);
        y_waypoints = Eigen::VectorXf(Point_number);
        z_waypoints = Eigen::VectorXf(Point_number);
        vx_mis = Eigen::VectorXf(Point_number);
        vy_mis = Eigen::VectorXf(Point_number);
        vz_mis = Eigen::VectorXf(Point_number);
        t_waypoints = Eigen::VectorXf::Zero(Point_number + 1);

        x_waypoints << 0.f  ,     1.5f   ,    11.5f    ,    11.5f  , 1.5f   , 0.f     , 0.f       ;
        y_waypoints << 0.f  ,     0.f   ,    0.f    ,   -4.9f  , -4.9f  , -4.9f    , -4.9f      ;
        z_waypoints << -3.0f,   -3.0f   ,   -3.0f    ,  -3.0f  , -3.0f , -3.0f   , 100.0f    ;

        vx_mis      <<  0.f ,     2.f  ,    1.5f    ,    0.f  , -1.5f  , -2.f   , 0.f       ;
        vy_mis      <<  0.f ,     0.f   ,    0.f    ,  -2.f  , 0.f   , 0.f     , 0.f       ;
        vz_mis      << -0.6f,     0.f   ,    0.f    ,   0.f   , 0.f   , 0.f     , 0.7       ;
    }
};
TrajectoryParams traj_prs;

//======================================================================================
struct Receives{
    Eigen::Vector3f pos_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f pos_body_{0.0f, 0.0f, 0.0f};    
    Eigen::Vector3f vel_{0.0f, 0.0f, 0.0f};
    float roll_{0.0f}, pitch_{0.0f}, yaw_{0.0f};
    float rc_offb_{0.0f}, rc_traj_{0.0f};

    //============================================================
    // Object
    Eigen::Vector2f size_img_{0.0f, 0.0f};
    Eigen::Vector2f pos_obj_img_{0.0f, 0.0f};
    Eigen::Vector2f pos_obj_uav_{0.0f, 0.0f};
    float yaw_obj_{0.0f};   
    float flag_obj_{0.0f};   
};
Receives recs;

//======================================================================================
struct Setpoints{
    Eigen::Vector3f pos_des_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f pos_home_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f pos_body_home_{0.0f, 0.0f, 0.0f};
    Eigen::Vector2f pos_obj_real_{0.0f, 0.0f};

    Eigen::Vector3f pos_pre_{0.0f, 0.0f, 0.0f};
    float yaw_pre_{0.0f};
    float angle_des_{0.0f}; 


    float f_thrust_{0.0f};
    float vxy_max_{0.8f};
    float roll_des_{0.0f}, pitch_des_{0.0f}, yaw_des_{0.0f};
    float roll_home_{0.0f}, pitch_home_{0.0f}, yaw_home_{0.0f};
};
Setpoints sets;

//======================================================================================
struct Camera{
    float sw_{3.6736f};
    float sh_{2.7384};
    float p_{90.0f}; //skew
    float f_{3.15f};
    float theta_{-90.0f};
};
Camera cam;

//======================================================================================
bool offboard_active_{false};
bool initialized_{true};
bool time_init_{true};
bool t_waypoint_flag{true};
bool obj_pre_flag_{false};
std::chrono::steady_clock::time_point time_start_;

//======================================================================================
float x_pre{0.f};
float y_pre{0.f};
float z_pre{0.f};
float time_last{0.f};
int step{0};

//======================================================================================
enum class MissionMode{
    HOLD,
    TRACKING,
    CIRCLE,
    TRAJECTORY,
    DETECTION,

};
MissionMode mission_mode;

//======================================================================================
class SubscriberObject : public rclcpp::Node
{
public:
    SubscriberObject() : Node("Object")
    {
        init_object_subscriber();
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_object_pos_;
    void init_object_subscriber(){
        auto qos = rclcpp::QoS(50).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        sub_object_pos_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "yolo_object_result", qos, 
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg){
                recs.pos_obj_img_ << msg->data[0], msg->data[1];
                recs.size_img_ << msg->data[2], msg->data[3];
                recs.yaw_obj_ = msg->data[4];
                recs.flag_obj_ = msg->data[5];

                //RCLCPP_INFO(this->get_logger(),"dx=%.3f, dy=%.3f, w=%.3f, h=%.3f", recs.pos_obj_.x(), recs.pos_obj_.y(), recs.size_img_.x(), recs.size_img_.y());
            });
    }
};

//======================================================================================
class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("Offboard")
    {
        params_ = params{};
        uavprs = UAVParams{};
        traj_prs = TrajectoryParams{};
        recs = Receives{};
        sets = Setpoints{};

        init_subscribers();
        init_publishers();

        timer_ = this->create_wall_timer(10ms, std::bind(&OffboardControl::main_control_loop, this));
    }

private:

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr sub_local_position_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr sub_attitude_;
    rclcpp::Subscription<InputRc>::SharedPtr sub_input_rc_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr pub_attitude_setpoint_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_command_;

    rclcpp::TimerBase::SharedPtr timer_;

    void init_subscribers(){
        auto qos = rclcpp::QoS(50).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        sub_local_position_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            [this](const VehicleLocalPosition::UniquePtr msg) {
                recs.pos_ << msg->x, msg->y, msg->z;
                recs.vel_ << msg->vx, msg->vy, msg->vz;
                auto body = local_to_body(recs.pos_.x(), recs.pos_.y(), recs.pos_.z(), recs.yaw_);
                recs.pos_body_ << body[0], body[1], body[2];

                RCLCPP_INFO(this->get_logger(),"pos_body(%.3f, %.3f, %.3f, %.3f), body_home_(%.3f, %.3f, %.3f, %.3f), pixel_uav(%.3f, %.3f), pixel(%.3f, %.3f, %3f)", 
                                                recs.pos_body_.x(), recs.pos_body_.y(), recs.pos_body_.z(), recs.yaw_*180.0/M_PI,
                                                sets.pos_body_home_.x(), sets.pos_body_home_.y(), sets.pos_body_home_.z(), sets.yaw_home_,
                                                recs.pos_obj_uav_[0], recs.pos_obj_uav_[1],
                                                recs.pos_obj_img_.x(), recs.pos_obj_img_.y(), recs.yaw_obj_);
            });

        sub_attitude_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const VehicleAttitude::UniquePtr msg){
                    double w = msg->q[0];
                    double x = msg->q[1];
                    double y = msg->q[2];
                    double z = msg->q[3];

                    //Quaternion to Euler angles
                    recs.roll_ = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
                    recs.pitch_ = asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
                    recs.yaw_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
            });

        sub_input_rc_ = this->create_subscription<InputRc>(
            "/fmu/out/input_rc", qos,
            [this](const InputRc::UniquePtr msg){
                recs.rc_offb_ = msg->values[5];
            });
    }

    void init_publishers() {
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        pub_offboard_mode_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        pub_attitude_setpoint_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", qos);
        pub_vehicle_command_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

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
        msg.timestamp = now_us();
        pub_vehicle_command_->publish(msg);
    }

    void publish_offboard_control_mode(){
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = now_us();
        pub_offboard_mode_->publish(msg);
    }

    void publish_attitude_setpoint(){
        VehicleAttitudeSetpoint msg{};
        Eigen::AngleAxisd rollAngle(sets.roll_des_, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(sets.pitch_des_, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(sets.yaw_des_, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        msg.q_d = {static_cast<float>(q.w()),
                   static_cast<float>(q.x()), 
                   static_cast<float>(q.y()), 
                   static_cast<float>(q.z())};
        msg.thrust_body[2] = sets.f_thrust_; 
        msg.timestamp = now_us();
        pub_attitude_setpoint_->publish(msg);
    }

    uint64_t now_us() const{
        return this->now().nanoseconds() / 1000;
    }

    std::array<float,3> body_to_local(float x, float y, float z, float psi){

        float cpsi = std::clamp((float)cos(psi), -1.0f, 1.0f);
        float spsi = std::clamp((float)sin(psi), -1.0f, 1.0f);

        float dx = x*cpsi - y*spsi;
        float dy = x*spsi + y*cpsi;
        float dz = z;

        return{dx, dy, dz};
    }

    std::array<float,3> local_to_body(float x, float y, float z, float psi){

        float cpsi = std::clamp((float)cos(psi), -1.0f, 1.0f);
        float spsi = std::clamp((float)sin(psi), -1.0f, 1.0f);

        float dx = x*cpsi + y*spsi;
        float dy = -x*spsi + y*cpsi;
        float dz = z;

        return{dx, dy, dz};
    }

    std::array<float,3> image_to_uav_position(float u, float v, float width_img, float height_img, float h){
        float cx = width_img/2.0f;
        float cy = height_img/2.0f;
        
        float fx = cam.f_*width_img/cam.sw_;
        float fy = cam.f_*height_img/cam.sh_;

        float xc = std::abs(h)*(((u - cx)/fx) + ((v - cy)*cos(cam.p_)/fy));
        float yc = std::abs(h)*((v - cy)*sin(cam.p_)/fy);

        float xu = -yc;
        float yu = xc;

        return{xu, yu};
    }

    std::array<float,1> pp_yaw_control(){
        float angle = 2.0f*M_PI/180.0f;

        float err = std::clamp(sets.angle_des_ - recs.yaw_, -angle, angle);

        float yaw = uavprs.Kp1_yaw*(uavprs.Kp2_yaw * err - recs.yaw_);
                
        return {yaw};
    }

    std::array<float,2> pp_z_control(){

        float err = std::clamp(sets.pos_des_.z() - recs.pos_.z(), -2.0f, 2.0f);

        float uz = uavprs.Kp1_z*(uavprs.Kp2_z * err - recs.vel_.z());

        float thrust = ((uavprs.gravity - uz) * uavprs.mass/(cos(recs.roll_) * cos(recs.pitch_)));

        if(std::fabs(thrust) < 1e-3f) thrust = (thrust >= 0.0f) ? 1e-3f : -1e-3f;
        
        // normalize
        float u1 = -(std::sqrt(std::clamp(thrust, 0.0f, 10000.0f) / (8 * uavprs.motor_coeff)) - uavprs.min_motor_vel) / (uavprs.max_motor_vel - uavprs.min_motor_vel);
        
        return {u1, thrust};
    }

    std::array<float,3> pp_position_control(){
        // z Controller
        auto z_cal = pp_z_control();

        float thrust_des = z_cal[0];
        float thrust = z_cal[1];

        // xy Controller
        float yaw = sets.yaw_home_;

        float dx = sets.pos_des_.x() - recs.pos_.x();
        float dy = sets.pos_des_.y() - recs.pos_.y();

        float alpha = atan2(dy, dx);

        float vx_max = (sets.vxy_max_ * std::fabs(cos(alpha)));
        float vy_max = (sets.vxy_max_ * std::fabs(sin(alpha)));

        float vx_sp = std::clamp(uavprs.Kp2_x * dx, -vx_max, vx_max);
        float vy_sp = std::clamp(uavprs.Kp2_y * dy, -vy_max, vy_max);

        float ax = uavprs.Kp1_x*(vx_sp - recs.vel_.x());
        float ay = uavprs.Kp1_y*(vy_sp - recs.vel_.y());

        float ux = ax*uavprs.mass/thrust;
        float uy = ay*uavprs.mass/thrust;
   
        float phi = asin(std::clamp((float)(-ux*sin(yaw) + uy*cos(yaw)), -1.0f, 1.0f));
        float theta = asin(std::clamp((float)(-(ux*cos(yaw) + uy*sin(yaw))/cos(recs.roll_)), -1.0f, 1.0f));

        float angle = 35.0f*M_PI/180.0f;
        float phi_des = std::clamp(phi, -angle, angle);
        float theta_des = std::clamp(theta, -angle, angle);

        return {phi_des, theta_des, thrust_des};    
    }

    void offboard_condition(){
        // Position control mode
        if(recs.rc_offb_ < 1500){
            if (offboard_active_){
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);  // Vào Position
                offboard_active_ = false;
            }
            initialized_ = true;
            time_init_ = true;
            step = 0;
            return;
        }
        // Offboard control mode
        if((!offboard_active_) && (recs.rc_offb_ > 1500)){
            if(initialized_){
                sets.yaw_home_ = recs.yaw_; 
                sets.pos_home_ = recs.pos_;
               
                auto body_home = local_to_body(sets.pos_home_.x(), sets.pos_home_.y(), sets.pos_home_.z(), sets.yaw_home_);
                sets.pos_body_home_ << body_home[0], body_home[1], body_home[2];
              
                initialized_ = false;
            }
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Vào Offboard
            offboard_active_ = true;
            mission_mode = MissionMode::HOLD;
        }
    }

    void mission(){
        switch (mission_mode){
            case MissionMode::HOLD:{
                sets.pos_des_ = sets.pos_home_;
                sets.yaw_des_ = sets.yaw_home_;
                break;
            }

            case MissionMode::TRAJECTORY:{
                if (time_init_){
                    time_start_ = std::chrono::steady_clock::now();
                    x_pre = 0.f;
                    y_pre = 0.f;
                    z_pre = 0.f;
                    time_last = 0.f;
                    time_init_ = false;
                }
                sets.yaw_des_ = sets.yaw_home_;

                float time_mis = std::chrono::duration<float>(
                                 std::chrono::steady_clock::now() - time_start_).count();
                 
                if (t_waypoint_flag){
                    float tx = 0.f;
                    float ty = 0.f;
                    float tz = 0.f;
                    if (traj_prs.vx_mis(step) != 0.f){
                        tx = fabsf((traj_prs.x_waypoints(step) - x_pre)/traj_prs.vx_mis(step));                  
                    }
                    else if (traj_prs.vy_mis(step) != 0.f){
                        ty = fabsf((traj_prs.y_waypoints(step) - y_pre)/traj_prs.vy_mis(step));  
                    }
                    else {
                        tz = fabsf((traj_prs.z_waypoints(step) - z_pre)/traj_prs.vz_mis(step));  
                    }

                    traj_prs.t_waypoints(step + 1) = traj_prs.t_waypoints(step) + tx + ty + tz;

                    x_pre = traj_prs.x_waypoints(step);
                    y_pre = traj_prs.y_waypoints(step);
                    z_pre = traj_prs.z_waypoints(step);

                    t_waypoint_flag = false;
                }           
                float delta_t = time_mis - time_last;  

                auto local_mis = body_to_local(traj_prs.vx_mis(step)*delta_t,
                                           traj_prs.vy_mis(step)*delta_t,
                                           traj_prs.vz_mis(step)*delta_t,
                                           sets.yaw_des_);

                sets.pos_des_.x() += local_mis[0];
                sets.pos_des_.y() += local_mis[1];
                sets.pos_des_.z() += local_mis[2];

                time_last = time_mis;

                if (time_mis > traj_prs.t_waypoints(step + 1)){
                    step ++;
                    auto local_wp = body_to_local(traj_prs.x_waypoints(step-1),
                                                 traj_prs.y_waypoints(step-1),
                                                 traj_prs.z_waypoints(step-1),
                                                 sets.yaw_des_);

                    sets.pos_des_.x() = sets.pos_home_.x() + local_wp[0];
                    sets.pos_des_.y() = sets.pos_home_.y() + local_wp[1];
                    sets.pos_des_.z() = sets.pos_home_.z() + local_wp[2];
                    t_waypoint_flag = true;
                }
                break;
            }

            case MissionMode::CIRCLE:{
                if (time_init_){
                    time_start_ = std::chrono::steady_clock::now();
                    time_init_ = false;
                }
                sets.yaw_des_ = sets.yaw_home_;
                float t = std::chrono::duration<float>(
                            std::chrono::steady_clock::now() - time_start_).count();
                float T = 2 * M_PI * traj_prs.circle_radius_ / traj_prs.circle_vel_xy_;
                float omega = 2 * M_PI / T;

                if (t <= traj_prs.time_circle_high_){
                    sets.pos_des_ = sets.pos_home_ + Eigen::Vector3f(0.f, 0.f, -std::clamp(traj_prs.circle_vel_z_ * t, 0.f, traj_prs.circle_high_));
                } 
                else if ((t > traj_prs.time_circle_high_) && (t <= traj_prs.time_circle_high_ + traj_prs.time_radius_)){
                    sets.pos_des_ = sets.pos_home_ + Eigen::Vector3f(traj_prs.circle_vel_xy_ * (t - traj_prs.time_circle_high_), 0.0f, -traj_prs.circle_high_);
                } 
                else if ((t > traj_prs.time_circle_high_ + traj_prs.time_radius_) && (t <= traj_prs.time_circle_high_ + traj_prs.time_radius_ + 2 * T)){
                    float t_circle = t - traj_prs.time_circle_high_ - traj_prs.time_radius_;
                    sets.pos_des_ = sets.pos_home_ + Eigen::Vector3f(
                        traj_prs.circle_radius_ * std::cos(omega * t_circle),
                        traj_prs.circle_radius_ * std::sin(omega * t_circle),
                        -traj_prs.circle_high_);
                } 
                else if ((t > traj_prs.time_circle_high_ + traj_prs.time_radius_ + 2 * T) && (t <= traj_prs.time_circle_high_ + 2 * traj_prs.time_radius_ + 2 * T)){
                    sets.pos_des_ = sets.pos_home_ + Eigen::Vector3f(traj_prs.circle_radius_ - traj_prs.circle_vel_xy_ * (t - traj_prs.time_circle_high_ - traj_prs.time_radius_ - 2 * T), 0.0f, -traj_prs.circle_high_);
                }
                else{
                    sets.pos_des_ = sets.pos_home_ + Eigen::Vector3f(0.f, 0.0f, -traj_prs.circle_high_ + traj_prs.circle_vel_z_ * (t - traj_prs.time_circle_high_ - 2 * traj_prs.time_radius_ - 2 * T));
                }
                break;
            }  
                        
        }
    }
    
    // === Main Loop ===
    void main_control_loop(){
        offboard_condition();
        if(offboard_active_){
            publish_offboard_control_mode();
            mission();
            auto xyz_cal = pp_position_control();
            sets.roll_des_ = xyz_cal[0];
            sets.pitch_des_ = xyz_cal[1];
            sets.f_thrust_ = xyz_cal[2];
            publish_attitude_setpoint();
        }
    }
    
};

// === Main Function ===
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto subscriber_object = std::make_shared<SubscriberObject>();
    auto offboard_control = std::make_shared<OffboardControl>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(subscriber_object);
    executor.add_node(offboard_control);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
