    #include <px4_msgs/msg/offboard_control_mode.hpp>
    #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
    #include <px4_msgs/msg/vehicle_command.hpp>
    #include <px4_msgs/msg/vehicle_local_position.hpp>
    #include <px4_msgs/msg/vehicle_attitude.hpp>
    #include <px4_msgs/msg/input_rc.hpp>

    #include <eigen3/Eigen/Geometry>
    #include <rclcpp/rclcpp.hpp>

    #include <chrono>
    #include <cmath>
    #include <array>

    using namespace std::chrono_literals;
    using namespace px4_msgs::msg;

    // === Parameter Structures ===
    struct UAVParams {
        float mass = 1.85f;
        float gravity = 9.80665f;
        float motor_coeff = 3.1171e-06f;
        float min_motor_vel = 224.0f;
        float max_motor_vel = 1633.0f;

        float Kp1_z = 2.0f * 0.85f * 2.5f;
        float Kp2_z = 2.5f / (2.0f * 0.85f);
        float Kff_z = 0.0f;
        float Kp1_x = 2.0f * 0.85f * 1.5f;
        float Kp2_x = 1.5f / (2.0f * 0.85f);
        float Kff_x = 0.0f;
        float Kp1_y = 2.0f * 0.85f * 1.5f;
        float Kp2_y = 1.5f / (2.0f * 0.85f);
        float Kff_y = 0.0f;
    };

    struct TrajectoryParams {
        Eigen::Vector3f way_point{0.5f, 0.5f, -0.5f};
        float high = 1.3;
        float vel_z = 0.2;
        float radius = 1.0;
        float vel_xy = 0.5;
        float th = high/vel_z;
        float tr = radius/vel_xy;


        //============================================================
        Eigen::VectorXf x_waypoints;
        Eigen::VectorXf y_waypoints;
        Eigen::VectorXf z_waypoints;
        Eigen::VectorXf vx_mis;
        Eigen::VectorXf vy_mis;
        Eigen::VectorXf vz_mis;
        Eigen::VectorXf t_waypoints;

        TrajectoryParams() {
            // x_waypoints = Eigen::VectorXf(7);
            // y_waypoints = Eigen::VectorXf(7);
            // z_waypoints = Eigen::VectorXf(7);
            // vx_mis = Eigen::VectorXf(7);
            // vy_mis = Eigen::VectorXf(7);
            // vz_mis = Eigen::VectorXf(7);
            // t_waypoints = Eigen::VectorXf::Zero(8);

            // x_waypoints << 0.f, 4.3f, 19.3f, 19.3f, 4.3f, 0.f, 0.f;
            // y_waypoints << 0.f, 0.f, 0.f, -32.f, -32.f, -32.f, -32.f;
            // z_waypoints << -2.5f, -2.5f, -2.5f, -2.5f, -2.5f, -2.5f, 2.5f;

            // vx_mis << 0.f, 1.f, 0.5f, 0.f, -0.5f, -1.f, 0.f;
            // vy_mis << 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f;
            // vz_mis << -0.3f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f;
            const int Point_number = 7;

            x_waypoints = Eigen::VectorXf(Point_number);
            y_waypoints = Eigen::VectorXf(Point_number);
            z_waypoints = Eigen::VectorXf(Point_number);
            vx_mis = Eigen::VectorXf(Point_number);
            vy_mis = Eigen::VectorXf(Point_number);
            vz_mis = Eigen::VectorXf(Point_number);
            t_waypoints = Eigen::VectorXf::Zero(Point_number+1);

            x_waypoints << 0.f  ,     1.5f   ,    11.5f    ,    11.5f  , 1.5f   , 0.f     , 0.f       ;
            y_waypoints << 0.f  ,     0.f   ,    0.f    ,   -4.9f  , -4.9f  , -4.9f    , -4.9f      ;
            z_waypoints << -3.0f,   -3.0f   ,   -3.0f    ,  -3.0f  , -3.0f , -3.0f   , 100.0f    ;

            vx_mis      <<  0.f ,     2.f  ,    1.5f    ,    0.f  , -1.5f  , -2.f   , 0.f       ;
            vy_mis      <<  0.f ,     0.f   ,    0.f    ,  -2.f  , 0.f   , 0.f     , 0.f       ;
            vz_mis      << -0.6f,     0.f   ,    0.f    ,   0.f   , 0.f   , 0.f     , 0.7       ;
        }
    };

    // === Trajectory Modes ===
    enum class TrajectoryMode {
        HOLD,
        TRACKING,
        CIRCLE,
        MISSION
    };

    // === Main Class _ OffboardControl ===
    class OffboardControl : public rclcpp::Node {
    public:
        OffboardControl() : Node("offboard_control") {
            params_ = UAVParams{};
            traj_params_ = TrajectoryParams{};
            init_publishers();
            init_subscribers();
            timer_ = this->create_wall_timer(10ms, std::bind(&OffboardControl::control_loop, this));
            //Step time: 10ms, frequency: 100Hz 
        }

    private:
        // === ROS Interface ===
        rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_mode_;
        rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr pub_attitude_sp_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_cmd_;

        rclcpp::Subscription<VehicleLocalPosition>::SharedPtr sub_local_pos_;
        rclcpp::Subscription<VehicleAttitude>::SharedPtr sub_attitude_;
        rclcpp::Subscription<InputRc>::SharedPtr sub_rc_;

        rclcpp::TimerBase::SharedPtr timer_;

        // === UAV State ===
        Eigen::Vector3f pos_{0.0f, 0.0f, 0.0f};
        Eigen::Vector3f vel_{0.0f, 0.0f, 0.0f};
        float roll_{0.0f}, pitch_{0.0f}, yaw_{0.0f};
        Eigen::Vector3f pos_home_{0.0f, 0.0f, 0.0f};
        float yaw_home_{0.0f};

        Eigen::Vector3f pos_des_{0.0f, 0.0f, 0.0f};
        float yaw_des_{0.0f};
        float thrust_des_{0.0f};
        float fz_{0.0f};
        float roll_des_{0.0f}, pitch_des_{0.0f};

        float rc_offb_{0.0f}, rc_traj_{0.0f};
        bool offboard_active_{false};
        bool th_init_{true};
        std::chrono::steady_clock::time_point th_start_;

        Eigen::Vector3f mocap_pos_{0.0f, 0.0f, 0.0f};

        int step{0};
        float x_pre{0.f};
        float y_pre{0.f};
        float z_pre{0.f};
        float t_last{0.f};
        bool t_waypoint_flag{true};

        UAVParams params_;
        TrajectoryParams traj_params_;
        TrajectoryMode traj_mode_;

        // === ROS Init ===
        void init_publishers() {
            pub_offboard_mode_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            pub_attitude_sp_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
            // pub_attitude_sp_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
            pub_vehicle_cmd_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        }

        void init_subscribers() {
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            qos_profile.depth = 50;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

            sub_local_pos_ = this->create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1", qos,
                [this](const VehicleLocalPosition::UniquePtr msg) {
                    pos_ << msg->x, msg->y, msg->z;
                    vel_ << msg->vx, msg->vy, msg->vz;
                    RCLCPP_INFO(this->get_logger(), "pos=%.3f", pos_(0));
                }); ///// =========================<<<<<<<< local_position_v1 neu khac version
            sub_attitude_ = this->create_subscription<VehicleAttitude>(
                "/fmu/out/vehicle_attitude", qos,
                [this](const VehicleAttitude::UniquePtr msg) {
                        double w = msg->q[0];
                        double x = msg->q[1];
                        double y = msg->q[2];
                        double z = msg->q[3];

                        //Quaternion to Euler angles
                        roll_ = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
                        pitch_ = asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
                        yaw_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
                });

            sub_rc_ = this->create_subscription<InputRc>(
                "/fmu/out/input_rc", qos,
                [this](const InputRc::UniquePtr msg) {
                    rc_offb_ = msg->values[5];
                    rc_traj_ = msg->values[8];
                });
        }

        // === Main Loop ===
        void control_loop() {
            handle_mode_switch();
            if (offboard_active_) {
                update_trajectory();
                publish_offboard_mode();
                auto z_res = pid_z();
                thrust_des_ = z_res[0];
                fz_ = z_res[1];
                auto xy_res = pid_xy();
                roll_des_ = xy_res[0];
                pitch_des_ = xy_res[1];
                publish_attitude_sp();
                // RCLCPP_INFO(this->get_logger(), "ex=%.3f, ey=%.3f, ez=%.3f", 
                //             pos_des_(0) - pos_(0), pos_des_(1) - pos_(1), pos_des_(2) - pos_(2));
                RCLCPP_INFO(this->get_logger(), "xdes=%.3f, ydes=%.3f, zdes=%.3f", pos_des_[0], pos_des_[1], pos_des_[2]);
                // RCLCPP_INFO(this->get_logger(), "thrust_des=%.3f", thrust_des_);
            }
        }

        // === Mode Switching ===
        void handle_mode_switch() {
            if (rc_offb_ >= 1500 && !offboard_active_) {
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                RCLCPP_INFO(this->get_logger(), "Switch to OFFBOARD MODE");
                offboard_active_ = true;
                pos_home_ = pos_;
                yaw_home_ = yaw_;
            } 
            else if (rc_offb_ < 1500 && offboard_active_) {
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
                RCLCPP_INFO(this->get_logger(), "Switch to POSITION MODE");
                offboard_active_ = false;
                th_init_ = true;
                step = 0;
            }
        }

        // === Trajectory Mode ===
        void update_trajectory() {
            trajectory_switch();
            switch (traj_mode_) {
                case TrajectoryMode::HOLD: {
                    pos_des_ = pos_home_;
                    yaw_des_ = yaw_home_;
                    break;
                }
        
                case TrajectoryMode::TRACKING: {
                    pos_des_ = Eigen::Vector3f(mocap_pos_.x(), mocap_pos_.y(), pos_home_.z());
                    yaw_des_ = yaw_home_;
                    break;
                }
        
                case TrajectoryMode::CIRCLE: {
                    if (th_init_) {
                        th_start_ = std::chrono::steady_clock::now();
                        th_init_ = false;
                    }
                    yaw_des_ = yaw_home_;
                    float t = std::chrono::duration<float>(
                                std::chrono::steady_clock::now() - th_start_).count();
                    float T = 2*M_PI*traj_params_.radius / traj_params_.vel_xy;
                    float omega = 2*M_PI / T;
                    if (t <= traj_params_.th) {
                        pos_des_ = pos_home_ + Eigen::Vector3f(0.f, 0.f, - std::clamp(traj_params_.vel_z*t, 0.f, traj_params_.high));
                    } else if ((t > traj_params_.th) && (t <= traj_params_.th + traj_params_.tr)) {
                        pos_des_ = pos_home_ + Eigen::Vector3f(traj_params_.vel_xy * (t - traj_params_.th), 0.0f, - traj_params_.high);
                    } else if ((t > traj_params_.th + traj_params_.tr) && (t <= traj_params_.th + traj_params_.tr + 2*T)){
                        float t_circle = t - traj_params_.th - traj_params_.tr;
                        pos_des_ = pos_home_ + Eigen::Vector3f(
                            traj_params_.radius * std::cos(omega * t_circle),
                            traj_params_.radius * std::sin(omega * t_circle),
                            - traj_params_.high);
                    } else if ((t > traj_params_.th + traj_params_.tr + 2*T) && (t <= traj_params_.th + 2*traj_params_.tr + 2*T)){
                        pos_des_ = pos_home_ + Eigen::Vector3f(traj_params_.radius - traj_params_.vel_xy * (t - traj_params_.th - traj_params_.tr - 2*T), 0.0f, - traj_params_.high);
                    } else {
                        pos_des_ = pos_home_ + Eigen::Vector3f(0.f, 0.0f, - traj_params_.high + traj_params_.vel_z * (t - traj_params_.th - 2*traj_params_.tr - 2*T));
                    }
                    break;
                }
                
                case TrajectoryMode::MISSION: {
                    if (th_init_) {
                        th_start_ = std::chrono::steady_clock::now();
                        th_init_ = false;

                        x_pre = 0.f;
                        y_pre = 0.f;
                        z_pre = 0.f;
                        t_last = 0.f;
                        pos_des_ = pos_home_;
                    }
                    yaw_des_ = yaw_home_;

                    float t_mis = std::chrono::duration<float>(
                            std::chrono::steady_clock::now() - th_start_).count();
                    
                    if (t_waypoint_flag) {
                        float tx = 0.f;
                        float ty = 0.f;
                        float tz = 0.f;
                        if (traj_params_.vx_mis(step) != 0.f){
                            tx = fabsf((traj_params_.x_waypoints(step) - x_pre)/traj_params_.vx_mis(step));
                        } else if (traj_params_.vy_mis(step) != 0.f){
                            ty = fabsf((traj_params_.y_waypoints(step) - y_pre)/traj_params_.vy_mis(step));
                        } else {
                            tz = fabsf((traj_params_.z_waypoints(step) - z_pre)/traj_params_.vz_mis(step));
                        }

                        traj_params_.t_waypoints(step + 1) = traj_params_.t_waypoints(step) + tx + ty + tz;

                        x_pre = traj_params_.x_waypoints(step);
                        y_pre = traj_params_.y_waypoints(step);
                        z_pre = traj_params_.z_waypoints(step);

                        t_waypoint_flag = false;
                    }
                                        
                    float delta_t = t_mis - t_last;
                    
                    pos_des_(0) = pos_des_(0) + traj_params_.vx_mis(step)*delta_t*std::cos(yaw_des_)
                                              - traj_params_.vy_mis(step)*delta_t*std::sin(yaw_des_);
                    pos_des_(1) = pos_des_(1) + traj_params_.vx_mis(step)*delta_t*std::sin(yaw_des_)
                                              + traj_params_.vy_mis(step)*delta_t*std::cos(yaw_des_);
                    pos_des_(2) = pos_des_(2) + traj_params_.vz_mis(step)*delta_t;
                    
                    t_last = t_mis;

                    if (t_mis > traj_params_.t_waypoints(step + 1)) {
                        step ++;
                        pos_des_(0) = pos_home_(0) + traj_params_.x_waypoints(step-1)*std::cos(yaw_des_) 
                                                - traj_params_.y_waypoints(step-1)*std::sin(yaw_des_);
                        pos_des_(1) = pos_home_(1) + traj_params_.x_waypoints(step-1)*std::sin(yaw_des_) 
                                                + traj_params_.y_waypoints(step-1)*std::cos(yaw_des_);
                        pos_des_(2) = pos_home_(2) + traj_params_.z_waypoints(step-1);
                        t_waypoint_flag = true;
                    }
                    break;
                }
            }
        }

        void trajectory_switch() {
            if (rc_traj_ >= 1700) {
                traj_mode_ = TrajectoryMode::HOLD;
            } else if (rc_traj_ >= 1300 && rc_traj_ < 1700) {
                traj_mode_ = TrajectoryMode::TRACKING;
            } else {
                traj_mode_ = TrajectoryMode::MISSION;
            }
        }
        
        // === Controllers ===
        std::array<float,2> pid_z() {
            float error = std::clamp(pos_des_.z() - pos_.z(), -2.0f, 2.0f);
            float uz = params_.Kp1_z * (params_.Kp2_z * error - vel_.z()) + params_.Kff_z * params_.Kp2_z * error;
            float fz = (params_.gravity - uz) * params_.mass / (std::cos(roll_) * std::cos(pitch_));
            if (std::fabs(fz) < 1e-3f) fz = (fz >= 0.0f) ? 1e-3f : -1e-3f;
            float u1 = -(std::sqrt(std::clamp(fz, 0.0f, 10000.0f) / (4 * params_.motor_coeff)) - params_.min_motor_vel) / 
                        (params_.max_motor_vel - params_.min_motor_vel);
            return {u1, fz};
        }

        std::array<float,2> pid_xy() {
            // //######################################
            // float dx = pos_des_.x() - pos_(0);
            // float dy = pos_des_.y() - pos_(1);
            // float cos_y = std::cos(yaw_);
            // float sin_y = std::sin(yaw_);

            // // world → body 
            // float pos_x_body =  cos_y * dx + sin_y * dy;
            // float pos_y_body = -sin_y * dx + cos_y * dy;
            // float vel_x_body =  cos_y * vel_.x() + sin_y * vel_.y();
            // float vel_y_body = -sin_y * vel_.x() + cos_y * vel_.y();

            // //######################################
            // float ux1 = params_.Kp1_x * (params_.Kp2_x * (pos_x_body) - vel_x_body) + 
            //             params_.Kff_x * params_.Kp2_x * (pos_x_body);
            // float uy1 = params_.Kp1_y * (params_.Kp2_y * (pos_y_body) - vel_y_body) +
            //             params_.Kff_y * params_.Kp2_y * (pos_y_body);

            // float ux = -ux1 * params_.mass / fz_;
            // float uy = -uy1 * params_.mass / fz_;

            // float a_roll = std::clamp(ux * std::sin(yaw_) - uy * std::cos(yaw_), -0.4f, 0.4f);
            // float phi_des = std::asin(a_roll);
            // float a_pitch = std::clamp((ux * std::cos(yaw_) + uy * std::sin(yaw_)) / std::cos(phi_des), -0.4f, 0.4f);
            // float theta_des = std::asin(a_pitch);

            // return {phi_des, theta_des};

            float ux1 = params_.Kp1_x * (params_.Kp2_x * (pos_des_.x() - pos_.x()) - vel_.x()) + 
                        params_.Kff_x * params_.Kp2_x * (pos_des_.x() - pos_.x());
            float uy1 = params_.Kp1_y * (params_.Kp2_y * (pos_des_.y() - pos_.y()) - vel_.y()) +
                        params_.Kff_y * params_.Kp2_y * (pos_des_.y() - pos_.y());
            float ux = -ux1 * params_.mass / fz_;
            float uy = -uy1 * params_.mass / fz_;

            float a_roll = std::clamp(ux * std::sin(yaw_des_) - uy * std::cos(yaw_des_), -0.4f, 0.4f);
            float phi_des = std::asin(a_roll);
            float a_pitch = std::clamp((ux * std::cos(yaw_des_) + uy * std::sin(yaw_des_)) / std::cos(phi_des), -0.4f, 0.4f);
            float theta_des = std::asin(a_pitch);
            return {phi_des, theta_des};
        }

        // === Publishers ===
        void publish_offboard_mode() {
            OffboardControlMode msg{};
            msg.attitude = true;
            msg.timestamp = now_us();
            pub_offboard_mode_->publish(msg);
        }

        void publish_attitude_sp() {
            VehicleAttitudeSetpoint msg{};
            Eigen::AngleAxisd rollAngle(roll_des_, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch_des_, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw_des_, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
            msg.q_d = {static_cast<float>(q.w()), static_cast<float>(q.x()), 
                    static_cast<float>(q.y()), static_cast<float>(q.z())};
            msg.thrust_body = {0.0f, 0.0f, thrust_des_};
            msg.timestamp = now_us();
            pub_attitude_sp_->publish(msg);
        }

        void send_vehicle_command(uint16_t cmd, float p1, float p2) {
            VehicleCommand msg{};
            msg.param1 = p1;
            msg.param2 = p2;
            msg.command = cmd;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = now_us();
            pub_vehicle_cmd_->publish(msg);
        }

        // === Utility ===
        uint64_t now_us() const {
            return this->now().nanoseconds() / 1000;
        }
    };

    // === Main Function ===
    int main(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OffboardControl>());
        rclcpp::shutdown();
        return 0;
    }
