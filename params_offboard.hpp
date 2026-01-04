#include <cmath>
#pragma once

constexpr float zero         = 0.0f;

constexpr char ARM           = 'A';
constexpr char DISARM        = 'D';
constexpr char LAND          = 'L';

// Variables
inline float f_thrust_        = zero;
inline float thrust_          = zero;

inline bool offboard_mode_    = false;
inline bool initialized_      = false;
inline bool time_initialized_ = false;

inline bool land_mode_        = false;

inline float psi_init_        = zero;
inline float z_init_          = zero;
inline float x_init_          = zero;
inline float y_init_          = zero;

inline float x_init_body_     = zero;
inline float y_init_body_     = zero;
inline float z_init_body_     = zero;

inline float qw_              = zero;
inline float qx_              = zero; 
inline float qy_              = zero;
inline float qz_              = zero;

inline float dx               = zero;
inline float dy               = zero;

inline float vxy_max_         = zero;

// Constant 
constexpr float radius_      = 2.0;
constexpr float vel_xy_      = 2.0;

constexpr float e_x          = 0.8;  //0.7
constexpr float w_x          = 2.67;  //0.33

constexpr float e_y          = 0.8;  //0.8
constexpr float w_y          = 2.67; //0.31

constexpr float e_z          = 0.8; //0.85
constexpr float w_z          = 5.1;  //4.5
 
constexpr float Kp1_x        = 2*e_x*w_x;
constexpr float Kp2_x        = w_x/(2*e_x);

constexpr float Kp1_y        = 2*e_y*w_y;
constexpr float Kp2_y        = w_y/(2*e_y);

constexpr float Kp1_z        = 2*e_z*w_z;
constexpr float Kp2_z        = w_z/(2*e_z);

constexpr float Jx           = 0.052380;
constexpr float Jy           = 0.052380;
constexpr float Jz           = 0.099032;

constexpr float m            = 2.72f;
constexpr float g            = 9.8066f;
constexpr float b            = 7.2706e-06;
constexpr float d            = 0.06*b;
constexpr float l            = 0.273;

constexpr float max_motor_vel = 1199.0f;
constexpr float min_motor_vel = 178.0f;

constexpr float f_thrust_max_ = 8*b*max_motor_vel*max_motor_vel;

constexpr float w0           = sqrt(m*g/(4*b));
constexpr float n0           = w0*60/(2*M_PI);
constexpr float u0           = (n0 -1075)/7525; 
constexpr float lamda        = (w0*w0)/u0;

// receive
inline float x_r_             = zero;
inline float y_r_             = zero;
inline float z_r_             = zero;

inline float x_body_r_        = zero;
inline float y_body_r_        = zero;

inline float phi_r_           = zero;
inline float theta_r_         = zero;
inline float psi_r_           = zero;

inline float vx_r_            = zero;
inline float vy_r_            = zero;
inline float vz_r_            = zero;

inline float omg_phi_r_       = zero;
inline float omg_theta_r_     = zero;
inline float omg_psi_r_       = zero;

inline float rc_offboard_sw6_ = zero;

// struct PIDParams {
//     float Kp_z = 2.0f;
//     float Ki_z = 0.0f;
//     float Kd_z = 0.0f;
//     float Kaw_z = 0.1f;

//     float thrust_ = zero;

//     float tau_z_ = 0.05f;   // filter derivative
//     float dt_ = 0.01f;      // time step
//     float i_limit_z_ = 2.0f; // integral limit
//     float integral_z_ = zero; // integral term
//     float derivative_z_ = zero; // derivative term
//     float prev_error_z_ = zero; // previous error for derivative calculation
// };
