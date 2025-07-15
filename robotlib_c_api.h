#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h> // For NAN

// ---- MOTION PROFILE PARAMETERS (CAN BE TWEAKED AT RUNTIME) ----
// Units: all *_ACCELERATION and *_DECELERATION in inches/sec^2 or deg/sec^2.
// *_SPEED in inches/sec or deg/sec.

// DRIVE (straight motion)
extern float DRIVE_MAX_ACCELERATION; // [in/sec^2] Default: 24.0
extern float DRIVE_MAX_DECELERATION; // [in/sec^2] Default: 24.0
extern float DRIVE_MIN_SPEED;        // [in/sec]   Default: 2.0
extern float DRIVE_MAX_SPEED;        // [in/sec]   Default: 12.0

// TURN (in-place rotation)
extern float TURN_MAX_ACCELERATION;  // [deg/sec^2] Default: 90.0
extern float TURN_MAX_DECELERATION;  // [deg/sec^2] Default: 90.0
extern float TURN_MIN_SPEED;         // [deg/sec]   Default: 10.0
extern float TURN_MAX_SPEED;         // [deg/sec]   Default: 180.0

// ---- TIMER PROTOCOL ----
void timer_reset(void);
unsigned long timer_elapsed_ms(void);
void timer_print(const char* label);

// ---- ROBOT POSE API ----
void init_robot_pose(float x, float y, float orientation_deg);
void get_robot_pose(float* x, float* y, float* orientation_deg);
void print_robot_pose(const char* label);

// ---- MOTION API (ramping built-in) ----
void drive(int ticks, int speed);  // [ticks]=encoder ticks, [speed]=cruise speed, ramping always applied
void turn(int degrees, int speed); // [degrees], [speed]=cruise speed, ramping always applied
void squareup(int direction, int speed);
void calibrate_turn(float* left_tpi, float* right_tpi);

// ---- SERVO API ----
void move_servo(int port, int target_pos, int duration_ms);
void move_servo_blocking(int port, int target_pos, int duration_ms);

// ---- GO TO POINT API ----
int goto_xy_heading(float target_x, float target_y, float target_heading_deg, int speed);

// ---- ROBOT BASE CONFIGURATION ----
void set_robot_base(float width_x, float length_y, float center_offset_x, float center_offset_y);
void get_robot_base(float* width_x, float* length_y, float* center_offset_x, float* center_offset_y);

// ---- ROBOT GEOMETRY HELPERS ----
void get_point_on_robot(float offset_x, float offset_y, float* field_x, float* field_y);
void get_robot_corners(float field_x[4], float field_y[4]);

// ---- OBSTACLE MANAGEMENT ----
void add_obstacle_rect(float x, float y, float width, float height);
void load_obstacles(void);
int get_obstacle_count(void);
int get_obstacle_rect(int n, float* x, float* y, float* width, float* height);
int remove_obstacle(int n);
void clear_obstacles(void);

// ---- OBSTACLE-AWARE GOTO ----
// Default: calls the advanced (A*) planner.
int goto_xy_heading_avoid(float target_x, float target_y, float target_heading_deg, int speed);
// Advanced (A*) version
int goto_xy_heading_avoid_advanced(float target_x, float target_y, float target_heading_deg, int speed);
// Simple detour version
int goto_xy_heading_avoid_simple(float target_x, float target_y, float target_heading_deg, int speed);

#ifdef __cplusplus
}
#endif