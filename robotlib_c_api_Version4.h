#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Hybrid drive/turn API
void drive(int ticks, int speed);
void turn(int degrees, int speed);
void squareup(int direction, int speed);

// Calibrate: records ticks per inch for +90 and -90 deg, for both motors, forward/backward
// Output arrays must be at least [2]: 0=forward, 1=backward
void calibrate_turn(float* left_tpi, float* right_tpi);

// Servo API
void move_servo(int port, int target_pos, int duration_ms);           // Non-blocking
void move_servo_blocking(int port, int target_pos, int duration_ms);  // Blocking

// Initialization (call at program start)
void init_robot_pose(float x, float y, float orientation_deg);

#ifdef __cplusplus
}
#endif