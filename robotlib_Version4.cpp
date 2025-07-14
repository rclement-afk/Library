#include "robotlib.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>

// ---- Internal Robot Pose and Gyro Bias ----
RobotPose robot_pose = {0, 0, 0};
float gyro_bias_value = 0.0f;

// --- ServoController Implementation ---

ServoController::ServoController() : stop_flag(false) {
    worker = std::thread(&ServoController::worker_loop, this);
}

ServoController::~ServoController() {
    stop_flag = true;
    cv_new_move.notify_all();
    if (worker.joinable()) worker.join();
}

ServoController& ServoController::instance() {
    static ServoController instance;
    return instance;
}

void ServoController::move_servo(int port, int target_pos, int duration_ms) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    moves.push({port, target_pos, duration_ms, false, nullptr, nullptr});
    cv_new_move.notify_one();
}

void ServoController::move_servo_blocking(int port, int target_pos, int duration_ms) {
    std::condition_variable cv;
    bool done = false;
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        moves.push({port, target_pos, duration_ms, true, &cv, &done});
        cv_new_move.notify_one();
    }
    std::mutex dummy;
    std::unique_lock<std::mutex> lock(dummy);
    cv.wait(lock, [&done]{ return done; });
}

void ServoController::worker_loop() {
    while (!stop_flag) {
        ServoMove move;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (moves.empty()) {
                cv_new_move.wait(lock, [this]{ return stop_flag || !moves.empty(); });
                if (stop_flag) break;
            }
            if (moves.empty()) continue;
            move = moves.front();
            moves.pop();
        }
        // Simulate servo movement
        // Replace with actual servo interface
        printf("[Servo] Moving port %d to %d over %d ms\n", move.port, move.target_pos, move.duration_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(move.duration_ms));
        if (move.blocking && move.cv_done && move.done_flag) {
            *(move.done_flag) = true;
            move.cv_done->notify_one();
        }
    }
}

// --------------- Robot Pose Initialization ---------------

extern "C" void init_robot_pose(float x, float y, float orientation_deg) {
    // Simulated gyro_bias() function (replace with real hardware call)
    auto gyro_bias = []() -> float {
        // TODO: Replace with actual hardware averaging
        printf("[Gyro] Measuring bias...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return 1.42f;
    };
    gyro_bias_value = gyro_bias();
    robot_pose.x = x;
    robot_pose.y = y;
    robot_pose.orientation_deg = orientation_deg;
    printf("[Init] Set pose to (%.2f, %.2f) heading %.2f°, gyro_bias=%.2f\n",
        x, y, orientation_deg, gyro_bias_value);
}

// --------------- Servo API Implementation ---------------

extern "C" void move_servo(int port, int target_pos, int duration_ms) {
    ServoController::instance().move_servo(port, target_pos, duration_ms);
}

extern "C" void move_servo_blocking(int port, int target_pos, int duration_ms) {
    ServoController::instance().move_servo_blocking(port, target_pos, duration_ms);
}

// --------------- Hybrid Drive/Turn API -------------------

// Simulated hardware functions (replace with actual robot API)
static void set_motor(int port, int speed) {
    printf("[Motors] Set port %d to speed %d\n", port, speed);
}
static int get_motor_ticks(int port) {
    // TODO: Replace with actual tick counter
    static int fake_ticks[2] = {0, 0};
    return fake_ticks[port];
}
static float read_gyro() {
    // TODO: Replace with actual gyro read (subtract bias)
    return 0.0f;
}
static void reset_motor_ticks(int port) {
    printf("[Motors] Reset ticks for port %d\n", port);
}

extern "C" void drive(int ticks, int speed) {
    // Reset encoders
    reset_motor_ticks(motors.Left);
    reset_motor_ticks(motors.Right);

    int left_start = get_motor_ticks(motors.Left);
    int right_start = get_motor_ticks(motors.Right);

    set_motor(motors.Left, speed);
    set_motor(motors.Right, speed);

    // Poll for completion
    while (true) {
        int left_ticks = std::abs(get_motor_ticks(motors.Left) - left_start);
        int right_ticks = std::abs(get_motor_ticks(motors.Right) - right_start);
        if (left_ticks >= std::abs(ticks) && right_ticks >= std::abs(ticks)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    set_motor(motors.Left, 0);
    set_motor(motors.Right, 0);
}

extern "C" void turn(int degrees, int speed) {
    // Hybrid: use gyro and ticks if available (simplified demo)
    reset_motor_ticks(motors.Left);
    reset_motor_ticks(motors.Right);

    int left_dir = (degrees > 0) ? 1 : -1;
    int right_dir = -left_dir;

    set_motor(motors.Left, speed * left_dir);
    set_motor(motors.Right, speed * right_dir);

    // TODO: Replace with actual angle tracking via gyro + ticks
    float target = std::abs(degrees);
    float turned = 0;
    while (turned < target) {
        // Simulate angle update
        turned += 1.0f;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    set_motor(motors.Left, 0);
    set_motor(motors.Right, 0);
}

extern "C" void squareup(int direction, int speed) {
    // Example: drive until both IRs see line (pseudo)
    set_motor(motors.Left, speed * direction);
    set_motor(motors.Right, speed * direction);
    // TODO: Replace with actual IR sensor check loop
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    set_motor(motors.Left, 0);
    set_motor(motors.Right, 0);
}

extern "C" void calibrate_turn(float* left_tpi, float* right_tpi) {
    // Calibrate both +90 and -90 deg, forward and backward
    // For demo: just fill with fake data
    for (int dir = 0; dir < 2; ++dir) {
        // dir==0: forward, dir==1: backward
        left_tpi[dir] = 42.0f + dir;
        right_tpi[dir] = 43.0f + dir;
        printf("[Calib] %s: left=%.2f, right=%.2f\n", dir==0?"FWD":"BWD", left_tpi[dir], right_tpi[dir]);
    }
}