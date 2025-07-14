#pragma once

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

// --- Device Structs ---

typedef struct {
    int Left;
    int Right;
} motor_pair_t;

static const motor_pair_t motors = { .Left = 0, .Right = 1 };

typedef struct {
    int Left;
    int Right;
    // int Center; // Uncomment if needed
    // int Rear;   // Uncomment if needed
} ir_sensor_pair_t;

static const ir_sensor_pair_t irs = { .Left = 2, .Right = 3 };

// --- Servo Movement Request Structure ---

struct ServoMove {
    int port;
    int target_pos;
    int duration_ms;
    bool blocking;
    std::condition_variable* cv_done;
    bool* done_flag;
};

// --- Servo Controller Class ---

class ServoController {
public:
    ServoController();
    ~ServoController();

    // Non-blocking move
    void move_servo(int port, int target_pos, int duration_ms);

    // Blocking move
    void move_servo_blocking(int port, int target_pos, int duration_ms);

    // Singleton instance for global access
    static ServoController& instance();

private:
    std::thread worker;
    std::mutex queue_mutex;
    std::queue<ServoMove> moves;
    std::condition_variable cv_new_move;
    std::atomic<bool> stop_flag;

    void worker_loop();
};

// --- Internal Robot Pose State ---

struct RobotPose {
    float x;
    float y;
    float orientation_deg;
};

extern RobotPose robot_pose;
extern float gyro_bias_value;

// --- C API Declarations ---

extern "C" {

void drive(int ticks, int speed);
void turn(int degrees, int speed);
void squareup(int direction, int speed);
void calibrate_turn(float* left_tpi, float* right_tpi);
void move_servo(int port, int target_pos, int duration_ms);
void move_servo_blocking(int port, int target_pos, int duration_ms);
void init_robot_pose(float x, float y, float orientation_deg);

}
