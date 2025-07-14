# RobotLib User Guide

Welcome to **RobotLib**, a C++-powered, C-callable robotics library designed for clean, reliable, and flexible control of motors, servos, and sensors on your robot. This guide covers setup, initialization, API functions, and integration examples.

---

## Table of Contents

1. [Overview](#overview)
2. [Setup & Integration](#setup--integration)
3. [Device Structs](#device-structs)
4. [Initialization](#initialization)
5. [API Reference](#api-reference)
    - [Driving & Turning](#driving--turning)
    - [Servo Control](#servo-control)
    - [Squareup](#squareup)
    - [Calibration](#calibration)
    - [Pose Initialization](#pose-initialization)
6. [Example Usage (C)](#example-usage-c)
7. [Customization](#customization)
8. [Extending the Library](#extending-the-library)
9. [Troubleshooting](#troubleshooting)

---

## Overview

- **Thread-safe servo/motor operations** using modern C++.
- **Hybrid drive/turn** using both encoder ticks and gyro sensor.
- **IR sensor integration** for line following/squareup.
- **Easy C API** for use in traditional robotics code.

---

## Setup & Integration

1. **Add Files to Your Project:**
    - `robotlib.hpp` (C++ header)
    - `robotlib.cpp` (C++ implementation)
    - `robotlib_c_api.h` (C header)

2. **Compile & Link:**
    - Compile `robotlib.cpp` as C++ and link with your project.
    - Include `robotlib_c_api.h` in your C files.

    ```c
    #include "robotlib_c_api.h"
    ```

3. **Edit Device Structs If Needed:**
    Change port numbers in `robotlib.hpp` to match your hardware.

---

## Device Structs

At the top of `robotlib.hpp`, you'll find:

```cpp
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
```

**Tip:**  
Edit these static consts to match your actual motor and sensor ports.

---

## Initialization

Call this at the very start of your main program to:
- Calibrate the gyro (zero out drift)
- Set your robot's pose/orientation

```c
init_robot_pose(float x, float y, float orientation_deg);
```

Example:
```c
init_robot_pose(0, 0, 0); // Start at origin, 0 degrees
```

---

## API Reference

### Driving & Turning

```c
void drive(int ticks, int speed);
// Drives forward (or backward for negative speed) by a given number of encoder ticks.

void turn(int degrees, int speed);
// Turns the robot in place by a given number of degrees (+ for right, - for left).
```

---

### Servo Control

```c
void move_servo(int port, int target_pos, int duration_ms);
// Moves a servo asynchronously (non-blocking) to the specified position over duration (ms).

void move_servo_blocking(int port, int target_pos, int duration_ms);
// Moves a servo and waits (blocking) until the move completes.
```

---

### Squareup

```c
void squareup(int direction, int speed);
// Drives (forward if direction=1, backward if direction=-1) until both IR sensors see a line.
```

---

### Calibration

```c
void calibrate_turn(float* left_tpi, float* right_tpi);
// Runs a +90 and -90 degree turn in both forward and backward directions.
// Fills output arrays with ticks-per-inch for left/right motors.
// Output arrays must be at least [2]: [0]=forward, [1]=backward.
```

---

### Pose Initialization

```c
void init_robot_pose(float x, float y, float orientation_deg);
// Sets robot pose (x,y,heading) and calibrates gyro bias.
```

---

## Example Usage (C)

```c
#include "robotlib_c_api.h"

int main() {
    init_robot_pose(0, 0, 0);

    // Drive forward 1000 ticks at speed 50
    drive(1000, 50);

    // Turn right 90 degrees at speed 40
    turn(90, 40);

    // Move servo on port 0 to position 1200 in 500 ms (blocking)
    move_servo_blocking(0, 1200, 500);

    // Square up with IR sensors, moving forward
    squareup(1, 30);

    // Calibrate turning (example)
    float left_tpi[2], right_tpi[2];
    calibrate_turn(left_tpi, right_tpi);
    // left_tpi[0] = forward, left_tpi[1] = backward, etc.

    return 0;
}
```

---

## Customization

- **Change port numbers** in `robotlib.hpp` to match your robot.
- **Add more sensors:**  
  Extend the IR struct or create new structs for bumpers, encoders, etc.

---

## Extending the Library

- **Live pose/orientation tracking:**  
  The library is ready for further expansion. Add functions to update/read the pose/orientation as you move.
- **Other device groups:**  
  Add more static structs for sensors, actuators, bumpers, or custom logic as needed.

---

## Troubleshooting

- **Servo/motor not responding:**  
  Ensure you've set port numbers correctly and your hardware API calls are properly hooked up in `robotlib.cpp`.
- **Compilation errors:**  
  Make sure `robotlib.cpp` is compiled as C++ and linked with your C code.
- **Gyro drift or inconsistent turns:**  
  Always call `init_robot_pose()` at startup to calibrate the gyro bias.

---

**Need more examples or want to see advanced usage? Just ask!**

---
