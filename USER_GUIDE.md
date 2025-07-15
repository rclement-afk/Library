# RobotLib User Guide

## Motion Profiles: Acceleration and Deceleration

All robot motion (drive/turn) uses smooth ramping for acceleration and deceleration, following a trapezoidal velocity profile. This improves accuracy and reduces wheel slip/wear.

### Adjustable Parameters

You can adjust these variables at runtime before any drive/turn/move command:

```c
DRIVE_MAX_ACCELERATION = 24.0f; // inches/sec^2 (default: 24.0)
DRIVE_MAX_DECELERATION = 24.0f; // inches/sec^2 (default: 24.0)
DRIVE_MIN_SPEED = 2.0f;         // inches/sec   (default: 2.0)
DRIVE_MAX_SPEED = 12.0f;        // inches/sec   (default: 12.0)

TURN_MAX_ACCELERATION = 90.0f;  // deg/sec^2    (default: 90.0)
TURN_MAX_DECELERATION = 90.0f;  // deg/sec^2    (default: 90.0)
TURN_MIN_SPEED = 10.0f;         // deg/sec      (default: 10.0)
TURN_MAX_SPEED = 180.0f;        // deg/sec      (default: 180.0)
```

You do not need to change your `drive` or `turn` calls—ramping is always active.

---

## Robot Geometry, Base Configuration, and Field Point Calculation

### Setting the Robot Base Dimensions and Centerpoint

```c
void set_robot_base(float width_x, float length_y, float center_offset_x, float center_offset_y);
```
- `width_x`: Robot width (side-to-side), in inches.
- `length_y`: Robot length (front-to-back), in inches.
- `center_offset_x`: X offset from the **bottom-left corner** (when facing forward) to your robot's centerpoint.
- `center_offset_y`: Y offset from the bottom-left corner to the centerpoint.

**Example:**
```c
set_robot_base(15.0, 18.0, 7.5, 9.0); // Centerpoint at center of 15x18" robot
```

---

### Calculating the Field Position of Any Point on the Robot

```c
void get_point_on_robot(float offset_x, float offset_y, float* field_x, float* field_y);
```
- `offset_x`: Offset (in inches) from the robot center, +x is right.
- `offset_y`: Offset (in inches) from the robot center, +y is forward.

**Example:**
```c
// Gripper 8" ahead of center:
float gx, gy;
get_point_on_robot(0.0, 8.0, &gx, &gy);
```

---

### Getting the Field Coordinates of the Robot Corners

```c
void get_robot_corners(float field_x[4], float field_y[4]);
```
- Returns the corners: [0]=front-left, [1]=front-right, [2]=back-right, [3]=back-left.

---

## Obstacle Management

### Adding, Removing, and Clearing Obstacles

- **Add:**  
  ```c
  add_obstacle_rect(10, 15, 4, 5);
  ```
- **Remove (by index):**  
  ```c
  remove_obstacle(0);
  ```
- **Clear all:**  
  ```c
  clear_obstacles();
  ```
- **Load obstacles at startup:**  
  ```c
  load_obstacles();
  ```

Obstacles are stored persistently in `obstacles.csv`.

---

### Querying Obstacles

```c
int n = get_obstacle_count();
for (int i = 0; i < n; ++i) {
    float x, y, w, h;
    get_obstacle_rect(i, &x, &y, &w, &h);
    printf("Obstacle %d: (%.1f, %.1f) size %.1fx%.1f\n", i, x, y, w, h);
}
```

---

## Obstacle-Aware Navigation

### Simple Detour Planner

```c
int goto_xy_heading_avoid_simple(float target_x, float target_y, float target_heading_deg, int speed);
```
- Checks if a straight path is clear. If not, detours around the nearest obstacle.

### Advanced A* Planner

```c
int goto_xy_heading_avoid_advanced(float target_x, float target_y, float target_heading_deg, int speed);
```
- Uses a grid-based A* algorithm to plan a path around all obstacles.
- Handles complex fields with multiple obstacles.

### Default (calls Advanced)

```c
int goto_xy_heading_avoid(float target_x, float target_y, float target_heading_deg, int speed);
```
- Calls the advanced version by default. You may switch to the simple version for comparison.

**Example:**
```c
load_obstacles();
goto_xy_heading_avoid(40.0, 40.0, 90.0, 50);
```

---

## Typical Workflow

1. **Set robot geometry and pose** at startup.
2. **Load obstacles** (once per run).
3. **Plan and run obstacle-aware navigation** as needed.

---
