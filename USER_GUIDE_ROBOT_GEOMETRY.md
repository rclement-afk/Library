## Robot Geometry, Base Configuration, and Field Point Calculation

### Overview
This library provides advanced functions to:
- **Configure the robot's physical base dimensions and centerpoint** for accurate field representation.
- **Calculate the absolute field position of any point** on your robot (such as a sensor, gripper, or corner).
- **Retrieve the coordinates of the robot's corners** for visualization, collision detection, or mapping.

---

### Setting the Robot Base Dimensions and Centerpoint

Use `set_robot_base` to configure your robot's shape and reference point.

#### Function
```c
void set_robot_base(float width_x, float length_y, float center_offset_x, float center_offset_y);
```

- `width_x` — Robot width (side-to-side), in inches.
- `length_y` — Robot length (front-to-back), in inches.
- `center_offset_x` — X offset (in inches) from the **bottom-left corner** (when facing forward) to your robot's centerpoint.
- `center_offset_y` — Y offset (in inches) from the bottom-left corner to the centerpoint.

#### Example
```c
// Robot is 15" wide, 18" long, centerpoint is at the center:
set_robot_base(15.0, 18.0, 7.5, 9.0);

// Robot is 15" wide, 18" long, centerpoint is 5" from left edge and 13" from back:
set_robot_base(15.0, 18.0, 5.0, 13.0);
```

---

### Getting Robot Base Dimensions and Centerpoint

```c
void get_robot_base(float* width_x, float* length_y, float* center_offset_x, float* center_offset_y);
```

---

### Calculating the Field Position of Any Point on the Robot

Use `get_point_on_robot` to compute the field coordinates of any point on your robot, given its local offset from the center.

#### Function
```c
void get_point_on_robot(float offset_x, float offset_y, float* field_x, float* field_y);
```
- `offset_x` — Offset (in inches) from the robot center, +x is right.
- `offset_y` — Offset (in inches) from the robot center, +y is forward.
- `field_x`, `field_y` — Output: Field (absolute) coordinates of this point.

#### Purpose
- **Sensor/Tool Positioning:** Find where a sensor or tool (not at the center) is on the field.
- **Interaction:** Approach or manipulate objects using off-center attachments.
- **Debugging:** Visualize robot parts in field coordinates.

#### Example
```c
// Find the field position of a gripper 8" in front of the robot center (no lateral offset):
float grip_x, grip_y;
get_point_on_robot(0.0, 8.0, &grip_x, &grip_y);
printf("Gripper is at (%.1f, %.1f)\n", grip_x, grip_y);

// Sensor 2" to the right and 3" ahead:
get_point_on_robot(2.0, 3.0, &field_x, &field_y);
```

---

### Getting the Field Coordinates of the Robot Corners

Use `get_robot_corners` to compute the absolute (x, y) coordinates of all four robot corners, for visualization or collision checks.

#### Function
```c
void get_robot_corners(float field_x[4], float field_y[4]);
```
- Returns arrays of the four corners:
  - [0] = front-left
  - [1] = front-right
  - [2] = back-right
  - [3] = back-left
  (All relative to the robot's heading—front is +Y.)

#### Purpose
- **Visualization:** Draw the robot on a map or GUI.
- **Collision Detection:** Check for field boundary or obstacle collisions.
- **Advanced Path Planning:** Precise robot footprint location.

#### Example
```c
float cx[4], cy[4];
get_robot_corners(cx, cy);
for (int i = 0; i < 4; ++i) {
    printf("Corner %d: (%.1f, %.1f)\n", i, cx[i], cy[i]);
}
```

---

### Typical Workflow

1. **Set your robot's base and centerpoint** at program startup:
   ```c
   set_robot_base(15.0, 18.0, 7.5, 9.0);
   ```
2. **Initialize the robot pose:**
   ```c
   init_robot_pose(0, 0, 90);
   ```
3. **When you need to know where a part is on the field:**
   ```c
   get_point_on_robot(0.0, 8.0, &px, &py); // e.g., gripper 8" ahead
   ```
4. **For drawing or checking robot corners:**
   ```c
   get_robot_corners(corner_x, corner_y);
   ```

---

### Notes

- **All geometry is based on the robot pose and heading, which is always referenced to the centerpoint you configure.**
- **Offsets and dimensions are always in inches.**
- **You can reconfigure the robot base and centerpoint at any time, but usually do this only at startup.**
