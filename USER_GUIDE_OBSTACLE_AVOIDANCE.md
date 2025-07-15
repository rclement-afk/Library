## Obstacle Management & Obstacle-Aware Navigation

### Adding, Removing, and Clearing Obstacles

- **Add:**  
  ```c
  add_obstacle_rect(10, 15, 4, 5); // Adds and saves a 4x5" obstacle at (10, 15)
  ```
- **Remove (by index):**  
  ```c
  remove_obstacle(0); // Removes the first obstacle
  ```
- **Clear all:**  
  ```c
  clear_obstacles();
  ```
- **Load obstacles at startup:**  
  ```c
  load_obstacles();
  ```

Obstacles are stored in `obstacles.csv` and are persistent.

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

### Obstacle-Aware Navigation

**Function:**  
```c
int goto_xy_heading_avoid(float target_x, float target_y, float target_heading_deg, int speed);
```

**Purpose:**  
Moves the robot to the specified field coordinate and heading, automatically planning a path that avoids all defined obstacles.

- If the straight-line path is clear, it proceeds directly.
- If blocked, it detours around the nearest obstacle, then resumes toward the target.
- Obstacles must be loaded (call `load_obstacles()`) before use.

**Example:**
```c
load_obstacles();
goto_xy_heading_avoid(40.0, 40.0, 90.0, 50);
```

---

**Note:**  
The current implementation uses a simple detour strategy (go around nearest obstacle). For complex fields, an advanced path planner (A*, RRT) is recommended.