## Working with Known Obstacles

### Adding Obstacles

Add an obstacle (rectangle) at any time:
```c
add_obstacle_rect(10.0, 15.0, 4.0, 5.0); // Adds and saves a 4x5" obstacle at (10, 15)
```

### Saving and Loading Obstacles

- Obstacles are **immediately saved** to the config file (`obstacles.csv`) each time you add one.
- At program startup, call:
  ```c
  load_obstacles();
  ```
  to load all previously saved obstacles.

### Accessing Obstacles

- Get the number of obstacles:
  ```c
  int n = get_obstacle_count();
  ```
- Retrieve details of each:
  ```c
  float x, y, w, h;
  for (int i = 0; i < get_obstacle_count(); ++i) {
      get_obstacle_rect(i, &x, &y, &w, &h);
      printf("Obstacle %d: (%.1f, %.1f) size %.1fx%.1f\n", i, x, y, w, h);
  }
  ```

---

**Purpose:**  
Use this system to record fixed obstacles (walls, barriers) on your field that the robot should avoid or account for in path planning.