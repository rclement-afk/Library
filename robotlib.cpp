#include "robotlib_c_api.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <queue>
#include <tuple>
#include <set>
#include <algorithm>

// ---- Motion profile parameters ----
float DRIVE_MAX_ACCELERATION = 24.0f;
float DRIVE_MAX_DECELERATION = 24.0f;
float DRIVE_MIN_SPEED = 2.0f;
float DRIVE_MAX_SPEED = 12.0f;

float TURN_MAX_ACCELERATION = 90.0f;
float TURN_MAX_DECELERATION = 90.0f;
float TURN_MIN_SPEED = 10.0f;
float TURN_MAX_SPEED = 180.0f;

// ---- Timer Protocol ----
static std::chrono::steady_clock::time_point timer_start;

extern "C" void timer_reset(void) {
    timer_start = std::chrono::steady_clock::now();
}

extern "C" unsigned long timer_elapsed_ms(void) {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - timer_start).count();
}

extern "C" void timer_print(const char* label) {
    printf("[%s] Elapsed: %lu ms\n", label ? label : "Timer", timer_elapsed_ms());
}

// ---- Robot Pose Tracking ----
static float robot_x = 0.0f;
static float robot_y = 0.0f;
static float robot_theta = 0.0f; // degrees

extern "C" void init_robot_pose(float x, float y, float orientation_deg) {
    robot_x = x;
    robot_y = y;
    robot_theta = std::fmod(orientation_deg, 360.0f);
    if (robot_theta < 0) robot_theta += 360.0f;
}

extern "C" void get_robot_pose(float* x, float* y, float* orientation_deg) {
    if(x) *x = robot_x;
    if(y) *y = robot_y;
    if(orientation_deg) *orientation_deg = robot_theta;
}

extern "C" void print_robot_pose(const char* label) {
    printf("[%s] Pose: (%.1f, %.1f) angle %.1f deg\n", 
        label ? label : "Robot", robot_x, robot_y, robot_theta);
}

static float normalize_angle(float deg) {
    float a = std::fmod(deg, 360.0f);
    if (a < 0) a += 360.0f;
    return a;
}

// ---- Motion API (drive/turn with ramping) ----

// Simulated sleep for demonstration (replace with real time control!)
static void sim_sleep_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
}

// Helper: trapezoidal velocity profile for drive/turn
static void ramped_motion(float total_dist, float max_speed, float min_speed, float accel, float decel, 
                          void (*set_motion)(float), const char* label) {
    // total_dist: inches or degrees
    // set_motion: callback that sets speed (positive or negative for direction)
    // label: for printf
    float dist = std::abs(total_dist);
    float dir = (total_dist >= 0) ? 1.0f : -1.0f;
    float v0 = min_speed;
    float vmax = std::max(min_speed, std::min(max_speed, dist)); // Don't allow cruise faster than distance!
    float a = accel;
    float d = decel;
    if (a <= 0) a = 24.0f;
    if (d <= 0) d = 24.0f;

    // Calculate distances for accel and decel
    float accel_dist = (vmax*vmax - v0*v0) / (2*a);
    float decel_dist = (vmax*vmax - v0*v0) / (2*d);
    float cruise_dist = dist - accel_dist - decel_dist;
    if (cruise_dist < 0) {
        // No cruise region, triangle profile
        accel_dist = decel_dist = dist/2;
        vmax = std::sqrt((a*d*dist + (d*v0*v0 + a*v0*v0))/(a + d));
        cruise_dist = 0;
    }

    // Acceleration phase
    float x = 0.0f, v = v0;
    float dt = 0.02f; // 20 ms steps
    printf("%s: ramp up\n", label);
    while (x < accel_dist) {
        v = std::min(vmax, std::sqrt(v0*v0 + 2*a*(x)));
        set_motion(v*dir);
        sim_sleep_ms(static_cast<int>(dt*1000));
        x += v*dt;
    }
    // Cruise phase
    printf("%s: cruise\n", label);
    float cruise_x = 0;
    while (cruise_x < cruise_dist) {
        set_motion(vmax*dir);
        sim_sleep_ms(static_cast<int>(dt*1000));
        cruise_x += vmax*dt;
    }
    // Deceleration phase
    printf("%s: ramp down\n", label);
    float decel_x = 0;
    v = vmax;
    while (decel_x < decel_dist) {
        float rem = decel_dist - decel_x;
        v = std::max(min_speed, std::sqrt(std::max(0.0f, vmax*vmax - 2*d*(decel_x))));
        set_motion(v*dir);
        sim_sleep_ms(static_cast<int>(dt*1000));
        decel_x += v*dt;
    }
    // Stop
    set_motion(0);
}

static void set_drive_speed(float speed) {
    // Placeholder: here you would set the actual hardware speed
    printf("  [HW] set drive speed: %.2f\n", speed);
    // For simulation: update odometry
    float dt = 0.02f;
    float theta_rad = robot_theta * static_cast<float>(M_PI) / 180.0f;
    robot_x += speed * dt * std::cos(theta_rad);
    robot_y += speed * dt * std::sin(theta_rad);
}

static void set_turn_speed(float speed) {
    // Placeholder: here you would set the actual hardware turn speed
    printf("  [HW] set turn speed: %.2f\n", speed);
    // For simulation: update odometry
    float dt = 0.02f;
    robot_theta = normalize_angle(robot_theta + speed * dt);
}

extern "C" void drive(int ticks, int speed) {
    // speed is the cruise speed (in ticks/sec, converted to in/sec)
    float ticks_per_inch = 100.0f; // Replace with calibration!
    float dist = ticks / ticks_per_inch;
    ramped_motion(dist, DRIVE_MAX_SPEED, DRIVE_MIN_SPEED, DRIVE_MAX_ACCELERATION, DRIVE_MAX_DECELERATION, set_drive_speed, "drive");
    printf("drive: %d ticks (%0.2f in) at speed %d\n", ticks, dist, speed);
}

extern "C" void turn(int degrees, int speed) {
    ramped_motion(degrees, TURN_MAX_SPEED, TURN_MIN_SPEED, TURN_MAX_ACCELERATION, TURN_MAX_DECELERATION, set_turn_speed, "turn");
    printf("turn: %d deg at speed %d\n", degrees, speed);
}

extern "C" void squareup(int direction, int speed) {
    printf("squareup: dir=%d speed=%d\n", direction, speed);
}

extern "C" void calibrate_turn(float* left_tpi, float* right_tpi) {
    if(left_tpi) *left_tpi = 100.0f;
    if(right_tpi) *right_tpi = 100.0f;
}

// --- Servo API ---
extern "C" void move_servo(int port, int target_pos, int duration_ms) {
    printf("move_servo: port=%d pos=%d dur=%d (non-blocking)\n", port, target_pos, duration_ms);
}
extern "C" void move_servo_blocking(int port, int target_pos, int duration_ms) {
    printf("move_servo_blocking: port=%d pos=%d dur=%d (blocking)\n", port, target_pos, duration_ms);
}

// ---- Go To Point API ----

extern "C" int goto_xy_heading(float target_x, float target_y, float target_heading_deg, int speed) {
    float dx = target_x - robot_x;
    float dy = target_y - robot_y;
    float distance = std::sqrt(dx*dx + dy*dy);
    if (distance < 0.01f) return 0;
    float target_angle = std::atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
    float angle_error = target_angle - robot_theta;
    while (angle_error < -180.0f) angle_error += 360.0f;
    while (angle_error >= 180.0f) angle_error -= 360.0f;
    turn(static_cast<int>(angle_error), speed);
    float ticks_per_inch = 100.0f; // Replace with your calibration!
    int drive_ticks = static_cast<int>(distance * ticks_per_inch);
    drive(drive_ticks, speed);
    if (!std::isnan(target_heading_deg)) {
        float theta_error = target_heading_deg - robot_theta;
        while (theta_error < -180.0f) theta_error += 360.0f;
        while (theta_error >= 180.0f) theta_error -= 360.0f;
        turn(static_cast<int>(theta_error), speed);
    }
    return 0;
}

// ---- Robot Base Configuration ----
static float robot_base_width_x = 12.0f;
static float robot_base_length_y = 12.0f;
static float robot_center_offset_x = 6.0f;
static float robot_center_offset_y = 6.0f;

extern "C" void set_robot_base(float width_x, float length_y, float center_offset_x, float center_offset_y) {
    robot_base_width_x = width_x;
    robot_base_length_y = length_y;
    robot_center_offset_x = center_offset_x;
    robot_center_offset_y = center_offset_y;
}

extern "C" void get_robot_base(float* width_x, float* length_y, float* center_offset_x, float* center_offset_y) {
    if (width_x) *width_x = robot_base_width_x;
    if (length_y) *length_y = robot_base_length_y;
    if (center_offset_x) *center_offset_x = robot_center_offset_x;
    if (center_offset_y) *center_offset_y = robot_center_offset_y;
}

// ---- Robot Geometry Helpers ----

extern "C" void get_point_on_robot(float offset_x, float offset_y, float* field_x, float* field_y) {
    float theta_rad = robot_theta * static_cast<float>(M_PI) / 180.0f;
    if (field_x) *field_x = robot_x + (offset_x * std::cos(theta_rad) - offset_y * std::sin(theta_rad));
    if (field_y) *field_y = robot_y + (offset_x * std::sin(theta_rad) + offset_y * std::cos(theta_rad));
}

extern "C" void get_robot_corners(float field_x[4], float field_y[4]) {
    float corners_local[4][2] = {
        {-(robot_center_offset_x), robot_base_length_y - robot_center_offset_y},
        {robot_base_width_x - robot_center_offset_x, robot_base_length_y - robot_center_offset_y},
        {robot_base_width_x - robot_center_offset_x, -robot_center_offset_y},
        {-(robot_center_offset_x), -robot_center_offset_y}
    };
    for (int i = 0; i < 4; ++i) {
        get_point_on_robot(corners_local[i][0], corners_local[i][1], &field_x[i], &field_y[i]);
    }
}

// ---- Obstacle Management ----

static const char* OBSTACLE_FILE = "obstacles.csv";
struct ObstacleRect { float x, y, w, h; };
static std::vector<ObstacleRect> obstacles;

static void save_obstacles_to_file() {
    std::ofstream ofs(OBSTACLE_FILE, std::ios::trunc);
    for (const auto& obs : obstacles) {
        ofs << obs.x << "," << obs.y << "," << obs.w << "," << obs.h << "\n";
    }
}

extern "C" void add_obstacle_rect(float x, float y, float width, float height) {
    obstacles.push_back({x, y, width, height});
    save_obstacles_to_file();
}

extern "C" void load_obstacles(void) {
    obstacles.clear();
    std::ifstream ifs(OBSTACLE_FILE);
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        ObstacleRect obs;
        char c;
        if (iss >> obs.x >> c >> obs.y >> c >> obs.w >> c >> obs.h) {
            obstacles.push_back(obs);
        }
    }
}

extern "C" int get_obstacle_count(void) {
    return obstacles.size();
}

extern "C" int get_obstacle_rect(int n, float* x, float* y, float* width, float* height) {
    if (n < 0 || n >= (int)obstacles.size()) return -1;
    if (x) *x = obstacles[n].x;
    if (y) *y = obstacles[n].y;
    if (width) *width = obstacles[n].w;
    if (height) *height = obstacles[n].h;
    return 0;
}

extern "C" int remove_obstacle(int n) {
    if (n < 0 || n >= (int)obstacles.size()) return -1;
    obstacles.erase(obstacles.begin() + n);
    save_obstacles_to_file();
    return 0;
}

extern "C" void clear_obstacles(void) {
    obstacles.clear();
    save_obstacles_to_file();
}

// ---- Simple Obstacle-Avoiding GOTO (detour around nearest obstacle) ----

static bool line_intersects_rect(float x1, float y1, float x2, float y2, const ObstacleRect& obs) {
    float rx0 = obs.x, ry0 = obs.y, rx1 = obs.x + obs.w, ry1 = obs.y + obs.h;
    auto seg_intersect = [](float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) {
        float denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);
        if (denom == 0) return false;
        float ua = ((x4-x3)*(y1-y3)-(y4-y3)*(x1-x3))/denom;
        float ub = ((x2-x1)*(y1-y3)-(y2-y1)*(x1-x3))/denom;
        return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
    };
    if (seg_intersect(x1,y1,x2,y2, rx0,ry0, rx1,ry0)) return true;
    if (seg_intersect(x1,y1,x2,y2, rx1,ry0, rx1,ry1)) return true;
    if (seg_intersect(x1,y1,x2,y2, rx1,ry1, rx0,ry1)) return true;
    if (seg_intersect(x1,y1,x2,y2, rx0,ry1, rx0,ry0)) return true;
    if (x1 > rx0 && x1 < rx1 && y1 > ry0 && y1 < ry1) return true;
    return false;
}

extern "C" int goto_xy_heading_avoid_simple(float target_x, float target_y, float target_heading_deg, int speed) {
    float start_x = robot_x, start_y = robot_y;
    bool clear = true;
    for (const auto& obs : obstacles) {
        if (line_intersects_rect(start_x, start_y, target_x, target_y, obs)) {
            clear = false;
            break;
        }
    }
    if (clear) {
        return goto_xy_heading(target_x, target_y, target_heading_deg, speed);
    }
    float min_dist = std::numeric_limits<float>::max();
    const ObstacleRect* closest = nullptr;
    for (const auto& obs : obstacles) {
        float cx = obs.x + obs.w/2, cy = obs.y + obs.h/2;
        float dist = std::hypot(cx - start_x, cy - start_y);
        if (dist < min_dist) { min_dist = dist; closest = &obs; }
    }
    if (!closest) return -1;
    float corner_x = closest->x - 1.0f, corner_y = closest->y + closest->h + 1.0f; // top-left + 1"
    goto_xy_heading(corner_x, corner_y, NAN, speed);
    return goto_xy_heading(target_x, target_y, target_heading_deg, speed);
}

// ---- Advanced (A*) Obstacle-Avoiding GOTO ----

struct Node {
    int x, y;
    float g, h;
    Node* parent;
    Node(int x, int y, float g, float h, Node* p=nullptr): x(x), y(y), g(g), h(h), parent(p) {}
    float f() const { return g + h; }
    bool operator<(const Node& o) const { return f() > o.f(); }
};

static bool cell_in_obstacle(int x, int y, float origin_x, float origin_y, float step) {
    float cx = origin_x + x*step;
    float cy = origin_y + y*step;
    for (const auto& obs : obstacles) {
        if (cx >= obs.x && cx <= obs.x+obs.w && cy >= obs.y && cy <= obs.y+obs.h)
            return true;
    }
    return false;
}

extern "C" int goto_xy_heading_avoid_advanced(float target_x, float target_y, float target_heading_deg, int speed) {
    // Parameters: grid size and resolution
    const float grid_step = 2.0f; // inches
    const int max_grid = 100; // 100x100 cells max
    // Get current position
    float sx = robot_x, sy = robot_y;
    // Build grid bounds
    float min_x = std::min(sx, target_x)-10, max_x = std::max(sx, target_x)+10;
    float min_y = std::min(sy, target_y)-10, max_y = std::max(sy, target_y)+10;
    int nx = std::min(max_grid, static_cast<int>((max_x-min_x)/grid_step)+1);
    int ny = std::min(max_grid, static_cast<int>((max_y-min_y)/grid_step)+1);
    // Map world to grid
    auto to_grid = [&](float wx, float wy, int& gx, int& gy) {
        gx = static_cast<int>((wx-min_x)/grid_step+0.5f);
        gy = static_cast<int>((wy-min_y)/grid_step+0.5f);
    };
    auto to_world = [&](int gx, int gy, float& wx, float& wy) {
        wx = min_x + gx*grid_step;
        wy = min_y + gy*grid_step;
    };
    // Start/end in grid
    int sxg, syg, txg, tyg;
    to_grid(sx, sy, sxg, syg);
    to_grid(target_x, target_y, txg, tyg);

    // A* Search
    std::priority_queue<Node> open;
    std::set<std::tuple<int,int>> closed;
    open.push(Node(sxg, syg, 0.0f, std::hypot(txg-sxg, tyg-syg), nullptr));
    Node* goal = nullptr;
    std::vector<Node*> all_nodes; // For cleanup

    while (!open.empty()) {
        Node* n = new Node(open.top());
        open.pop();
        if (closed.count({n->x, n->y})) { delete n; continue; }
        closed.insert({n->x, n->y});
        all_nodes.push_back(n);
        if (n->x == txg && n->y == tyg) { goal = n; break; }
        // neighbors: 8-connected
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
            if (dx==0 && dy==0) continue;
            int nxg=n->x+dx, nyg=n->y+dy;
            if (nxg<0||nyg<0||nxg>=nx||nyg>=ny) continue;
            if (closed.count({nxg,nyg})) continue;
            if (cell_in_obstacle(nxg, nyg, min_x, min_y, grid_step)) continue;
            float g = n->g + grid_step*std::hypot(dx,dy);
            float h = std::hypot(txg-nxg, tyg-nyg);
            open.push(Node(nxg, nyg, g, h, n));
        }
    }
    if (!goal) { for (auto* n : all_nodes) delete n; return -1; }
    // Extract path
    std::vector<std::pair<float,float>> path;
    for (Node* n = goal; n; n = n->parent) {
        float wx, wy; to_world(n->x, n->y, wx, wy);
        path.push_back({wx, wy});
    }
    std::reverse(path.begin(), path.end());
    // Follow path (skip first point if close)
    for (size_t i = 1; i < path.size(); ++i) {
        goto_xy_heading(path[i].first, path[i].second, NAN, speed);
    }
    int ret = goto_xy_heading(target_x, target_y, target_heading_deg, speed);
    for (auto* n : all_nodes) delete n;
    return ret;
}

// ---- Main Obstacle-Aware GOTO ----
extern "C" int goto_xy_heading_avoid(float target_x, float target_y, float target_heading_deg, int speed) {
    // By default, use advanced
    return goto_xy_heading_avoid_advanced(target_x, target_y, target_heading_deg, speed);
}