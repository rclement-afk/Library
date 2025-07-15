#include <stdio.h>
#include <math.h>
#include "robotlib_c_api.h"

int main() {
    timer_reset();
    init_robot_pose(0, 0, 90);

    // Go to (24, 36), end at 90 degrees
    goto_xy_heading(24.0, 36.0, 90.0, 50);
    print_robot_pose("After goto_xy_heading");

    // Go to (48, 48), keep last heading (use NAN)
    goto_xy_heading(48.0, 48.0, NAN, 50);
    print_robot_pose("After goto_xy_heading (same)");

    return 0;
}