## Motion Profiles: Acceleration and Deceleration

All robot motion (drive and turn) now uses smooth ramping for both acceleration and deceleration, following a trapezoidal velocity profile. This improves accuracy and reduces wheel slip/wear.

### Adjustable Parameters

You can adjust these at runtime before any drive/turn/move command:
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

### How Ramping Works

- **Acceleration:** Robot smoothly increases speed up to the requested cruise speed, limited by `*_MAX_ACCELERATION`.
- **Cruise:** Maintains max speed as possible.
- **Deceleration:** Slows smoothly to stop, limited by `*_MAX_DECELERATION`.
- **Minimum/Maximum Speed:** Ensures robot never goes slower/faster than limits.

You do not need to change your `drive` or `turn` calls—ramping is always active.

---