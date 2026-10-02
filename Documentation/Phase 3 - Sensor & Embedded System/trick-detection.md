# Phase 1 — Trick Detection

Counting kickflips the rider actually lands, and telling them apart from the ones they bail.

## Hardware

| Part | Connection |
|---|---|
| Seeed XIAO ESP32-C3 | Board under the deck |
| Adafruit BNO055 IMU | I2C, address 0x28 |

The BNO055 runs in its fused mode, so the sketch reads Euler angles for tilt and the raw gyroscope vector for rotation rate. Startup blocks until `bno.begin()` succeeds, because a board that cannot see its IMU has nothing to report.

## How it works

Detection is a four-stage state machine over the gyro's Y axis, which is the axis a kickflip rotates around:

| Stage | Meaning |
|---|---|
| `IDLE` | Nothing happening. Rotation rate near zero. |
| `ROTATING` | Rotation passed the trigger rate; degrees are being accumulated. |
| `LAND_CONFIRM` | A full flip's worth of rotation was seen; the board now watches how the deck settles. |
| `COOLDOWN` | A trick was just counted; further motion is ignored briefly. |

A trick that reaches `LAND_CONFIRM` and settles within the allowed swing counts as a **land**. One that keeps swinging past the limit, or runs out of time, counts as a **bail**. Speed wobble is separate: it needs a run of oscillations before it is reported, so one swerve does not trigger it.

## Tuning constants

| Constant | Value | Purpose |
|---|---|---|
| `MOTION_WINDOW_MS` | 2000 | How long the active rotation window stays open |
| `MOTION_ABSOLUTE_TIMEOUT_MS` | 4000 | Hard limit on any single trick |
| `MOTION_COOLDOWN_MS` | 500 | Quiet period after a counted trick |
| `LAND_CONFIRM_WINDOW_MS` | 1000 | Time allowed to confirm the landing |
| `LAND_CONFIRM_MAX_DEG` | 83.0 | Swing past which the landing is rejected as a bail |
| `WOBBLE_OSCILLATIONS_REQUIRED` | 3 | Oscillations before a wobble is called |

## Testing

Two tools were built into the firmware for tuning:

- **A live debug page**, served by the board, showing the current stage, gyro rate, accumulated rotation, each window against its limit, and the counters. It refreshes twice a second, so a trick can be watched as it is classified.
- **`stage_log.csv`**, a row every 500 ms holding the same values plus the event that fired. It is capped at 2000 rows and cleared at every power-up, and can be downloaded from the dashboard for offline analysis.

The main loop is also instrumented: it records its slowest and average period. This matters because a stalled loop shows up as *missed tricks* rather than as anything obviously timing-shaped, so the loop figures are the first thing to check when counts look low.

## Open issues

- Tuned for kickflips. Other tricks rotate around different axes and are not classified.
- Very slow, deliberate flips can exceed the absolute window and be dropped.
- Counts run from power-up, not per session; a session stores the difference between its start and end.
