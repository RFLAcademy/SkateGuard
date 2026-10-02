# Phase 3 — Sensor and Embedded System (Week 3)

**Planned:** firmware reading live IMU and GPS data, the wobble algorithm, and kickflip detection from rotational signatures.

**Delivered:** a continuous sensor loop, kickflip detection as a four-stage state machine, and wobble detection on counted oscillations with no speed gate.

| Document | What it covers |
|---|---|
| [trick-detection.md](trick-detection.md) | The motion state machine, the land-versus-bail rule, the tuning constants and the stage log |
| [gps-speed-and-route.md](gps-speed-and-route.md) | Fix quality rules, the jitter and jump filters, route sampling and GPS diagnostics |

## Detection in one page

Detection is a four-stage state machine over the gyro axis a kickflip rotates around:

| Stage | Meaning |
|---|---|
| `IDLE` | Rotation rate near zero, waiting for a trick |
| `ROTATING` | Degrees accumulating; 2000 ms window, 4000 ms hard limit |
| `LAND_CONFIRM` | Watching how the deck settles, within 1000 ms and 83 degrees |
| `COOLDOWN` | 500 ms quiet period, counter updated |

A trick that reaches `LAND_CONFIRM` and settles inside the limit counts as a **land**. One that keeps swinging past 83 degrees, or runs out of time, counts as a **bail**. Speed wobble is separate: three counted oscillations are needed before it is reported, so one swerve does not trigger an alert.

| Constant | Value | Purpose |
|---|---|---|
| `MOTION_WINDOW_MS` | 2000 | How long the active rotation window stays open |
| `MOTION_ABSOLUTE_TIMEOUT_MS` | 4000 | Hard limit on any single trick |
| `MOTION_COOLDOWN_MS` | 500 | Quiet period after a counted trick |
| `LAND_CONFIRM_WINDOW_MS` | 1000 | Time allowed to confirm the landing |
| `LAND_CONFIRM_MAX_DEG` | 83.0 | Swing past which the landing is rejected as a bail |
| `WOBBLE_OSCILLATIONS_REQUIRED` | 3 | Oscillations before a wobble is called |

A GPS fix is trusted only when the position is valid, less than 3 seconds old and backed by at least 4 satellites. Speeds under 2 km/h report as zero so a parked board does not drift, and a position step implying more than 20 m/s is rejected as multipath.

## Still open

`WOBBLE_AXIS` and `KICKFLIP_AXIS` are an assumption about how the IMU sits on this mount. `stage_log.csv` now records gyro_x and gyro_z beside gyro_y, so one real kickflip shows which axis carries the rotation and settles it from data.
