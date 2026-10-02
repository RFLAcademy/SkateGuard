# Phase 6 — Testing, Tuning and Demonstration (Week 6)

**Planned:** full system testing across speeds and surfaces, tuning to cut false triggers, final assembly, demonstration and documentation.

**Delivered so far:** both units assembled and running, with bench, browser and first field testing complete. On-road tuning and the demonstration are the remaining work.

## What has been tested

| Area | Method | Result |
|---|---|---|
| Trick detection | `stage_log.csv`, a row every 500 ms holding stage, all three gyro axes, accumulated degrees, each window against its limit and the event that fired | Thresholds set to the Phase 3 values; log downloads cleanly and is capped at 2000 rows |
| Loop health | Slowest and average loop period recorded and shown on the debug page | Instrumented, because a stalled loop shows up as missed tricks rather than as a timing symptom |
| GPS | Status line every 5 seconds: satellites, HDOP, characters processed, fix age, checksum failures; first fix after boot timed | Fix acquired outdoors with 4 or more satellites; no fix indoors, as expected |
| Radio link | Power-up in both orders, walking out of range until `NO SIGNAL` and back, changing the hotspot so the channel moves | Wrist unit re-finds the board without a reboot in each case |
| Dashboard | Exercised in a browser against a simulated board: save and discard, the 7-session cap, deleting sessions, downloads with and without a route | All paths work; the route-alignment fault was reproduced and fixed with a no-GPS session saved between two GPS sessions |
| Field use | Real rides recorded, then opened as live map and session route maps | Routes render over OpenStreetMap tiles and match the ride |

Detection accuracy on real rides has not been quantified yet. That is the open item.

## What remains

1. Confirm the rotation axis from one real kickflip, comparing gyro_x, gyro_y and gyro_z in `stage_log.csv`, and confirm the I2C and ADC pins from the boot log.
2. Ride with a hand-counted tally of flips, score land and bail counts against it, and tune the thresholds to cut false triggers.
3. Trigger a real speed wobble at riding speed and confirm the wrist motor fires.
4. Decide whether the 25 km/h figure returns as a configurable alert level.
5. Measure battery life on both 1500 mAh units over a full session.
6. Record print material, layer height, infill and enclosure dimensions, then prepare the demonstration.
