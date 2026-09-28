# SkateGuard — Documentation

Build documentation for SkateGuard, one folder per phase, plus the finished user manual.

![SkateGuard dashboard](_assets/dashboard.png)

| Folder | What is in it |
|---|---|
| [Phase 1 - Trick Detection](Phase%201%20-%20Trick%20Detection) | BNO055 motion state machine: lands, bails and speed wobble |
| [Phase 2 - GPS Speed & Route](Phase%202%20-%20GPS%20Speed%20%26%20Route) | NEO-8M speed, distance and route sampling, and the fix quality rules |
| [Phase 3 - Wearable Receiver](Phase%203%20-%20Wearable%20Receiver) | ESP-NOW link, channel hunting and the wrist OLED |
| [Phase 4 - Local Dashboard](Phase%204%20-%20Local%20Dashboard) | Hotspot join, stable address and the board's own web server |
| [Phase 5 - Sessions & Downloads](Phase%205%20-%20Sessions%20%26%20Downloads) | On-board session storage, maps and per-session downloads |
| [Final Report](Final%20Report) | The user manual as PDF, Word and a web page |

Each phase folder describes what was built, the parts and pins it uses, the constants that tune it, how it was tested and what is still open.

## The system in one paragraph

A Seeed XIAO ESP32-C3 under the deck reads a BNO055 IMU and a NEO-8M GPS. It counts kickflip lands and bails, measures speed and distance, and serves a dashboard from its own flash over the rider's phone hotspot. A second XIAO ESP32-C3 worn on the wrist receives speed and battery over ESP-NOW and shows the dashboard address so the rider never has to look it up. Sessions are stored on the board itself and can be downloaded from the phone.

![A real ride on the live map](_assets/live_map_real.jpg)
