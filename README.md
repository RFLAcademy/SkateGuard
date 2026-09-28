# SkateGuard

A skateboard trick counter and ride tracker built on two Seeed XIAO ESP32-C3 boards. The board under the deck detects kickflip lands and bails with a BNO055 IMU, tracks speed, distance and route with a NEO-8M GPS, and serves its own dashboard to the rider's phone over a hotspot — no app, no internet, no account. A wrist-worn receiver mirrors live speed over ESP-NOW and shows the dashboard address so the rider never has to look it up.

## ✨ Features

- **Kickflip land vs bail detection** — a four-stage motion state machine (IDLE → ROTATING → LAND_CONFIRM → COOLDOWN) over BNO055 gyro data, so a rotation that ends upright counts as a land and one that does not counts as a bail
- **Speed wobble detection** — three counted oscillations before a wobble is reported, so one swerve does not trigger it
- **GPS speed, distance and route** — TinyGPS++ with a fix-age check, a four-satellite minimum and a jump filter, so a stale or weak fix cannot inflate distance
- **Self-hosted dashboard** — the board runs its own web server and serves the whole page from flash
- **Stable address** — the board takes a DHCP lease, then pins itself to `<subnet>.200` and answers to `skateguard.local`
- **Session recording** — start, stop, then save or discard; the last 7 sessions live on the board
- **Per-session download** — the ride's numbers and route points as CSV, plus a rendered route map picture
- **Live map** — OpenStreetMap tiles, auto-follow that pauses while you pan, and an overlay of speed, distance, average speed and trick counts
- **Debug page** — motion state machine, loop timing, GPS diagnostics, radio and memory, refreshed twice a second
- **Wearable receiver** — speed, battery, link state and the dashboard address on a 128×32 OLED, with ESP-NOW channel hunting
- **Installable** — a web app manifest, so "Add to Home screen" gives a full-screen icon like a native app

## 🧩 Hardware

| Component | Interface |
|---|---|
| Seeed XIAO ESP32-C3 (board) | — |
| BNO055 IMU | I2C, address 0x28 |
| NEO-8M GPS | UART1 — RX GPIO20, TX GPIO21, 9600 baud |
| Battery sense (board) | ADC through a resistor divider |
| Seeed XIAO ESP32-C3 (wearable) | — |
| SSD1306 128×32 OLED | I2C — SDA GPIO6 (D4), SCL GPIO7 (D5) |
| Haptic / motor | GPIO3 |
| Status LED | GPIO8 |
| Battery sense (wearable) | ADC GPIO2 (D0), divider R1 = 30k / R2 = 10k |
| Board ↔ wearable link | ESP-NOW broadcast, channel follows the hotspot |

## 📋 Requirements

- Arduino IDE with the **ESP32 core 3.2.0**, board set to **XIAO_ESP32C3**
- Libraries: `Adafruit BNO055`, `Adafruit Unified Sensor`, `Adafruit BusIO`, `TinyGPSPlus`, `U8g2`
- A phone hotspot. Its name and password go in the `WIFI_NETWORKS` table at the top of the board sketch; three entries are supported and tried in order.

## 🚀 Usage

1. Flash `Code/skateguard_local_sender_2/skateguard_local_sender_2.ino` to the board.
2. Flash `Code/skateguard_local_receiver/skateguard_local_receiver.ino` to the wearable.
3. Turn on the phone hotspot. The board joins it and moves itself to `<subnet>.200`.
4. Read the address off the wearable screen and open it in the phone browser.

The board sketch uses 91% of the default 1.25 MB program partition and 12% of RAM. Keep the same **Partition Scheme** between flashes, or stored sessions are erased.

There is no login. Anyone on the hotspot can open the dashboard, record and delete sessions.

## 🌐 Board Endpoints

| Endpoint | Action |
|---|---|
| `/` | The dashboard page |
| `/api/live` | Telemetry JSON, polled twice a second |
| `/api/debug` | Motion state machine, GPS, radio and memory values |
| `/api/sessions` | Saved sessions and their routes |
| `/api/command?cmd=start` | Begin recording a session |
| `/api/command?cmd=stop` | Stop recording, pending save or discard |
| `/api/command?cmd=save` | Write the stopped session to the board |
| `/api/command?cmd=discard` | Throw the stopped session away |
| `/api/command?cmd=delete:<n>` | Delete saved session n |
| `/log`, `/clear` | Download or clear the motion log |
| `/manifest.json` | Web app manifest for "Add to Home screen" |

## 🎬 Recording a Session

Tricks are counted from power-up, but only a recorded session is stored with its distance, speed and route.

| Step | What happens |
|---|---|
| 1 | Tap **Start**. Start dims, Stop stays bright, the card reads "Recording…" |
| 2 | Ride. Distance and session speed climb; a route point is stored every 30 seconds |
| 3 | Tap **Stop**. Nothing is written yet |
| 4 | Choose **Save** to keep the ride, or **Discard** to drop it |

A saved session appears in Recorded Sessions, where each row can be opened as a map, downloaded, or deleted.

## ⚙️ Configuration

Key constants in `Code/skateguard_local_sender_2/skateguard_local_sender_2.ino`:

| Constant | Default | Description |
|---|---|---|
| `WIFI_NETWORKS` | 1 entry | Hotspot name and password list, tried in order |
| `MOTION_WINDOW_MS` | 2000 | How long the active trick window stays open |
| `MOTION_ABSOLUTE_TIMEOUT_MS` | 4000 | Hard limit on a single trick |
| `MOTION_COOLDOWN_MS` | 500 | Pause after a trick before the next is counted |
| `LAND_CONFIRM_WINDOW_MS` | 1000 | Time allowed to confirm a landing |
| `LAND_CONFIRM_MAX_DEG` | 83.0 | Swing past which a landing is rejected as a bail |
| `WOBBLE_OSCILLATIONS_REQUIRED` | 3 | Oscillations before a wobble is reported |
| `MAX_SAVED_SESSIONS` | 7 | Sessions kept before the oldest is dropped |
| `SESSION_MAX_ROUTE_POINTS` | 20 | Route points stored per session |
| `SESSION_ROUTE_SAMPLE_INTERVAL_MS` | 30000 | Gap between route points |
| `MAX_STAGE_LOG_ROWS` | 2000 | Rows kept in the motion log |
| `GPS_FIX_MAX_AGE_MS` | 3000 | Age past which a fix counts as stale |
| `GPS_MAX_PLAUSIBLE_SPEED_MS` | 20.0 | Speed used to reject impossible position jumps |
| `JITTER_REJECT_THRESHOLD_KMH` | 2.0 | Speed below which the dashboard reads 0.0 |

A fix also requires 4 or more satellites before speed, distance or maps are used.

## 📊 Session Data

Saved sessions are appended to `Data/sessions.csv` on the board:

```
timestamp_ms,avg_speed_kmh,distance_km,land_count,bail_count
842113,11.84,0.874,7,2
1904552,0.00,0.000,14,5
3120870,13.20,1.215,9,3
```

`Data/routes.csv` holds one line per session, as `latitude:longitude` pairs separated by semicolons. A session recorded without GPS writes a single `-`, so line N always belongs to session N:

```
19.207431:72.837215;19.207890:72.837902;19.208344:72.838511
-
19.209120:72.839550;19.209744:72.840122;19.210301:72.840688
```

The files in `Data/` are examples of the format, not a recorded ride.

## 🗂️ Project Structure

```
├── Code/
│   ├── skateguard_local_sender_2/   # Board firmware: IMU, GPS, web server, ESP-NOW, session storage
│   ├── skateguard_local_receiver/   # Wearable firmware: OLED, ESP-NOW receiver, channel hunting
│   └── dashboard/
│       └── dashboard.html           # Dashboard source, embedded into the firmware as a raw string
├── Data/
│   ├── sessions.csv                 # Session list format
│   └── routes.csv                   # Route file format, one line per session
├── Documentation/
│   ├── Phase 1 - Trick Detection/
│   ├── Phase 2 - GPS Speed & Route/
│   ├── Phase 3 - Wearable Receiver/
│   ├── Phase 4 - Local Dashboard/
│   ├── Phase 5 - Sessions & Downloads/
│   ├── Final Report/                # User manual as PDF, Word and a web page
│   └── _assets/
└── README.md
```

## 🔭 Roadmap

| Phase | Status | Description |
|---|---|---|
| Phase 1 | ✅ Complete | BNO055 trick detection: land, bail and wobble |
| Phase 2 | ✅ Complete | NEO-8M GPS: speed, distance and route sampling |
| Phase 3 | ✅ Complete | ESP-NOW wearable receiver with channel hunting |
| Phase 4 | ✅ Complete | Self-hosted dashboard over the phone hotspot |
| Phase 5 | ✅ Complete | Session recording, storage and per-session download |
| Phase 6 | 🔧 In progress | Field testing and trick-detection tuning on real rides |

## ⚠️ Known Limitations / Open Issues

- The dashboard has no login. Anyone on the hotspot can open it, record and delete sessions; the hotspot password is the only barrier.
- A GPS cold start can take several minutes, and nothing is reported until 4 satellites are visible.
- Map tiles come from OpenStreetMap, so the phone needs internet for the map screens. Recording works without it.
- Program space is 91% full on the default partition. Larger features need a different partition scheme, which erases stored sessions.
- Only the last 7 sessions are kept, and the route is sampled every 30 seconds, so the drawn line cuts corners.
- Deleting a session from the list happens immediately, with no confirmation.
- `skateguard.local` resolves on some phones and not others, which is why the wearable shows the numeric address.
- Detection is tuned for kickflips. Other tricks may not be counted correctly.
- The wearable must hunt for the hotspot's channel at power-up, so the first link can take a few seconds.

## 📄 License

This project is for educational and prototype purposes.
