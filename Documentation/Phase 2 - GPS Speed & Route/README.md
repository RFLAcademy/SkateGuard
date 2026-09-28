# Phase 2 — GPS Speed & Route

Turning a NEO-8M into trustworthy speed, distance and a route worth drawing.

## Hardware

| Part | Connection |
|---|---|
| NEO-8M GPS module | UART1 — RX GPIO20, TX GPIO21, 9600 baud |

The module is left in its default configuration. No UBX setup commands are sent, because a malformed command to a clone or a different M8 variant can leave the receiver in a state the sketch cannot recover from. NMEA sentences are parsed with TinyGPS++.

## How it works

The UART is drained twice per loop, at the start and again near the end, with a 512-character ceiling per pass. WiFi, the filesystem, ESP-NOW and the web server all take time while the GPS keeps transmitting in the background, so a single drain per loop lets the buffer overflow and sentences arrive broken.

A fix is only trusted when three things hold at once:

- the position is valid,
- it is less than **3 seconds** old, and
- at least **4 satellites** are in use.

The age check matters because TinyGPS++ keeps reporting the last known position after signal loss. Without it, a board that just lost the sky keeps claiming the same location.

Distance accumulates between accepted positions, with two filters. Speeds below **2 km/h** report as zero, so a parked board does not drift. A step is rejected when it implies more than **20 m/s**, scaled by the real time elapsed since the last accepted position, which kills the jumps that multipath produces near buildings.

## Tuning constants

| Constant | Value | Purpose |
|---|---|---|
| `GPS_FIX_MAX_AGE_MS` | 3000 | Age past which a fix counts as stale |
| `GPS_MAX_PLAUSIBLE_SPEED_MS` | 20.0 | Speed used to reject impossible jumps |
| `JITTER_REJECT_THRESHOLD_KMH` | 2.0 | Below this, speed reads 0.0 |
| `SESSION_ROUTE_SAMPLE_INTERVAL_MS` | 30000 | Gap between stored route points |
| `SESSION_MAX_ROUTE_POINTS` | 20 | Points kept per session |

## Testing

The board prints a GPS status line every 5 seconds over its network log: satellites, HDOP, characters processed, fix state, seconds since the last byte, fix age, failed checksums and uptime. The first fix after boot is logged with how long it took. The same values appear on the dashboard's debug page, which is what to read when a fix will not come:

- **GPS Data In** climbing but no fix — the module is alive, the sky is not good enough.
- **GPS Data In** stuck at zero — wiring or baud, not reception.
- **Checksum fails** climbing — noise on the line or a bad connection.

## Open issues

- A cold start after days unpowered can take several minutes.
- Route points every 30 seconds keep storage small but cut corners on fast laps.
- Indoors, satellites usually stop at two or three, so the board never reports a fix.
