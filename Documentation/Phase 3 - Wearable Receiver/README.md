# Phase 3 — Wearable Receiver

A wrist unit that shows speed while riding, and the dashboard address when stopped.

## Hardware

| Part | Connection |
|---|---|
| Seeed XIAO ESP32-C3 | Worn on the wrist |
| SSD1306 128×32 OLED | I2C — SDA GPIO6 (D4), SCL GPIO7 (D5), driven by U8g2 |
| Status LED | GPIO8 |
| Haptic / motor | GPIO3 |
| Battery sense | ADC GPIO2 (D0), divider R1 = 30k / R2 = 10k, 11 dB attenuation |

Battery voltage is averaged over several ADC reads, because a single read on this chip is noisy enough to swing the reported percentage.

## How it works

The board broadcasts telemetry over **ESP-NOW**, which runs alongside its hotspot connection and needs no pairing. The peer is registered on channel 0 so it follows whatever channel the phone's hotspot picks.

The receiver does not know that channel, so at power-up it **hunts**: it parks on each channel for 300 ms until packets arrive. Once a packet lands, the receiver also reads the channel the board states inside the packet and retunes to it. That second step exists because adjacent-channel leakage let the receiver lock onto a neighbouring channel, where packets arrived just often enough to look correct and then stopped.

The packet carries speed, the board's battery, the board's IP address and its channel. The screen shows:

| State | Display |
|---|---|
| Hunting | `SCAN ch<n>` |
| Linked, stopped, first 20 s | `Dashboard:` and the address, with a countdown |
| Linked, moving | Live speed in large digits |
| Linked, stopped after the window | Battery icon |
| Link lost for 5 s | `NO SIGNAL`, then hunting again |

The address window is gated on being stopped as well as on the timer, so rolling away inside the window hands the screen straight back to speed.

## Constants

| Constant | Value | Purpose |
|---|---|---|
| `CHANNEL_DWELL_MS` | 300 | Time parked on each channel while hunting |
| `LINK_LOST_MS` | 5000 | Silence before the link counts as lost |
| `IP_DISPLAY_MS` | 20000 | How long the address stays on screen |

## Testing

Serial printing was moved out of the ESP-NOW receive callback and into a drained queue. Printing inside the callback blocks the WiFi task long enough to drop the next packets, which made the link look unreliable while the radio was fine.

Field checks: power the board first and the wearable second, then the other way round; walk out of range until `NO SIGNAL` appears and back until the address returns; change the phone hotspot so the channel moves, and confirm the wearable finds the board again without a reboot.

## Open issues

- First link after power-up takes a few seconds while channels are swept.
- The wearable reports its own battery back to the board, so a wearable that never powers on leaves that indicator greyed out.
