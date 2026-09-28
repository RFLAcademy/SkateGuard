# Phase 4 — Local Dashboard

The board serving its own web app to the rider's phone, with no app store, no server and no internet.

## How it works

The board joins the rider's phone hotspot as a normal WiFi client. It takes a DHCP lease first, then moves itself to `<subnet>.200` on whatever subnet it landed on, and verifies the move before keeping it. An earlier build hard-coded a full static address, which worked on one phone and failed silently on any hotspot using a different subnet.

The address is therefore predictable without being fixed: on a phone handing out `192.168.43.x`, the dashboard is always at `192.168.43.200`. The board also registers `skateguard.local` over mDNS and announces a DHCP hostname, though phone support for both is inconsistent, which is why the wearable displays the numeric address.

The page itself is stored in flash as a raw string inside the sketch and served whole from `/`. `dashboard.html` in `Code/dashboard/` is the editable source; it is pasted between the `R"HTMLDOC(` markers before flashing, so the two never drift apart.

The browser polls `/api/live` twice a second and redraws speed, GPS state, wobble, trick status, the land/bail bar and the session card. Power save is disabled on the WiFi interface, since the default power-saving mode adds enough latency to make the readout feel sticky.

## Endpoints

| Endpoint | Purpose |
|---|---|
| `/` | The dashboard page |
| `/api/live` | Telemetry JSON, polled twice a second |
| `/api/debug` | Motion state machine, GPS, radio and memory values |
| `/api/sessions` | Saved sessions and their routes |
| `/api/command?cmd=` | `start`, `stop`, `save`, `discard`, `delete`, `delete:<n>` |
| `/log`, `/clear` | Download or clear the motion log |
| `/manifest.json` | Web app manifest for "Add to Home screen" |

The telemetry JSON is field-for-field identical to the one the earlier Bluetooth build sent to the Android app, so both front ends read exactly the same data.

## Installing on the phone

The manifest makes the dashboard installable: "Add to Home screen" gives a full-screen icon with no address bar, so after a one-time setup there is no address to type again. The icon is an inline SVG data URI rather than a separate image route, so no binary asset has to live in flash.

## Testing

- Joining hotspots on different subnets, confirming the board lands on `.200` each time.
- Killing the hotspot mid-ride: the page shows "Connection lost — retrying…" and recovers on its own when the network returns.
- Loading the dashboard from a second phone on the same hotspot, which sees the same data and the same sessions.

## Open issues

- No login. Anyone on the hotspot can open the dashboard and delete sessions; the hotspot password is the only barrier.
- `skateguard.local` resolves on some phones and not others.
- Map tiles come from OpenStreetMap, so the map screens need the phone to have internet even though recording does not.
- The sketch uses 91% of the default 1.25 MB program partition, so there is little room left for new features.
