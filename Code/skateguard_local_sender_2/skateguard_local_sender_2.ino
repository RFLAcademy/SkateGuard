/*
  SkateGuard — LOCAL SENDER (Seeed XIAO ESP32-C3)
  -----------------------------------------------------------------------
  Replaces skateguard_firebase_sender.ino — same sensors, same
  trick-detection FSM (BNO055 fusion, gyro clamp + 5-sample median
  filter, the wobble/land/bail state machine with the oscillation
  re-baseline fix), but Firebase is gone entirely:

    OLD (skateguard_firebase_sender.ino): board joins the phone hotspot,
    pushes data out to Firebase over the internet, phone loads a
    Firebase-hosted webapp that polls Firebase. Every telemetry update
    and every button press did a DOUBLE internet round-trip even though
    phone and board are inches apart on the same hotspot — that's what
    caused the latency.

    NEW (this file): board still joins the phone's hotspot (for GPS-less
    testing this also means it has internet if the hotspot provides it,
    but nothing here actually needs internet). The board now runs its
    OWN local HTTP server and serves the dashboard page directly — same
    UI as before (dashboard.html in this folder is the source; embedded
    below as DASHBOARD_HTML). The phone's browser talks to the board
    over ONE local hop on the shared hotspot network, no internet, no
    Firebase, no round-trip through anywhere else.

  WEARABLE (ESP-NOW) — SUPPORTED, but pair it with the matching receiver:
    skateguard_local_receiver.ino, NOT reciver_skateboard_xiaoc3.ino.
    The old receiver hardcodes ESPNOW_CHANNEL 11 to match the BLE build's
    softAP; this board has no softAP and sits on whatever channel the
    phone hotspot picked, so a hardcoded 11 will simply never hear us.
    The new receiver channel-hops until it finds us and then locks on.
    See the ESP-NOW section below for the full reasoning.

  BLE IS STILL DROPPED HERE:
    The phone talks to this board over plain HTTP on the hotspot instead,
    so the BLE GATT server would be dead weight (and more radio load on a
    single 2.4GHz radio already running STA + ESP-NOW). Use
    skateboard_xiao_c3_ble_stable.ino + SkateGuardBLEApp if you want the
    native-app/BLE path — that pair is untouched and still works.

  SETUP — fill in the WIFI_NETWORKS table below (one row per phone you
  want to ride with), then flash. That's it — no Firebase project, no API
  keys, no webapp to deploy. Adding a second phone doesn't need a
  reflash if you've already got a spare row filled in.

  FINDING THE DASHBOARD — the board takes a DHCP lease, then re-addresses
  itself to <that subnet>.200 and prints the result:
    [WIFI] DHCP lease 192.168.43.7
    [WIFI] Pinned to 192.168.43.200
    [WIFI] Dashboard at http://192.168.43.200/
  So the address always ends in .200 and is stable across reboots, on any
  hotspot — only the subnet prefix changes if you swap phones. Open that
  URL from any device on the same hotspot.

  http://skateguard.local/ also works on iOS/macOS/Windows, but NOT in
  most Android browsers (no mDNS resolver), which is why the numeric
  address is the one worth bookmarking on a phone.

  If anything about the re-addressing doesn't apply (no gateway offered,
  a non-/24 subnet, the interface refusing the change) the board just
  keeps its DHCP lease and prints that instead — it never ends up
  unreachable, it just might not end in .200. Read the Serial line rather
  than assuming.

  LOCAL API served by this board (same-origin fetches from the page):
    GET  /              -> the dashboard page (DASHBOARD_HTML)
    GET  /api/live       -> JSON telemetry, same shape the dashboard reads
    GET  /api/command?cmd=start|stop|save|discard|delete|delete:<n>
                          -> executes immediately, no polling/dedup needed
                             now that this is a direct request/response
                             instead of eventually-consistent Firebase writes
    GET  /api/sessions    -> { sessionsCsv, routesCsv } from local LittleFS
    GET  /log             -> stage_log.csv download
    GET  /clear           -> wipes stage_log.csv

  webServer.handleClient() is called unconditionally every loop() cycle,
  NOT gated on MOTION_IDLE like the Firebase version's uploads were. That
  gating existed because HTTPS calls block for 100ms-1s+ over the
  internet; handling an already-open local socket and building a small
  JSON string from RAM is on the order of microseconds, so it can't
  distort trick timing the way a stalled HTTPS POST could.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <LittleFS.h>

// ---------------------------------------------------------------------
// >>> FILL THIS IN BEFORE FLASHING <<<
//
// Every hotspot you want this board to work with. It tries them one per
// retry cycle until something connects, so you can have several phones
// set up at once and just turn on whichever one you're riding with —
// no reflashing to switch between them.
//
// Leave unused slots as they are; empty entries are skipped.
// ---------------------------------------------------------------------
struct WifiCredential {
  const char* ssid;
  const char* password;
};

WifiCredential WIFI_NETWORKS[] = {
  { "12345678",  "12345678" },   // primary phone
  { "",     ""         },   // spare — second phone
  { "",     ""         },   // spare — third phone / home router
};

const uint8_t WIFI_NETWORK_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Which entry the next connection attempt uses. Advances on every failed
// retry so a dead first entry can't block the rest.
uint8_t wifiNetworkIndex = 0;

// Stable last octet for the dashboard URL. The board takes a normal DHCP
// lease first, then moves itself to <whatever subnet it landed on>.200 —
// so the address ends in .200 on EVERY hotspot, not just the one this was
// written against.
//
// Why not just hardcode the whole address: a fixed 10.236.16.200 breaks
// silently on any other phone. The board still associates fine (that's
// layer 2, unrelated to IP), still reports WL_CONNECTED, and still prints
// a confident "Connected, IP 10.236.16.200" — but it's on a subnet nobody
// else is on, so the dashboard is simply unreachable with no error to
// explain it. Learning the subnet at runtime removes that failure mode
// entirely.
//
// Pick something above the DHCP pool. Phone hotspots hand out from the
// low end (.2 upward) and rarely serve more than a handful of clients,
// so .200 is clear in practice. If you ever do collide, change this.
const uint8_t STATIC_HOST_OCTET = 200;
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Dashboard page — same UI as the Firebase version, embedded verbatim.
// Source of truth is dashboard.html in this folder (edit that, then
// re-paste it here). ESP32 maps flash into the normal address space,
// so a plain const char[] here already lives in flash without needing
// PROGMEM (unlike AVR boards).
// ---------------------------------------------------------------------
const char DASHBOARD_HTML[] = R"HTMLDOC(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>SkateGuard</title>

<!-- "Add to Home screen" support. With these, the board's dashboard
     installs as a standalone icon: one tap opens it full-screen, no URL
     bar, no address to remember on that phone again. The manifest is
     served by the board itself (see handleManifest() in the .ino); the
     icon here is the same inline SVG so the tab/bookmark matches. -->
<link rel="manifest" href="/manifest.json">
<meta name="theme-color" content="#0D0D0F">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="SkateGuard">
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 192 192'><rect width='192' height='192' rx='42' fill='%230D0D0F'/><ellipse cx='60' cy='143' rx='15' ry='11' fill='%232ECC71'/><ellipse cx='132' cy='143' rx='15' ry='11' fill='%232ECC71'/><path d='M26 104q70-24 140 0-70 27-140 0z' fill='%232ECC71'/></svg>">
<link rel="apple-touch-icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 192 192'><rect width='192' height='192' rx='42' fill='%230D0D0F'/><ellipse cx='60' cy='143' rx='15' ry='11' fill='%232ECC71'/><ellipse cx='132' cy='143' rx='15' ry='11' fill='%232ECC71'/><path d='M26 104q70-24 140 0-70 27-140 0z' fill='%232ECC71'/></svg>">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="" />
<style>
  /* Recreation of SkateGuardBLEApp's native Android UI as a web page,
     served DIRECTLY by the ESP32 (see skateguard_local_sender.ino) —
     no Firebase, no internet round-trip for telemetry/commands. The
     phone and board are both on the same phone hotspot, so every
     fetch() below is a single local hop (relative URLs — same origin
     as whatever served this page, i.e. the board itself). Only the
     Leaflet library and OpenStreetMap map tiles come from the internet
     (fetched by the phone's own connection, not routed through the
     board) — everything else here works with the board reachable but
     no internet at all. */
  :root {
    --bg: #0D0D0F;
    --card: #1B1D21;
    --card-border: #2A2D33;
    --text: #F5F5F7;
    --text-dim: #9AA0AA;
    --text-faint: #5A5F68;
    --green: #2ECC71;
    --red: #E53935;
    --amber: #FFC107;
    --blue: #3B82F6;
    --orange: #FFA726;
    --locator-blue: #4285F4;
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body { height: 100%; }
  body {
    margin: 0;
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    overflow: hidden;
  }
  button { font-family: inherit; cursor: pointer; border: none; }
  button:focus-visible { outline: 2px solid var(--blue); outline-offset: 2px; }

  #app { position: fixed; inset: 0; }

  /* ---------- home screen ---------- */
  #homeLayout {
    position: absolute; inset: 0;
    display: flex; flex-direction: column; align-items: center; justify-content: center;
    padding: 26px 32px;
    gap: 0;
  }
  #homeLayout .title { font-size: 22px; font-weight: 700; margin-bottom: 24px; }
  .btn-pill {
    padding: 11px 22px;
    border-radius: 24px;
    font-size: 14px;
    font-weight: 600;
    color: #06210f;
  }
  .btn-green { background: var(--green); color: #06210f; }
  .btn-red { background: var(--red); color: #fff; }
  .btn-blue { background: var(--blue); color: #fff; }
  .btn-grey { background: var(--text-faint); color: #fff; }
  /* Applies to EVERY button, not just .btn-pill. The session Start/Stop/
     Save/Discard buttons only carry .btn-green/.btn-red/etc, so a
     .btn-pill-scoped rule left them looking fully active while disabled —
     you couldn't tell whether a press had registered. Android greys out
     disabled buttons automatically, which is what the BLE app relies on;
     the web needs it spelled out. Desaturating as well as dimming makes
     "off" unmistakable against the bright green/red fills. */
  button:disabled {
    opacity: .35;
    filter: grayscale(70%);
    cursor: not-allowed;
  }
  #statusText { margin-top: 16px; color: var(--text-dim); font-size: 11px; text-align: center; max-width: 280px; }

  /* ---------- dashboard ---------- */
  #dashboardScroll {
    position: absolute; inset: 0;
    overflow-y: auto;
    -webkit-overflow-scrolling: touch;
    display: none;
  }
  #dashboardInner {
    max-width: 480px;
    margin: 0 auto;
    padding: 13px 16px;
  }

  .dash-head {
    position: relative;
    height: 77px;
    margin-top: 16px;
    margin-bottom: 12px;
    display: flex;
    align-items: center;
  }
  .dash-title {
    position: absolute; left: 0; right: 0; text-align: center;
    font-size: 28px; font-weight: 700;
  }
  .batteries { display: flex; align-items: center; gap: 6px; z-index: 1; }
  .batt {
    width: 27px; height: 39px;
    position: relative;
    display: flex; align-items: center; justify-content: center;
  }
  .batt-body {
    width: 27px; height: 19px;
    border: 1.5px solid var(--text);
    border-radius: 4px;
    background: #1B1D21;
    position: relative;
    overflow: hidden;
    display: flex;
    flex-direction: column-reverse;
  }
  .batt-body::before {
    content: "";
    position: absolute;
    top: -5px; left: 50%; transform: translateX(-50%);
    width: 9px; height: 3px;
    background: var(--text);
    border-radius: 2px;
  }
  .batt-fill {
    width: 100%;
    height: 0%;
    background: var(--green);
    transition: height .3s ease, background .3s ease;
  }
  .batt-icon {
    position: absolute;
    font-size: 12px;
    opacity: .9;
    pointer-events: none;
    filter: drop-shadow(0 0 1px rgba(0,0,0,.6));
  }
  .batt.dim { opacity: .35; }
  .map-toggle-btn {
    position: absolute; right: 0; top: 50%; transform: translateY(-50%);
    width: 40px; height: 40px;
    display: flex; align-items: center; justify-content: center;
    background: transparent; border-radius: 50%;
    color: var(--text); font-size: 19px;
    z-index: 1;
  }
  .map-toggle-btn:active { background: rgba(255,255,255,.08); }

  .metrics-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 7px;
    margin-top: 20px;
  }
  .metric-cell {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 18px;
    padding: 20px;
    min-height: 96px;
    display: flex; flex-direction: column; align-items: center; justify-content: center;
    text-align: center;
  }
  .metric-value { font-size: 24px; font-weight: 700; margin-top: 6px; }
  .metric-label { font-size: 11px; color: var(--text-dim); letter-spacing: .02em; margin-top: 0; }

  .card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 18px;
    margin: 4px 6px 5px;
    padding: 13px 16px;
  }

  .compare-row { display: flex; align-items: center; }
  .compare-col { flex: 1; display: flex; flex-direction: column; align-items: center; gap: 4px; }
  .compare-count { font-size: 16px; font-weight: 700; }
  .compare-track {
    width: 150px; height: 8px;
    margin: 0 20px 6px;
    border-radius: 5px;
    background: #2A2D33;
    display: flex;
    overflow: hidden;
    flex: none;
    align-self: flex-end;
  }
  .compare-land { background: var(--green); }
  .compare-bail { background: var(--red); }

  .card-head { position: relative; display: flex; justify-content: center; align-items: center; min-height: 20px; }
  .view-recordings-btn {
    position: absolute; right: 0; top: 50%; transform: translateY(-50%);
    width: 28px; height: 28px;
    display: flex; align-items: center; justify-content: center;
    background: transparent; border-radius: 50%;
    color: var(--text); font-size: 15px;
  }
  .view-recordings-btn:active { background: rgba(255,255,255,.08); }

  .session-metrics { display: flex; align-items: center; margin-top: 10px; }
  .session-metric { flex: 1; display: flex; flex-direction: column; align-items: center; }
  .session-metric .metric-value { font-size: 18px; }

  #sessionStatusText { text-align: center; margin-top: 8px; color: var(--text-faint); font-size: 10px; }

  .btn-row { display: flex; gap: 8px; width: 273px; max-width: 100%; margin: 12px auto 0; }
  .btn-row button { flex: 1; padding: 10px; border-radius: 24px; font-size: 14px; font-weight: 600; }
  #saveDiscardRow { display: none; }

  #connectionStatusText { text-align: center; margin-top: 16px; margin-bottom: 20px; color: var(--text-faint); font-size: 10px; }

  /* ---------- disconnected overlay ---------- */
  #disconnectedOverlay {
    position: absolute; inset: 0;
    background: rgba(0,0,0,.7);
    display: none;
    align-items: center; justify-content: center;
    z-index: 20;
  }
  .spinner {
    width: 28px; height: 28px;
    border: 3px solid rgba(245,245,247,.25);
    border-top-color: var(--text);
    border-radius: 50%;
    animation: spin 0.8s linear infinite;
    margin: 0 auto 12px;
  }
  @keyframes spin { to { transform: rotate(360deg); } }
  @media (prefers-reduced-motion: reduce) { .spinner { animation-duration: 2s; } }
  #disconnectedOverlay .msg { text-align: center; font-size: 15px; font-weight: 700; padding: 0 24px; }

  /* ---------- generic dialog shell ---------- */
  .dialog-backdrop {
    position: fixed; inset: 0;
    background: rgba(0,0,0,.7);
    display: none;
    align-items: center; justify-content: center;
    z-index: 30;
    padding: 20px;
  }
  .dialog-backdrop.open { display: flex; }
  .dialog-box {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 18px;
    overflow: hidden;
    width: 100%;
  }

  /* map dialog */
  #mapDialogBox { max-width: 520px; height: 75vh; position: relative; }
  #routeMapDiv { width: 100%; height: 100%; }
  /* Anything layered over the map needs an explicit z-index. Leaflet gives
     its own panes z-index 200-700 (tiles 200, overlays 400, markers 600),
     and a positioned sibling left at z-index:auto paints UNDERNEATH all of
     them — so the close button and these badges were being covered by the
     map tiles the moment tiles actually loaded. Above Leaflet's controls
     (1000) so nothing the library adds later can bury them either. */
  .map-overlay-badge {
    position: absolute;
    z-index: 1200;
    /* Also needs real offsets: with position:absolute and no top/left it
       sits at its static position, which is after the full-height map div
       — i.e. below the box, clipped away by overflow:hidden. Top-left
       matches mapSessionStatsText's layout_gravity in the Android app. */
    top: 8px;
    left: 8px;
    background: rgba(0,0,0,.6);
    padding: 10px 12px;
    border-radius: 10px;
    /* Large enough to read at a glance on a phone while riding. */
    font-size: 16px;
    font-weight: 600;
    color: var(--text);
    line-height: 1.45;
    white-space: pre-line;
  }
  /* The app centres this one (layout_gravity="center") — it's the "waiting
     for a fix" message, not a stats corner. */
  #mapStatusText,
  #sessionRouteStatusText {
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    /* Font size, weight and padding come from .map-overlay-badge, so these
       messages look the same as the live map's stats overlay. */
    text-align: center;
    /* With left:50% an absolute box may only shrink-to-fit into the right
       half, so longer messages would wrap early; size to the text instead,
       capped to the dialog. */
    width: max-content;
    max-width: 85%;
  }
  /* Centre over the map area, not the info panel below it (84px). */
  #sessionRouteStatusText { top: calc((100% - 84px) / 2); }
  #mapSessionStatsText { top: 8px; left: 8px; }
  .dialog-close-btn {
    position: absolute; top: 8px; right: 8px;
    /* See .map-overlay-badge — without this the map tiles paint over it and
       the close button is both invisible and unclickable. */
    z-index: 1200;
    width: 40px; height: 40px;
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 10px;
    display: flex; align-items: center; justify-content: center;
    color: var(--text); font-size: 18px;
  }

  /* sessions dialog */
  #sessionsDialogBox { max-width: 480px; max-height: 80vh; display: flex; flex-direction: column; }

  /* ---- Debug dialog ---- */
  #debugDialogBox { max-width: 480px; max-height: 80vh; display: flex; flex-direction: column; }
  #debugBody { overflow-y: auto; padding: 4px 16px 16px; }
  .debug-table { width: 100%; border-collapse: collapse; font-size: 12px; }
  .debug-table td { padding: 5px 4px; border-bottom: 1px solid var(--card-border); }
  .debug-table td:last-child { border-bottom: 1px solid var(--card-border); }
  .debug-table tr:last-child td { border-bottom: none; }
  .debug-table td:first-child { color: var(--text-dim); }
  /* Values are monospace + tabular so rapidly-changing numbers (gyro,
     accumulated rotation) don't make the column jitter while you read it. */
  .debug-table td:last-child {
    text-align: right;
    font-family: ui-monospace, Menlo, Consolas, monospace;
    font-variant-numeric: tabular-nums;
    color: var(--text);
  }
  .debug-links { display: flex; gap: 16px; justify-content: center; margin-top: 14px; }
  .debug-links a { color: var(--blue); font-size: 12px; text-decoration: none; }
  .debug-links a:hover { text-decoration: underline; }
  .sessions-head {
    position: relative; display: flex; align-items: center; justify-content: center;
    padding: 8px; min-height: 36px;
    font-size: 13px; font-weight: 700;
  }
  .icon-btn-corner {
    position: absolute; top: 50%; transform: translateY(-50%);
    width: 36px; height: 36px;
    display: flex; align-items: center; justify-content: center;
    background: transparent; border-radius: 50%;
    color: var(--text); font-size: 16px;
  }
  .icon-btn-corner:active { background: rgba(255,255,255,.08); }
  #sessionsList { overflow-y: auto; padding: 0 16px 12px; }
  .session-row {
    display: flex; align-items: center; gap: 8px;
    padding: 8px 0;
    border-bottom: 1px solid var(--card-border);
  }
  .session-row:last-child { border-bottom: none; }
  .session-row .label { flex: 1; font-size: 11px; background: transparent; color: var(--text); text-align: left; }
  .session-row .del-btn {
    width: 32px; height: 32px; flex: none;
    background: transparent; color: var(--red); font-size: 15px;
    border-radius: 50%;
  }
  .session-row .del-btn:active { background: rgba(229,57,53,.15); }
  .session-row .dl-btn {
    width: 32px; height: 32px; flex: none;
    background: transparent; color: var(--blue); font-size: 15px;
    border-radius: 50%;
  }
  .session-row .dl-btn:active { background: rgba(59,130,246,.15); }
  #sessionsEmpty { color: var(--text-dim); padding: 12px 0; font-size: 12px; }

  /* session route dialog */
  #sessionRouteDialogBox { max-width: 480px; height: 55vh; position: relative; }
  /* The map gives up exactly the info panel's fixed height below it. Leaflet
     needs a concrete size to lay tiles out, so keep these two 84px in step. */
  #sessionRouteMapDiv { width: 100%; height: calc(100% - 84px); }
  #sessionRouteInfoText {
    height: 84px; padding: 8px 10px 10px;
    display: flex; flex-direction: column; justify-content: center; gap: 6px;
    border-top: 1px solid var(--card-border);
  }
  .route-info-title { font-size: 12px; font-weight: 600; color: var(--text-dim); text-align: center; }
  .route-info-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; }
  .route-info-cell { display: flex; flex-direction: column; align-items: center; min-width: 0; }
  .route-info-value {
    font-size: 18px; font-weight: 700; line-height: 1.1;
    color: var(--text); font-variant-numeric: tabular-nums;
  }
  .route-info-value.land { color: var(--green); }
  .route-info-value.bail { color: var(--red); }
  .route-info-label { font-size: 10px; color: var(--text-dim); margin-top: 2px; white-space: nowrap; }

  ::-webkit-scrollbar { width: 6px; }
  ::-webkit-scrollbar-thumb { background: #2A2D33; border-radius: 3px; }
</style>
</head>
<body>
<div id="app">

  <!-- ============ HOME ============ -->
  <div id="homeLayout">
    <div class="title">SkateGuard</div>
    <button id="openDashboardButton" class="btn-pill btn-green">Open Dashboard</button>
    <div id="statusText"></div>
  </div>

  <!-- ============ DASHBOARD ============ -->
  <div id="dashboardScroll">
    <div id="dashboardInner">

      <div class="dash-head">
        <div class="batteries">
          <div class="batt" id="battWrap" title="Board battery">
            <div class="batt-body"><div class="batt-fill" id="battFill"></div></div>
            <span class="batt-icon">🛹</span>
          </div>
          <div class="batt dim" id="rxBattWrap" title="No wearable in this build">
            <div class="batt-body"><div class="batt-fill" id="rxBattFill"></div></div>
            <span class="batt-icon">⌚</span>
          </div>
        </div>
        <div class="dash-title">Dashboard</div>
        <button class="map-toggle-btn" id="mapToggleButton" aria-label="Open route map">🗺️</button>
      </div>

      <div class="metrics-grid">
        <div class="metric-cell"><div class="metric-value" id="speedValue">--</div><div class="metric-label">Speed (km/h)</div></div>
        <div class="metric-cell"><div class="metric-value" id="gpsValue">--</div><div class="metric-label">GPS</div></div>
        <div class="metric-cell"><div class="metric-value" id="wobbleValue">--</div><div class="metric-label">Wobble</div></div>
        <div class="metric-cell"><div class="metric-value" id="trickValue">--</div><div class="metric-label">Trick Status</div></div>
      </div>

      <div class="card">
        <div class="compare-row">
          <div class="compare-col">
            <div class="metric-label">LAND</div>
            <div class="compare-count" style="color:var(--green)" id="landCountText">0</div>
          </div>
          <div class="compare-track">
            <!-- 50/50 to match what updateCompareBar() renders at 0/0, so
                 the very first paint (before any telemetry arrives) already
                 looks right rather than flashing a sliver. -->
            <div class="compare-land" id="landBar" style="flex:50"></div>
            <div class="compare-bail" id="bailBar" style="flex:50"></div>
          </div>
          <div class="compare-col">
            <div class="metric-label">BAIL</div>
            <div class="compare-count" style="color:var(--red)" id="bailCountText">0</div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-head">
          <div class="metric-label">SESSION RECORDING</div>
          <button class="view-recordings-btn" id="viewRecordingsButton" aria-label="View recorded sessions">📋</button>
        </div>

        <div class="session-metrics">
          <div class="session-metric"><div class="metric-value" id="sessionDistanceValue">0.00</div><div class="metric-label">Distance (km)</div></div>
          <div class="session-metric"><div class="metric-value" id="sessionSpeedValue">0.0</div><div class="metric-label">Session Speed (km/h)</div></div>
        </div>

        <div id="sessionStatusText">Not recording</div>

        <div class="btn-row" id="startStopRow">
          <button id="startRecordingButton" class="btn-green">Start</button>
          <button id="stopRecordingButton" class="btn-red" disabled>Stop</button>
        </div>
        <div class="btn-row" id="saveDiscardRow">
          <button id="saveRecordingButton" class="btn-blue">Save</button>
          <button id="discardRecordingButton" class="btn-grey">Discard</button>
        </div>
      </div>

      <div id="connectionStatusText"></div>
    </div>
  </div>

  <div id="disconnectedOverlay">
    <div>
      <div class="spinner"></div>
      <div class="msg">Trying to connect…</div>
    </div>
  </div>

  <!-- ============ MAP DIALOG ============ -->
  <div class="dialog-backdrop" id="mapDialogBackdrop">
    <div class="dialog-box" id="mapDialogBox">
      <div id="routeMapDiv"></div>
      <div class="map-overlay-badge" id="mapStatusText">Waiting for GPS fix…</div>
      <div class="map-overlay-badge" id="mapSessionStatsText" style="display:none"></div>
      <button class="dialog-close-btn" id="closeMapButton" aria-label="Close map">✕</button>
    </div>
  </div>

  <!-- ============ SESSIONS DIALOG ============ -->
  <div class="dialog-backdrop" id="sessionsDialogBackdrop">
    <div class="dialog-box" id="sessionsDialogBox">
      <div class="sessions-head">
        <!-- Same corner the BLE app puts its debug button in (see
             sessionsDebugButton in dialog_sessions.xml). -->
        <button class="icon-btn-corner" id="sessionsDebugButton" style="left:8px" aria-label="Debug">🐞</button>
        <span>Recorded Sessions (last 7)</span>
        <button class="icon-btn-corner" id="sessionsCloseButton" style="right:8px" aria-label="Close">✕</button>
      </div>
      <div id="sessionsList"></div>
    </div>
  </div>

  <!-- ============ DEBUG DIALOG ============
       The BLE app's debug button has to walk you through switching the
       phone's WiFi over to the board's own AP, then loads the board's
       debug page in a WebView. None of that applies here — this page is
       already served by the board, so the same figures come straight from
       GET /api/debug. -->
  <div class="dialog-backdrop" id="debugDialogBackdrop">
    <div class="dialog-box" id="debugDialogBox">
      <div class="sessions-head">
        <span>Debug</span>
        <button class="icon-btn-corner" id="debugCloseButton" style="right:8px" aria-label="Close">✕</button>
      </div>
      <div id="debugBody">
        <table class="debug-table" id="debugTable"></table>
        <div class="debug-links">
          <a href="/log" target="_blank" rel="noopener">Download stage_log.csv</a>
          <a href="/clear" id="debugClearLink">Clear log</a>
        </div>
      </div>
    </div>
  </div>

  <!-- ============ SESSION ROUTE DIALOG ============ -->
  <div class="dialog-backdrop" id="sessionRouteDialogBackdrop">
    <div class="dialog-box" id="sessionRouteDialogBox">
      <div id="sessionRouteMapDiv"></div>
      <div class="map-overlay-badge" id="sessionRouteStatusText">Loading…</div>
      <button class="dialog-close-btn" id="closeSessionMapButton" aria-label="Close">✕</button>
      <div id="sessionRouteInfoText"></div>
    </div>
  </div>

</div>

<script>
(() => {
  "use strict";

  // ---- Local API — same origin as this page (the board itself), so
  // these are always relative fetches, one local hop over the shared
  // hotspot. No host to configure, no internet needed for any of this.
  // 500ms matches SkateGuardBLEApp exactly — that's the board's
  // BLE_NOTIFY_INTERVAL_MS in the BLE build, so the numbers on screen move
  // at the same rate here as they do in the native app. It's a floor set by
  // taste, not by the transport: /api/live builds fresh on every request
  // from RAM over one local hop, so 250ms also works fine if you want it
  // snappier. Don't raise it above ~1000ms or trick pulses (latched for
  // 2000ms board-side) start getting missed between polls.
  const POLL_INTERVAL_MS = 500;
  const STALE_AFTER_MS = 4000;   // no updatedAtMs change in this long -> "disconnected"
  const SESSION_ROUTE_SAMPLE_MS = 30000;

  async function apiGet(path) {
    const res = await fetch(path);
    if (!res.ok) throw new Error("HTTP " + res.status);
    return res.json();
  }
  async function apiCommand(cmd) {
    const res = await fetch("/api/command?cmd=" + encodeURIComponent(cmd));
    if (!res.ok) throw new Error("HTTP " + res.status);
    return res.json();
  }

  // ---------------- elements ----------------
  const homeLayout = document.getElementById("homeLayout");
  const dashboardScroll = document.getElementById("dashboardScroll");
  const statusText = document.getElementById("statusText");
  const openDashboardButton = document.getElementById("openDashboardButton");

  const speedValue = document.getElementById("speedValue");
  const gpsValue = document.getElementById("gpsValue");
  const wobbleValue = document.getElementById("wobbleValue");
  const trickValue = document.getElementById("trickValue");
  const landCountText = document.getElementById("landCountText");
  const bailCountText = document.getElementById("bailCountText");
  const landBar = document.getElementById("landBar");
  const bailBar = document.getElementById("bailBar");
  const battFill = document.getElementById("battFill");
  const rxBattFill = document.getElementById("rxBattFill");
  const rxBattWrap = document.getElementById("rxBattWrap");
  const connectionStatusText = document.getElementById("connectionStatusText");
  const disconnectedOverlay = document.getElementById("disconnectedOverlay");

  const sessionDistanceValue = document.getElementById("sessionDistanceValue");
  const sessionSpeedValue = document.getElementById("sessionSpeedValue");
  const sessionStatusText = document.getElementById("sessionStatusText");
  const startStopRow = document.getElementById("startStopRow");
  const saveDiscardRow = document.getElementById("saveDiscardRow");
  const startRecordingButton = document.getElementById("startRecordingButton");
  const stopRecordingButton = document.getElementById("stopRecordingButton");
  const saveRecordingButton = document.getElementById("saveRecordingButton");
  const discardRecordingButton = document.getElementById("discardRecordingButton");
  const viewRecordingsButton = document.getElementById("viewRecordingsButton");

  let connected = false;
  let pollHandle = null;
  let sessionActionInFlight = false;

  // ---------------- battery ----------------
  function setBattery(fillEl, percent) {
    const clamped = Math.max(0, Math.min(100, percent));
    let color = "var(--green)";
    if (clamped <= 25) color = "var(--red)";
    else if (clamped <= 50) color = "var(--amber)";
    fillEl.style.height = clamped + "%";
    fillEl.style.background = color;
  }

  // ---------------- compare bar ----------------
  function updateCompareBar(land, bail) {
    // The app uses Android LinearLayout weights, which ALWAYS normalise —
    // weight 0.05 vs 0.05 gives a 50/50 split of the full track. CSS
    // flex-grow doesn't: it only fills the container once the values sum
    // to >= 1, so copying the app's numbers directly left both bars at
    // 0.05 (sum 0.1) filling a tenth of the track and the rest empty.
    // That's the "0 land / 0 bail looks broken until the first trick"
    // case — at 7 vs 3 the sum is 10 and it happens to look right.
    //
    // Normalising to a fixed total reproduces the Android behaviour at
    // every value, including 0/0.
    const l = Math.max(land, 0.05);
    const b = Math.max(bail, 0.05);
    const total = l + b;
    landBar.style.flex = String((l / total) * 100);
    bailBar.style.flex = String((b / total) * 100);
  }

  // ---------------- session buttons ----------------
  function updateSessionButtons(sessionState) {
    if (sessionActionInFlight) return;
    if (sessionState === "recording") {
      sessionStatusText.textContent = "Recording…";
      startStopRow.style.display = "flex";
      saveDiscardRow.style.display = "none";
      startRecordingButton.disabled = true;
      stopRecordingButton.disabled = false;
    } else if (sessionState === "stopped") {
      sessionStatusText.textContent = "Stopped — save this session or discard it?";
      startStopRow.style.display = "none";
      saveDiscardRow.style.display = "flex";
      saveRecordingButton.disabled = false;
      discardRecordingButton.disabled = false;
    } else {
      sessionStatusText.textContent = "Not recording";
      startStopRow.style.display = "flex";
      saveDiscardRow.style.display = "none";
      startRecordingButton.disabled = false;
      stopRecordingButton.disabled = true;
    }
  }

  async function runSessionAction(cmd) {
    if (sessionActionInFlight) return;
    sessionActionInFlight = true;
    startRecordingButton.disabled = true;
    stopRecordingButton.disabled = true;
    saveRecordingButton.disabled = true;
    discardRecordingButton.disabled = true;
    try {
      await apiCommand(cmd);
    } catch (e) { /* next poll cycle reflects whatever the board's state actually is */ }
    setTimeout(() => { sessionActionInFlight = false; }, 300);
  }
  startRecordingButton.addEventListener("click", () => runSessionAction("start"));
  stopRecordingButton.addEventListener("click", () => runSessionAction("stop"));
  saveRecordingButton.addEventListener("click", () => runSessionAction("save"));
  discardRecordingButton.addEventListener("click", () => runSessionAction("discard"));

  // ---------------- map state (live route) ----------------
  let sessionPathPoints = [];   // {lat,lng} sampled every 30s while recording
  let lastSessionSampleMs = 0;
  let wasRecordingLastUpdate = false;
  let liveLocator = null;       // {lat,lng} — current position

  function updateSessionPath(t) {
    const isRecording = t.sessionState === "recording";
    if (isRecording && !wasRecordingLastUpdate) {
      sessionPathPoints = [];
      lastSessionSampleMs = 0;
    }
    wasRecordingLastUpdate = isRecording;

    if (!isRecording || !t.gpsFix || (t.gpsLat === 0 && t.gpsLng === 0)) return;
    const now = Date.now();
    if (lastSessionSampleMs !== 0 && now - lastSessionSampleMs < SESSION_ROUTE_SAMPLE_MS) return;
    lastSessionSampleMs = now;
    sessionPathPoints.push({ lat: t.gpsLat, lng: t.gpsLng });
  }

  // ---------------- telemetry render ----------------
  function updateUi(t) {
    speedValue.textContent = (t.speedKmh ?? 0).toFixed(1);
    gpsValue.textContent = t.gpsFix ? "YES" : "NO";
    wobbleValue.textContent = t.wobbleActive ? "YES" : "No";
    trickValue.textContent = t.landActive ? "LAND" : (t.bailActive ? "BAIL" : "--");
    landCountText.textContent = t.kickflipCount ?? 0;
    bailCountText.textContent = t.bailCount ?? 0;
    updateCompareBar(t.kickflipCount ?? 0, t.bailCount ?? 0);

    setBattery(battFill, t.batteryPercent ?? 0);
    if (t.hasReceiver) {
      rxBattWrap.classList.remove("dim");
      setBattery(rxBattFill, t.receiverBatteryPercent ?? 0);
    } else {
      rxBattWrap.classList.add("dim");
      rxBattFill.style.height = "0%";
    }

    sessionDistanceValue.textContent = (t.sessionDistanceKm ?? 0).toFixed(2);
    // Live speed, but only while a session is actually recording — 0.0 when
    // idle or stopped. That makes it this session's speed rather than a
    // second copy of the always-on Speed tile above. (The map overlay
    // still shows the session average from sessionAvgSpeedKmh.)
    sessionSpeedValue.textContent =
      (t.sessionState === "recording" ? (t.speedKmh ?? 0) : 0).toFixed(1);
    sessionActionInFlight = false;
    updateSessionButtons(t.sessionState || "idle");

    if (t.gpsFix && (t.gpsLat !== 0 || t.gpsLng !== 0)) {
      liveLocator = { lat: t.gpsLat, lng: t.gpsLng };
    }
    updateSessionPath(t);

    if (mapDialogOpen) renderLiveMap();
    if (mapDialogOpen) updateMapSessionStats(t);
  }

  function updateMapSessionStats(t) {
    const el = document.getElementById("mapSessionStatsText");
    el.style.display = "block";
    el.textContent =
      // Live speed first — on a map you're watching while riding, how fast
      // you're going right now matters more than the session totals.
      `Speed: ${(t.speedKmh ?? 0).toFixed(1)} km/h\n` +
      `Distance: ${(t.sessionDistanceKm ?? 0).toFixed(2)} km\n` +
      `Avg speed: ${(t.sessionAvgSpeedKmh ?? 0).toFixed(1)} km/h\n` +
      `Land: ${t.kickflipCount ?? 0}   Bail: ${t.bailCount ?? 0}`;
  }

  // ---------------- polling / connection state ----------------
  // updatedAtMs is the BOARD's millis(), not wall-clock — track how long
  // it's been since WE last saw it change; if it hasn't advanced in
  // STALE_AFTER_MS of our own polling time, treat the link as stale.
  let lastSeenBoardMs = -1;
  let lastSeenChangedAtWall = 0;

  async function pollLoop() {
    try {
      const t = await apiGet("/api/live");
      if (t && typeof t.updatedAtMs === "number") {
        if (t.updatedAtMs !== lastSeenBoardMs) {
          lastSeenBoardMs = t.updatedAtMs;
          lastSeenChangedAtWall = Date.now();
        }
        const stale = Date.now() - lastSeenChangedAtWall > STALE_AFTER_MS;
        setConnected(!stale);
        if (!stale) updateUi(t);
      } else {
        setConnected(false);
      }
    } catch (e) {
      setConnected(false);
    }
  }

  function setConnected(isConnected) {
    if (isConnected === connected) return;
    connected = isConnected;
    if (isConnected) {
      disconnectedOverlay.style.display = "none";
      connectionStatusText.textContent = "";
    } else {
      disconnectedOverlay.style.display = "flex";
      connectionStatusText.textContent = "Connection lost — retrying…";
    }
  }

  function startDashboard() {
    homeLayout.style.display = "none";
    dashboardScroll.style.display = "block";
    statusText.textContent = "";
    pollLoop();
    pollHandle = setInterval(pollLoop, POLL_INTERVAL_MS);
  }

  openDashboardButton.addEventListener("click", () => {
    statusText.textContent = "Connecting…";
    startDashboard();
  });

  // ---------------- Leaflet loader (OpenStreetMap tiles — free, no key) ----------------
  let leafletPromise = null;
  function loadLeaflet() {
    if (leafletPromise) return leafletPromise;
    leafletPromise = new Promise((resolve, reject) => {
      if (window.L) { resolve(window.L); return; }
      const script = document.createElement("script");
      script.src = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js";
      script.integrity = "sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=";
      script.crossOrigin = "";
      script.onload = () => resolve(window.L);
      script.onerror = () => reject(new Error("Failed to load Leaflet"));
      document.head.appendChild(script);
    });
    return leafletPromise;
  }

  const OSM_TILE_URL = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png";
  const OSM_ATTRIBUTION = '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors';

  // Dashed line + ringed dot markers, matching the native app's own
  // PatternItem dash(20)/gap(12) route and hand-drawn locator/waypoint
  // bitmaps (see MainActivity.kt locatorDotIcon()/waypointDotIcon()).
  function dashedRoute(L, points) {
    return L.polyline(points.map(p => [p.lat, p.lng]), { color: "#FFA726", weight: 3, dashArray: "10, 6" });
  }
  function dotMarker(L, p, fillColor, radius) {
    return L.circleMarker([p.lat, p.lng], { radius, fillColor, fillOpacity: 1, color: "#FFFFFF", weight: 2 });
  }

  // ---------------- live route map ----------------
  const mapDialogBackdrop = document.getElementById("mapDialogBackdrop");
  const mapStatusText = document.getElementById("mapStatusText");
  let mapDialogOpen = false;
  let liveMap = null;
  let livePolyline = null;
  let liveLocatorMarker = null;
  let liveWaypointMarkers = [];
  let hasCenteredLiveCamera = false;
  // Timestamp rather than an "is dragging" boolean, deliberately: a flag
  // cleared only by dragend stays stuck ON if that event is ever missed
  // (cancelled touch, lost pointer capture, an exception mid-gesture), and
  // auto-follow would then be silently dead for the rest of the ride with
  // no way to get it back. A timestamp always heals itself.
  let lastUserMapInteractionMs = 0;
  const FOLLOW_RESUME_AFTER_MS = 3000;

  async function ensureLiveMap() {
    if (liveMap) return liveMap;
    const L = await loadLeaflet();
    // Zoom buttons go bottom-right: Leaflet's default top-left corner is
    // where the stats badge sits, and the badge (z-index 1200) covers them.
    liveMap = L.map("routeMapDiv", { zoomControl: false }).setView([0, 0], 17);
    L.control.zoom({ position: "bottomright" }).addTo(liveMap);
    // Only USER-driven events, never move/moveend — panTo below fires those
    // itself, so following would suppress itself after its first pan.
    // drag/dragstart/dragend come solely from Leaflet's Drag handler, and
    // panTo never changes zoom, so zoomstart here is the rider too.
    liveMap.on("dragstart drag dragend zoomstart", () => {
      lastUserMapInteractionMs = Date.now();
    });
    L.tileLayer(OSM_TILE_URL, { attribution: OSM_ATTRIBUTION, maxZoom: 19 }).addTo(liveMap);
    livePolyline = dashedRoute(L, []).addTo(liveMap);
    return liveMap;
  }

  function renderLiveMap() {
    if (!liveMap || !window.L) return;
    const hasAny = sessionPathPoints.length > 0 || liveLocator;
    if (!hasAny) {
      mapStatusText.style.display = "block";
      mapStatusText.textContent = "Waiting for GPS fix…";
      return;
    }
    mapStatusText.style.display = "none";

    livePolyline.setLatLngs(sessionPathPoints.map(p => [p.lat, p.lng]));

    liveWaypointMarkers.forEach(m => m.remove());
    liveWaypointMarkers = sessionPathPoints.map(p => dotMarker(L, p, "#E53935", 5).addTo(liveMap));

    if (liveLocator) {
      if (!liveLocatorMarker) {
        liveLocatorMarker = dotMarker(L, liveLocator, "#4285F4", 9).addTo(liveMap);
      } else {
        liveLocatorMarker.setLatLng([liveLocator.lat, liveLocator.lng]);
      }
      // Mirrors updateLocator() in MainActivity.kt: jump to the first fix
      // and set the zoom, then FOLLOW every fix after that. The web port
      // originally implemented only the first half, so the dot kept moving
      // while the camera stayed frozen where you started — ride a hundred
      // metres and you're off-screen until you drag the map yourself.
      //
      // panTo is the Leaflet equivalent of the app's animateCamera(newLatLng):
      // animated, and it preserves whatever zoom the rider has chosen
      // (setView would yank them back to 17 on every single update).
      if (!hasCenteredLiveCamera) {
        liveMap.setView([liveLocator.lat, liveLocator.lng], 17);
        hasCenteredLiveCamera = true;
      } else if (Date.now() - lastUserMapInteractionMs > FOLLOW_RESUME_AFTER_MS) {
        // Held off while the rider is panning or zooming: at a 500ms poll,
        // yanking the map back under a moving finger makes it impossible to
        // look ahead. Following resumes on its own a few seconds after they
        // stop, so there's no stuck state and no recentre button to hunt for.
        liveMap.panTo([liveLocator.lat, liveLocator.lng]);
      }
    }
  }

  document.getElementById("mapToggleButton").addEventListener("click", async () => {
    mapDialogOpen = true;
    mapDialogBackdrop.classList.add("open");
    mapStatusText.style.display = "block";
    mapStatusText.textContent = "Loading map…";
    try {
      await ensureLiveMap();
      liveMap.invalidateSize();
      renderLiveMap();
    } catch (e) {
      mapStatusText.textContent = "Map unavailable on this device";
    }
  });
  document.getElementById("closeMapButton").addEventListener("click", () => {
    mapDialogOpen = false;
    mapDialogBackdrop.classList.remove("open");
    document.getElementById("mapSessionStatsText").style.display = "none";
  });

  // ---------------- sessions dialog ----------------
  const sessionsDialogBackdrop = document.getElementById("sessionsDialogBackdrop");
  const sessionsList = document.getElementById("sessionsList");

  function parseSessionsCsv(csv) {
    if (!csv || !csv.trim()) return [];
    const rows = csv.trim().split(/\r?\n/).slice(1);
    return rows.map((line, i) => {
      const p = line.split(",");
      if (p.length < 3) return null;
      return { index: i + 1, avgSpeedKmh: p[1] || "0.00", distanceKm: p[2] || "0.000", landCount: p[3] || "0", bailCount: p[4] || "0" };
    }).filter(Boolean);
  }

  // Positional: line N of routes.csv belongs to session N. A session saved
  // without GPS has no route — current firmware writes "-" for that, older
  // firmware wrote a blank line — and both must stay in the list as real
  // entries. This used to filter blank lines out, which slid every later
  // route onto the wrong session. Only the file's single trailing newline
  // is dropped, and "-" is normalised to "" so it reads as "no route".
  function parseRoutesCsv(csv) {
    if (!csv) return [];
    const lines = csv.split(/\r?\n/);
    if (lines.length > 0 && lines[lines.length - 1] === "") lines.pop();
    return lines.map(l => (l.trim() === "-" ? "" : l));
  }

  function parseRouteLine(line) {
    if (!line) return [];
    return line.split(";").map(seg => {
      const [lat, lng] = seg.split(":").map(Number);
      return Number.isFinite(lat) && Number.isFinite(lng) ? { lat, lng } : null;
    }).filter(Boolean);
  }

  async function openSessionsDialog() {
    sessionsDialogBackdrop.classList.add("open");
    sessionsList.innerHTML = `<div id="sessionsEmpty">Loading…</div>`;
    let device;
    try { device = await apiGet("/api/sessions"); } catch (e) { device = null; }
    const sessions = parseSessionsCsv(device?.sessionsCsv);
    const routeLines = parseRoutesCsv(device?.routesCsv);

    if (sessions.length === 0) {
      sessionsList.innerHTML = `<div id="sessionsEmpty">No sessions recorded yet.</div>`;
      return;
    }
    sessionsList.innerHTML = "";
    sessions.forEach(s => {
      const row = document.createElement("div");
      row.className = "session-row";
      row.innerHTML = `
        <button class="label">Session ${s.index}:  ${s.distanceKm} km   •   avg ${s.avgSpeedKmh} km/h   •   L${s.landCount}/B${s.bailCount}</button>
        <button class="dl-btn" aria-label="Download session ${s.index}">⬇</button>
        <button class="del-btn" aria-label="Delete session ${s.index}">🗑</button>
      `;
      row.querySelector(".label").addEventListener("click", () => {
        sessionsDialogBackdrop.classList.remove("open");
        openSessionRouteDialog(s, routeLines[s.index - 1]);
      });
      // Just this one session: its CSV row, plus its map image only when it
      // actually has a route. A session with no route (e.g. practising
      // kickflips in one spot, where GPS never logs a path) is perfectly
      // normal, so it simply gets the CSV and no image.
      // Uses `device` — the data this list was drawn from — so the file
      // always matches the row that was tapped, even if the list refreshes.
      row.querySelector(".dl-btn").addEventListener("click", async (ev) => {
        const dlBtn = ev.currentTarget;
        if (dlBtn.disabled) return;
        dlBtn.disabled = true;   // no double-saves while the image is drawn
        const stamp = downloadStamp();

        saveBlob(
          new Blob([buildSessionsExportCsv(device, s.index)], { type: "text/csv" }),
          `skateguard_session_${s.index}_${stamp}.csv`
        );

        const points = parseRouteLine(routeLines[s.index - 1]);
        if (points.length > 0) {
          dlBtn.textContent = "…";
          try {
            const png = await renderSessionRouteImage(s, points);
            saveBlob(png, `skateguard_session_${s.index}_map_${stamp}.png`);
          } catch (e) {
            console.warn("Route map for session " + s.index + " failed:", e);
          }
          dlBtn.textContent = "⬇";
        }
        dlBtn.disabled = false;
      });
      row.querySelector(".del-btn").addEventListener("click", async () => {
        await apiCommand("delete:" + s.index);
        setTimeout(openSessionsDialog, 300);
      });
      sessionsList.appendChild(row);
    });
  }
  viewRecordingsButton.addEventListener("click", openSessionsDialog);
  document.getElementById("sessionsCloseButton").addEventListener("click", () => sessionsDialogBackdrop.classList.remove("open"));

  // ---------------- download sessions ----------------
  // Built in the browser from the same /api/sessions data the list shows and
  // saved through a local blob link. That keeps this a page-only change — no
  // new firmware route to add (and keep in sync) in both sender sketches.
  //
  // One row per saved session, routes included, so the file stands alone.
  // saved_at_uptime_ms is the board's millis() when Save was pressed (time
  // since that boot), not a calendar date — the board has no clock.
  function csvField(value) {
    const s = String(value ?? "");
    return /[",\r\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
  }

  // onlyIndex (1-based, as shown in the list) limits the file to that one
  // session — used by each row's own download button.
  function buildSessionsExportCsv(data, onlyIndex = null) {
    const header = "session,saved_at_uptime_ms,avg_speed_kmh,distance_km,land_count,bail_count,route_points,route";
    const rawRows = (data?.sessionsCsv || "").trim().split(/\r?\n/).slice(1);
    const routeLines = parseRoutesCsv(data?.routesCsv);
    const rows = [];
    rawRows.forEach((line, i) => {
      const p = line.split(",");
      if (p.length < 3) return;   // same skip rule parseSessionsCsv uses
      if (onlyIndex !== null && i + 1 !== onlyIndex) return;
      const route = routeLines[i] || "";
      rows.push([
        i + 1, p[0], p[1], p[2], p[3] ?? "0", p[4] ?? "0",
        parseRouteLine(route).length, route,
      ].map(csvField).join(","));
    });
    return header + "\n" + rows.join("\n") + "\n";
  }

  // ---------------- route map images ----------------
  // One PNG per saved session: the route over a street map, with the ride
  // stats underneath — something you can open in Gallery or share, unlike
  // the raw coordinates in the CSV.
  //
  // Drawn onto a canvas by hand (tile maths + the tiles themselves) rather
  // than screenshotting the Leaflet map: that needs no extra library, and it
  // works straight from the session list without opening that map on screen.
  // OpenStreetMap's tile server sends CORS headers, so tiles loaded with
  // crossOrigin="anonymous" can be exported from the canvas.
  const ROUTE_IMG_W = 1080;
  const ROUTE_IMG_MAP_H = 1080;
  const ROUTE_IMG_INFO_H = 300;
  const ROUTE_IMG_PAD = 150;        // keep the route clear of the image edges
  // OSM tiles are 256px; drawn at 2x so street names are still readable when
  // the image is viewed full-width on a phone rather than tiny and crisp.
  const ROUTE_IMG_TILE_PX = 512;
  const ROUTE_IMG_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
  const ROUTE_IMG_FONT = "system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif";

  // Web Mercator: lat/lng -> pixel position in the whole world map at zoom z.
  function routeWorldPx(lat, lng, z) {
    const scale = ROUTE_IMG_TILE_PX * Math.pow(2, z);
    const latR = lat * Math.PI / 180;
    return {
      x: (lng + 180) / 360 * scale,
      y: (1 - Math.log(Math.tan(latR) + 1 / Math.cos(latR)) / Math.PI) / 2 * scale,
    };
  }

  // Closest zoom at which the whole route still fits inside the padded map
  // area. A single point has no extent, so it gets the app's usual zoom 17.
  function pickRouteZoom(points) {
    if (points.length < 2) return 17;
    for (let z = 18; z >= 3; z--) {
      const px = points.map(p => routeWorldPx(p.lat, p.lng, z));
      const w = Math.max(...px.map(p => p.x)) - Math.min(...px.map(p => p.x));
      const h = Math.max(...px.map(p => p.y)) - Math.min(...px.map(p => p.y));
      if (w <= ROUTE_IMG_W - 2 * ROUTE_IMG_PAD && h <= ROUTE_IMG_MAP_H - 2 * ROUTE_IMG_PAD) return z;
    }
    return 3;
  }

  // Resolves to the tile image, or null if it fails or takes over 8s — a
  // missing tile shouldn't stop the image from being produced.
  function loadRouteTile(z, x, y) {
    return new Promise((resolve) => {
      const img = new Image();
      img.crossOrigin = "anonymous";
      const timer = setTimeout(() => resolve(null), 8000);
      img.onload = () => { clearTimeout(timer); resolve(img); };
      img.onerror = () => { clearTimeout(timer); resolve(null); };
      img.src = ROUTE_IMG_TILE_URL.replace("{z}", z).replace("{x}", x).replace("{y}", y);
    });
  }

  function canvasToPngBlob(canvas) {
    return new Promise((resolve, reject) => {
      try {
        canvas.toBlob(b => (b ? resolve(b) : reject(new Error("toBlob returned null"))), "image/png");
      } catch (e) {
        reject(e);   // SecurityError if a tile arrived without CORS permission
      }
    });
  }

  async function renderSessionRouteImage(session, points, withTiles = true) {
    const W = ROUTE_IMG_W, MAP_H = ROUTE_IMG_MAP_H, TILE = ROUTE_IMG_TILE_PX;
    const canvas = document.createElement("canvas");
    canvas.width = W;
    canvas.height = MAP_H + ROUTE_IMG_INFO_H;
    const ctx = canvas.getContext("2d");

    // Plain map-coloured ground: what shows wherever a tile couldn't load.
    ctx.fillStyle = "#E9E5DC";
    ctx.fillRect(0, 0, W, MAP_H);

    const z = pickRouteZoom(points);
    const world = points.map(p => routeWorldPx(p.lat, p.lng, z));
    const minX = Math.min(...world.map(p => p.x)), maxX = Math.max(...world.map(p => p.x));
    const minY = Math.min(...world.map(p => p.y)), maxY = Math.max(...world.map(p => p.y));
    // Whole pixels, so every tile lands on an integer offset and no hairline
    // seams appear between neighbouring tiles.
    const left = Math.round((minX + maxX) / 2 - W / 2);
    const top = Math.round((minY + maxY) / 2 - MAP_H / 2);

    let tilesWanted = 0, tilesMissing = 0;
    if (withTiles) {
      const n = Math.pow(2, z);
      const jobs = [];
      for (let ty = Math.floor(top / TILE); ty <= Math.floor((top + MAP_H - 1) / TILE); ty++) {
        if (ty < 0 || ty >= n) continue;
        for (let tx = Math.floor(left / TILE); tx <= Math.floor((left + W - 1) / TILE); tx++) {
          const wrappedX = ((tx % n) + n) % n;
          jobs.push(loadRouteTile(z, wrappedX, ty).then(img => ({ img, dx: tx * TILE - left, dy: ty * TILE - top })));
        }
      }
      tilesWanted = jobs.length;
      (await Promise.all(jobs)).forEach(({ img, dx, dy }) => {
        if (img) ctx.drawImage(img, dx, dy, TILE, TILE);
        else tilesMissing++;
      });
    }

    // Route: solid white casing under the dashed orange line, so the dashes
    // stay visible over any map colour, then the red waypoint dots — the
    // same look as the route on the in-app maps.
    const pts = world.map(p => ({ x: p.x - left, y: p.y - top }));
    const tracePath = () => {
      ctx.beginPath();
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)));
    };
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    if (pts.length > 1) {
      ctx.setLineDash([]);
      ctx.strokeStyle = "rgba(255,255,255,0.9)";
      ctx.lineWidth = 16;
      tracePath(); ctx.stroke();
      ctx.setLineDash([32, 20]);
      ctx.strokeStyle = "#FFA726";
      ctx.lineWidth = 9;
      tracePath(); ctx.stroke();
    }
    ctx.setLineDash([]);
    pts.forEach(p => {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 13, 0, Math.PI * 2);
      ctx.fillStyle = "#FFFFFF";
      ctx.fill();
      ctx.beginPath();
      ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
      ctx.fillStyle = "#E53935";
      ctx.fill();
    });

    // OpenStreetMap's licence requires this credit on anything showing its map.
    ctx.font = `500 24px ${ROUTE_IMG_FONT}`;
    ctx.textBaseline = "middle";
    ctx.textAlign = "left";
    const credit = "© OpenStreetMap contributors";
    const creditW = ctx.measureText(credit).width;
    ctx.fillStyle = "rgba(255,255,255,0.85)";
    ctx.fillRect(W - creditW - 28, MAP_H - 44, creditW + 28, 44);
    ctx.fillStyle = "#333333";
    ctx.fillText(credit, W - creditW - 14, MAP_H - 22);

    // Say so on the image when the map behind the route is incomplete, so a
    // blank background isn't mistaken for the route being somewhere empty.
    let notice = "";
    if (!withTiles || (tilesWanted > 0 && tilesMissing === tilesWanted)) notice = "Map unavailable — route only";
    else if (tilesMissing > 0) notice = "Some map tiles didn't load";
    if (notice) {
      ctx.font = `600 28px ${ROUTE_IMG_FONT}`;
      const nW = ctx.measureText(notice).width;
      ctx.fillStyle = "rgba(0,0,0,0.6)";
      ctx.fillRect(24, 24, nW + 32, 56);
      ctx.fillStyle = "#FFFFFF";
      ctx.fillText(notice, 40, 52);
    }

    // Ride stats strip — same four values as the session map dialog.
    const infoTop = MAP_H;
    ctx.fillStyle = "#1B1D21";
    ctx.fillRect(0, infoTop, W, ROUTE_IMG_INFO_H);
    ctx.fillStyle = "#2A2D33";
    ctx.fillRect(0, infoTop, W, 2);

    ctx.textAlign = "center";
    ctx.textBaseline = "alphabetic";
    ctx.fillStyle = "#9AA0AA";
    ctx.font = `600 34px ${ROUTE_IMG_FONT}`;
    ctx.fillText(`SkateGuard  ·  Session ${session.index}`, W / 2, infoTop + 66);

    const num = (v, digits) => {
      const x = Number(v);
      return Number.isFinite(x) ? x.toFixed(digits) : "--";
    };
    const cells = [
      [num(session.distanceKm, 2),  "Distance (km)",    "#F5F5F7"],
      [num(session.avgSpeedKmh, 1), "Avg Speed (km/h)", "#F5F5F7"],
      [num(session.landCount, 0),   "Land",             "#2ECC71"],
      [num(session.bailCount, 0),   "Bail",             "#E53935"],
    ];
    const colW = W / cells.length;
    cells.forEach(([value, label, color], i) => {
      const cx = colW * i + colW / 2;
      ctx.fillStyle = color;
      ctx.font = `700 66px ${ROUTE_IMG_FONT}`;
      ctx.fillText(value, cx, infoTop + 172);
      ctx.fillStyle = "#9AA0AA";
      ctx.font = `500 26px ${ROUTE_IMG_FONT}`;
      ctx.fillText(label, cx, infoTop + 224);
    });

    try {
      return await canvasToPngBlob(canvas);
    } catch (e) {
      // A tile without CORS permission "taints" the canvas and export is
      // refused outright. Rather than lose the image, redraw without tiles.
      if (withTiles && e && e.name === "SecurityError") {
        return renderSessionRouteImage(session, points, false);
      }
      throw e;
    }
  }

  function saveBlob(blob, filename) {
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 4000);
  }

  // Phone's local date and time for filenames, so repeat downloads don't pile
  // up as "(1)", "(2)" copies of the same name.
  function downloadStamp() {
    const d = new Date();
    const pad = (n) => String(n).padStart(2, "0");
    return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}`;
  }

  // ---------------- debug dialog ----------------
  const debugDialogBackdrop = document.getElementById("debugDialogBackdrop");
  const debugTable = document.getElementById("debugTable");
  const DEBUG_POLL_INTERVAL_MS = 500;
  let debugPollHandle = null;

  // [label, formatter] — order here is the row order on screen, and
  // mirrors the board debug page the BLE app opens in its WebView.
  const DEBUG_ROWS = [
    ["Stage",               d => d.stage ?? "--"],
    ["Gyro Y",              d => fmt(d.gyroY, 1) + " deg/s"],
    ["Accum Rotation",      d => fmt(d.accumDeg, 1) + " deg"],
    ["Land Confirm Swing",  d => fmt(d.landConfirmSwingDeg, 1) + " deg"],
    ["Active Window",       d => `${d.activeWindowMs ?? 0} / ${d.activeWindowMaxMs ?? 0} ms`],
    ["Absolute Window",     d => `${d.absoluteWindowMs ?? 0} / ${d.absoluteWindowMaxMs ?? 0} ms`],
    ["Cooldown Window",     d => `${d.cooldownWindowMs ?? 0} / ${d.cooldownWindowMaxMs ?? 0} ms`],
    ["Oscillations",        d => `${d.oscillations ?? 0} / ${d.oscillationsRequired ?? 0}`],
    ["Loop Max / Avg",      d => `${d.loopMaxMs ?? 0} / ${fmt(d.loopAvgMs, 1)} ms`],
    ["Land Count",          d => d.kickflipCount ?? 0],
    ["Bail Count",          d => d.bailCount ?? 0],
    ["Battery",             d => `${d.batteryPercent ?? 0}%  (${fmt(d.batteryVoltage, 2)} V)`],
    ["GPS Satellites",      d => `${d.gpsSatellites ?? 0}${(d.gpsSatellites ?? 0) >= 4 ? "" : "  (need 4+)"}`],
    ["GPS HDOP",            d => (d.gpsHdop > 0 ? fmt(d.gpsHdop, 1) : "--")],
    ["GPS Fix Age",         d => (d.gpsFixAgeMs != null && d.gpsFixAgeMs >= 0) ? `${(d.gpsFixAgeMs / 1000).toFixed(1)} s` : "no fix yet"],
    ["GPS Data In",         d => `${d.gpsCharsProcessed ?? 0} chars`],
    ["GPS Checksum Fails",  d => d.gpsFailedChecksum ?? 0],
    ["WiFi",                d => `ch ${d.wifiChannel ?? "--"}   ${d.wifiRssi ?? "--"} dBm`],
    ["ESP-NOW",             d => d.espNowReady ? (d.receiverLinked ? "wearable linked" : "ready, no wearable") : "off"],
    ["Free Heap",           d => `${Math.round((d.freeHeap ?? 0) / 1024)} KB`],
    ["Uptime",              d => formatUptime(d.uptimeMs ?? 0)],
  ];

  function fmt(v, digits) {
    return (typeof v === "number" && isFinite(v)) ? v.toFixed(digits) : "--";
  }

  function formatUptime(ms) {
    const totalSec = Math.floor(ms / 1000);
    const h = Math.floor(totalSec / 3600);
    const m = Math.floor((totalSec % 3600) / 60);
    const s = totalSec % 60;
    return h > 0 ? `${h}h ${m}m ${s}s` : (m > 0 ? `${m}m ${s}s` : `${s}s`);
  }

  function renderDebug(d) {
    // Rebuilding only the value cells (not the whole table) keeps text
    // selection and scroll position stable across the 500ms refresh.
    if (debugTable.rows.length !== DEBUG_ROWS.length) {
      debugTable.innerHTML = "";
      DEBUG_ROWS.forEach(([label]) => {
        const tr = debugTable.insertRow();
        tr.insertCell().textContent = label;
        tr.insertCell().textContent = "--";
      });
    }
    DEBUG_ROWS.forEach(([, format], i) => {
      let text;
      try { text = String(format(d)); } catch (e) { text = "--"; }
      const cell = debugTable.rows[i].cells[1];
      if (cell.textContent !== text) cell.textContent = text;
    });
  }

  async function debugPoll() {
    try {
      renderDebug(await apiGet("/api/debug"));
    } catch (e) {
      // Board unreachable mid-session — leave the last values on screen
      // rather than blanking the table; the connection banner already
      // reports the disconnect.
    }
  }

  function openDebugDialog() {
    debugDialogBackdrop.classList.add("open");
    debugPoll();
    if (debugPollHandle === null) {
      debugPollHandle = setInterval(debugPoll, DEBUG_POLL_INTERVAL_MS);
    }
  }

  function closeDebugDialog() {
    debugDialogBackdrop.classList.remove("open");
    if (debugPollHandle !== null) {
      clearInterval(debugPollHandle);
      debugPollHandle = null;
    }
  }

  document.getElementById("sessionsDebugButton").addEventListener("click", openDebugDialog);
  document.getElementById("debugCloseButton").addEventListener("click", closeDebugDialog);

  // /clear redirects back to "/", which would replace this page with a
  // fresh load and drop you out of the dialog. Intercept it, clear via
  // fetch, and stay put.
  document.getElementById("debugClearLink").addEventListener("click", async (e) => {
    e.preventDefault();
    try { await fetch("/clear"); } catch (err) { /* next poll shows the truth */ }
  });

  // ---------------- session route dialog (a saved session's own route) ----------------
  const sessionRouteDialogBackdrop = document.getElementById("sessionRouteDialogBackdrop");
  const sessionRouteStatusText = document.getElementById("sessionRouteStatusText");
  let sessionRouteMap = null;
  let sessionRoutePolyline = null;
  let sessionRouteMarkers = [];

  function clearSessionRouteOverlays() {
    if (sessionRoutePolyline) { sessionRoutePolyline.remove(); sessionRoutePolyline = null; }
    sessionRouteMarkers.forEach(m => m.remove());
    sessionRouteMarkers = [];
  }

  // Ride info under a saved session's map: a title plus four large values,
  // instead of the old single 11px line, which was hard to read on a phone.
  // Built with textContent rather than innerHTML since the values come
  // straight from the board's CSV.
  function renderSessionRouteInfo(session) {
    const box = document.getElementById("sessionRouteInfoText");
    box.textContent = "";

    const title = document.createElement("div");
    title.className = "route-info-title";
    title.textContent = `Session ${session.index}`;

    const num = (v, digits) => {
      const n = Number(v);
      return Number.isFinite(n) ? n.toFixed(digits) : "--";
    };

    const grid = document.createElement("div");
    grid.className = "route-info-grid";
    [
      [num(session.distanceKm, 2),  "Distance (km)",    ""],
      [num(session.avgSpeedKmh, 1), "Avg Speed (km/h)", ""],
      [num(session.landCount, 0),   "Land",             "land"],
      [num(session.bailCount, 0),   "Bail",             "bail"],
    ].forEach(([value, label, tone]) => {
      const cell = document.createElement("div");
      cell.className = "route-info-cell";
      const v = document.createElement("div");
      v.className = "route-info-value" + (tone ? " " + tone : "");
      v.textContent = value;
      const l = document.createElement("div");
      l.className = "route-info-label";
      l.textContent = label;
      cell.append(v, l);
      grid.appendChild(cell);
    });

    box.append(title, grid);
  }

  async function openSessionRouteDialog(session, routeLine) {
    sessionRouteDialogBackdrop.classList.add("open");
    renderSessionRouteInfo(session);

    const points = parseRouteLine(routeLine);
    if (points.length === 0) {
      sessionRouteStatusText.style.display = "block";
      sessionRouteStatusText.textContent = "No route recorded for this session";
      clearSessionRouteOverlays();
      return;
    }

    sessionRouteStatusText.style.display = "block";
    sessionRouteStatusText.textContent = "Loading map…";
    try {
      const L = await loadLeaflet();
      if (!sessionRouteMap) {
        sessionRouteMap = L.map("sessionRouteMapDiv").setView([points[0].lat, points[0].lng], 17);
        L.tileLayer(OSM_TILE_URL, { attribution: OSM_ATTRIBUTION, maxZoom: 19 }).addTo(sessionRouteMap);
      }
      sessionRouteMap.invalidateSize();
      clearSessionRouteOverlays();

      if (points.length > 1) {
        sessionRoutePolyline = dashedRoute(L, points).addTo(sessionRouteMap);
      }
      sessionRouteMarkers = points.map(p => dotMarker(L, p, "#E53935", 5).addTo(sessionRouteMap));

      if (points.length > 1) {
        sessionRouteMap.fitBounds(points.map(p => [p.lat, p.lng]), { padding: [40, 40] });
      } else {
        sessionRouteMap.setView([points[0].lat, points[0].lng], 17);
      }
      sessionRouteStatusText.style.display = "none";
    } catch (e) {
      sessionRouteStatusText.style.display = "block";
      sessionRouteStatusText.textContent = "Map unavailable on this device";
    }
  }
  document.getElementById("closeSessionMapButton").addEventListener("click", () => sessionRouteDialogBackdrop.classList.remove("open"));

  // Close any dialog by tapping its backdrop, matching standard dialog behavior.
  [mapDialogBackdrop, sessionsDialogBackdrop, sessionRouteDialogBackdrop].forEach(bd => {
    bd.addEventListener("click", (e) => { if (e.target === bd) bd.classList.remove("open"); });
  });
  // Separate from the loop above: closing this one also has to stop its
  // poller, or it keeps hitting /api/debug forever behind a hidden dialog.
  debugDialogBackdrop.addEventListener("click", (e) => {
    if (e.target === debugDialogBackdrop) closeDebugDialog();
  });

})();
</script>
</body>
</html>
)HTMLDOC";

// Named axis constants per ADR rule — never hardcode axis indices inline.
enum GyroAxis { GYRO_X = 0, GYRO_Y = 1, GYRO_Z = 2 };

// ---------------------------------------------------------------------
// Serial logging only in this file — no TCP monitor, no BLE, no local
// web dashboard. Plain Serial.print, kept under the same netPrint/
// netPrintln names so the IMU/FSM/session code below (copied from
// skateboard_xiao_c3_ble_stable.ino) needed zero edits at the call sites.
// ---------------------------------------------------------------------
template<typename T>
void netPrint(T v) { Serial.print(v); }
template<typename T>
void netPrint(T v, int fmt) { Serial.print(v, fmt); }
template<typename T>
void netPrintln(T v) { Serial.println(v); }
template<typename T>
void netPrintln(T v, int fmt) { Serial.println(v, fmt); }
void netPrintln() { Serial.println(); }

// ---------------------------------------------------------------------
// Battery — read via voltage divider (30k on VCC side, 10k on GND side)
// into an ADC pin. Same calibration as skateboard_xiao_c3_ble.ino.
// ---------------------------------------------------------------------
#define BATTERY_ADC_PIN 4  // GPIO4 (ADC1_CH4) — UNVERIFIED

const float BATTERY_DIVIDER_RATIO = 4.165f;
const float ADC_MAX_COUNTS = 4095.0f;
const float ADC_REF_VOLTAGE = 3.3f;
const float ADC_VOLTAGE_CALIBRATION = 0.8815f;
const float BATTERY_MIN_V = 3.3f;  // considered 0% — UNVERIFIED
const float BATTERY_MAX_V = 4.2f;  // considered 100% — UNVERIFIED

uint8_t batteryPercent = 0;
float lastPinVoltage = 0.0f;
float lastBatteryVoltage = 0.0f;
unsigned long lastBatteryReadMs = 0;
const unsigned long BATTERY_READ_INTERVAL_MS = 2000;
const uint8_t BATTERY_SAMPLE_COUNT = 10;

void batteryInit() {
  analogReadResolution(12);
}

void batteryUpdate() {
  unsigned long now = millis();
  if (now - lastBatteryReadMs < BATTERY_READ_INTERVAL_MS) return;
  lastBatteryReadMs = now;

  long rawSum = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    rawSum += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }
  float raw = rawSum / (float)BATTERY_SAMPLE_COUNT;

  float rawPinVoltage = (raw / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
  float pinVoltage = rawPinVoltage * ADC_VOLTAGE_CALIBRATION;
  float batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;

  lastPinVoltage = pinVoltage;
  lastBatteryVoltage = batteryVoltage;

  float pct = (batteryVoltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
  pct = constrain(pct, 0.0f, 100.0f);
  batteryPercent = (uint8_t)(roundf(pct / 5.0f) * 5.0f);
}

// ---------------------------------------------------------------------
// IMU — BNO055
// ---------------------------------------------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define PIN_SDA 6   // UNVERIFIED
#define PIN_SCL 7   // UNVERIFIED

const float FILTER_ALPHA = 0.2f;
float filteredTiltAngleZ = 0.0f;

const GyroAxis WOBBLE_AXIS    = GYRO_Y;  // UNVERIFIED — re-check axis on this mount
const GyroAxis KICKFLIP_AXIS  = GYRO_Y;  // UNVERIFIED

void imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);

  netPrintln(F("[IMU] Initializing BNO055..."));
  while (!bno.begin()) {
    netPrintln(F("[IMU] BNO055 not found - check wiring, power, and I2C address."));
    delay(1000);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  netPrintln(F("[IMU] BNO055 found. Move through slow figure-8s after boot "
                    "to help magnetometer calibration settle."));
}

float readEulerAngleZ() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.z();
}

void imuUpdate() {
  float raw = readEulerAngleZ();
  filteredTiltAngleZ = (FILTER_ALPHA * raw) + ((1.0f - FILTER_ALPHA) * filteredTiltAngleZ);
}

float imuGetTiltAngleZ() {
  return filteredTiltAngleZ;
}

imu::Vector<3> readGyroRaw() {
  return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

float gyroAxisValue(const imu::Vector<3>& v, GyroAxis axis) {
  switch (axis) {
    case GYRO_X: return v.x();
    case GYRO_Y: return v.y();
    case GYRO_Z: return v.z();
  }
  return 0.0f;
}

// Hard clamp on physically-realistic gyro rate. See MEMORY.md — 700 was
// too aggressive (saturated constantly, truncated real ~900 deg/s trick
// peaks into undershoot bails); 1200 sits above observed real peaks
// (~1121 deg/s in bench logs) while still rejecting the absurd.
const float MAX_REALISTIC_GYRO_DPS = 1200.0f;

float clampGyroRate(float dps) {
  if (dps > MAX_REALISTIC_GYRO_DPS) return MAX_REALISTIC_GYRO_DPS;
  if (dps < -MAX_REALISTIC_GYRO_DPS) return -MAX_REALISTIC_GYRO_DPS;
  return dps;
}

// 5-sample median filter — rejects a single wild spike without smearing
// out a real, sustained fast rotation the way a low-pass filter would.
float medianOf5(float v0, float v1, float v2, float v3, float v4) {
  float a[5] = {v0, v1, v2, v3, v4};
  for (uint8_t i = 1; i < 5; i++) {
    float key = a[i];
    int8_t j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
  return a[2];
}

struct MedianFilter5 {
  float history[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t count = 0;

  float apply(float newValue) {
    for (uint8_t i = 0; i < 4; i++) history[i] = history[i + 1];
    history[4] = newValue;
    if (count < 5) count++;
    if (count < 5) return newValue;
    return medianOf5(history[0], history[1], history[2], history[3], history[4]);
  }
};

MedianFilter5 gyroYMedianFilter;

// ---------------------------------------------------------------------
// UNIFIED MOTION FSM — copied verbatim from skateboard_xiao_c3_ble_stable.ino,
// including the reversal oscillation re-baseline fix. Do not diverge this
// from that file without a reason; they should stay in sync.
// ---------------------------------------------------------------------
const float         MOTION_TRIGGER_DPS = 150.0f;
const float         LAND_RANGE1_MIN_DEG = 320.0f;
const float         LAND_RANGE1_MAX_DEG = 400.0f;
const float         BAIL_MIN_ANGLE_DEG  = 200.0f;
const unsigned long MOTION_WINDOW_MS   = 2000;
const unsigned long MOTION_COOLDOWN_MS = 500;
const uint8_t       WOBBLE_REVERSAL_SAMPLES_REQUIRED = 5;
const float         WOBBLE_MAX_CONSIDER_DEG = 60.0f;
const float         WOBBLE_MIN_SPEED_KMH = 5.0f;

const unsigned long LAND_CONFIRM_WINDOW_MS = 1000;
const float         LAND_CONFIRM_MAX_DEG   = 83.0f;

const float MOTION_STABLE_DPS = 20.0f;
const unsigned long MOTION_ABSOLUTE_TIMEOUT_MS = 4000;
const uint8_t WOBBLE_OSCILLATIONS_REQUIRED = 3;

enum MotionState { MOTION_IDLE, MOTION_ROTATING, MOTION_LAND_CONFIRM, MOTION_COOLDOWN };
MotionState motionState = MOTION_IDLE;

const char* motionStateName() {
  switch (motionState) {
    case MOTION_IDLE:         return "IDLE";
    case MOTION_ROTATING:     return "ROTATING";
    case MOTION_LAND_CONFIRM: return "LAND_CONFIRM";
    case MOTION_COOLDOWN:     return "COOLDOWN";
  }
  return "UNKNOWN";
}

unsigned long motionWindowStartMs = 0;
unsigned long motionLastSampleMs = 0;
unsigned long motionCooldownStartMs = 0;
unsigned long motionActiveElapsedMs = 0;
float motionAccumulatedDeg = 0.0f;
bool motionInitialSignPositive = true;
uint8_t motionReversalStreak = 0;
uint8_t motionOscillationCount = 0;

unsigned long landConfirmStartMs = 0;
unsigned long landConfirmLastSampleMs = 0;
float landConfirmAccumulatedDeg = 0.0f;

bool wobbleEventPending = false;
unsigned long wobbleLastEventMs = 0;
bool kickflipEventPending = false;

float lastWobbleAccumulatedDeg = 0.0f;
float lastWobbleSpeedKmh = 0.0f;

float lastLandRotationDeg = 0.0f;
float lastLandConfirmSwingDeg = 0.0f;
bool bailEventPending = false;
uint16_t kickflipCount = 0;
uint16_t bailCount = 0;
bool cooldownRequiresStability = false;

enum BailReason { BAIL_REASON_OVERSHOOT, BAIL_REASON_POST_LAND_WOBBLE };
BailReason lastBailReason = BAIL_REASON_OVERSHOOT;

const char* bailReasonName() {
  switch (lastBailReason) {
    case BAIL_REASON_OVERSHOOT:        return "overshoot (rotation missed the land window)";
    case BAIL_REASON_POST_LAND_WOBBLE: return "post-land wobble (swung past land-confirm limit)";
  }
  return "unknown";
}

uint8_t latchedStageEvent = 0;  // 0=none, 1=wobble, 2=land, 3=bail
unsigned long stageEventLatchedAtMs = 0;
const unsigned long STAGE_EVENT_LATCH_MS = 800;

void classifyAndFinishMotion(unsigned long now) {
  bool isLandCandidate = (motionAccumulatedDeg >= LAND_RANGE1_MIN_DEG &&
                          motionAccumulatedDeg <= LAND_RANGE1_MAX_DEG);

  if (isLandCandidate) {
    lastLandRotationDeg = motionAccumulatedDeg;
    motionState = MOTION_LAND_CONFIRM;
    landConfirmStartMs = now;
    landConfirmLastSampleMs = now;
    landConfirmAccumulatedDeg = 0.0f;
  } else if (motionAccumulatedDeg >= BAIL_MIN_ANGLE_DEG) {
    bailCount++;
    bailEventPending = true;
    lastBailReason = BAIL_REASON_OVERSHOOT;
    motionState = MOTION_COOLDOWN;
    motionCooldownStartMs = now;
    cooldownRequiresStability = true;
  } else {
    motionState = MOTION_IDLE;
  }
  motionAccumulatedDeg = 0.0f;
}

void motionFsmUpdate(float axisDps, float speedKmh) {
  unsigned long now = millis();
  float magnitude = fabsf(axisDps);

  switch (motionState) {
    case MOTION_IDLE:
      if (magnitude >= MOTION_TRIGGER_DPS) {
        motionState = MOTION_ROTATING;
        motionWindowStartMs = now;
        motionLastSampleMs = now;
        motionAccumulatedDeg = 0.0f;
        motionActiveElapsedMs = 0;
        motionInitialSignPositive = (axisDps >= 0);
        motionReversalStreak = 0;
        motionOscillationCount = 0;
      }
      break;

    case MOTION_ROTATING: {
      unsigned long dtMs = now - motionLastSampleMs;
      float dtSeconds = dtMs / 1000.0f;
      motionLastSampleMs = now;

      bool currentSignPositive = (axisDps >= 0);
      bool reversed = (magnitude >= MOTION_TRIGGER_DPS && currentSignPositive != motionInitialSignPositive);

      if (reversed && motionAccumulatedDeg < WOBBLE_MAX_CONSIDER_DEG) {
        motionReversalStreak++;
        if (motionReversalStreak >= WOBBLE_REVERSAL_SAMPLES_REQUIRED) {
          motionReversalStreak = 0;
          motionOscillationCount++;

          if (motionOscillationCount >= WOBBLE_OSCILLATIONS_REQUIRED) {
            if (speedKmh > WOBBLE_MIN_SPEED_KMH) {
              wobbleEventPending = true;
              wobbleLastEventMs = now;
              lastWobbleAccumulatedDeg = motionAccumulatedDeg;
              lastWobbleSpeedKmh = speedKmh;
            }
            motionState = MOTION_COOLDOWN;
            motionCooldownStartMs = now;
            cooldownRequiresStability = false;
          } else {
            motionInitialSignPositive = currentSignPositive;
            motionAccumulatedDeg = 0.0f;
          }
        }
        break;
      } else if (reversed) {
        motionReversalStreak = 0;
        classifyAndFinishMotion(now);
        break;
      } else {
        motionReversalStreak = 0;
      }

      motionAccumulatedDeg += magnitude * dtSeconds;

      if (magnitude >= MOTION_STABLE_DPS) {
        motionActiveElapsedMs += dtMs;
      }

      if (motionActiveElapsedMs >= MOTION_WINDOW_MS ||
          (now - motionWindowStartMs) >= MOTION_ABSOLUTE_TIMEOUT_MS) {
        classifyAndFinishMotion(now);
      }
      break;
    }

    case MOTION_LAND_CONFIRM: {
      float dtSeconds = (now - landConfirmLastSampleMs) / 1000.0f;
      landConfirmLastSampleMs = now;
      landConfirmAccumulatedDeg += axisDps * dtSeconds;

      if (fabsf(landConfirmAccumulatedDeg) >= LAND_CONFIRM_MAX_DEG) {
        bailCount++;
        bailEventPending = true;
        lastBailReason = BAIL_REASON_POST_LAND_WOBBLE;
        motionState = MOTION_COOLDOWN;
        motionCooldownStartMs = now;
        cooldownRequiresStability = true;
        break;
      }

      if (now - landConfirmStartMs >= LAND_CONFIRM_WINDOW_MS) {
        lastLandConfirmSwingDeg = landConfirmAccumulatedDeg;
        kickflipCount++;
        kickflipEventPending = true;
        motionState = MOTION_COOLDOWN;
        motionCooldownStartMs = now;
        cooldownRequiresStability = false;
      }
      break;
    }

    case MOTION_COOLDOWN:
      if (cooldownRequiresStability && magnitude >= MOTION_STABLE_DPS) {
        motionCooldownStartMs = now;
        break;
      }
      if (now - motionCooldownStartMs >= MOTION_COOLDOWN_MS) {
        motionState = MOTION_IDLE;
        cooldownRequiresStability = false;
      }
      break;
  }
}

// ---------------------------------------------------------------------
// GPS — NEO-8M
// ---------------------------------------------------------------------
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
#define GPS_BAUD   9600

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

const float JITTER_REJECT_THRESHOLD_KMH = 2.0f;

// Position older than this counts as no fix — see gpsHasFix().
const unsigned long GPS_FIX_MAX_AGE_MS = 3000;

// Fastest speed a jump between two accepted positions is allowed to imply.
// 20 m/s = 72 km/h, well above any real skateboard speed, so only glitches
// exceed it. Used as speed x elapsed time rather than a fixed distance —
// see gpsUpdateDistance().
const float GPS_MAX_PLAUSIBLE_SPEED_MS = 20.0f;

// GPS diagnostics. These are deliberately passive. The receiver is left at
// its normal 9600 baud / default navigation configuration so this sketch does
// not risk sending a malformed UBX command to a clone or different M8 variant.
unsigned long gpsBootMs = 0;
unsigned long gpsLastCharMs = 0;
unsigned long gpsLastFixMs = 0;
unsigned long gpsLastStatusMs = 0;
bool gpsEverHadFix = false;

bool gpsHasFix();

// Process as much pending NMEA data as possible without blocking. This is
// called at the beginning AND near the end of loop(). That matters because
// WiFi, filesystem writes, ESP-NOW and the web server can all consume time
// while the GPS UART keeps receiving bytes in the background.
void gpsProcessSerial() {
  uint16_t processed = 0;

  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();
    gps.encode(c);
    gpsLastCharMs = millis();
    processed++;

    // Safety guard. At 9600 baud this is far above the amount normally
    // waiting between two calls, but it prevents one unexpectedly large UART
    // backlog from monopolising the motion loop.
    if (processed >= 512) break;
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsLastFixMs = millis();

    // gpsHasFix(), not just isValid(): a weak 3-satellite fix makes the
    // position valid, but the rest of the sketch still treats GPS as NO, so
    // announcing "FIRST FIX" then would contradict the dashboard.
    if (!gpsEverHadFix && gpsHasFix()) {
      gpsEverHadFix = true;
      netPrint(F("[GPS] FIRST FIX after "));
      netPrint((gpsLastFixMs - gpsBootMs) / 1000);
      netPrintln(F(" seconds"));
    }
  }
}

float gpsGetSpeedKmh() {
  if (!gpsHasFix() || !gps.speed.isValid()) return 0.0f;

  float raw = gps.speed.kmph();
  if (!isfinite(raw) || raw < JITTER_REJECT_THRESHOLD_KMH) return 0.0f;

  return raw;
}

bool gpsHasFix() {
  // A location alone is not enough. Require a current valid position and at
  // least four satellites, which avoids treating weak / incomplete 2D data
  // as a reliable skateboard position.
  //
  // The age check covers the case the satellite count can't: TinyGPS++
  // keeps the last position AND the last satellite count forever once
  // valid. While sentences keep arriving the count refreshes and drops when
  // signal is lost, but if the GPS stops sending entirely (loose wire, GPS
  // brownout) nothing refreshes, and without this the dashboard would show
  // GPS: YES on frozen data indefinitely. The NEO-8M reports once per
  // second, so 3s allows a couple of missed sentences before giving up.
  return gps.location.isValid() &&
         gps.location.age() < GPS_FIX_MAX_AGE_MS &&
         gps.satellites.isValid() &&
         gps.satellites.value() >= 4;
}

double gpsLastLat = 0.0;
double gpsLastLng = 0.0;
bool   gpsHasLastPosition = false;
float  totalDistanceMeters = 0.0f;

// When gpsLastLat/gpsLastLng were last accepted. Lets the jump check scale
// with how long the GPS was without a usable fix.
unsigned long gpsLastPositionMs = 0;

void gpsUpdateDistance() {
  if (!gps.location.isUpdated() || !gps.location.isValid()) return;

  // Do not accept a position update unless the receiver currently has the
  // minimum satellite count. TinyGPS++ can retain the last valid location
  // while newer sentences are still being parsed.
  if (!gpsHasFix()) return;

  double lat = gps.location.lat();
  double lng = gps.location.lng();

  if (gpsHasLastPosition && gpsGetSpeedKmh() > 0.0f) {
    float deltaM = TinyGPSPlus::distanceBetween(
      gpsLastLat, gpsLastLng, lat, lng
    );

    // Reject impossible jumps caused by a bad fix or multipath — but judge
    // "impossible" against how long it's been since the last accepted
    // position, not against a fixed distance.
    //
    // While there's no usable fix, gpsLastLat/gpsLastLng stay frozen, so the
    // first fix after a signal gap measures the WHOLE gap as one jump. A
    // fixed 100m cap threw that away: at 15 km/h you cover 100m in ~24s, so
    // any gap longer than that (a bridge, tall buildings, dipping below 4
    // satellites) silently lost the distance actually ridden. Allowing
    // GPS_MAX_PLAUSIBLE_SPEED_MS x elapsed keeps glitch rejection on a
    // normal 1s update (still ~20m) while crediting real distance across
    // gaps. Floored at 1s so update-timing jitter can't shrink the limit.
    float elapsedS = (millis() - gpsLastPositionMs) / 1000.0f;
    if (elapsedS < 1.0f) elapsedS = 1.0f;
    float maxJumpM = GPS_MAX_PLAUSIBLE_SPEED_MS * elapsedS;

    if (isfinite(deltaM) && deltaM >= 0.0f && deltaM <= maxJumpM) {
      totalDistanceMeters += deltaM;
    }
  }

  gpsLastLat = lat;
  gpsLastLng = lng;
  gpsLastPositionMs = millis();
  gpsHasLastPosition = true;
}

void gpsPrintStatus() {
  unsigned long now = millis();
  if (now - gpsLastStatusMs < 5000) return;
  gpsLastStatusMs = now;

  netPrint(F("[GPS] SAT="));
  if (gps.satellites.isValid()) netPrint(gps.satellites.value());
  else netPrint(F("?"));

  netPrint(F(" HDOP="));
  if (gps.hdop.isValid()) netPrint(gps.hdop.hdop(), 1);
  else netPrint(F("?"));

  netPrint(F(" CHARS="));
  netPrint(gps.charsProcessed());

  netPrint(F(" FIX="));
  netPrint(gpsHasFix() ? F("YES") : F("NO"));

  netPrint(F(" DATA="));
  if (gpsLastCharMs == 0) netPrint(F("NONE"));
  else {
    netPrint((now - gpsLastCharMs) / 1000);
    netPrint(F("s ago"));
  }

  netPrint(F(" FIXAGE="));
  if (gps.location.isValid()) {
    netPrint(gps.location.age());
    netPrint(F("ms"));
  } else {
    netPrint(F("NEVER"));
  }

  netPrint(F(" CHKSUM="));
  netPrint(gps.failedChecksum());

  netPrint(F(" UPTIME="));
  netPrint((now - gpsBootMs) / 1000);
  netPrint(F("s"));

  if (gpsHasFix()) {
    netPrint(F(" LAT="));
    netPrint(gps.location.lat(), 6);
    netPrint(F(" LNG="));
    netPrint(gps.location.lng(), 6);
  }

  netPrintln();
}

// ---------------------------------------------------------------------
// Session recording — mirrors skateboard_xiao_c3_ble.ino's local CSV
// behavior exactly (sessions.csv / routes.csv, FIFO capped at
// MAX_SAVED_SESSIONS). Commands arrive over the local HTTP API (see
// handleApiCommand() further down) instead of a BLE characteristic
// write; the local CSV stays the source of truth either way, and
// GET /api/sessions reads it directly on each request — no separate
// mirror/dirty-flag needed now that there's no round-trip to keep in
// sync with.
// ---------------------------------------------------------------------
#define SESSIONS_CSV_PATH "/sessions.csv"
#define ROUTES_CSV_PATH   "/routes.csv"
const uint16_t MAX_SAVED_SESSIONS = 7;

#define SESSION_MAX_ROUTE_POINTS 20
const unsigned long SESSION_ROUTE_SAMPLE_INTERVAL_MS = 30000;
struct RoutePoint { double lat; double lng; };
RoutePoint sessionRoute[SESSION_MAX_ROUTE_POINTS];
uint8_t sessionRouteCount = 0;
unsigned long sessionLastRouteSampleMs = 0;

enum SessionRecState { SESSION_IDLE, SESSION_RECORDING, SESSION_STOPPED };
SessionRecState sessionState = SESSION_IDLE;
float    sessionStartDistanceKm = 0.0f;
double   sessionSpeedSum = 0.0;
uint32_t sessionSpeedCount = 0;
float    sessionFinalDistanceKm = 0.0f;
float    sessionFinalAvgSpeedKmh = 0.0f;
uint16_t sessionKickflipCountStart = 0;
uint16_t sessionBailCountStart = 0;
uint16_t sessionFinalKickflipCount = 0;
uint16_t sessionFinalBailCount = 0;

void sessionUpdate(float speedKmh) {
  if (sessionState != SESSION_RECORDING) return;
  sessionSpeedSum += speedKmh;
  sessionSpeedCount++;
}

void sessionUpdateRoute() {
  if (sessionState != SESSION_RECORDING) return;
  if (!gpsHasLastPosition) return;
  if (sessionRouteCount >= SESSION_MAX_ROUTE_POINTS) return;

  unsigned long now = millis();
  if (sessionLastRouteSampleMs != 0 && now - sessionLastRouteSampleMs < SESSION_ROUTE_SAMPLE_INTERVAL_MS) return;
  sessionLastRouteSampleMs = now;

  sessionRoute[sessionRouteCount].lat = gpsLastLat;
  sessionRoute[sessionRouteCount].lng = gpsLastLng;
  sessionRouteCount++;
}

float sessionLiveDistanceKm(float distanceKm) {
  switch (sessionState) {
    case SESSION_RECORDING: {
      float d = distanceKm - sessionStartDistanceKm;
      return (d > 0.0f) ? d : 0.0f;
    }
    case SESSION_STOPPED: return sessionFinalDistanceKm;
    default:              return 0.0f;
  }
}

float sessionLiveAvgSpeedKmh() {
  if (sessionState == SESSION_STOPPED) return sessionFinalAvgSpeedKmh;
  if (sessionSpeedCount == 0) return 0.0f;
  return (float)(sessionSpeedSum / sessionSpeedCount);
}

const char* sessionStateName() {
  switch (sessionState) {
    case SESSION_RECORDING: return "recording";
    case SESSION_STOPPED:   return "stopped";
    default:                return "idle";
  }
}

void sessionStart() {
  sessionStartDistanceKm = totalDistanceMeters / 1000.0f;
  sessionSpeedSum = 0.0;
  sessionSpeedCount = 0;
  sessionKickflipCountStart = kickflipCount;
  sessionBailCountStart = bailCount;
  sessionRouteCount = 0;
  sessionLastRouteSampleMs = 0;
  sessionState = SESSION_RECORDING;
  netPrintln(F("[SESSION] Recording started"));
}

void sessionStop() {
  if (sessionState != SESSION_RECORDING) return;
  float distanceKm = totalDistanceMeters / 1000.0f;
  float d = distanceKm - sessionStartDistanceKm;
  sessionFinalDistanceKm = (d > 0.0f) ? d : 0.0f;
  sessionFinalAvgSpeedKmh = (sessionSpeedCount > 0) ? (float)(sessionSpeedSum / sessionSpeedCount) : 0.0f;
  sessionFinalKickflipCount = kickflipCount - sessionKickflipCountStart;
  sessionFinalBailCount = bailCount - sessionBailCountStart;
  sessionState = SESSION_STOPPED;
  netPrintln(F("[SESSION] Recording stopped"));
}

uint16_t countCsvDataLines(const char* path, bool hasHeader) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return 0;
  if (hasHeader) f.readStringUntil('\n');
  uint16_t count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() > 0) count++;
  }
  f.close();
  return count;
}

void skipOldestCsvLines(const char* path, uint16_t skip, bool hasHeader) {
  if (skip == 0 || !LittleFS.exists(path)) return;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  String kept = "";
  if (hasHeader) kept = f.readStringUntil('\n') + "\n";
  uint16_t rowIndex = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    // Every line counts, blank ones included. This also runs on routes.csv,
    // where a blank line is a real entry: a session saved with no route by
    // firmware older than the "-" placeholder in sessionSave(). Skipping
    // blanks here dropped that session's slot, so after a trim every later
    // route was paired with the wrong session. sessions.csv never has blank
    // rows, so it's unaffected. The file's final newline doesn't yield a
    // phantom empty line: once it's consumed, available() is false.
    rowIndex++;
    if (rowIndex > skip) kept += line + "\n";
  }
  f.close();
  File out = LittleFS.open(path, FILE_WRITE);
  if (!out) return;
  out.print(kept);
  out.close();
}

void deleteLineFromFile(const char* path, uint16_t index, bool hasHeader) {
  if (index == 0 || !LittleFS.exists(path)) return;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  String kept = "";
  if (hasHeader) kept = f.readStringUntil('\n') + "\n";
  uint16_t rowIndex = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    // Positional, blank lines included — same reason as skipOldestCsvLines():
    // on routes.csv a blank line is a session with no route, and skipping it
    // made "delete session N" remove some OTHER session's route.
    rowIndex++;
    if (rowIndex != index) kept += line + "\n";
  }
  f.close();
  File out = LittleFS.open(path, FILE_WRITE);
  if (!out) return;
  out.print(kept);
  out.close();
}

void trimSessionsToMax() {
  uint16_t totalRows = countCsvDataLines(SESSIONS_CSV_PATH, true);
  if (totalRows <= MAX_SAVED_SESSIONS) return;
  uint16_t skip = totalRows - MAX_SAVED_SESSIONS;
  skipOldestCsvLines(SESSIONS_CSV_PATH, skip, true);
  skipOldestCsvLines(ROUTES_CSV_PATH, skip, false);
}

void sessionSave() {
  if (sessionState != SESSION_STOPPED) return;

  bool isNewFile = !LittleFS.exists(SESSIONS_CSV_PATH);
  File f = LittleFS.open(SESSIONS_CSV_PATH, FILE_APPEND);
  if (!f) {
    netPrintln(F("[SESSION] Failed to open sessions.csv"));
    return;
  }
  if (isNewFile) {
    f.print("timestamp_ms,avg_speed_kmh,distance_km,land_count,bail_count\n");
  }
  f.printf("%lu,%.2f,%.3f,%u,%u\n", millis(), sessionFinalAvgSpeedKmh, sessionFinalDistanceKm,
            sessionFinalKickflipCount, sessionFinalBailCount);
  f.close();

  String route = "";
  for (uint8_t i = 0; i < sessionRouteCount; i++) {
    if (i > 0) route += ";";
    route += String(sessionRoute[i].lat, 6) + ":" + String(sessionRoute[i].lng, 6);
  }
  // A session recorded without any usable GPS fix has no route. Write "-"
  // instead of leaving the line blank: routes.csv is paired with
  // sessions.csv purely by line position, and blank lines are exactly what
  // gets skipped or tidied away (older dashboards filtered them out), which
  // silently slides every later route onto the wrong session.
  if (route.length() == 0) route = "-";

  File rf = LittleFS.open(ROUTES_CSV_PATH, FILE_APPEND);
  if (rf) {
    rf.print(route);
    rf.print("\n");
    rf.close();
  }

  trimSessionsToMax();
  sessionState = SESSION_IDLE;
  netPrintln(F("[SESSION] Saved to sessions.csv"));
}

void sessionDiscard() {
  if (sessionState != SESSION_STOPPED) return;
  sessionState = SESSION_IDLE;
  netPrintln(F("[SESSION] Discarded (not saved)"));
}

void sessionDeleteAll() {
  if (LittleFS.exists(SESSIONS_CSV_PATH)) LittleFS.remove(SESSIONS_CSV_PATH);
  if (LittleFS.exists(ROUTES_CSV_PATH)) LittleFS.remove(ROUTES_CSV_PATH);
  sessionState = SESSION_IDLE;
  netPrintln(F("[SESSION] All saved sessions deleted"));
}

void sessionDeleteIndex(uint16_t index) {
  deleteLineFromFile(SESSIONS_CSV_PATH, index, true);
  deleteLineFromFile(ROUTES_CSV_PATH, index, false);
  netPrint(F("[SESSION] Deleted session #"));
  netPrintln(index);
}

String readSessionsCsv() {
  if (!LittleFS.exists(SESSIONS_CSV_PATH)) return "";
  File f = LittleFS.open(SESSIONS_CSV_PATH, FILE_READ);
  if (!f) return "";
  String content = f.readString();
  f.close();
  return content;
}

String readRoutesCsv() {
  if (!LittleFS.exists(ROUTES_CSV_PATH)) return "";
  File f = LittleFS.open(ROUTES_CSV_PATH, FILE_READ);
  if (!f) return "";
  String content = f.readString();
  f.close();
  return content;
}

// ---------------------------------------------------------------------
// Local stage log — same LittleFS CSV format as the other SkateGuard
// sketches, downloadable from the dashboard via GET /log for
// after-the-fact analysis (same format the webapp-cloud run-log viewer
// already knows how to parse, if you want that view again later).
// ---------------------------------------------------------------------
#define STAGE_LOG_CSV_PATH "/stage_log.csv"
const uint16_t MAX_STAGE_LOG_ROWS = 2000;
const unsigned long STAGE_LOG_WRITE_INTERVAL_MS = 500;

const char* STAGE_LOG_HEADER =
  "timestamp_ms,stage,gyro_y,accum_deg,land_confirm_swing_deg,event,kickflip_count,bail_count,active_window_ms,absolute_window_ms,cooldown_window_ms,oscillation_count";

void stageLogClearOnBoot() {
  if (LittleFS.exists(STAGE_LOG_CSV_PATH)) {
    LittleFS.remove(STAGE_LOG_CSV_PATH);
  }
}

uint16_t countDataLines(const char* path, bool hasHeader) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return 0;
  if (hasHeader) f.readStringUntil('\n');
  uint16_t count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() > 0) count++;
  }
  f.close();
  return count;
}

void skipOldestLines(const char* path, uint16_t skip, bool hasHeader) {
  if (skip == 0 || !LittleFS.exists(path)) return;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;

  String kept = "";
  if (hasHeader) kept = f.readStringUntil('\n') + "\n";

  uint16_t rowIndex = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;
    rowIndex++;
    if (rowIndex > skip) kept += line + "\n";
  }
  f.close();

  File out = LittleFS.open(path, FILE_WRITE);
  if (!out) return;
  out.print(kept);
  out.close();
}

void stageLogAppend(unsigned long timestampMs, const char* stageName, float gyroY,
                     float accumDeg, float landConfirmSwingDeg, uint8_t event,
                     uint16_t kfCount, uint16_t blCount,
                     unsigned long activeWindowMs, unsigned long absoluteWindowMs,
                     unsigned long cooldownWindowMs, uint8_t oscillationCount) {
  bool isNewFile = !LittleFS.exists(STAGE_LOG_CSV_PATH);
  File f = LittleFS.open(STAGE_LOG_CSV_PATH, FILE_APPEND);
  if (!f) {
    netPrintln(F("[STAGELOG] Failed to open stage_log.csv"));
    return;
  }
  if (isNewFile) {
    f.print(STAGE_LOG_HEADER);
    f.print("\n");
  }

  char line[192];
  int n = snprintf(line, sizeof(line), "%lu,%s,%.2f,%.2f,%.2f,%u,%u,%u,%lu,%lu,%lu,%u",
                    timestampMs, stageName, gyroY, accumDeg, landConfirmSwingDeg, event,
                    kfCount, blCount, activeWindowMs, absoluteWindowMs, cooldownWindowMs,
                    oscillationCount);
  f.print(line);
  f.print("\n");
  f.close();

  static uint16_t writesSinceTrim = 0;
  writesSinceTrim++;
  if (writesSinceTrim >= 50) {
    writesSinceTrim = 0;
    uint16_t totalRows = countDataLines(STAGE_LOG_CSV_PATH, true);
    if (totalRows > MAX_STAGE_LOG_ROWS) {
      skipOldestLines(STAGE_LOG_CSV_PATH, totalRows - MAX_STAGE_LOG_ROWS, true);
    }
  }
}

// ---------------------------------------------------------------------
// WiFi — joins the phone's hotspot as a station. No Firebase, no
// internet dependency; this is purely so the board and phone are on the
// same local network for the web server below.
// ---------------------------------------------------------------------
bool wifiConnected = false;
unsigned long lastWifiAttemptMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;

// Starts an attempt against WIFI_NETWORKS[wifiNetworkIndex], skipping
// blank slots. Non-blocking: WiFi.begin() returns immediately and
// wifiUpdate() watches for the result, so nothing here can stall loop()
// and distort trick timing.
void wifiBeginCurrentNetwork() {
  // Find the next non-empty entry, giving up after one full pass so an
  // all-blank table can't spin forever.
  for (uint8_t tried = 0; tried < WIFI_NETWORK_COUNT; tried++) {
    const WifiCredential& net = WIFI_NETWORKS[wifiNetworkIndex];
    if (net.ssid != nullptr && net.ssid[0] != '\0') {
      netPrint(F("[WIFI] Trying \""));
      netPrint(net.ssid);
      netPrintln(F("\""));
      WiFi.begin(net.ssid, net.password);
      return;
    }
    wifiNetworkIndex = (wifiNetworkIndex + 1) % WIFI_NETWORK_COUNT;
  }
  netPrintln(F("[WIFI] No networks configured - fill in WIFI_NETWORKS"));
}

void wifiInit() {
  WiFi.mode(WIFI_STA);

  // Announce a DHCP hostname (DHCP option 12). Android's hotspot runs
  // dnsmasq, which on many builds resolves the hostnames its own DHCP
  // clients registered — so http://skateguard/ (no ".local") often just
  // works on the phone, no IP to remember and no mDNS support needed.
  // Costs nothing when the phone ignores it; the .200 address below stays
  // the guaranteed path either way. Must be set after mode(), before
  // begin().
  WiFi.setHostname("skateguard");

  // Deliberately plain DHCP here — see pinStaticIp(), which runs once the
  // lease tells us what subnet we're actually on.
  wifiBeginCurrentNetwork();
}

// Moves us to <current subnet>.STATIC_HOST_OCTET so the dashboard URL is
// predictable, using the gateway and mask the hotspot just told us about.
// Called after the DHCP lease lands.
//
// Every failure path here leaves the working DHCP address in place. An
// unreachable board is much worse than one whose address ends in an
// unexpected number — the DHCP lease is always a functioning fallback, so
// nothing in here is allowed to be fatal.
void pinStaticIp() {
  IPAddress gateway = WiFi.gatewayIP();
  IPAddress subnet  = WiFi.subnetMask();
  IPAddress current = WiFi.localIP();

  // A hotspot that gave us no gateway is not something to start rewriting
  // interface config against.
  if (gateway == IPAddress(0, 0, 0, 0)) {
    netPrintln(F("[WIFI] No gateway from DHCP - keeping the leased address"));
    return;
  }

  // Only safe on a /24. On anything wider the host octet isn't the only
  // thing that identifies us within the subnet, and blindly rewriting the
  // last octet could land outside the usable range.
  if (subnet != IPAddress(255, 255, 255, 0)) {
    netPrint(F("[WIFI] Subnet isn't /24 ("));
    netPrint(subnet);
    netPrintln(F(") - keeping the leased address"));
    return;
  }

  IPAddress desired(gateway[0], gateway[1], gateway[2], STATIC_HOST_OCTET);
  if (current == desired) return;   // already where we want to be

  if (!WiFi.config(desired, gateway, subnet, gateway)) {
    netPrintln(F("[WIFI] Static config rejected - keeping the leased address"));
    return;
  }

  // Verify rather than assume: if the interface didn't actually take the
  // new address, say so instead of printing a URL that doesn't work.
  IPAddress applied = WiFi.localIP();
  if (applied != desired) {
    netPrint(F("[WIFI] Static config didn't take, still on "));
    netPrintln(applied);
    return;
  }

  netPrint(F("[WIFI] Pinned to "));
  netPrintln(applied);
}

void wifiUpdate() {
  bool nowConnected = (WiFi.status() == WL_CONNECTED);
  if (nowConnected && !wifiConnected) {
    netPrint(F("[WIFI] Joined \""));
    netPrint(WiFi.SSID());
    netPrint(F("\", DHCP lease "));
    netPrintln(WiFi.localIP());

    // Re-address BEFORE announcing the dashboard URL, so the address
    // printed below is the one to actually open.
    pinStaticIp();

    netPrint(F("[WIFI] Dashboard at http://"));
    netPrint(WiFi.localIP());
    netPrintln(F("/"));
    netPrintln(F("[WIFI] Try http://skateguard/ too - works on most Android hotspots"));
    if (MDNS.begin("skateguard")) {
      netPrintln(F("[WIFI] And http://skateguard.local/ on iOS/macOS/Windows"));
    }

    // Modem sleep OFF — a connected STA otherwise powers the radio down
    // between AP beacon windows, and ESP-NOW packets from the wearable
    // that arrive during those naps are silently dropped. This is the
    // single most important line for making ESP-NOW reliable while also
    // joined to a hotspot (reciver_skateboard_xiaoc3.ino does the same
    // on its side, for the same reason).
    esp_wifi_set_ps(WIFI_PS_NONE);

    // The hotspot dictates our channel — we can't choose it, and ESP-NOW
    // can only talk to peers on the SAME channel. Print it so the wearable
    // side is debuggable; the wearable finds this channel by scanning
    // (see espnow_channel discovery in the receiver sketch).
    netPrint(F("[ESPNOW] Radio is on hotspot channel "));
    netPrint(WiFi.channel());
    netPrintln(F(" - wearable must hop to this channel"));
  }
  wifiConnected = nowConnected;

  if (!wifiConnected) {
    unsigned long now = millis();
    if (now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiAttemptMs = now;

      // Move to the next configured network before retrying, so a phone
      // that's switched off doesn't hold the board hostage — it cycles
      // through the table until one of them answers.
      wifiNetworkIndex = (wifiNetworkIndex + 1) % WIFI_NETWORK_COUNT;
      WiFi.disconnect();

      // Back to DHCP before every retry. Once pinStaticIp() has applied a
      // static address it STICKS across reconnects — so without this, a
      // board that had pinned itself on one hotspot would rejoin a
      // different hotspot still clinging to the old subnet's address, and
      // be unreachable exactly as if the address had been hardcoded. All
      // zeroes re-enables DHCP, so each reconnect re-learns the subnet.
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
      wifiBeginCurrentNetwork();
    }
  }
}

// ---------------------------------------------------------------------
// ESP-NOW — to/from the wearable receiver, running ALONGSIDE the hotspot
// connection above. This is the part that was previously declared
// impossible in this variant's header, so here's why it does work:
//
//   The old objection was that ESP-NOW needs a fixed, known channel while
//   a hotspot dictates the channel. Both halves are true; the wrong
//   conclusion was that the channel has to be known AT COMPILE TIME.
//   It doesn't — it only has to MATCH at runtime. So:
//
//     - This board does NOT call esp_wifi_set_channel(). Doing so would
//       yank the radio off the hotspot's channel and drop the WiFi link
//       (that's what reciver_skateboard_xiaoc3.ino does, but it isn't
//       joined to any AP, so it's free to).
//     - The broadcast peer is added with channel = 0, which means "use
//       whatever channel the interface is currently on". So ESP-NOW
//       automatically follows the hotspot, including if it moves.
//     - The WEARABLE does the adapting: it channel-hops until it hears
//       us, then locks on. See skateguard_local_receiver.ino.
//
// Three structs of deliberately distinct sizes (22 / 1 / 6 bytes) so
// neither end can confuse them — the receiver dispatches on length.
//
// Two additions versus skateboard_xiao_c3_ble_stable.ino's layout, which
// is why the wearable must be skateguard_local_receiver.ino (the older
// receiver expects the 18-byte layout and would reject these):
//
//   boardIp      — this board's dashboard address, so the wearable's OLED
//                  can show you where to point a browser instead of
//                  needing a laptop and Serial Monitor after a phone swap.
//
//   boardChannel — the channel this radio is actually on. 2.4GHz channels
//                  are 20MHz wide but spaced 5MHz apart, so a receiver
//                  scanning for us hears us on neighbouring channels too
//                  (a sender on 6 leaks onto 4-8). A receiver that locks
//                  onto the first channel it hears anything on can easily
//                  settle 1-2 channels off, where only the occasional
//                  packet survives — which looks exactly like a flapping
//                  link, not like a mistuned one. Stating the channel
//                  explicitly lets the wearable retune to the real one
//                  instead of guessing from where it happened to hear us.
// ---------------------------------------------------------------------
typedef struct {
  float    tiltAngleZ;
  float    speedKmh;
  float    distanceKm;
  uint32_t boardIp;        // IPv4 as a raw 32-bit value, 0 until connected
  bool     gpsFix;
  bool     wobbleActive;
  bool     kickflipActive;
  uint16_t kickflipCount;
  uint8_t  boardChannel;   // the channel we're ACTUALLY on — see below
} TelemetryPacket;

typedef struct {
  uint8_t batteryPercent;
} ReceiverStatusPacket;

// Optional third packet — consumed only by motion_stage_display_test.ino's
// OLED if that board happens to be in range. The real wearable ignores it
// (size doesn't match TelemetryPacket).
typedef struct {
  uint8_t  stage;
  uint8_t  event;
  uint16_t kickflipCount;
  uint16_t bailCount;
} StagePacket;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
TelemetryPacket telemetry;
StagePacket stagePacket;

// Wearable's battery, reported back over the same broadcast link.
uint8_t receiverBatteryPercent = 0;
unsigned long lastReceiverStatusMs = 0;
const unsigned long RECEIVER_STATUS_TIMEOUT_MS = 12000;

bool espNowReady = false;

void onReceiverStatusRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ReceiverStatusPacket)) return;
  ReceiverStatusPacket status;
  memcpy(&status, incomingData, sizeof(status));
  receiverBatteryPercent = status.batteryPercent;
  lastReceiverStatusMs = millis();
}

void espNowInit() {
  if (esp_now_init() != ESP_OK) {
    netPrintln(F("[ESPNOW] init failed - wearable link disabled (dashboard still works)"));
    return;
  }
  esp_now_register_recv_cb(onReceiverStatusRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;              // 0 = follow the STA's current channel
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    netPrintln(F("[ESPNOW] failed to add broadcast peer"));
    return;
  }

  espNowReady = true;
  netPrint(F("[ESPNOW] Ready. Sender STA MAC: "));
  netPrintln(WiFi.macAddress());
}

void espNowSendTelemetry(float tiltAngleZ, float speedKmh, float distanceKm, bool fix,
                          bool wobble, bool kickflip, uint16_t kfCount) {
  if (!espNowReady) return;
  telemetry.tiltAngleZ = tiltAngleZ;
  telemetry.speedKmh = speedKmh;
  telemetry.distanceKm = distanceKm;
  // Stays 0 while unconnected, which the wearable reads as "no address to
  // show yet" rather than displaying 0.0.0.0 as if it were real.
  telemetry.boardIp = wifiConnected ? (uint32_t)WiFi.localIP() : 0;
  telemetry.boardChannel = (uint8_t)WiFi.channel();
  telemetry.gpsFix = fix;
  telemetry.wobbleActive = wobble;
  telemetry.kickflipActive = kickflip;
  telemetry.kickflipCount = kfCount;
  esp_now_send(broadcastAddress, (uint8_t*)&telemetry, sizeof(telemetry));
}

// ---------------------------------------------------------------------
// Local web server — the dashboard page itself (DASHBOARD_HTML, defined
// further down) plus the JSON API it talks to over same-origin fetch()
// calls. Replaces the Firebase live-telemetry PUT / command polling with
// direct request/response: a command takes effect the instant the
// request lands, no id-based dedup needed since there's no
// eventually-consistent write to race against.
// ---------------------------------------------------------------------
WebServer webServer(80);

// ---- Debug snapshot — the FSM internals behind GET /api/debug, which
// backs the dashboard's Debug dialog. Same figures the BLE build exposes
// on its own softAP debug page (handleWebRoot() in
// skateboard_xiao_c3_ble_stable.ino), which is what SkateGuardBLEApp's
// debug button loads in a WebView after switching WiFi networks. Here
// the page is already served by this board, so no network switching is
// needed — the dialog just polls this endpoint directly.
//
// Snapshotted once per loop() because wobbleAxisRate and friends are
// local to loop(); a handler firing between cycles has no other way to
// see them.
float liveGyroY = 0.0f;
float liveAccumDeg = 0.0f;
float liveLandConfirmSwingDeg = 0.0f;
unsigned long liveActiveWindowMs = 0;
unsigned long liveAbsoluteWindowMs = 0;
unsigned long liveCooldownWindowMs = 0;

// Loop-period stats, reset each time they're written to the stage log.
// motionAccumulatedDeg integrates as a rectangle (one gyro sample x the
// whole dt), so a stalled loop makes that integration coarse, and the
// 320-400 deg land window is the narrowest target in the FSM — it
// degrades first. If loopMax climbs well above the typical period,
// timing is the problem rather than the detection thresholds.
unsigned long loopMaxPeriodMs = 0;
unsigned long loopPeriodSumMs = 0;
unsigned long loopPeriodCount = 0;

bool dashboardWobbleLatched = false;
unsigned long dashboardWobbleShownAtMs = 0;
bool dashboardLandLatched = false;
unsigned long dashboardLandShownAtMs = 0;
bool dashboardBailLatched = false;
unsigned long dashboardBailShownAtMs = 0;
const unsigned long DASHBOARD_EVENT_DISPLAY_MS = 2000;

// Same field shape the dashboard's updateUi() reads. Built fresh on every
// GET /api/live — cheap (a handful of String concatenations from RAM),
// which is exactly why this doesn't need the MOTION_IDLE gating the old
// Firebase PUT needed: there's no blocking network call happening here,
// just building a small string in response to a request the phone
// already initiated.
String buildLiveJson() {
  unsigned long now = millis();
  if (dashboardWobbleLatched && now - dashboardWobbleShownAtMs >= DASHBOARD_EVENT_DISPLAY_MS) dashboardWobbleLatched = false;
  if (dashboardLandLatched && now - dashboardLandShownAtMs >= DASHBOARD_EVENT_DISPLAY_MS) dashboardLandLatched = false;
  if (dashboardBailLatched && now - dashboardBailShownAtMs >= DASHBOARD_EVENT_DISPLAY_MS) dashboardBailLatched = false;

  float distanceKm = totalDistanceMeters / 1000.0f;
  String body = "{";
  // Field-for-field identical to bleNotifyTelemetry()'s JSON in
  // skateboard_xiao_c3_ble_stable.ino, so the dashboard reads exactly what
  // SkateGuardBLEApp's Telemetry.fromJson() reads.
  body += "\"tiltAngleZ\":" + String(imuGetTiltAngleZ(), 2) + ",";
  body += "\"speedKmh\":" + String(gpsGetSpeedKmh(), 2) + ",";
  body += "\"distanceKm\":" + String(distanceKm, 3) + ",";
  body += "\"gpsFix\":" + String(gpsHasFix() ? "true" : "false") + ",";
  body += "\"gpsLat\":" + String(gpsHasLastPosition ? gpsLastLat : 0.0, 6) + ",";
  body += "\"gpsLng\":" + String(gpsHasLastPosition ? gpsLastLng : 0.0, 6) + ",";
  body += "\"wobbleActive\":" + String(dashboardWobbleLatched ? "true" : "false") + ",";
  body += "\"landActive\":" + String(dashboardLandLatched ? "true" : "false") + ",";
  body += "\"bailActive\":" + String(dashboardBailLatched ? "true" : "false") + ",";
  body += "\"kickflipCount\":" + String(kickflipCount) + ",";
  body += "\"bailCount\":" + String(bailCount) + ",";
  body += "\"batteryPercent\":" + String(batteryPercent) + ",";
  body += "\"batteryPinV\":" + String(lastPinVoltage, 3) + ",";
  body += "\"batteryVoltage\":" + String(lastBatteryVoltage, 3) + ",";

  // Same staleness handling as the BLE build: keep reporting the last
  // known value even once stale, so a momentary radio drop doesn't
  // collapse the wearable's battery bar to empty. hasReceiver stays false
  // until a wearable has EVER been heard, which is what greys the
  // indicator out on a board running solo.
  bool receiverBatteryFresh = (now - lastReceiverStatusMs) < RECEIVER_STATUS_TIMEOUT_MS;
  bool receiverEverSeen = (lastReceiverStatusMs != 0);
  body += "\"hasReceiver\":" + String(receiverEverSeen ? "true" : "false") + ",";
  body += "\"receiverBatteryPercent\":" + String(receiverEverSeen ? receiverBatteryPercent : 0) + ",";
  body += "\"receiverBatteryFresh\":" + String(receiverBatteryFresh ? "true" : "false") + ",";
  body += "\"sessionState\":\"" + String(sessionStateName()) + "\",";
  body += "\"sessionDistanceKm\":" + String(sessionLiveDistanceKm(distanceKm), 3) + ",";
  body += "\"sessionAvgSpeedKmh\":" + String(sessionLiveAvgSpeedKmh(), 2) + ",";
  body += "\"updatedAtMs\":" + String(now);
  body += "}";
  return body;
}

void applyCommand(const String& cmd) {
  if (cmd == "start") sessionStart();
  else if (cmd == "stop") sessionStop();
  else if (cmd == "save") sessionSave();
  else if (cmd == "discard") sessionDiscard();
  else if (cmd == "delete") sessionDeleteAll();
  else if (cmd.startsWith("delete:")) sessionDeleteIndex((uint16_t)cmd.substring(7).toInt());
  else netPrintln("[CMD] Unknown command: " + cmd);
}

void handleWebRoot() {
  webServer.send(200, "text/html", DASHBOARD_HTML);
}

// Web app manifest — this is what turns "Add to Home screen" into a real
// standalone app icon instead of a browser bookmark: tap it and the
// dashboard opens full-screen with no URL bar, so after a one-time setup
// there's no address to type or remember on that phone ever again.
//
// The icon is an inline SVG data URI rather than a separate PNG route, so
// there's no binary blob to embed in flash. Single quotes inside the SVG
// are deliberate — they keep it legal JSON and legal C++ with no escaping.
// '#' has to be percent-encoded as %23 inside a data URI.
void handleManifest() {
  String icon = "data:image/svg+xml,"
                "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 192 192'>"
                "<rect width='192' height='192' rx='42' fill='%230D0D0F'/>"
                "<ellipse cx='60' cy='143' rx='15' ry='11' fill='%232ECC71'/>"
                "<ellipse cx='132' cy='143' rx='15' ry='11' fill='%232ECC71'/>"
                "<path d='M26 104q70-24 140 0-70 27-140 0z' fill='%232ECC71'/>"
                "</svg>";

  String body = "{";
  body += "\"name\":\"SkateGuard\",";
  body += "\"short_name\":\"SkateGuard\",";
  body += "\"start_url\":\"/\",";
  body += "\"scope\":\"/\",";
  body += "\"display\":\"standalone\",";
  body += "\"orientation\":\"portrait\",";
  body += "\"background_color\":\"#0D0D0F\",";
  body += "\"theme_color\":\"#0D0D0F\",";
  body += "\"icons\":[{\"src\":\"" + icon + "\",\"sizes\":\"any\",\"type\":\"image/svg+xml\",\"purpose\":\"any\"}]";
  body += "}";
  webServer.send(200, "application/manifest+json", body);
}

void handleApiLive() {
  webServer.send(200, "application/json", buildLiveJson());
}

// Polled only while the dashboard's Debug dialog is open, which is why
// these fields aren't folded into /api/live — no reason to carry FSM
// internals in the 500ms telemetry poll every rider makes.
void handleApiDebug() {
  String body = "{";
  body += "\"stage\":\"" + String(motionStateName()) + "\",";
  body += "\"gyroY\":" + String(liveGyroY, 1) + ",";
  body += "\"accumDeg\":" + String(liveAccumDeg, 1) + ",";
  body += "\"landConfirmSwingDeg\":" + String(liveLandConfirmSwingDeg, 1) + ",";
  body += "\"activeWindowMs\":" + String(liveActiveWindowMs) + ",";
  body += "\"activeWindowMaxMs\":" + String(MOTION_WINDOW_MS) + ",";
  body += "\"absoluteWindowMs\":" + String(liveAbsoluteWindowMs) + ",";
  body += "\"absoluteWindowMaxMs\":" + String(MOTION_ABSOLUTE_TIMEOUT_MS) + ",";
  body += "\"cooldownWindowMs\":" + String(liveCooldownWindowMs) + ",";
  body += "\"cooldownWindowMaxMs\":" + String(MOTION_COOLDOWN_MS) + ",";
  body += "\"oscillations\":" + String(motionOscillationCount) + ",";
  body += "\"oscillationsRequired\":" + String(WOBBLE_OSCILLATIONS_REQUIRED) + ",";
  body += "\"loopMaxMs\":" + String(loopMaxPeriodMs) + ",";
  body += "\"loopAvgMs\":" + String(loopPeriodCount ? (float)loopPeriodSumMs / loopPeriodCount : 0.0f, 1) + ",";
  body += "\"kickflipCount\":" + String(kickflipCount) + ",";
  body += "\"bailCount\":" + String(bailCount) + ",";
  body += "\"batteryPercent\":" + String(batteryPercent) + ",";
  body += "\"batteryVoltage\":" + String(lastBatteryVoltage, 3) + ",";

  // ---- GPS acquisition diagnostics ----
  // Slow fixes are almost never a code problem, but they're impossible to
  // tell apart without these. The three cases they separate:
  //   charsProcessed ~0        -> nothing arriving: wiring/baud/power
  //   chars OK, satellites 0   -> receiver alive but sees no sky: antenna
  //                               orientation, indoors, or shielding
  //   satellites climbing, no  -> it IS working, just still downloading
  //   fix yet                     orbital data (see the header note on
  //                               cold starts and the backup battery)
  // failedChecksum rising alongside good chars points at a noisy or
  // marginal serial line rather than at the GPS itself.
  body += "\"gpsSatellites\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  body += "\"gpsHdop\":" + String(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 1) + ",";
  // age() reports ULONG_MAX until the first valid fix; -1 reads as "never"
  // on the dashboard instead of a nonsense 49-day age.
  body += "\"gpsFixAgeMs\":" + String(gps.location.isValid() ? (long)gps.location.age() : -1L) + ",";
  body += "\"gpsCharsProcessed\":" + String(gps.charsProcessed()) + ",";
  body += "\"gpsSentencesWithFix\":" + String(gps.sentencesWithFix()) + ",";
  body += "\"gpsFailedChecksum\":" + String(gps.failedChecksum()) + ",";

  body += "\"wifiRssi\":" + String(WiFi.RSSI()) + ",";
  body += "\"wifiChannel\":" + String(WiFi.channel()) + ",";
  body += "\"espNowReady\":" + String(espNowReady ? "true" : "false") + ",";
  body += "\"receiverLinked\":" + String((lastReceiverStatusMs != 0 &&
            millis() - lastReceiverStatusMs < RECEIVER_STATUS_TIMEOUT_MS) ? "true" : "false") + ",";
  body += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  body += "\"uptimeMs\":" + String(millis());
  body += "}";
  webServer.send(200, "application/json", body);
}

void handleApiCommand() {
  if (!webServer.hasArg("cmd")) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }
  String cmd = webServer.arg("cmd");
  netPrintln("[CMD] " + cmd);
  applyCommand(cmd);
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// JSON-escapes and returns { sessionsCsv, routesCsv } — same shape the
// dashboard's openSessionsDialog() already parses.
void handleApiSessions() {
  String sc = readSessionsCsv();
  sc.replace("\\", "\\\\"); sc.replace("\"", "\\\""); sc.replace("\n", "\\n");
  String rc = readRoutesCsv();
  rc.replace("\\", "\\\\"); rc.replace("\"", "\\\""); rc.replace("\n", "\\n");
  webServer.send(200, "application/json", "{\"sessionsCsv\":\"" + sc + "\",\"routesCsv\":\"" + rc + "\"}");
}

void handleWebLog() {
  if (!LittleFS.exists(STAGE_LOG_CSV_PATH)) {
    webServer.send(404, "text/plain", "No stage_log.csv yet");
    return;
  }
  File f = LittleFS.open(STAGE_LOG_CSV_PATH, FILE_READ);
  webServer.streamFile(f, "text/csv");
  f.close();
}

void handleWebClear() {
  if (LittleFS.exists(STAGE_LOG_CSV_PATH)) LittleFS.remove(STAGE_LOG_CSV_PATH);
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void webServerInit() {
  webServer.on("/", handleWebRoot);
  webServer.on("/manifest.json", handleManifest);
  webServer.on("/api/live", handleApiLive);
  webServer.on("/api/debug", handleApiDebug);
  webServer.on("/api/command", handleApiCommand);
  webServer.on("/api/sessions", handleApiSessions);
  webServer.on("/log", handleWebLog);
  webServer.on("/clear", handleWebClear);
  webServer.begin();
  netPrintln(F("[WEB] Local server listening on port 80"));
}

// ---------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------
unsigned long lastStageLogWriteMs = 0;
bool wobblePendingSend = false;
bool kickflipPendingSend = false;

unsigned long lastEspNowSendMs = 0;
const unsigned long ESPNOW_SEND_INTERVAL_MS = 100;  // matches the BLE build's wearable cadence

void setup() {
  Serial.begin(115200);
  delay(500);
  imuInit();
  batteryInit();
  if (!LittleFS.begin(true)) {
    netPrintln(F("[FS] LittleFS mount failed"));
  }
  stageLogClearOnBoot();

  // Start GPS as early as possible. Do not send UBX configuration commands
  // here: the NEO-8M is already capable of streaming NMEA at 9600 baud, and
  // passive startup is safer across different M8 breakout variants.
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsBootMs = millis();
  netPrintln(F("[GPS] NEO-8M serial started at 9600 baud"));
  netPrintln(F("[GPS] Waiting for satellite fix..."));

  wifiInit();
  // esp_now_init() needs the WiFi driver started. The broadcast peer uses
  // channel 0 so it follows the hotspot channel once the STA connects.
  espNowInit();
  webServerInit();

  netPrintln(F("[BENCH] Local sender running (XIAO ESP32-C3): BNO055 trick detection -> local dashboard + ESP-NOW wearable"));
}

void loop() {
  // GPS must be serviced first. The UART can continue receiving while the
  // rest of the application is doing IMU, WiFi, ESP-NOW and filesystem work.
  gpsProcessSerial();

  // Loop-period instrumentation — see loopMaxPeriodMs's declaration for
  // why a stalled loop shows up as missed lands rather than as anything
  // obviously timing-shaped.
  {
    static unsigned long loopLastMs = 0;
    unsigned long loopNow = millis();
    if (loopLastMs != 0) {
      unsigned long period = loopNow - loopLastMs;
      if (period > loopMaxPeriodMs) loopMaxPeriodMs = period;
      loopPeriodSumMs += period;
      loopPeriodCount++;
    }
    loopLastMs = loopNow;
  }

  wifiUpdate();
  batteryUpdate();
  imuUpdate();

  imu::Vector<3> gyroRaw = readGyroRaw();
  float wobbleAxisRateRaw = gyroAxisValue(gyroRaw, WOBBLE_AXIS);
  float wobbleAxisRate = gyroYMedianFilter.apply(clampGyroRate(wobbleAxisRateRaw));

  motionFsmUpdate(wobbleAxisRate, gpsGetSpeedKmh());

  // Snapshot for GET /api/debug — these are loop() locals otherwise.
  liveGyroY = wobbleAxisRate;
  liveAccumDeg = motionAccumulatedDeg;
  liveLandConfirmSwingDeg = landConfirmAccumulatedDeg;
  liveActiveWindowMs = motionActiveElapsedMs;
  liveAbsoluteWindowMs = millis() - motionWindowStartMs;
  liveCooldownWindowMs = millis() - motionCooldownStartMs;

  if (wobbleEventPending) {
    wobbleEventPending = false;
    wobblePendingSend = true;
    latchedStageEvent = 1; stageEventLatchedAtMs = millis();
    // Clearing the other two latches is the "land and bail crash with each
    // other" fix from skateboard_xiao_c3_ble_stable.ino: each latch runs on
    // its own independent 2s window, so without this a land followed
    // shortly by a bail leaves both flags true and the UI silently shows
    // only whichever wins its fixed priority order.
    dashboardLandLatched = false;
    dashboardBailLatched = false;
    dashboardWobbleLatched = true; dashboardWobbleShownAtMs = millis();
    netPrint(F("[WOBBLE] Detected!  Reason: direction reversed after "));
    netPrint(lastWobbleAccumulatedDeg, 1);
    netPrint(F(" deg of rotation while at "));
    netPrint(lastWobbleSpeedKmh, 1);
    netPrintln(F(" km/h"));
  }

  if (kickflipEventPending) {
    kickflipEventPending = false;
    kickflipPendingSend = true;
    latchedStageEvent = 2; stageEventLatchedAtMs = millis();
    dashboardWobbleLatched = false;
    dashboardBailLatched = false;
    dashboardLandLatched = true; dashboardLandShownAtMs = millis();
    netPrint(F("[LAND] Kickflip confirmed! Count: "));
    netPrint(kickflipCount);
    netPrint(F("  Reason: rotated "));
    netPrint(lastLandRotationDeg, 1);
    netPrint(F(" deg, stayed within "));
    netPrint(fabsf(lastLandConfirmSwingDeg), 1);
    netPrint(F(" deg swing during confirm window"));
    netPrintln();
  }

  if (bailEventPending) {
    bailEventPending = false;
    latchedStageEvent = 3; stageEventLatchedAtMs = millis();
    dashboardWobbleLatched = false;
    dashboardLandLatched = false;
    dashboardBailLatched = true; dashboardBailShownAtMs = millis();
    netPrint(F("[BAIL] Attempt failed! Count: "));
    netPrint(bailCount);
    netPrint(F("  Reason: "));
    netPrintln(bailReasonName());
  }

  // Drain anything that arrived while the rest of loop() was running.
  gpsProcessSerial();
  gpsUpdateDistance();
  gpsPrintStatus();

  unsigned long now = millis();
  sessionUpdate(gpsGetSpeedKmh());
  sessionUpdateRoute();

  // Wearable broadcast — same 100ms cadence as the BLE build, which is
  // what the watch's display timing was tuned against. wobble/kickflip
  // ride along as one-shot flags, cleared right after the send so the
  // wearable sees each event exactly once.
  if (now - lastEspNowSendMs >= ESPNOW_SEND_INTERVAL_MS) {
    lastEspNowSendMs = now;
    espNowSendTelemetry(imuGetTiltAngleZ(), gpsGetSpeedKmh(),
                        totalDistanceMeters / 1000.0f, gpsHasFix(),
                        wobblePendingSend, kickflipPendingSend, kickflipCount);
    wobblePendingSend = false;
    kickflipPendingSend = false;

    if (latchedStageEvent != 0 && now - stageEventLatchedAtMs >= STAGE_EVENT_LATCH_MS) {
      latchedStageEvent = 0;
    }
    if (espNowReady) {
      stagePacket.stage = (uint8_t)motionState;
      stagePacket.event = latchedStageEvent;
      stagePacket.kickflipCount = kickflipCount;
      stagePacket.bailCount = bailCount;
      esp_now_send(broadcastAddress, (uint8_t*)&stagePacket, sizeof(stagePacket));
    }
  }

  if (now - lastStageLogWriteMs >= STAGE_LOG_WRITE_INTERVAL_MS) {
    lastStageLogWriteMs = now;

    // Periodic diagnostic line — this file only printed on wobble/land/bail
    // events, so "nothing's counting" was undebuggable: no way to tell
    // whether the gyro is reading real values, whether MOTION_TRIGGER_DPS
    // is ever being crossed, or whether the FSM is stuck somewhere. This
    // mirrors the "stage:" line from motion_stage_sender_test.ino, printed
    // unconditionally (not gated on WiFi/Firebase) so it's visible on
    // Serial regardless of network state.
    netPrint("stage:"); netPrint(motionStateName());
    netPrint("  gyroY:"); netPrint(wobbleAxisRate, 1);
    netPrint("  accumDeg:"); netPrintln(motionAccumulatedDeg, 1);

    stageLogAppend(now, motionStateName(), wobbleAxisRate, motionAccumulatedDeg,
                   landConfirmAccumulatedDeg, latchedStageEvent, kickflipCount, bailCount,
                   motionActiveElapsedMs, now - motionWindowStartMs,
                   now - motionCooldownStartMs, motionOscillationCount);

    // Reset the loop-period window alongside the log write, so loopMax
    // always means "worst stall since the last logged row" rather than
    // "worst stall since boot" (which would latch high forever after one
    // unlucky flash write and stop being useful).
    loopMaxPeriodMs = 0;
    loopPeriodSumMs = 0;
    loopPeriodCount = 0;
  }

  // Handling an already-open local socket and building a small JSON
  // string from RAM is microseconds, not the 100ms-1s+ an HTTPS call
  // could take — unlike the old Firebase push/poll, this doesn't need to
  // be gated on MOTION_IDLE or WiFi status (WebServer itself is a no-op
  // when nothing's connected).
  webServer.handleClient();
}
