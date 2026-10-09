/*
  SkateGuard — wearable receiver (Seeed XIAO ESP32-C3)

  Wrist unit for skateguard_local_sender_2.ino. Shows live speed, link
  state, battery and the dashboard address, and buzzes on a speed wobble.

  Channel hunting: ESP-NOW only works between radios on the same 2.4GHz
  channel, and the board's channel is whatever the rider's hotspot picked
  — unknown at compile time and able to change. The board can't move
  without dropping the dashboard, so this unit adapts: it hops channels
  1..13, dwelling CHANNEL_DWELL_MS on each, until telemetry arrives, then
  locks on. Silence for LINK_LOST_MS starts the hunt again. A full sweep
  takes about 4s; the OLED shows "SCAN ch N" meanwhile.

  The ESP32-C3 is 2.4GHz only. A hotspot set to 5GHz is one the board
  cannot join in the first place.

  Pins:
    OLED SDA         -> D4 / GPIO6
    OLED SCL         -> D5 / GPIO7
    Vibration motor  -> D1 / GPIO3   (via transistor/MOSFET + flyback diode)
    Battery ADC      -> D0 / GPIO2   (divider R1 = 30k / R2 = 10k)
    LED              -> D8 / GPIO8
*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>

// Pin map
#define OLED_SDA 6
#define OLED_SCL 7
#define LED_PIN 8
#define MOTOR_PIN 3
#define BATTERY_ADC_PIN 2

// 0.91" 128x32 OLED — different constructor/geometry than a 128x64 panel.
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Byte-identical to the sender's TelemetryPacket. Do not reorder or
// resize: the length check in onDataRecv() is what tells this apart from
// the sender's other broadcasts.
typedef struct {
  float    tiltAngleZ;      // degrees, fused BNO055 Euler Z — DISPLAY ONLY
  float    speedKmh;
  float    distanceKm;      // total distance traveled this session
  uint32_t boardIp;         // board's dashboard address, 0 until it's on WiFi
  bool     gpsFix;
  bool     wobbleActive;    // true for one packet when a wobble is confirmed
  bool     kickflipActive;  // true for one packet when a kickflip is confirmed
  uint16_t kickflipCount;   // running total this session
  uint8_t  boardChannel;    // channel the board is really on — see onDataRecv
} TelemetryPacket;

TelemetryPacket latest = {0.0f, 0.0f, 0.0f, 0, false, false, false, 0, 0};
volatile unsigned long lastPacketMs = 0;
const unsigned long LINK_TIMEOUT_MS = 1000;   // no packet this long -> "NO SIGNAL" on screen

// Dashboard address readout. The board's address changes with the phone,
// and without this the only way to learn the new one is Serial Monitor.
// Shown briefly after the link comes up, and only while stopped.
const unsigned long IP_DISPLAY_MS = 20000;
unsigned long linkEstablishedAtMs = 0;
uint32_t lastShownBoardIp = 0;

// Channel discovery
const uint8_t CHANNEL_MIN = 1;
const uint8_t CHANNEL_MAX = 13;

// Sender broadcasts every 100ms, so ~300ms gives about three chances to
// hear it per channel before moving on — long enough to be reliable
// through a little radio contention, short enough that a full 13-channel
// sweep still finishes in ~4 seconds.
const unsigned long CHANNEL_DWELL_MS = 300;

// Deliberately much longer than LINK_TIMEOUT_MS: the screen should say
// "NO SIGNAL" after 1s of silence, but tearing down a good channel lock
// and re-hunting is expensive, so only do that after a real outage.
const unsigned long LINK_LOST_MS = 5000;

uint8_t currentChannel = CHANNEL_MIN;
bool channelLocked = false;
unsigned long lastChannelHopMs = 0;

// onDataRecv() runs in the WiFi task, where Serial writes can block long
// enough to make the radio miss the packets right behind the one being
// logged — the exact failure the logging was meant to help diagnose.
// So the callback only records what to say, and loop() says it.
volatile uint8_t pendingChannelLog = 0;
volatile uint32_t pendingIpLog = 0;
volatile bool pendingWobbleLog = false;

void drainPendingLogs() {
  if (pendingWobbleLog) {
    pendingWobbleLog = false;
    Serial.println(F("[WOBBLE] Received from sender!"));
  }
  if (pendingChannelLog != 0) {
    uint8_t ch = pendingChannelLog;
    pendingChannelLog = 0;
    Serial.print(F("[ESPNOW] Locked onto board on channel "));
    Serial.println(ch);
  }
  if (pendingIpLog != 0) {
    uint32_t ip = pendingIpLog;
    pendingIpLog = 0;
    Serial.print(F("[LINK] Board dashboard at http://"));
    Serial.print(IPAddress(ip));
    Serial.println(F("/"));
  }
}

void setRadioChannel(uint8_t channel) {
  // Promiscuous mode toggled around the set — on a STA that isn't
  // associated to an AP this is the supported way to force a channel.
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

// Battery — resistor-divider into an ADC1 pin
const float BATTERY_DIVIDER_R1 = 30000.0f;  // ohms, Vbat side
const float BATTERY_DIVIDER_R2 = 10000.0f;  // ohms, GND side

const unsigned long BATTERY_READ_INTERVAL_MS = 2000;
unsigned long lastBatteryReadMs = 0;

uint8_t batteryPercent = 0;

// 1S LiPo discharge curve, roughly. Piecewise-linear interpolation
// between these voltage/percent points.
const float BATT_CURVE_V[] = {3.30f, 3.45f, 3.68f, 3.74f, 3.77f, 3.79f, 3.82f, 3.87f, 3.92f, 3.98f, 4.06f, 4.20f};
const uint8_t BATT_CURVE_PCT[] = {0,     0,     10,    20,    30,    40,    50,    60,    70,    80,    90,    100};
const uint8_t BATT_CURVE_LEN = sizeof(BATT_CURVE_V) / sizeof(BATT_CURVE_V[0]);

float readBatteryVoltage() {
  // Average several samples — a single ADC read is noisy enough to swing
  // the reported percent by 10%+ between consecutive reads.
  const uint8_t SAMPLES = 16;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    sum += analogReadMilliVolts(BATTERY_ADC_PIN);
  }
  float mv = sum / (float)SAMPLES;
  return (mv / 1000.0f) * (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2;
}

uint8_t batteryVoltageToPercent(float voltage) {
  if (voltage <= BATT_CURVE_V[0]) return BATT_CURVE_PCT[0];
  if (voltage >= BATT_CURVE_V[BATT_CURVE_LEN - 1]) return BATT_CURVE_PCT[BATT_CURVE_LEN - 1];

  for (uint8_t i = 0; i < BATT_CURVE_LEN - 1; i++) {
    if (voltage >= BATT_CURVE_V[i] && voltage <= BATT_CURVE_V[i + 1]) {
      float span = BATT_CURVE_V[i + 1] - BATT_CURVE_V[i];
      float frac = (voltage - BATT_CURVE_V[i]) / span;
      return BATT_CURVE_PCT[i] + (uint8_t)(frac * (BATT_CURVE_PCT[i + 1] - BATT_CURVE_PCT[i]));
    }
  }
  return 0;
}

void batteryUpdate() {
  unsigned long now = millis();
  if (now - lastBatteryReadMs < BATTERY_READ_INTERVAL_MS) return;
  lastBatteryReadMs = now;

  float voltage = readBatteryVoltage();
  batteryPercent = batteryVoltageToPercent(voltage);

  Serial.print(F("[BATT] adcV:")); Serial.print(voltage, 3);
  Serial.print(F("  pct:")); Serial.println(batteryPercent);
}

// Reverse channel: broadcast our own battery status to the board, which
// surfaces it in the web dashboard's second battery bar (the
// receiverBatteryPercent / receiverBatteryFresh fields in /api/live).
typedef struct {
  uint8_t batteryPercent;
} ReceiverStatusPacket;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
ReceiverStatusPacket receiverStatus;
unsigned long lastStatusSendMs = 0;
const unsigned long STATUS_SEND_INTERVAL_MS = 1000;  // sent often since some broadcast packets get lost to radio contention

void onStatusSendResult(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Only interesting while locked — during hunting we're transmitting on
  // channels nobody is listening on, so failures there are expected.
  if (!channelLocked) return;
  Serial.print(F("[BATT] link-layer delivery: "));
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ACKED" : "NO ACK / FAILED");
}

// Display
void drawBatteryIcon(int x, int y, uint8_t percent, int w = 24, int h = 12) {
  int nubW = max(2, w / 9);   // terminal nub scales roughly with body size
  int nubH = h / 3;

  display.drawFrame(x, y, w, h);
  display.drawBox(x + w, y + (h - nubH) / 2, nubW, nubH);

  int innerW = w - 4;  // 2px margin each side
  int innerH = h - 4;
  int fillWidth = map(constrain(percent, 0, 100), 0, 100, 0, innerW);
  if (fillWidth > 0) {
    display.drawBox(x + 2, y + 2, fillWidth, innerH);
  }
}

// Latch the wobble flag briefly on-screen even though the source event
// is momentary (one packet) — avoids a flash too fast to read.
unsigned long wobbleFlagShownAtMs = 0;
const unsigned long WOBBLE_FLAG_DISPLAY_MS = 500;
bool wobbleFlagLatched = false;

// LED + vibration motor "haptic" feedback — non-blocking blink on wobble
const unsigned long LED_BLINK_ON_MS  = 100;
const unsigned long LED_BLINK_OFF_MS = 100;
const uint8_t       LED_BLINK_COUNT  = 3;

bool ledBlinking = false;
uint8_t ledBlinksRemaining = 0;
bool ledCurrentlyOn = false;
unsigned long ledLastToggleMs = 0;

// How long after the last received wobble to still consider it "ongoing".
// Sender's own cooldown means back-to-back wobbles land roughly 1.1s
// apart, so this window needs to comfortably span that gap.
const unsigned long WOBBLE_ACTIVE_WINDOW_MS  = 1500;
const unsigned long LED_PERIODIC_INTERVAL_MS = 1000;
unsigned long wobbleLastEventMs = 0;
unsigned long ledLastPeriodicMs = 0;

// LED polarity varies by board — flip these two if your LED behaves
// backwards (on when it should be off).
void ledInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
}

void ledStartBlinkBurst() {
  ledBlinking = true;
  ledBlinksRemaining = LED_BLINK_COUNT;
  ledCurrentlyOn = true;
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(MOTOR_PIN, HIGH);
  ledLastToggleMs = millis();
}

void ledUpdate() {
  if (!ledBlinking) return;

  unsigned long now = millis();
  unsigned long interval = ledCurrentlyOn ? LED_BLINK_ON_MS : LED_BLINK_OFF_MS;

  if (now - ledLastToggleMs >= interval) {
    ledCurrentlyOn = !ledCurrentlyOn;
    digitalWrite(LED_PIN, ledCurrentlyOn ? HIGH : LOW);
    digitalWrite(MOTOR_PIN, ledCurrentlyOn ? HIGH : LOW);
    ledLastToggleMs = now;

    if (!ledCurrentlyOn) {
      ledBlinksRemaining--;
      if (ledBlinksRemaining == 0) {
        ledBlinking = false;
        digitalWrite(LED_PIN, LOW);
        digitalWrite(MOTOR_PIN, LOW);
      }
    }
  }
}

// ESP-NOW
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // The sender also broadcasts a 6-byte StagePacket; the size check is
  // what keeps us from parsing it as telemetry.
  if (len != sizeof(TelemetryPacket)) return;
  memcpy(&latest, incomingData, sizeof(latest));

  // Restart the address readout whenever the link is newly up, or the
  // board reports a different address than last time (you switched
  // phones, or it re-pinned itself after a reconnect) — those are exactly
  // the moments the address you had written down went stale.
  // Deliberately LINK_LOST_MS, not the twitchier LINK_TIMEOUT_MS the
  // display uses: a one-second gap is a blip, and restarting the address
  // readout on every blip is what made the screen flip back and forth.
  // Only a real outage counts as "new link worth re-announcing".
  bool linkWasDown = (millis() - lastPacketMs) > LINK_LOST_MS;
  if (latest.boardIp != 0 && (linkWasDown || latest.boardIp != lastShownBoardIp)) {
    linkEstablishedAtMs = millis();
    lastShownBoardIp = latest.boardIp;
    pendingIpLog = latest.boardIp;   // printed from loop(), not here
  }

  lastPacketMs = millis();

  // Lock onto the channel the board SAYS it's on, not the one we happened
  // to hear it on. Adjacent 2.4GHz channels overlap heavily, so hearing a
  // packet here doesn't mean we're tuned correctly — and a 1-2 channel
  // error still passes the occasional packet, which presents as a link
  // that keeps dropping rather than as an obvious tuning fault.
  //
  // Retune whenever the stated channel differs from ours, which also
  // covers the board moving (hotspot restart, different phone) without
  // waiting for the link-lost timeout to expire first.
  if (latest.boardChannel >= CHANNEL_MIN && latest.boardChannel <= CHANNEL_MAX &&
      latest.boardChannel != currentChannel) {
    currentChannel = latest.boardChannel;
    setRadioChannel(currentChannel);
    pendingChannelLog = currentChannel;   // printed from loop(), not here
  }

  if (!channelLocked) {
    channelLocked = true;
    pendingChannelLog = currentChannel;
  }

  if (latest.wobbleActive) {
    wobbleFlagLatched = true;
    wobbleFlagShownAtMs = millis();
    ledStartBlinkBurst();
    wobbleLastEventMs = millis();
    ledLastPeriodicMs = millis();
    pendingWobbleLog = true;   // printed from loop(), same reason as above
  }
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Disable WiFi modem sleep — otherwise the radio periodically powers
  // down between wake windows and misses incoming ESP-NOW packets during
  // that window, causing brief unexplained "NO SIGNAL" gaps. Matters even
  // more here than before: a nap during a 300ms dwell can make us skip
  // right past the correct channel while hunting.
  esp_wifi_set_ps(WIFI_PS_NONE);

  setRadioChannel(currentChannel);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ESPNOW] init failed"));
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onStatusSendResult);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;              // 0 = whatever channel we're on right now
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("[ESPNOW] failed to add broadcast peer for status send"));
  }

  Serial.print(F("[ESPNOW] Receiver MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.println(F("[ESPNOW] Hunting for the board across channels 1-13..."));
}

// Advances the hunt, or drops a stale lock. Cheap enough to call every
// loop; all the real work is behind time checks.
void channelHuntUpdate() {
  unsigned long now = millis();

  if (channelLocked) {
    if (now - lastPacketMs > LINK_LOST_MS) {
      channelLocked = false;
      lastChannelHopMs = now;
      Serial.println(F("[ESPNOW] Link lost - resuming channel hunt"));
    }
    return;
  }

  if (now - lastChannelHopMs < CHANNEL_DWELL_MS) return;
  lastChannelHopMs = now;

  currentChannel++;
  if (currentChannel > CHANNEL_MAX) currentChannel = CHANNEL_MIN;
  setRadioChannel(currentChannel);
}

void renderDisplay() {
  bool linkAlive = (millis() - lastPacketMs) < LINK_TIMEOUT_MS;

  // Expire the latched wobble flag after its display window.
  if (wobbleFlagLatched && (millis() - wobbleFlagShownAtMs >= WOBBLE_FLAG_DISPLAY_MS)) {
    wobbleFlagLatched = false;
  }

  display.clearBuffer();

  if (!linkAlive) {
    // Distinguish "actively looking for the board" from "board found but
    // gone quiet" — otherwise a hunt in progress is indistinguishable
    // from a dead link, which is exactly the case where you want to know.
    display.setFont(u8g2_font_helvB10_tf);
    if (channelLocked) {
      display.drawStr(28, 21, "NO SIGNAL");
    } else {
      char buf[20];
      snprintf(buf, sizeof(buf), "SCAN ch%u", currentChannel);
      display.drawStr(24, 21, buf);
    }
  } else if (latest.boardIp != 0 &&
             latest.speedKmh < 3.0f &&
             millis() - linkEstablishedAtMs < IP_DISPLAY_MS) {
    // Dashboard address, shown for IP_DISPLAY_MS after the link comes up.
    // Gated on being stopped as well as on the timer: if you roll away
    // inside the window, live speed takes the screen back immediately.
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(0, 8, "Dashboard:");
    display.setFont(u8g2_font_6x10_tf);
    // 15 chars max ("255.255.255.255") at 6px = 90px, fits 128px wide.
    display.drawStr(0, 22, IPAddress(latest.boardIp).toString().c_str());

    // Countdown so it's obvious the screen is about to change back on its
    // own, rather than looking frozen.
    display.setFont(u8g2_font_5x8_tf);
    char secs[8];
    snprintf(secs, sizeof(secs), "%lus",
             (IP_DISPLAY_MS - (millis() - linkEstablishedAtMs)) / 1000);
    display.drawStr(108, 31, secs);
  } else {
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 8, "SkateGuard");

    if (latest.speedKmh < 3.0f) {
      // Same slot the speed text occupies, so the icon reads as replacing
      // it rather than as a small badge off to the side.
      drawBatteryIcon(34, 16, batteryPercent, 44, 14);
    } else {
      display.setFont(u8g2_font_fub14_tf);
      char buf[20];
      snprintf(buf, sizeof(buf), "%.1f", latest.speedKmh);

      if (latest.speedKmh < 10.0f) {
        display.drawStr(46, 30, buf);   // single digit, e.g. "5.3"
      } else {
        display.drawStr(34, 30, buf);   // double digit, e.g. "12.3"
      }
    }
  }

  display.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  ledInit();
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  batteryUpdate();

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_helvB10_tf);
  display.drawStr(10, 21, "Waiting...");
  display.sendBuffer();

  espNowInit();
  Serial.println(F("[BENCH] Local receiver running (XIAO ESP32-C3): channel-hunting ESP-NOW -> OLED + motor + battery"));
}

void loop() {
  drainPendingLogs();
  channelHuntUpdate();
  renderDisplay();

  // Keep re-blinking every second while wobbling continues; stops on its
  // own once no new wobble packet has arrived for WOBBLE_ACTIVE_WINDOW_MS.
  unsigned long now = millis();
  if (now - wobbleLastEventMs < WOBBLE_ACTIVE_WINDOW_MS &&
      now - ledLastPeriodicMs >= LED_PERIODIC_INTERVAL_MS) {
    ledStartBlinkBurst();
    ledLastPeriodicMs = now;
  }

  ledUpdate();
  batteryUpdate();

  // Only worth transmitting once we know we're on the board's channel —
  // while hunting, nobody is listening on whatever channel we're parked on.
  if (channelLocked && now - lastStatusSendMs >= STATUS_SEND_INTERVAL_MS) {
    lastStatusSendMs = now;
    receiverStatus.batteryPercent = batteryPercent;
    esp_err_t sendResult = esp_now_send(broadcastAddress, (uint8_t*)&receiverStatus, sizeof(receiverStatus));
    Serial.print(F("[BATT] sent pct:")); Serial.print(receiverStatus.batteryPercent);
    Serial.print(F("  result:")); Serial.println(sendResult == ESP_OK ? "OK" : "FAIL");
  }

  // During a hunt this is also the dwell timer's resolution, and the
  // "SCAN ch N" readout changes every CHANNEL_DWELL_MS.
  delay(50);
}
