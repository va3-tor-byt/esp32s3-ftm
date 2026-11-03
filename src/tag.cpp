// FTM tag on the 1.14" TFT board: ranges to FTM_ANCHOR_1..3 without
// connecting, smooths each distance, trilaterates (x, y), and shows it on
// the screen and over Serial.
//
// Built-in calibration:
//   - Hold BOOT (or send 'c' over Serial) to start the calibration wizard.
//   - For each anchor: place the tag CAL_DIST_CM away, press BOOT.
//   - Offsets are saved to flash (NVS) and survive reboots/re-flashing.
//   - Serial 'p' prints offsets, 'x' clears them.
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Preferences.h>
#include <algorithm>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "esp_wifi.h"

// ---- Display pins (Feather S3 TFT layout; fallbacks if the variant lacks them)
#ifndef TFT_CS
#define TFT_CS 7
#endif
#ifndef TFT_DC
#define TFT_DC 39
#endif
#ifndef TFT_RST
#define TFT_RST 40
#endif
#ifndef TFT_BACKLITE
#define TFT_BACKLITE 45
#endif
#ifndef TFT_I2C_POWER
#define TFT_I2C_POWER 21
#endif
const int BTN = 0;   // BOOT button, active LOW

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;

// ---- Setup you edit -------------------------------------------------------
// Anchor positions in metres, indexed by anchor ID (index 0 unused).
struct Pt { float x, y; };
const Pt ANCHOR_POS[4] = { {0, 0}, {0.0, 0.0}, {4.0, 0.0}, {0.0, 3.0} };

const int32_t CAL_DIST_CM = 200;  // tag-to-anchor distance during calibration
const int     CAL_SAMPLES = 40;   // readings per anchor

const uint8_t  FRAME_COUNT  = 16;   // 0, 16, 24, 32, 64
const uint16_t BURST_PERIOD = 2;    // x100 ms
const float    ALPHA        = 0.3;  // smoothing (0..1, lower = smoother)
// ---------------------------------------------------------------------------

int32_t offsetCm[4] = {0};  // loaded from NVS

struct Anchor { int id; uint8_t bssid[6]; uint8_t channel; };
Anchor anchors[3];
int anchorCount = 0;

float dist[4]  = {0};
bool  haveD[4] = {false};

SemaphoreHandle_t ftmDone;
volatile bool ftmOk = false;
volatile int32_t lastDistCm = 0;

// ---------------- Offsets in flash ----------------
void loadOffsets() {
  prefs.begin("ftm", true);
  for (int id = 1; id <= 3; id++) {
    char k[4]; snprintf(k, sizeof k, "o%d", id);
    offsetCm[id] = prefs.getInt(k, 0);
  }
  prefs.end();
}
void saveOffset(int id, int32_t v) {
  prefs.begin("ftm", false);
  char k[4]; snprintf(k, sizeof k, "o%d", id);
  prefs.putInt(k, v);
  prefs.end();
}
void clearOffsets() {
  prefs.begin("ftm", false); prefs.clear(); prefs.end();
  for (int id = 1; id <= 3; id++) offsetCm[id] = 0;
}
void printOffsets() {
  Serial.printf("Offsets (cm): A1=%ld A2=%ld A3=%ld\n",
    (long)offsetCm[1], (long)offsetCm[2], (long)offsetCm[3]);
}

// ---------------- Screen helpers ----------------
void showMsg(const char *l1, const char *l2 = "", const char *l3 = "", const char *l4 = "") {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  const char *lines[4] = {l1, l2, l3, l4};
  for (int i = 0; i < 4; i++) { tft.setCursor(4, 6 + i * 30); tft.print(lines[i]); }
}

bool buttonHeld() { return digitalRead(BTN) == LOW; }
void waitRelease() { while (buttonHeld()) delay(10); delay(30); }
void waitPress()   { waitRelease(); while (!buttonHeld()) delay(10); waitRelease(); }

// ---------------- FTM ----------------
void onFtmReport(arduino_event_t *event) {
  wifi_event_ftm_report_t *r = &event->event_info.wifi_ftm_report;
  ftmOk = (r->status == FTM_STATUS_SUCCESS);
  if (ftmOk) lastDistCm = r->dist_est;
  if (r->ftm_report_data) free(r->ftm_report_data);
  xSemaphoreGive(ftmDone);
}

void scanForAnchors() {
  anchorCount = 0;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n && anchorCount < 3; i++) {
    String ssid = WiFi.SSID(i);
    if (!ssid.startsWith("FTM_ANCHOR_")) continue;
    int id = ssid.substring(11).toInt();
    if (id < 1 || id > 3) continue;
    Anchor &a = anchors[anchorCount++];
    a.id = id;
    memcpy(a.bssid, WiFi.BSSID(i), 6);
    a.channel = WiFi.channel(i);
    Serial.printf("Found A%d %s ch%d RSSI %d\n", id,
                  WiFi.BSSIDstr(i).c_str(), a.channel, WiFi.RSSI(i));
  }
  WiFi.scanDelete();
}

const Anchor *findAnchor(int id) {
  for (int i = 0; i < anchorCount; i++) if (anchors[i].id == id) return &anchors[i];
  return nullptr;
}

bool rangeRawCm(const Anchor &a, int32_t &cm) {
  wifi_ftm_initiator_cfg_t cfg = {};
  memcpy(cfg.resp_mac, a.bssid, 6);
  cfg.channel = a.channel;
  cfg.frm_count = FRAME_COUNT;
  cfg.burst_period = BURST_PERIOD;
  if (esp_wifi_ftm_initiate_session(&cfg) != ESP_OK) return false;
  if (xSemaphoreTake(ftmDone, pdMS_TO_TICKS(5000)) != pdTRUE) return false;
  if (!ftmOk) return false;
  cm = lastDistCm;
  return true;
}

bool rangeTo(const Anchor &a, float &m) {
  int32_t cm;
  if (!rangeRawCm(a, cm)) return false;
  m = (cm - offsetCm[a.id]) / 100.0f;
  return true;
}

// ---------------- Calibration wizard ----------------
void calibrate() {
  char l1[24], l2[24], l3[24];
  if (anchorCount < 3) { showMsg("Scanning..."); scanForAnchors(); }

  for (int id = 1; id <= 3; id++) {
    const Anchor *a = findAnchor(id);
    if (!a) {
      snprintf(l1, sizeof l1, "A%d not found", id);
      showMsg(l1, "skipping");
      delay(2000);
      continue;
    }

    snprintf(l1, sizeof l1, "Calibrate A%d", id);
    snprintf(l2, sizeof l2, "Tag at %.2f m", CAL_DIST_CM / 100.0);
    showMsg(l1, l2, "then press BOOT");
    Serial.printf("Place tag %.2f m from A%d, press BOOT\n", CAL_DIST_CM / 100.0, id);
    waitPress();

    int32_t s[CAL_SAMPLES];
    int n = 0, tries = 0;
    snprintf(l1, sizeof l1, "Measuring A%d", id);
    showMsg(l1, "keep still...");
    while (n < CAL_SAMPLES && tries < CAL_SAMPLES * 3) {
      tries++;
      int32_t cm;
      if (rangeRawCm(*a, cm)) s[n++] = cm;
      tft.setCursor(4, 66);
      snprintf(l3, sizeof l3, "%d/%d   ", n, CAL_SAMPLES);
      tft.print(l3);
    }

    if (n < 10) {
      snprintf(l1, sizeof l1, "A%d failed", id);
      showMsg(l1, "too few readings", "press BOOT");
      waitPress();
      continue;
    }

    std::sort(s, s + n);
    int32_t median = s[n / 2];
    int32_t spread = s[(3 * n) / 4] - s[n / 4];  // interquartile range
    int32_t off = median - CAL_DIST_CM;
    offsetCm[id] = off;
    saveOffset(id, off);

    Serial.printf("A%d: median %ld cm, IQR %ld cm -> offset %ld cm\n",
                  id, (long)median, (long)spread, (long)off);
    snprintf(l1, sizeof l1, "A%d offset %ldcm", id, (long)off);
    snprintf(l2, sizeof l2, "spread %ld cm", (long)spread);
    showMsg(l1, l2, spread > 50 ? "noisy - redo?" : "looks good", "press BOOT");
    waitPress();
  }

  for (int id = 1; id <= 3; id++) haveD[id] = false;  // restart smoothing
  printOffsets();
  tft.fillScreen(ST77XX_BLACK);
}

// ---------------- Positioning ----------------
bool trilaterate(float &x, float &y) {
  if (!(haveD[1] && haveD[2] && haveD[3])) return false;
  Pt p1 = ANCHOR_POS[1], p2 = ANCHOR_POS[2], p3 = ANCHOR_POS[3];
  float r1 = dist[1], r2 = dist[2], r3 = dist[3];
  float A = 2 * (p2.x - p1.x), B = 2 * (p2.y - p1.y);
  float C = r1*r1 - r2*r2 - p1.x*p1.x + p2.x*p2.x - p1.y*p1.y + p2.y*p2.y;
  float D = 2 * (p3.x - p1.x), E = 2 * (p3.y - p1.y);
  float F = r1*r1 - r3*r3 - p1.x*p1.x + p3.x*p3.x - p1.y*p1.y + p3.y*p3.y;
  float det = A * E - B * D;
  if (fabsf(det) < 1e-6) return false;
  x = (C * E - B * F) / det;
  y = (A * F - C * D) / det;
  return true;
}

void drawScreen(bool havePos, float x, float y) {
  char line[24];
  tft.setTextSize(2);
  for (int id = 1; id <= 3; id++) {
    tft.setCursor(4, 4 + (id - 1) * 22);
    tft.setTextColor(haveD[id] ? ST77XX_WHITE : ST77XX_RED, ST77XX_BLACK);
    if (haveD[id]) snprintf(line, sizeof line, "A%d %6.2f m   ", id, dist[id]);
    else           snprintf(line, sizeof line, "A%d   --       ", id);
    tft.print(line);
  }
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(4, 74);
  if (havePos) snprintf(line, sizeof line, "x%5.2f  ", x); else snprintf(line, sizeof line, "x  --   ");
  tft.print(line);
  tft.setCursor(4, 104);
  if (havePos) snprintf(line, sizeof line, "y%5.2f  ", y); else snprintf(line, sizeof line, "y  --   ");
  tft.print(line);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') calibrate();
    else if (c == 'p') printOffsets();
    else if (c == 'x') { clearOffsets(); Serial.println("Offsets cleared"); }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN, INPUT_PULLUP);

  pinMode(TFT_I2C_POWER, OUTPUT); digitalWrite(TFT_I2C_POWER, HIGH);
  pinMode(TFT_BACKLITE, OUTPUT);  digitalWrite(TFT_BACKLITE, HIGH);
  delay(10);
  tft.init(135, 240);
  tft.setRotation(3);
  showMsg("Scanning...", "", "Hold BOOT to", "calibrate");

  loadOffsets();
  printOffsets();

  ftmDone = xSemaphoreCreateBinary();
  WiFi.onEvent(onFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  scanForAnchors();
  tft.fillScreen(ST77XX_BLACK);
}

void loop() {
  handleSerial();
  if (buttonHeld()) { waitRelease(); calibrate(); return; }
  if (anchorCount < 3) scanForAnchors();

  for (int i = 0; i < anchorCount; i++) {
    if (buttonHeld()) return;  // let the top of loop() catch it
    float m;
    int id = anchors[i].id;
    if (rangeTo(anchors[i], m)) {
      dist[id] = haveD[id] ? ALPHA * m + (1 - ALPHA) * dist[id] : m;
      haveD[id] = true;
      Serial.printf("A%d %.2f m (smoothed %.2f)\n", id, m, dist[id]);
    }
  }

  float x, y;
  bool ok = trilaterate(x, y);
  if (ok) Serial.printf("POS x=%.2f y=%.2f\n", x, y);
  Serial.println("---");
  drawScreen(ok, x, y);
  delay(200);
}
