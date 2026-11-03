// FTM anchor: a soft AP with the 802.11mc FTM responder enabled.
#include <Arduino.h>
#include <WiFi.h>

#ifndef ANCHOR_ID
#define ANCHOR_ID 1
#endif

const char *PASS = "ftm_anchor";   // >= 8 chars
const int CHANNEL = 1;

void setup() {
  Serial.begin(115200);
  delay(500);

  String ssid = "FTM_ANCHOR_" + String(ANCHOR_ID);

  // softAP(ssid, pass, channel, hidden, max_conn, ftm_responder)
  if (WiFi.softAP(ssid.c_str(), PASS, CHANNEL, 0, 4, true)) {
    Serial.printf("Anchor %d up: SSID=%s  BSSID=%s  ch=%d\n",
                  ANCHOR_ID, ssid.c_str(), WiFi.softAPmacAddress().c_str(), CHANNEL);
  } else {
    Serial.println("softAP failed");
  }
}

void loop() {
  delay(5000);
  Serial.printf("Anchor %d alive\n", ANCHOR_ID);
}
