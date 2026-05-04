/* ═══════════════════════════════════════════════════════════════════════════
   FOREST SENTINEL — RECEIVER / BASE STATION
   Board  : ESP32 DevKit V1
   Module : LoRa Ra-02 (SX1278, 433 MHz) — 3.3 V logic only!

   ── Wiring (VSPI) ───────────────────────────────────────────────────────
   NSS  → GPIO 5
   RST  → GPIO 14
   DIO0 → GPIO 4   ← interrupt pin (GPIO 2 avoided: boot / LED conflict)
   SCK  → GPIO 18
   MISO → GPIO 19
   MOSI → GPIO 23
   VCC  → 3.3 V    ← NEVER connect to 5 V
   GND  → GND

   ── Required libraries (install via Library Manager) ────────────────────
   • arduino-LoRa  (Sandeep Mistry)

   ── Dashboard connection ─────────────────────────────────────────────────
   Plug this receiver's USB cable into your PC.
   Open forest-sentinel-dashboard.html in Chrome or Edge (≥89).
   Click "CONNECT SERIAL" and select this device's COM port.
   Baud rate is 115200.

   Every received LoRa packet is printed as a single-line JSON object
   on Serial, e.g.:
     {"id":1,"s":0,"c":"noise","p":0.10,"t":28.5,"db":42.3,"pr":1013.2,"rssi":-72,"snr":8.5,"dup":0}

   The dashboard's readSerialLoop() picks up any line starting with '{'.
═══════════════════════════════════════════════════════════════════════════ */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ── PIN MAP ───────────────────────────────────────────────────────────────
#define LORA_NSS_PIN   5
#define LORA_RST_PIN  14
#define LORA_DIO0_PIN  4
// SCK=18, MISO=19, MOSI=23 are VSPI defaults on ESP32 — no explicit define needed

// ── LORA PARAMETERS ───────────────────────────────────────────────────────
// Must match transmitter exactly
#define LORA_FREQUENCY       433E6
#define LORA_SPREADING_FACTOR    7
#define LORA_BANDWIDTH       125E3
#define LORA_CODING_RATE         5

// ── DUPLICATE SUPPRESSION ─────────────────────────────────────────────────
// Tracks the last packet fingerprint per node (id + state) to flag duplicates.
// A "duplicate" means: same node, same state, arrived within DUP_WINDOW_MS.
#define MAX_NODES         8
#define DUP_WINDOW_MS  6000UL   // suppress repeats within 6 s (alert burst)

struct NodeTrack {
  uint8_t  id;
  uint8_t  state;
  uint32_t lastMs;
};

static NodeTrack nodeTrack[MAX_NODES] = {};

static int isDuplicate(uint8_t id, uint8_t state) {
  uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; ++i) {
    if (nodeTrack[i].id == id) {
      if (nodeTrack[i].state == state &&
          (now - nodeTrack[i].lastMs) < DUP_WINDOW_MS) {
        return 1;   // duplicate
      }
      // Update record
      nodeTrack[i].state  = state;
      nodeTrack[i].lastMs = now;
      return 0;
    }
  }
  // New node — find empty slot
  for (int i = 0; i < MAX_NODES; ++i) {
    if (nodeTrack[i].id == 0) {
      nodeTrack[i].id     = id;
      nodeTrack[i].state  = state;
      nodeTrack[i].lastMs = now;
      return 0;
    }
  }
  return 0;  // table full — don't suppress
}

// ═════════════════════════════════════════════════════════════════════════
//  PACKET PARSER
//  Extracts id and state from the raw JSON so we can add receiver-side
//  RSSI / SNR and the duplicate flag before forwarding to the dashboard.
// ═════════════════════════════════════════════════════════════════════════
/**
 * Very lightweight JSON field extractor — no heap allocation, no library.
 * Looks for "key":VALUE and returns the integer value, or -999 on failure.
 */
static int extractInt(const char* json, const char* key) {
  char search[32];
  snprintf(search, sizeof(search), "\"%s\":", key);
  const char* pos = strstr(json, search);
  if (!pos) return -999;
  pos += strlen(search);
  while (*pos == ' ') pos++;
  return atoi(pos);
}

// ═════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("[BOOT] Forest Sentinel BASE STATION");

  // VSPI uses default pins on ESP32; LoRa.setPins handles NSS/RST/DIO0
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[BOOT] LoRa INIT FAILED — check wiring!");
    while (true) { delay(1000); }
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth((long)LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.enableCrc();

  Serial.println("[BOOT] LoRa OK  @ 433 MHz  SF7  BW125 — listening...");
}

// ═════════════════════════════════════════════════════════════════════════
//  LOOP — poll for LoRa packets, enrich, print as JSON for the dashboard
// ═════════════════════════════════════════════════════════════════════════
void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize == 0) {
    delay(10);
    return;
  }

  // ── Read raw payload ────────────────────────────────────────────────
  char rawBuf[256];
  size_t idx = 0;
  while (LoRa.available() && idx < sizeof(rawBuf) - 1) {
    rawBuf[idx++] = (char)LoRa.read();
  }
  rawBuf[idx] = '\0';

  // Validate: must start with '{' to be our JSON
  if (rawBuf[0] != '{') {
    Serial.printf("[RX]  Ignored non-JSON packet: %s\n", rawBuf);
    return;
  }

  // ── Receiver-side metadata ──────────────────────────────────────────
  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();

  // ── Parse node id and state ─────────────────────────────────────────
  int nodeId    = extractInt(rawBuf, "id");
  int nodeState = extractInt(rawBuf, "s");

  if (nodeId <= 0) {
    Serial.println("[RX]  Malformed packet — no 'id' field");
    return;
  }

  // ── Duplicate detection ─────────────────────────────────────────────
  int dup = isDuplicate((uint8_t)nodeId, (uint8_t)nodeState);

  // ── Strip closing '}', append receiver fields, close ────────────────
  // rawBuf ends with '}'; we rewrite it to inject rssi/snr/dup/rxms
  size_t jsonLen = strlen(rawBuf);
  if (rawBuf[jsonLen - 1] == '}') {
    rawBuf[jsonLen - 1] = '\0';   // remove closing brace
  }

  // Build enriched output on Serial — dashboard reads this line
  // Format matches processPacket() expectations in the HTML
  Serial.printf(
    "%s,\"rssi\":%d,\"snr\":%.1f,\"dup\":%d,\"rxms\":%lu}\n",
    rawBuf, rssi, snr, dup, (unsigned long)millis()
  );

  // Local debug (NOT parsed by dashboard — on a separate debug line)
  if (dup) {
    Serial.printf("[RX]  Node %d — DUPLICATE suppressed\n", nodeId);
  } else {
    Serial.printf("[RX]  Node %d  RSSI=%d dBm  SNR=%.1f dB  state=%d\n",
                  nodeId, rssi, snr, nodeState);
  }
}

