/* ═══════════════════════════════════════════════════════════════════════════
   FOREST SENTINEL — TRANSMITTER / SENSOR NODE
   Board  : ESP32-S3 DevKitC-1
   IDE    : Arduino (esp32 board package ≥ 3.x)

   ── Wiring ──────────────────────────────────────────────────────────────
   INMP441 I²S Mic   WS  → GPIO 4   SD  → GPIO 5   SCK → GPIO 6
   BMP280  I²C       SDA → GPIO 18  SCL → GPIO 17
   SX1276  SPI(FSPI) NSS → GPIO 14  SCK → GPIO 13  MOSI→ GPIO 12
                     MISO→ GPIO 11  RST → GPIO 10  DIO0→ GPIO 9

   ── Required libraries (install via Library Manager) ────────────────────
   • arduino-LoRa  (Sandeep Mistry)
   • Adafruit BMP280 Library  (+ Adafruit Unified Sensor)
   • Edge Impulse Arduino SDK  (import the .zip via
       Sketch → Include Library → Add .ZIP Library…)
       The SDK provides: <forest_inferencing.h>

   ── Developer hand-off notes ────────────────────────────────────────────
   1. Replace NODE_ID with a unique integer per physical node.
   2. Tune DB_OFFSET_CORRECTION so that ambient reads ~40-50 dB.
   3. After flashing, open Serial Monitor @115200 and watch the JSON lines —
      they should match the dashboard's expected format exactly.
   4. CPU profiling: enable EI_CLASSIFIER_TIMING_ENABLED 1 in your model
      config to get per-stage timing in the Serial output.
═══════════════════════════════════════════════════════════════════════════ */

// ── NODE IDENTITY ─────────────────────────────────────────────────────────
#define NODE_ID   1          // ← CHANGE for each deployed unit

// ── EDGE IMPULSE SDK ──────────────────────────────────────────────────────
// Reduces RAM by ~10 KB — safe for this model; remove if you see accuracy drops
#define EIDSP_QUANTIZE_FILTERBANK   0
#include <forest_inferencing.h>    // Generated library from your EI project

// ── LIBRARIES ─────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <driver/i2s.h>          // ESP-IDF I2S (built-in)
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>

// ── PIN MAP ───────────────────────────────────────────────────────────────
// I2S — INMP441
#define I2S_WS_PIN     4
#define I2S_SD_PIN     6
#define I2S_SCK_PIN    5

// I2C — BMP280
#define BMP_SDA_PIN    18
#define BMP_SCL_PIN    17

// SPI (FSPI) — SX1276 LoRa
#define LORA_NSS_PIN   14
#define LORA_SCK_PIN   13
#define LORA_MOSI_PIN  12
#define LORA_MISO_PIN  11
#define LORA_RST_PIN   10
#define LORA_DIO0_PIN   9

// ── AUDIO PARAMETERS ──────────────────────────────────────────────────────
#define SAMPLE_RATE       16000          // Hz
#define I2S_DMA_BUF_COUNT      4
#define I2S_DMA_BUF_LEN    1024         // samples per DMA buffer
// Edge Impulse window size — matches EI_CLASSIFIER_RAW_SAMPLE_COUNT (16000)
#define EI_SAMPLE_COUNT   EI_CLASSIFIER_RAW_SAMPLE_COUNT
#define AUDIO_STACK_BYTES (32 * 1024)   // 32 KB — covers tensor arena

// ── AUDIO SIGNAL PROCESSING ───────────────────────────────────────────────
#define DB_OFFSET_CORRECTION  94.0f     // Calibrate: raise to lower readings
                                        // Lower to raise readings
                                        // Target: ambient ~40-50 dB

// ── INFERENCE THRESHOLDS ──────────────────────────────────────────────────
#define CONFIDENCE_THRESHOLD  0.85f
#define LOUDNESS_GATE_DB      65.0f

// ── LORA PARAMETERS ───────────────────────────────────────────────────────
#define LORA_FREQUENCY    433E6
#define LORA_TX_POWER        17         // dBm (max 20 for RA-02)
#define LORA_SPREADING_FACTOR 7         // SF7 — best throughput
#define LORA_BANDWIDTH    125E3
#define LORA_CODING_RATE    5           // 4/5

// ── TELEMETRY INTERVALS ───────────────────────────────────────────────────
#define HEARTBEAT_INTERVAL_MS  20000UL  // 20 s idle heartbeat
#define ALERT_INTERVAL_MS       5000UL  //  5 s alert burst
#define ALERT_BURST_COUNT           3   // redundant packets per alert event

// ── INTER-CORE COMMUNICATION ──────────────────────────────────────────────
// Packed result sent from Core 0 → Core 1 via FreeRTOS queue
typedef struct {
  uint8_t  state;           // 0 = IDLE, 1 = ALERT
  char     label[16];       // "chainsaw" | "noise"
  float    confidence;
  float    decibels;
} InferenceResult_t;

static QueueHandle_t resultQueue = nullptr;

// Shared BMP280 readings (Core 1 writes, Core 1 reads — no race)
static volatile float g_temperature = 0.0f;
static volatile float g_pressure    = 0.0f;

// ── OBJECTS ───────────────────────────────────────────────────────────────
static Adafruit_BMP280 bmp;

// ── PACKET COUNTER ────────────────────────────────────────────────────────
static volatile uint32_t g_packetCount = 0;

// ── EI AUDIO SIGNAL BRIDGE ────────────────────────────────────────────────
// Pointer set by AI_Task before each run_classifier() call so the
// static C-style callback can reach the current PCM frame.
static const int16_t* g_ei_pcm_buf = nullptr;

/**
 * Edge Impulse signal callback — converts int16 PCM → float on demand.
 * Called by run_classifier() internally during DSP feature extraction.
 */
static int ei_get_audio_signal_data(size_t offset, size_t length, float* out_ptr) {
    numpy::int16_to_float(g_ei_pcm_buf + offset, out_ptr, length);
    return 0;
}

// ═════════════════════════════════════════════════════════════════════════
//  UTILITY: RMS → dBSPL
// ═════════════════════════════════════════════════════════════════════════
/**
 * Converts a buffer of 16-bit PCM samples to an approximate dBSPL reading.
 *
 * The INMP441 has a sensitivity of -26 dBFS @ 94 dBSPL (1 Pa).
 * We normalise to full-scale then apply the sensitivity correction.
 *
 * DB_OFFSET_CORRECTION: adjust during calibration so a calibrated
 * 94 dBSPL source (or known environment) reads correctly.
 * Ambient room noise should read ~40-50 dB when correctly tuned.
 */
float computeDbSPL(const int16_t* samples, size_t count) {
  if (count == 0) return 0.0f;
  double sum = 0.0;
  for (size_t i = 0; i < count; ++i) {
    double s = (double)samples[i] / 32768.0;
    sum += s * s;
  }
  double rms = sqrt(sum / (double)count);
  if (rms < 1e-10) return 0.0f;
  // 20*log10(rms) gives dBFS; add correction to get dBSPL
  float dbfs = 20.0f * log10f((float)rms);
  return dbfs + DB_OFFSET_CORRECTION;
}

// ═════════════════════════════════════════════════════════════════════════
//  I2S INITIALISATION
// ═════════════════════════════════════════════════════════════════════════
static void i2s_init() {
  const i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,  // INMP441 sends 24-bit in 32-bit frame
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = I2S_DMA_BUF_COUNT,
    .dma_buf_len          = I2S_DMA_BUF_LEN,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0,
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK_PIN,
    .ws_io_num    = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD_PIN,
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));
}

// ═════════════════════════════════════════════════════════════════════════
//  CORE 0 — AI / INFERENCE TASK
// ═════════════════════════════════════════════════════════════════════════
/**
 * Pinned to Core 0, priority 2.
 * Stack: AUDIO_STACK_BYTES (≥30 KB) to hold the TFLite Micro tensor arena.
 *
 * Pipeline:
 *   1. Fill audio buffer from I2S DMA (16-bit PCM, 16 kHz).
 *   2. Compute dBSPL for loudness gate.
 *   3. Run Edge Impulse inference (MFE → 1D-CNN → int8 TFLite Micro).
 *   4. Apply confidence + loudness gates.
 *   5. Push result to resultQueue (Core 1 consumes).
 */
void AI_Task(void* param) {
  Serial.println("[AI]  Task started on core " + String(xPortGetCoreID()));

  // ── Audio sample buffer ──────────────────────────────────────────────
  // 32-bit DMA words from I2S (INMP441 shifts 24-bit data into MSBs)
  static int32_t  i2s_raw[I2S_DMA_BUF_LEN];
  // 16-bit PCM for Edge Impulse (size = EI_CLASSIFIER_RAW_SAMPLE_COUNT = 16000)
  static int16_t  pcm_frame[EI_SAMPLE_COUNT];
  size_t          pcm_idx = 0;

  while (true) {
    // ── 1. Drain DMA and fill PCM frame ─────────────────────────────
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, i2s_raw, sizeof(i2s_raw), &bytes_read, portMAX_DELAY);
    i2s_read(I2S_NUM_0, i2s_raw, sizeof(i2s_raw), &bytes_read, portMAX_DELAY);

// ── TEMP DIAGNOSTIC — remove after confirming ──
if (pcm_idx == 0) {
    Serial.printf("[DBG] bytes_read=%u  raw[0]=%ld  raw[1]=%ld  raw[2]=%ld\n",
                  bytes_read, i2s_raw[0], i2s_raw[1], i2s_raw[2]);
}
// ──────────────────────────────────────────────
    size_t samples_read = bytes_read / sizeof(int32_t);

    for (size_t i = 0; i < samples_read && pcm_idx < EI_SAMPLE_COUNT; ++i) {
      // Shift right 8 to discard unused LSBs → 24-bit → truncate to 16-bit
      pcm_frame[pcm_idx++] = (int16_t)(i2s_raw[i] >> 14);

    }

    // Wait until we have a full inference window
    if (pcm_idx < EI_SAMPLE_COUNT) {
      vTaskDelay(1);   // yield; do NOT block watchdog
      continue;
    }
    pcm_idx = 0;  // reset for next window

    // ── 2. Loudness gate ─────────────────────────────────────────────
    float db = computeDbSPL(pcm_frame, EI_SAMPLE_COUNT);

    // ── 3. Edge Impulse Inference ─────────────────────────────────────
    // Point the global bridge pointer to this frame so the callback
    // (ei_get_audio_signal_data) can convert it to float on demand.
    g_ei_pcm_buf = pcm_frame;

    signal_t signal;
    signal.total_length = EI_SAMPLE_COUNT;   // == EI_CLASSIFIER_RAW_SAMPLE_COUNT
    signal.get_data     = &ei_get_audio_signal_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    if (err != EI_IMPULSE_OK) {
      Serial.printf("[AI]  Inference error: %d\n", err);
      vTaskDelay(10);
      continue;
    }

    // Label order confirmed in model_variables.h:
    //   ei_classifier_inferencing_categories = { "chainsaw", "noise" }
    //   index 0 → "chainsaw"   index 1 → "noise"
    float conf_chainsaw = result.classification[0].value;
    float conf_noise    = result.classification[1].value;
    const char* top_label = (conf_chainsaw > conf_noise) ? "chainsaw" : "noise";
    float top_conf        = max(conf_chainsaw, conf_noise);

    // ── 4. Logic gate ─────────────────────────────────────────────────
    InferenceResult_t res;
    res.decibels   = db;
    res.confidence = top_conf;
    strncpy(res.label, top_label, sizeof(res.label) - 1);
    res.label[sizeof(res.label) - 1] = '\0';

    bool chainsawDetected = (strcmp(top_label, "chainsaw") == 0)
                          && (top_conf  >= CONFIDENCE_THRESHOLD)
                          && (db        >= LOUDNESS_GATE_DB);

    res.state = chainsawDetected ? 1 : 0;

    // ── 5. Push to telemetry queue (non-blocking; drop if full) ───────
    xQueueOverwrite(resultQueue, &res);

    // Debug output — visible in Serial Monitor
    Serial.printf("[AI]  dB=%.1f  label=%s  conf=%.2f  state=%d\n",
                  db, top_label, top_conf, res.state);

    vTaskDelay(1);   // yield to prevent TWDT trigger
  }
}

// ═════════════════════════════════════════════════════════════════════════
//  JSON PACKET BUILDER
// ═════════════════════════════════════════════════════════════════════════
/**
 * Fills buf with a minified JSON payload.
 * Format: {"id":N,"s":S,"c":"label","p":0.00,"t":00.0,"db":00.0,"pr":000.0}
 *
 * "pr" (pressure in hPa) is added here so the dashboard can display it
 * once the firmware comment at the bottom of the HTML is resolved.
 */
static void buildPacket(char* buf, size_t bufLen,
                         const InferenceResult_t& ir,
                         float temperature, float pressure) {
  snprintf(buf, bufLen,
    "{\"id\":%d,\"s\":%d,\"c\":\"%s\",\"p\":%.2f,\"t\":%.1f,\"db\":%.1f,\"pr\":%.1f}",
    NODE_ID, ir.state, ir.label, ir.confidence,
    temperature, ir.decibels, pressure / 100.0f  // Pa → hPa
  );
}

// ═════════════════════════════════════════════════════════════════════════
//  LORA TRANSMIT (with retry)
// ═════════════════════════════════════════════════════════════════════════
static bool loraSend(const char* payload) {
  LoRa.beginPacket();
  LoRa.print(payload);
  if (!LoRa.endPacket()) {
    Serial.println("[LoRa] TX FAILED");
    return false;
  }
  Serial.printf("[LoRa] TX → %s\n", payload);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  CORE 1 — TELEMETRY / RADIO TASK
// ═════════════════════════════════════════════════════════════════════════
/**
 * Pinned to Core 1, priority 1.
 *
 * State machine:
 *   IDLE  → send heartbeat every HEARTBEAT_INTERVAL_MS
 *   ALERT → burst ALERT_BURST_COUNT packets every ALERT_INTERVAL_MS
 *
 * BMP280 is polled at the start of each cycle.
 */
void Radio_Task(void* param) {
  Serial.println("[Radio] Task started on core " + String(xPortGetCoreID()));

  InferenceResult_t ir = {};
  ir.state      = 0;
  ir.decibels   = 0.0f;
  ir.confidence = 0.0f;
  strncpy(ir.label, "noise", sizeof(ir.label));

  char jsonBuf[192];

  uint32_t lastTxMs   = 0;
  uint8_t  burstCount = 0;

  while (true) {
    // ── Poll BMP280 ──────────────────────────────────────────────────
    float t  = bmp.readTemperature();
    float p  = bmp.readPressure();
    if (!isnan(t)) g_temperature = t;
    if (!isnan(p)) g_pressure    = p;

    // ── Pull latest inference result (non-blocking) ──────────────────
    InferenceResult_t newIr;
    if (xQueueReceive(resultQueue, &newIr, 0) == pdTRUE) {
      ir          = newIr;
      burstCount  = 0;    // reset burst counter on fresh result
    }

    uint32_t now = millis();

    if (ir.state == 1) {
      // ── ALERT state ────────────────────────────────────────────────
      if (burstCount < ALERT_BURST_COUNT &&
          (now - lastTxMs) >= ALERT_INTERVAL_MS) {
        buildPacket(jsonBuf, sizeof(jsonBuf), ir, g_temperature, g_pressure);
        loraSend(jsonBuf);
        lastTxMs = now;
        burstCount++;
      }
      if (burstCount >= ALERT_BURST_COUNT) {
        // Burst complete — revert to idle until next AI result
        ir.state = 0;
        burstCount = 0;
      }
    } else {
      // ── IDLE state: heartbeat every 20 s ───────────────────────────
      if ((now - lastTxMs) >= HEARTBEAT_INTERVAL_MS) {
        buildPacket(jsonBuf, sizeof(jsonBuf), ir, g_temperature, g_pressure);
        loraSend(jsonBuf);
        lastTxMs = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms loop — prevents TWDT
  }
}

// ═════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n[BOOT] Forest Sentinel Node — ID " + String(NODE_ID));

  // ── I2C for BMP280 ────────────────────────────────────────────────────
  Wire.begin(BMP_SDA_PIN, BMP_SCL_PIN);
  if (!bmp.begin(BMP280_ADDRESS_ALT)) {    // try 0x76 first
    if (!bmp.begin()) {                    // fallback to 0x77
      Serial.println("[BOOT] BMP280 NOT FOUND — check wiring!");
      // Continue anyway; temperature/pressure will read as 0
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temperature
                  Adafruit_BMP280::SAMPLING_X16,  // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println("[BOOT] BMP280 OK");

  // ── FSPI for SX1276 ───────────────────────────────────────────────────
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[BOOT] LoRa INIT FAILED — check wiring and frequency!");
    while (true) { delay(1000); }  // halt — no point continuing
  }
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth((long)LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.enableCrc();
  Serial.println("[BOOT] LoRa OK  @ 433 MHz  SF7  BW125");

  // ── I2S for INMP441 ───────────────────────────────────────────────────
  i2s_init();
  Serial.println("[BOOT] I2S OK   @ 16 kHz  16-bit PCM");

  // ── Inter-core queue (depth=1, always gets the freshest result) ───────
  resultQueue = xQueueCreate(1, sizeof(InferenceResult_t));
  configASSERT(resultQueue);

  // ── Launch tasks ──────────────────────────────────────────────────────
  //   AI_Task   → Core 0, priority 2, 32 KB stack
  //   Radio_Task → Core 1, priority 1,  8 KB stack
  xTaskCreatePinnedToCore(
    AI_Task,          // function
    "AI_Task",        // name
    AUDIO_STACK_BYTES,// stack in bytes  (≥30 KB for tensor arena)
    nullptr,          // param
    2,                // priority — HIGHER than Radio_Task
    nullptr,          // handle (not needed)
    0                 // Core 0
  );

  xTaskCreatePinnedToCore(
    Radio_Task,
    "Radio_Task",
    8 * 1024,
    nullptr,
    1,                // priority — lower
    nullptr,
    1                 // Core 1
  );

  Serial.println("[BOOT] Tasks launched — entering loop()");
}

// ── loop() is intentionally empty — all work is in FreeRTOS tasks ─────────
void loop() {
  vTaskDelay(portMAX_DELAY);
}

