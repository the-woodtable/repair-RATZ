/*
  EDI PIPE CAM — ESP32-CAM firmware (flash to BOTH cameras, identical)
  -------------------------------------------------------------------
  Streams 0xAA55-framed JPEG over UART0 (baud set in Serial.begin below).
  Frame format OUT of the camera:  0xAA 0x55 | uint32 LE length | JPEG

  The camera does NOT identify itself as left or right — identity is
  decided by which S3 UART it is wired into (see StereoCameras.h).
  So both cameras run this exact sketch with zero configuration.

  Wiring per camera (4 wires):
    5V  -> shared 5V rail (each cam can spike ~500 mA, budget for it —
           add a 470-1000uF electrolytic + 100nF ceramic decoupling cap
           pair as close to this module's 5V/GND pins as possible)
    GND -> shared GND
    U0T (GPIO1, TX) -> the S3 RX pin assigned to this camera  [video out]
    U0R (GPIO3, RX) <- the S3 TX pin assigned to this camera  [commands in]

  IMPORTANT: U0T/U0R are also the flashing pins. DISCONNECT both from the
  S3 before flashing this sketch with a USB-serial adapter, then reconnect.

  Board: "AI Thinker ESP32-CAM" in Arduino IDE.
  To flash: connect a USB-serial adapter to U0R/U0T/GND, hold GPIO0 to
  GND while resetting to enter bootloader. Disconnect adapter after.

  -------------------------------------------------------------------
  RESOLUTION CHANGE (bumped from QVGA up toward VGA for sharper CV
  detection, then dialed back to CIF once VGA proved too much data for
  BOTH cameras to stream together reliably):
    frame_size raised 320x240 -> 400x296 (CIF). This INVALIDATES any
    existing stereo_calib.npz (it was computed for 320x240 images) --
    recalibrate with stereo_calibrate.py after reflashing BOTH cameras.
    jpeg_quality was also lowered (= less compression, more detail) to
    complement the resolution bump. If fps still feels low with both
    cameras running, that's the two-camera bandwidth-sharing limit --
    drop back toward FRAMESIZE_QVGA rather than raising jpeg_quality
    further, since quality has less room to give before frames grow
    close to VGA size again.
  -------------------------------------------------------------------
  RECOVERY BEHAVIOR (added after field testing showed the stream could
  freeze permanently on a single glitch):
    1. Task watchdog — reboots the whole module if loop() ever fails to
       complete a full frame send within WDT_TIMEOUT_S seconds. This is
       the last-resort catch-all for any hang not handled below.
    2. Bounded-time serial writes — if the S3 stops draining its UART
       (e.g. busy servicing the other camera), don't block forever;
       give up on that frame after WRITE_TIMEOUT_MS and let loop()
       continue grabbing fresh frames instead of wedging on one write.
    3. Sensor re-init on repeated capture failure — if the OV5640's
       SCCB/I2C bus wedges (a known failure mode, often triggered by
       power glitches or bus noise), esp_camera_fb_get() will return
       NULL forever. Reinitializing the camera driver (not the whole
       chip) usually clears it without needing a full power cycle.
  -------------------------------------------------------------------
*/

#include "esp_camera.h"
#include "esp_task_wdt.h"

// ---- AI-Thinker ESP32-CAM pin map ----
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static const uint8_t MAGIC[2] = { 0xAA, 0x55 };

// --- Recovery tuning constants ---
#define WDT_TIMEOUT_S 5           // reboot if no full frame completes within this many seconds
#define WRITE_TIMEOUT_MS 1000     // give up on a single Serial.write() burst after this long
#define MAX_CONSECUTIVE_FAILS 5

#define REINIT_BLINK 0

#define FIXED_EXPOSURE 200    // 0-1200  (was 600: overexposed + halved fps)
#define FIXED_GAIN     4      // 0-30    (was 8: gain amplifies channel imbalance)

#define WB_MODE 3

#define FLIP_VERTICAL     0   // 0 or 1
#define MIRROR_HORIZONTAL 0   // 0 or 1

camera_config_t config = {};

int consecutiveFails = 0;

bool writeWithTimeout(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  uint32_t start = millis();
  size_t sent = 0;
  while (sent < len) {
    size_t avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t chunk = min(avail, len - sent);
      sent += Serial.write(data + sent, chunk);
    }
    if (millis() - start > timeout_ms) {
      return false;
    }
  }
  return true;
}

void applyFixedImageSettings();

void setup() {
  // MUST equal CAM_BAUD in THIS ONE/include/StereoCameras.h.
  // Changing it means reflashing BOTH cameras AND the S3 — three devices.
  // Symptom of getting it wrong: telemetry still works, frames stay at 0.
  Serial.begin(460800);

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };

  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Raised from FRAMESIZE_QVGA (320x240) for sharper crack detection,
  // set to CIF rather than VGA -- VGA alone streamed fine on ONE camera
  // but caused stream desync once BOTH ran together (too much combined
  // data for the shared link to the S3/laptop). CIF is a middle ground:
  // ~1.6x QVGA's pixel count (a real detail improvement) but only ~40%
  // of VGA's data size.
  // INVALIDATES stereo_calib.npz (computed for the old 320x240 size) --
  // recalibrate with stereo_calibrate.py after reflashing BOTH cameras.
  config.frame_size = FRAMESIZE_SVGA;   // 400x296

  if (psramFound()) {
    // Lowered from 20 (less compression, more detail).
    config.jpeg_quality = 14;          // was 20
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.jpeg_quality = 16;          // was 22
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    pinMode(4, OUTPUT);
    while (true) {
      digitalWrite(4, HIGH);
      delay(100);
      digitalWrite(4, LOW);
      delay(900);
    }
  }

  applyFixedImageSettings();
}

void applyFixedImageSettings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;

  s->set_exposure_ctrl(s, 0);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 0);

  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 0);
  s->set_wb_mode(s, WB_MODE);

  s->set_aec_value(s, FIXED_EXPOSURE);
  s->set_agc_gain(s, FIXED_GAIN);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, MIRROR_HORIZONTAL);
  s->set_vflip(s, FLIP_VERTICAL);
}

void loop() {

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    consecutiveFails++;
    if (consecutiveFails > MAX_CONSECUTIVE_FAILS) {
#if REINIT_BLINK
      pinMode(4, OUTPUT);
      digitalWrite(4, HIGH);
      delay(40);
      digitalWrite(4, LOW);
#endif
      esp_camera_deinit();
      delay(100);
      esp_camera_init(&config);
      consecutiveFails = 0;
    }
    delay(10);
    return;
  }

  uint32_t len = fb->len;

  bool ok = writeWithTimeout(MAGIC, 2, WRITE_TIMEOUT_MS);
  if (ok) ok = writeWithTimeout((uint8_t *)&len, 4, WRITE_TIMEOUT_MS);
  if (ok) ok = writeWithTimeout(fb->buf, fb->len, WRITE_TIMEOUT_MS);

  esp_camera_fb_return(fb);

#if REINIT_BLINK
  if (!ok) {
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    delay(40);
    digitalWrite(4, LOW);
  }
#endif

  if (ok) {
    consecutiveFails = 0;
    esp_task_wdt_reset();
  }
}