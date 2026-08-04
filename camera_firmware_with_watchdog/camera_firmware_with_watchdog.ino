/*
  EDI PIPE CAM — ESP32-CAM firmware (flash to BOTH cameras, identical)
  -------------------------------------------------------------------
  Streams 0xAA55-framed JPEG over UART0 at 921600 baud.
  Frame format OUT of the camera:  0xAA 0x55 | uint32 LE length | JPEG

  The camera does NOT identify itself as left or right — identity is
  decided by which S3 UART it is wired into (see s3_cam_forwarder.h).
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
#define MAX_CONSECUTIVE_FAILS 5  // this many back-to-back fb_get() failures triggers a sensor re-init

// camera_config_t is moved to file scope (was local to setup() originally)
// so that loop() can reuse the exact same config to re-init the sensor
// without needing to duplicate/rebuild it.
camera_config_t config = {};

// Counts consecutive esp_camera_fb_get() failures. Reset to 0 on any
// successful frame; once it crosses MAX_CONSECUTIVE_FAILS we assume the
// sensor itself is wedged (not just a transient buffer underrun) and
// force a driver-level re-init.
int consecutiveFails = 0;

// Sends `len` bytes from `data` over Serial, but gives up after
// timeout_ms instead of blocking forever. Serial.write() by default
// blocks indefinitely once the TX ring buffer fills — if the S3 stalls
// (e.g. busy forwarding the other camera's frame), this used to hang
// the whole loop with no way out. Returns false if it couldn't send
// everything in time; the caller should just move on to the next frame
// rather than treat this as fatal.
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
      return false;  // bail out — don't let one stalled receiver wedge the camera
    }
  }
  return true;
}

void setup() {
  Serial.begin(460800);

  // Arm the task watchdog on the loop task. `true` panics (reboots) on
  // trigger rather than just printing a warning — we want automatic
  // recovery here, not a message nobody will see on an unattended rig.
  // NOTE: esp32 Arduino core 3.x (IDF 5.x) changed esp_task_wdt_init()
  // from (timeout_s, panic_bool) to taking an esp_task_wdt_config_t*.
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,     // don't watch idle tasks, just this one
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
  config.xclk_freq_hz = 20000000;  // OV5640 needs this. Lowering it to 10 MHz
                                   // starves the sensor -> truncated frames
                                   // (mostly-grey images) and low fps.
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 — keep! stereo calib assumes this
  config.jpeg_quality = 15;            // ~8-15 KB/frame -> ~6-10 fps at 921600 baud
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    // Camera failed — blink onboard flash LED (GPIO4) forever
    pinMode(4, OUTPUT);
    while (true) {
      digitalWrite(4, HIGH);
      delay(100);
      digitalWrite(4, LOW);
      delay(900);
    }
  }
}

void loop() {

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    // No frame available. Could be a transient underrun (fine, just
    // retry) or a wedged sensor (SCCB/I2C lockup). We can't tell the
    // difference from a single failure, so we count consecutive misses
    // and only intervene once it's clearly not transient.
    consecutiveFails++;
    if (consecutiveFails > MAX_CONSECUTIVE_FAILS) {
      esp_camera_deinit();  // tear down the wedged driver/sensor state
      delay(100);
      esp_camera_init(&config);  // bring it back up fresh using the same config
      consecutiveFails = 0;
    }
    // NOTE: watchdog is deliberately NOT fed here. A real sensor wedge
    // that the re-init above can't clear will keep failing until the
    // WDT_TIMEOUT_S deadline, at which point the watchdog reboots the
    // whole module as the final fallback.
    delay(10);
    return;
  }

  uint32_t len = fb->len;

  // Send header + length + payload, each bounded by writeWithTimeout so
  // a stalled S3 receiver can't block this loop indefinitely. If any
  // part times out, the frame is abandoned (receiver-side framing will
  // reject/resync on the next valid magic bytes) and we move on rather
  // than hang.
  bool ok = writeWithTimeout(MAGIC, 2, WRITE_TIMEOUT_MS);
  if (ok) ok = writeWithTimeout((uint8_t *)&len, 4, WRITE_TIMEOUT_MS);  // little-endian on ESP32
  if (ok) ok = writeWithTimeout(fb->buf, fb->len, WRITE_TIMEOUT_MS);

  esp_camera_fb_return(fb);

  if (ok) {
    // Only reset the failure counter and feed the watchdog on a fully
    // successful frame — this is the "proof of life" signal that the
    // whole path (sensor -> encode -> UART) is actually working, not
    // just that loop() is spinning.
    consecutiveFails = 0;
    esp_task_wdt_reset();
  }
}
