/*
  EDI PIPE CAM — ESP32-CAM firmware (flash to BOTH cameras, identical)
  -------------------------------------------------------------------
  Streams 0xAA55-framed JPEG over UART0 at 921600 baud.
  Frame format OUT of the camera:  0xAA 0x55 | uint32 LE length | JPEG

  The camera does NOT identify itself as left or right — identity is
  decided by which S3 UART it is wired into (see s3_cam_forwarder.h).
  So both cameras run this exact sketch with zero configuration.

  Wiring per camera (4 wires):
    5V  -> shared 5V rail (each cam can spike ~500 mA, budget for it)
    GND -> shared GND
    U0T (GPIO1, TX) -> the S3 RX pin assigned to this camera  [video out]
    U0R (GPIO3, RX) <- the S3 TX pin assigned to this camera  [commands in]

  IMPORTANT: U0T/U0R are also the flashing pins. DISCONNECT both from the
  S3 before flashing this sketch with a USB-serial adapter, then reconnect.

  Board: "AI Thinker ESP32-CAM" in Arduino IDE.
  To flash: connect a USB-serial adapter to U0R/U0T/GND, hold GPIO0 to
  GND while resetting to enter bootloader. Disconnect adapter after.
*/

#include "esp_camera.h"

// ---- AI-Thinker ESP32-CAM pin map ----
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

static const uint8_t MAGIC[2] = {0xAA, 0x55};

void setup() {
  Serial.begin(460800);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;   config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;   config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;   config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;   config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240 — keep! stereo calib assumes this
  config.jpeg_quality = 15;               // ~8-15 KB/frame -> ~6-10 fps at 921600 baud
  config.fb_count     = 2;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    // Camera failed — blink onboard flash LED (GPIO4) forever
    pinMode(4, OUTPUT);
    while (true) { digitalWrite(4, HIGH); delay(100); digitalWrite(4, LOW); delay(900); }
  }
}

// ---------------- live control commands (from the S3 TX line) ----------------
// Single ASCII chars, applied instantly, no reflashing needed:
//   q / Q  jpeg quality worse / better   (10 = best, 40 = worst; smaller
//                                         frames = higher fps)
//   b / B  brightness down / up          (-2 .. +2)
//   c / C  contrast down / up            (-2 .. +2)
//   1      QQVGA 160x120     2  QVGA 320x240 (default, calibrated)
//   3      VGA   640x480     4  SVGA 800x600
//   f / F  flash LED off / on            (GPIO 4, useful inside a pipe)
//
// NOTE: changing resolution INVALIDATES the stereo calibration — the panel
// will report wrong distances until you recalibrate at that resolution.
// Use 3/4 only for capturing detailed stills for the crack dataset.
void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    switch (c) {
      case 'q': s->set_quality(s, min(40, s->status.quality + 2)); break;
      case 'Q': s->set_quality(s, max(10, s->status.quality - 2)); break;
      case 'b': s->set_brightness(s, max(-2, s->status.brightness - 1)); break;
      case 'B': s->set_brightness(s, min(2, s->status.brightness + 1)); break;
      case 'c': s->set_contrast(s, max(-2, s->status.contrast - 1)); break;
      case 'C': s->set_contrast(s, min(2, s->status.contrast + 1)); break;
      case '1': s->set_framesize(s, FRAMESIZE_QQVGA); break;
      case '2': s->set_framesize(s, FRAMESIZE_QVGA);  break;
      case '3': s->set_framesize(s, FRAMESIZE_VGA);   break;
      case '4': s->set_framesize(s, FRAMESIZE_SVGA);  break;
      case 'f': pinMode(4, OUTPUT); digitalWrite(4, LOW);  break;
      case 'F': pinMode(4, OUTPUT); digitalWrite(4, HIGH); break;
    }
  }
}

void loop() {
  handleCommands();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { delay(10); return; }

  uint32_t len = fb->len;
  Serial.write(MAGIC, 2);
  Serial.write((uint8_t *)&len, 4);        // little-endian on ESP32
  Serial.write(fb->buf, fb->len);          // blocks until sent — natural rate limit

  esp_camera_fb_return(fb);
}
