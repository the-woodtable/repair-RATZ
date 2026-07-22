/*
  esp32cam_stream.ino
  EDI Pipe Cam - camera node firmware (flash this identically onto BOTH ESP32-CAM boards)

  Board: AI-Thinker ESP32-CAM (OV2640), mounted on an ESP32-CAM-MB USB dock
  Role: capture JPEG frames and push them out over UART2 (IO13 TX / IO15 RX)
        to the ESP32-S3 hub. WiFi/Bluetooth are left off entirely - this board
        never talks to the laptop directly, so we don't burn power or pins on
        a radio.

  Frame format sent on the wire:
    [0xAA][0x55][len: uint32 little-endian][ <len> bytes of JPEG payload ]

  The ESP32-S3 hub re-frames this (adding a camera-ID byte) before relaying
  it on to the laptop - see esp32s3_hub.ino for the full protocol.

  NOTE ON PINS: the ESP32-CAM-MB dock's onboard USB-serial chip is
  permanently wired to GPIO1/GPIO3 (UART0), so those pins can't be reused
  for the S3 link while the dock is attached - that's why this sketch uses
  a second UART (UART2) on IO13/IO15 instead. That also means Serial
  (UART0/USB) stays completely free for debug prints over the same cable
  you flash it with - no need to disconnect the dock after flashing.
  IO13 = TX (frames out to the S3), IO15 = RX (reserved, currently unused).
*/

#include "esp_camera.h"
#include <HardwareSerial.h>

// ---- AI-Thinker ESP32-CAM pin map (standard) ----
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static const uint32_t UART_BAUD = 921600;   // must match ESP32-S3 hub's CamSerialX.begin()
static const uint8_t  MAGIC0 = 0xAA;
static const uint8_t  MAGIC1 = 0x55;

static const int LINK_RX_PIN = 15;   // IO15 - unused/reserved, wired to S3's TX pin
static const int LINK_TX_PIN = 13;   // IO13 - frames out, wired to S3's RX pin

HardwareSerial CamLink(2);   // UART2, custom pins (UART0 stays free for debug/USB)

void setCameraConfig(camera_config_t &config) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;   // 640x480 - good balance for crack-detection YOLO input
    config.jpeg_quality = 12;            // lower number = higher quality/larger frame
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 14;
    config.fb_count = 1;
  }
}

void sendFramedJPEG(camera_fb_t *fb) {
  uint32_t len = fb->len;
  uint8_t header[6];
  header[0] = MAGIC0;
  header[1] = MAGIC1;
  header[2] = (uint8_t)(len & 0xFF);
  header[3] = (uint8_t)((len >> 8) & 0xFF);
  header[4] = (uint8_t)((len >> 16) & 0xFF);
  header[5] = (uint8_t)((len >> 24) & 0xFF);

  CamLink.write(header, sizeof(header));
  CamLink.write(fb->buf, fb->len);
}

void setup() {
  Serial.begin(115200);   // free for debug prints via the dock's USB - not the data link anymore
  CamLink.begin(UART_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  // Disable brownout detector chatter on some ESP32-CAM boards during power-up
  // (safe to leave default if you don't hit resets)

  camera_config_t config = {};
  setCameraConfig(config);

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("esp_camera_init failed: 0x%x\n", err);
    // Also blink the onboard flash LED (GPIO4) in case nobody has the
    // serial monitor open.
    pinMode(4, OUTPUT);
    while (true) {
      digitalWrite(4, HIGH); delay(150);
      digitalWrite(4, LOW);  delay(150);
    }
  }
  Serial.println("Camera init OK, streaming to S3 over UART2 (IO13 TX / IO15 RX)");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(5);
    return;
  }
  sendFramedJPEG(fb);
  esp_camera_fb_return(fb);
}
