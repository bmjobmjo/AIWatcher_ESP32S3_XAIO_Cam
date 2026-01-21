#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <type_traits>
#include <esp_wifi.h>

/* =========================================================
   GENERAL CONFIG
   ========================================================= */
#define AP_SSID         "AIWatcher-Setup"
#define AP_PASS         "12345678"
#define DNS_PORT        53
#define LED_PIN         LED_BUILTIN
#define LED_TEST_PIN_1  21
#define LED_TEST_PIN_2  2
#define LED_TEST_PIN_3  48
#define LED_ACTIVE_LOW  1
#define DEFAULT_IMAGESEND_DELAY 10000

/* =========================================================
   CAMERA PINS - FOR ESP32-S3 (SENSE / S3-CAM)
   ========================================================= */
#define PWDN_GPIO_NUM     -1 
#define RESET_GPIO_NUM    -1 
#define XCLK_GPIO_NUM     10 
#define SIOD_GPIO_NUM     40 
#define SIOC_GPIO_NUM     39 
#define Y9_GPIO_NUM       48 
#define Y8_GPIO_NUM       11 
#define Y7_GPIO_NUM       12 
#define Y6_GPIO_NUM       14 
#define Y5_GPIO_NUM       16 
#define Y4_GPIO_NUM       18 
#define Y3_GPIO_NUM       17 
#define Y2_GPIO_NUM       15 
#define VSYNC_GPIO_NUM    38 
#define HREF_GPIO_NUM     47 
#define PCLK_GPIO_NUM     13 

/* =========================================================
   API CONFIG
   ========================================================= */
const char* API_HOST = "aiwatcher-api-160403254713.asia-south1.run.app";
const int   API_PORT = 443;
const char* API_PATH = "/process-image";
const char* HARDCODED_APP_KEY = "9d025fcc-0050-4b47-ae85-52608257f9f4";

/* =========================================================
   GLOBALS & ENUMS
   ========================================================= */
enum LedMode {
  LED_PROVISIONING,
  LED_PRE_RESTART,
  LED_HOTSPOT_MODE,
  LED_HOTSPOT_FAST,
  LED_CAPTURE_SOLID,
  LED_NORMAL_IDLE
};

LedMode ledMode = LED_PROVISIONING;
unsigned long lastLedToggle = 0;
bool ledState = false;
int fastBlinkCount = 0;
unsigned long lastLedPulse = 0;

WebServer server(80);
DNSServer dns;
Preferences prefs;
const char* DEFAULT_SERIAL = "1111222233334444";

unsigned long lastCaptureTime = 0;
unsigned long nextCaptureDelay = 10000UL;
unsigned long minuteWindowStart = 0;
int imagesThisMinute = 0;

bool provisioningMode = false;
bool provisioningStarted = false;
bool provisioningRequested = false;
bool motionDetectionEnabled = false;

uint32_t resetCounter = 0;
bool resetWindowActive = false;
unsigned long nextResetTickMs = 0;
uint16_t resetTickCount = 0;
static const char* PREF_RESET_COUNTER_KEY = "rst_cnt";

camera_config_t cameraConfig;
bool cameraInitialized = false;

/* =========================================================
   TLS DIAGNOSTICS HELPERS (Templates for compatibility)
   ========================================================= */
template <typename T> struct has_lastError {
  template <typename U> static auto test(int) -> decltype(((U*)0)->lastError((char*)0, 0), std::true_type());
  template <typename> static std::false_type test(...);
  static const bool value = std::is_same<decltype(test<T>(0)), std::true_type>::value;
};
template <typename T> struct has_getLastSSLError {
  template <typename U> static auto test(int) -> decltype(((U*)0)->getLastSSLError((char*)0, 0), std::true_type());
  template <typename> static std::false_type test(...);
  static const bool value = std::is_same<decltype(test<T>(0)), std::true_type>::value;
};

/* =========================================================
   LED HANDLER
   ========================================================= */
void setLedMode(LedMode mode) {
  ledMode = mode;
  if (mode == LED_NORMAL_IDLE) digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
  if (mode == LED_CAPTURE_SOLID) digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
}

void handleLed() {
  unsigned long now = millis();
  switch (ledMode) {
    case LED_PROVISIONING:
      if (now - lastLedToggle >= 500) {
        ledState = !ledState;
        digitalWrite(LED_PIN, LED_ACTIVE_LOW ? !ledState : ledState);
        lastLedToggle = now;
      }
      break;
    case LED_PRE_RESTART:
      if (now - lastLedToggle >= 100) {
        ledState = !ledState;
        digitalWrite(LED_PIN, LED_ACTIVE_LOW ? !ledState : ledState);
        lastLedToggle = now;
        if (!ledState) {
          fastBlinkCount++;
          if (fastBlinkCount >= 10) ESP.restart();
        }
      }
      break;
    case LED_HOTSPOT_FAST:
      if (now - lastLedToggle >= 100) {
        ledState = !ledState;
        digitalWrite(LED_PIN, LED_ACTIVE_LOW ? !ledState : ledState);
        lastLedToggle = now;
      }
      break;
    case LED_CAPTURE_SOLID:
      digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
      break;
    default: break;
  }
}

/* =========================================================
   CAMERA LOGIC
   ========================================================= */
void startCamera() {
  // 1. Diagnostics: Check RAM status BEFORE touching the camera
  Serial.println("[CAM] Checking Memory Status...");
  size_t freeHeap = ESP.getFreeHeap();
  size_t freePsram = ESP.getFreePsram();
  size_t totalPsram = ESP.getPsramSize();
  bool psram_ok = psramFound();
  
  Serial.printf("  > Internal Heap: %d bytes\n", freeHeap);
  Serial.printf("  > Total PSRAM:   %d bytes\n", totalPsram);
  Serial.printf("  > Free PSRAM:    %d bytes\n", freePsram);
  Serial.printf("  > PSRAM Found:   %s\n", psram_ok ? "yes" : "no");

  bool hasValidPsram = (totalPsram > 0) && (freePsram > 200000); // Need at least ~200KB for RGB QVGA

  // 2. Configure Camera
  Serial.println("[CAM] Pin Map:");
  Serial.printf("  > XCLK: %d, PCLK: %d, VSYNC: %d, HREF: %d\n", XCLK_GPIO_NUM, PCLK_GPIO_NUM, VSYNC_GPIO_NUM, HREF_GPIO_NUM);
  Serial.printf("  > SIOD: %d, SIOC: %d, PWDN: %d, RESET: %d\n", SIOD_GPIO_NUM, SIOC_GPIO_NUM, PWDN_GPIO_NUM, RESET_GPIO_NUM);
  Serial.printf("  > D0: %d D1: %d D2: %d D3: %d D4: %d D5: %d D6: %d D7: %d\n",
                Y2_GPIO_NUM, Y3_GPIO_NUM, Y4_GPIO_NUM, Y5_GPIO_NUM, Y6_GPIO_NUM, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM);
  cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  cameraConfig.ledc_timer   = LEDC_TIMER_0;
  cameraConfig.pin_d0 = Y2_GPIO_NUM;
  cameraConfig.pin_d1 = Y3_GPIO_NUM;
  cameraConfig.pin_d2 = Y4_GPIO_NUM;
  cameraConfig.pin_d3 = Y5_GPIO_NUM;
  cameraConfig.pin_d4 = Y6_GPIO_NUM;
  cameraConfig.pin_d5 = Y7_GPIO_NUM;
  cameraConfig.pin_d6 = Y8_GPIO_NUM;
  cameraConfig.pin_d7 = Y9_GPIO_NUM;
  cameraConfig.pin_xclk = XCLK_GPIO_NUM;
  cameraConfig.pin_pclk = PCLK_GPIO_NUM;
  cameraConfig.pin_vsync = VSYNC_GPIO_NUM;
  cameraConfig.pin_href = HREF_GPIO_NUM;
  cameraConfig.pin_sscb_sda = SIOD_GPIO_NUM;
  cameraConfig.pin_sscb_scl = SIOC_GPIO_NUM;
  cameraConfig.pin_pwdn = PWDN_GPIO_NUM;
  cameraConfig.pin_reset = RESET_GPIO_NUM;
  
  // LOWERED: 10MHz for maximum stability during debug
  cameraConfig.xclk_freq_hz = 10000000; 
  
  // Reverted to JPEG for Stability + Color
  Serial.println("[CAM] Config: Stable JPEG Mode (Color).");
  cameraConfig.pixel_format = PIXFORMAT_JPEG; 
  if (hasValidPsram) {
    cameraConfig.frame_size = FRAMESIZE_QVGA;
    cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
    cameraConfig.fb_count = 2;
  } else {
    // Fallback for boards with no PSRAM detected
    cameraConfig.frame_size = FRAMESIZE_QQVGA;
    cameraConfig.fb_location = CAMERA_FB_IN_DRAM;
    cameraConfig.fb_count = 1;
  }
  cameraConfig.jpeg_quality = 12; // High quality JPEG

  esp_err_t err = esp_camera_init(&cameraConfig);
  if (err != ESP_OK) {
    printf("[CAM] Init failed: 0x%x\n", err);
    cameraInitialized = false;
    return;
  }
  cameraInitialized = true;
  printf("[CAM] Initialized successfully!\n");
}

/* =========================================================
   MOTION DETECTION (Smart Thumbnail Method)
   ========================================================= */
uint8_t* prev_thumb = NULL;
size_t thumb_len = 0; 
int motion_frame_count = 0; // Fixed: Variable declared

void resetMotionState() {
  motion_frame_count = 0; 
  if (prev_thumb) { free(prev_thumb); prev_thumb = NULL; }
  thumb_len = 0;
}

bool checkMotion() {
  if (!cameraInitialized) return false;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  // 1. Decode a tiny 1/8th scale RGB thumbnail (approx 40x30 pixels for QVGA)
  // This is fast and low-memory.
  uint8_t* curr_thumb = NULL;
  size_t curr_thumb_len = 0; 
  size_t thumb_w = fb->width / 8;
  size_t thumb_h = fb->height / 8;
  curr_thumb_len = thumb_w * thumb_h * 2; // RGB565 = 2 bytes per pixel
  curr_thumb = (uint8_t*)malloc(curr_thumb_len);
  if (!curr_thumb) {
    Serial.println("[ERR] Thumb buffer alloc failed");
    esp_camera_fb_return(fb);
    return false;
  }
  // Fixed: Correct function name is jpg2rgb565
  bool decoded = jpg2rgb565(fb->buf, fb->len, curr_thumb, JPG_SCALE_8X);
  
  if (!decoded || !curr_thumb) {
    Serial.println("[ERR] Thumb decode failed");
    free(curr_thumb);
    esp_camera_fb_return(fb);
    return false;
  }
  
  int pixel_count = (int)(thumb_w * thumb_h);

  if (prev_thumb == NULL) {
    // First frame, save as reference
    prev_thumb = curr_thumb; // Take ownership
    esp_camera_fb_return(fb);
    return false;
  }

  // 2. Compare RGB Thumbnails (Pixel by Pixel)
  int changed_pixels = 0;
  
  uint16_t* curr16 = (uint16_t*)curr_thumb;
  uint16_t* prev16 = (uint16_t*)prev_thumb;
  for (int i = 0; i < pixel_count; i++) {
    uint16_t c = curr16[i];
    uint16_t p = prev16[i];
    uint8_t cr = (uint8_t)(((c >> 11) & 0x1F) * 255 / 31);
    uint8_t cg = (uint8_t)(((c >> 5) & 0x3F) * 255 / 63);
    uint8_t cb = (uint8_t)((c & 0x1F) * 255 / 31);
    uint8_t pr = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
    uint8_t pg = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
    uint8_t pb = (uint8_t)((p & 0x1F) * 255 / 31);
    int diff = abs(cr - pr) + abs(cg - pg) + abs(cb - pb);
               
    if (diff > 50) changed_pixels++; // Sensitivity threshold
  }

  float motion_percent = (float)changed_pixels / (float)pixel_count * 100.0;
  Serial.printf("[MOTION] Diff: %.2f%%\n", motion_percent);

  // 3. Update Reference
  free(prev_thumb);
  prev_thumb = curr_thumb; // Take ownership of new thumb

  esp_camera_fb_return(fb);
  return (motion_percent > 10.0);
}

/* =========================================================
   IMAGE CAPTURE & UPLOAD
   ========================================================= */
void captureAndSendImage() {
  if (WiFi.status() != WL_CONNECTED) {
    printf("[API] Skip: WiFi not connected\n");
    return;
  }
  if (!cameraInitialized) {
    printf("[API] Skip: Camera not initialized\n");
    return;
  }

  printf("[API] Starting capture sequence...\n");
  setLedMode(LED_CAPTURE_SOLID);
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    printf("[API] Error: Failed to capture frame buffer\n");
    setLedMode(LED_NORMAL_IDLE);
    return;
  }
  printf("[API] JPEG captured. Size: %d bytes\n", fb->len);

  // No conversion needed! We are already in JPEG mode.

  WiFiClientSecure client;
  client.setInsecure();
  printf("[API] Connecting to %s:%d...\n", API_HOST, API_PORT);

  if (client.connect(API_HOST, API_PORT)) {
    printf("[API] Connected! Preparing payload...\n");
    String serialNumber = prefs.getString("serial", DEFAULT_SERIAL);
    String apiKey = prefs.getString("app_key", HARDCODED_APP_KEY);
    String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    
    String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"serialNumber\"\r\n\r\n" + serialNumber + "\r\n--" + boundary + "\r\nContent-Disposition: form-data; name=\"apiKey\"\r\n\r\n" + apiKey + "\r\n--" + boundary + "\r\nContent-Disposition: form-data; name=\"image\"; filename=\"a.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";

    uint32_t totalLen = head.length() + fb->len + tail.length();
    printf("[API] Sending POST request (%d bytes)...\n", totalLen);

    client.printf("POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: multipart/form-data; boundary=%s\r\nContent-Length: %d\r\n\r\n", API_PATH, API_HOST, boundary.c_str(), totalLen);
    client.print(head);
    
    size_t written = client.write(fb->buf, fb->len);
    printf("[API] Image data sent: %d bytes\n", written);
    
    client.print(tail);
    printf("[API] Payload complete. Waiting for response...\n");

    // Minimal response parsing for next_interval
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") break;
    }
    String response = client.readString();
    printf("[API] Response: %s\n", response.c_str());

    // Parse JSON response to update configuration
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      JsonObject instructions = doc["instructions"];
      
      if (instructions.containsKey("next_interval")) {
        unsigned long intervalSec = instructions["next_interval"];
        // Sanity check: ensure interval is at least 5 seconds
        if (intervalSec >= 5) {
          unsigned long newDelay = intervalSec * 1000UL;
          if (nextCaptureDelay != newDelay) {
            nextCaptureDelay = newDelay;
            printf("[CONFIG] Updated Capture Interval: %lu ms\n", nextCaptureDelay);
          }
        }
      }

      if (instructions.containsKey("motion")) {
        bool serverMotionEnabled = instructions["motion"];
        if (motionDetectionEnabled != serverMotionEnabled) {
          motionDetectionEnabled = serverMotionEnabled;
          printf("[CONFIG] Motion Detection Mode changed to: %s\n", motionDetectionEnabled ? "ENABLED" : "DISABLED");
          
          // Reset motion state if enabling
          if (motionDetectionEnabled) resetMotionState();
        }
      }
    } else {
      printf("[API] Error parsing JSON response: %s\n", error.c_str());
    }
  } else {
    printf("[API] Connection failed!\n");
  }
  
  // Clean up
  esp_camera_fb_return(fb);

  lastCaptureTime = millis();
  setLedMode(LED_NORMAL_IDLE);
  printf("[API] Capture sequence finished.\n");
}

/* =========================================================
   PROVISIONING PORTAL & WEB SERVER
   ========================================================= */
void startProvisioningPortal() {
  provisioningMode = true;
  provisioningStarted = true;
  setLedMode(LED_HOTSPOT_FAST);
  
  WiFi.softAP(AP_SSID, AP_PASS);
  dns.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<h1>AIWatcher Setup</h1><form action='/save' method='POST'>SSID: <input name='s'><br>Pass: <input name='p'><br><input type='submit'></form>");
  });

  server.on("/save", HTTP_POST, []() {
    prefs.putString("ssid", server.arg("s"));
    prefs.putString("password", server.arg("p"));
    server.send(200, "text/plain", "Saved. Restarting...");
    delay(1000);
    ESP.restart();
  });

  server.begin();
  printf("Portal Started\n");
}

void handleResetWindowTimer() {
  if (!resetWindowActive) return;
  if (millis() > nextResetTickMs) {
    resetTickCount++;
    resetCounter++;
    prefs.putUInt(PREF_RESET_COUNTER_KEY, resetCounter);
    nextResetTickMs = millis() + 100;
    if (resetTickCount > 100) {
      prefs.putUInt(PREF_RESET_COUNTER_KEY, 0);
      resetWindowActive = false;
    }
  }
}
/* =========================================================
   SETUP 
   ========================================================= */
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n\n========================================");
  Serial.println("      AIWATCHER SYSTEM BOOTING         ");
  Serial.println("========================================");

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_TEST_PIN_1, OUTPUT);
  pinMode(LED_TEST_PIN_2, OUTPUT);
  pinMode(LED_TEST_PIN_3, OUTPUT);

  // Brief LED identification test at boot
  digitalWrite(LED_TEST_PIN_1, LED_ACTIVE_LOW ? LOW : HIGH);
  delay(200);
  digitalWrite(LED_TEST_PIN_1, LED_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(LED_TEST_PIN_2, LED_ACTIVE_LOW ? LOW : HIGH);
  delay(200);
  digitalWrite(LED_TEST_PIN_2, LED_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(LED_TEST_PIN_3, LED_ACTIVE_LOW ? LOW : HIGH);
  delay(200);
  digitalWrite(LED_TEST_PIN_3, LED_ACTIVE_LOW ? HIGH : LOW);

  // STEP 1: Preferences Initialization
  Serial.println("[STEP 1] Opening Preferences...");
  if (prefs.begin("aiwatcher", false)) {
    Serial.println("[OK] Preferences partition 'aiwatcher' loaded.");
  } else {
    Serial.println("[ERROR] Failed to initialize Preferences! Check partition table.");
  }

  // STEP 2: Handle Reset Counter (Double-Reset Detection)
  resetCounter = prefs.getUInt(PREF_RESET_COUNTER_KEY, 0);
  Serial.print("[STEP 2] Reset Counter Value: ");
  Serial.println(resetCounter);

  // Deactivated as requested: Multiple reset detection
  /*
  if (resetCounter >= 3 && resetCounter < 10) { 
    Serial.println("[ACTION] Multiple resets detected. Entering Setup Portal...");
    prefs.putUInt(PREF_RESET_COUNTER_KEY, 0);
    startProvisioningPortal();
  } else 
  */
  {
    // Standard boot sequence
    resetWindowActive = true;
    nextResetTickMs = millis() + 100;
    Serial.println("[INFO] Normal boot. Reset window active for 10 seconds.");
    
    // STEP 3: WiFi Connection & Logging
    String s = prefs.getString("ssid", "");
    String p = prefs.getString("password", "");

    s = "Bmjo";//"Airtel_biju_0971";
    p = "bmjobmjo";//"Air@67766";

    Serial.println("[STEP 3] WiFi Credentials Check:");
    if (s != "" && s != "NULL") {
      Serial.print("  > SSID:     "); Serial.println(s);
      Serial.println("  > Status:   Initializing WiFi stack for XIAO ESP32S3...");

      // 1. Reset WiFi stack cleanly
      WiFi.disconnect(true); // Clear credentials and switch off
      delay(1000);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false); // Disable power saving for max performance
      
      // 2. Scan to verify signal presence (disabled due to scan crash on some builds)
      bool ssidFound = false;
      int32_t channel = 0;
      Serial.println("  > Action:   Skipping WiFi scan for stability. Using auto channel.");

      // 3. Connection Loop
      int retryCount = 0;
      const int maxRetries = 3; 
      bool connected = false;
      WiFi.setAutoReconnect(true);

      while (retryCount < maxRetries && !connected) {
        Serial.printf("\n[CONNECT] Attempt %d/%d to '%s' (Channel: %d)...\n", retryCount + 1, maxRetries, s.c_str(), channel);
        
        // Use specific channel if found, otherwise scan auto (channel 0)
        WiFi.begin(s.c_str(), p.c_str(), channel);
        
        unsigned long startAttemptTime = millis();
        // Wait up to 15 seconds for WiFi connection
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
          delay(500);
          Serial.print(".");
          handleLed(); 
        }

        if (WiFi.status() == WL_CONNECTED) {
          connected = true;
        } else {
          Serial.print(" Failed! (Status Code: ");
          Serial.print(WiFi.status());
          Serial.println(")");
          
          retryCount++;
          if (retryCount < maxRetries) {
            Serial.println("  > Retrying in 3 seconds...");
            WiFi.disconnect(); 
            delay(3000); // Cool down
          }
        }
      }

      Serial.println("");

      if (connected) {
        Serial.println("[OK] WiFi Connected Successfully!");
        Serial.print("  > IP Address: "); Serial.println(WiFi.localIP());
        Serial.print("  > Signal (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
        
        // STEP 4: Camera Initialization
        Serial.println("[STEP 4] Initializing Camera Hardware...");
        startCamera();
        if (cameraInitialized) {
          Serial.println("[OK] Camera System Online.");
        } else {
          Serial.println("[ERROR] Camera initialization failed! Check wiring/power.");
        }
      } else {
        Serial.println("[TIMEOUT] Could not connect to WiFi after multiple attempts.");
        Serial.println("[INFO] Switching to Access Point mode for reconfiguration.");
        startProvisioningPortal();
      }
    } else {
      Serial.println("[INFO] No credentials stored. Switching to Access Point mode.");
      startProvisioningPortal();
    }
  }
  
  Serial.println("========================================");
  Serial.println("      SETUP SEQUENCE FINISHED          ");
  Serial.println("========================================\n");
}

/* =========================================================
   LOOP
   ========================================================= */
void loop() {
  // Always handle background tasks: LED patterns and Reset Timer
  handleLed();
  handleResetWindowTimer();
  
  if (provisioningMode) {
    // If in Hotspot mode, handle the Setup Web Server
    server.handleClient();
    dns.processNextRequest();
  } else {
    // If in Normal mode, check for Motion or Timer-based Capture
    if (WiFi.status() == WL_CONNECTED) {
      
      if (motionDetectionEnabled) {
        // Option A: Upload only when motion is detected
        if (checkMotion()) {
          Serial.println("[EVENT] Motion detected! Uploading...");
          captureAndSendImage();
        }
      } else {
        // Option B: Regular interval upload
        if (millis() - lastCaptureTime >= nextCaptureDelay) {
          Serial.println("[EVENT] Timer trigger: Capturing image...");
          captureAndSendImage();
        }
      }

    } else {
      // WiFi dropped during operation
      Serial.println("[WARN] WiFi Connection lost. Attempting to reconnect...");
      delay(5000); 
      
    }
  }
}
