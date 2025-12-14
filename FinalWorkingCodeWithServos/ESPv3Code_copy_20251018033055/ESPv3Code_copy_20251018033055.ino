// ============================================================================
// ESP32-CAM 3-CLASS SORTER - DUAL UART VERSION
// ============================================================================
// Board: AI Thinker ESP32-CAM
// Upload Speed: 115200
// Partition Scheme: Huge APP (3MB No OTA)
// Flash Frequency: 80MHz
// UART0 (Serial): 115200 baud - USB-TTL Serial Monitor
// UART2 (Serial2): 57600 baud - Arduino Communication


#include <BSc_GP_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// ============================================================================
// UART2 PINS FOR ARDUINO COMMUNICATION (ESP32-CAM)
// ============================================================================
#define UART2_TX_PIN 14  // ESP32 GPIO14 -> Arduino D3 (RX) - SD bus, safe when SD unused
#define UART2_RX_PIN 13  // ESP32 GPIO13 <- Arduino D2 (TX via divider) - SD bus, safe
#define UART2_BAUD 38400 // Max supported by NeoSWSerial on Arduino

// ============================================================================
// CAMERA PINS (AI-THINKER ESP32-CAM)
// ============================================================================
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

// ============================================================================
// CONFIGURATION
// ============================================================================
#define RAW_WIDTH 320
#define RAW_HEIGHT 240
#define BYTES_PER_PIXEL 3
#define MAX_RETRY 3
#define MIN_CONFIDENCE 0.65f
#define NMS_IOU_THRESHOLD 0.2f
#define MIN_BOX_SIZE 2
#define SPATIAL_THRESHOLD 60
#define CONF_DIFF_THRESHOLD 0.25f
#define HIGH_CONF_THRESHOLD 0.85f

// ============================================================================
// BOUNDING BOX STRUCTURE
// ============================================================================
struct BoundingBox {
  uint32_t x, y, width, height;
  float confidence;
  char label[32];
  bool suppressed;

  BoundingBox() : x(0), y(0), width(0), height(0), confidence(0.0f), suppressed(false) {
    label[0] = '\0';
  }

  uint32_t area() const { return width * height; }
  uint32_t centerX() const { return x + (width / 2); }
  uint32_t centerY() const { return y + (height / 2); }
  
  bool isValid() const {
    return width >= MIN_BOX_SIZE && 
           height >= MIN_BOX_SIZE &&
           confidence >= MIN_CONFIDENCE;
  }
};

// ============================================================================
// CAMERA MANAGER
// ============================================================================
class CameraManager {
private:
  bool active;
  uint8_t* imageBuffer;

  camera_config_t getConfig() {
    camera_config_t config;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    return config;
  }

public:
  CameraManager() : active(false), imageBuffer(nullptr) {}

  bool initialize() {
    if (active) return true;

    camera_config_t config = getConfig();
    esp_err_t result = esp_camera_init(&config);
    
    if (result != ESP_OK) {
      Serial.println("[ERROR] Camera init failed");
      Serial2.println("ERR:CAM_INIT_FAILED");
      return false;
    }

    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->id.PID == OV3660_PID) {
      sensor->set_vflip(sensor, 1);
      sensor->set_brightness(sensor, 1);
      sensor->set_saturation(sensor, 0);
    }

    active = true;
    Serial.println("[OK] Camera initialized");
    return true;
  }

  bool capture(uint16_t targetWidth, uint16_t targetHeight) {
    if (!active) return false;

    if (imageBuffer) {
      free(imageBuffer);
      imageBuffer = nullptr;
    }

    size_t bufferSize = RAW_WIDTH * RAW_HEIGHT * BYTES_PER_PIXEL;
    imageBuffer = (uint8_t*)malloc(bufferSize);
    if (!imageBuffer) {
      Serial.println("[ERROR] Memory allocation failed");
      return false;
    }

    camera_fb_t* frame = nullptr;
    for (uint8_t retry = 0; retry < MAX_RETRY && !frame; retry++) {
      frame = esp_camera_fb_get();
      if (!frame && retry < MAX_RETRY - 1) delay(10);
    }

    if (!frame || !frame->buf || frame->len == 0) {
      if (frame) esp_camera_fb_return(frame);
      free(imageBuffer);
      imageBuffer = nullptr;
      Serial.println("[ERROR] Frame capture failed");
      return false;
    }

    bool ok = fmt2rgb888(frame->buf, frame->len, PIXFORMAT_JPEG, imageBuffer);
    esp_camera_fb_return(frame);

    if (!ok) {
      free(imageBuffer);
      imageBuffer = nullptr;
      Serial.println("[ERROR] Image conversion failed");
      return false;
    }

    if (targetWidth != RAW_WIDTH || targetHeight != RAW_HEIGHT) {
      ei::image::processing::crop_and_interpolate_rgb888(
        imageBuffer, RAW_WIDTH, RAW_HEIGHT,
        imageBuffer, targetWidth, targetHeight
      );
    }

    Serial.println("[OK] Image captured and processed");
    return true;
  }

  uint8_t* getBuffer() const { return imageBuffer; }

  void release() {
    if (imageBuffer) {
      free(imageBuffer);
      imageBuffer = nullptr;
    }
  }

  ~CameraManager() {
    release();
    if (active) esp_camera_deinit();
  }
};

// ============================================================================
// DETECTION FILTER
// ============================================================================
class DetectionFilter {
public:
  static float calculateIoU(const BoundingBox& a, const BoundingBox& b) {
    uint32_t x1 = max(a.x, b.x);
    uint32_t y1 = max(a.y, b.y);
    uint32_t x2 = min(a.x + a.width, b.x + b.width);
    uint32_t y2 = min(a.y + a.height, b.y + b.height);

    if (x1 >= x2 || y1 >= y2) return 0.0f;

    uint32_t intersection = (x2 - x1) * (y2 - y1);
    uint32_t unionArea = a.area() + b.area() - intersection;
    
    return unionArea > 0 ? (float)intersection / unionArea : 0.0f;
  }

  static float distance(const BoundingBox& a, const BoundingBox& b) {
    int32_t dx = (int32_t)a.centerX() - (int32_t)b.centerX();
    int32_t dy = (int32_t)a.centerY() - (int32_t)b.centerY();
    return sqrt(dx * dx + dy * dy);
  }

  static bool sameClass(const BoundingBox& a, const BoundingBox& b) {
    return strcmp(a.label, b.label) == 0;
  }

  static void sortByConfidence(BoundingBox* boxes, uint32_t count) {
    for (uint32_t i = 0; i < count - 1; i++) {
      for (uint32_t j = 0; j < count - i - 1; j++) {
        if (boxes[j].confidence < boxes[j + 1].confidence) {
          BoundingBox temp = boxes[j];
          boxes[j] = boxes[j + 1];
          boxes[j + 1] = temp;
        }
      }
    }
  }

  static void applyNMS(BoundingBox* boxes, uint32_t count) {
    sortByConfidence(boxes, count);
    
    for (uint32_t i = 0; i < count; i++) {
      if (boxes[i].suppressed) continue;
      
      for (uint32_t j = i + 1; j < count; j++) {
        if (boxes[j].suppressed) continue;
        if (!sameClass(boxes[i], boxes[j])) continue;
        
        float iou = calculateIoU(boxes[i], boxes[j]);
        float dist = distance(boxes[i], boxes[j]);
        
        if (iou > NMS_IOU_THRESHOLD || dist < SPATIAL_THRESHOLD) {
          boxes[j].suppressed = true;
        }
      }
    }
  }

  static void filterInvalid(BoundingBox* boxes, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
      if (boxes[i].suppressed) continue;
      
      if (!boxes[i].isValid()) {
        boxes[i].suppressed = true;
        continue;
      }
      
      if (boxes[i].x + boxes[i].width > EI_CLASSIFIER_INPUT_WIDTH ||
          boxes[i].y + boxes[i].height > EI_CLASSIFIER_INPUT_HEIGHT) {
        boxes[i].suppressed = true;
        continue;
      }
      
      float aspectRatio = (float)boxes[i].width / boxes[i].height;
      if (aspectRatio > 10.0f || aspectRatio < 0.1f) {
        boxes[i].suppressed = true;
      }
    }
  }

  static void removeContained(BoundingBox* boxes, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
      if (boxes[i].suppressed) continue;
      
      for (uint32_t j = 0; j < count; j++) {
        if (i == j || boxes[j].suppressed) continue;
        
        bool contained = (boxes[j].x >= boxes[i].x && 
                         boxes[j].y >= boxes[i].y &&
                         boxes[j].x + boxes[j].width <= boxes[i].x + boxes[i].width && 
                         boxes[j].y + boxes[j].height <= boxes[i].y + boxes[i].height);
        
        if (contained && boxes[i].confidence > boxes[j].confidence) {
          boxes[j].suppressed = true;
        }
      }
    }
  }

  static void keepBestPerClass(BoundingBox* boxes, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
      if (boxes[i].suppressed) continue;
      
      for (uint32_t j = i + 1; j < count; j++) {
        if (boxes[j].suppressed) continue;
        if (!sameClass(boxes[i], boxes[j])) continue;
        
        float scorei = boxes[i].confidence * 0.7f + (boxes[i].area() / 1000.0f) * 0.3f;
        float scorej = boxes[j].confidence * 0.7f + (boxes[j].area() / 1000.0f) * 0.3f;
        
        if (scorei >= scorej) {
          boxes[j].suppressed = true;
        } else {
          boxes[i].suppressed = true;
          break;
        }
      }
    }
  }
};

// ============================================================================
// GLOBAL IMAGE DATA FOR INFERENCE
// ============================================================================
static uint8_t* g_imageData = nullptr;

static int getImageData(size_t offset, size_t length, float* out) {
  if (!g_imageData || !out) return -1;
  
  size_t maxPixels = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  if (offset + length > maxPixels) return -1;

  size_t byteOffset = offset * 3;
  
  for (size_t i = 0; i < length; i++) {
    out[i] = (g_imageData[byteOffset + 2] << 16) + 
             (g_imageData[byteOffset + 1] << 8) + 
              g_imageData[byteOffset];
    byteOffset += 3;
  }
  
  return 0;
}

// ============================================================================
// INFERENCE RUNNER
// ============================================================================
bool runInference(uint8_t* imageData) {
  if (!imageData) {
    Serial.println("[ERROR] Null image data");
    Serial2.println("ERR:NULL_IMAGE");
    return false;
  }

  Serial.println("[INFO] Starting inference...");
  g_imageData = imageData;

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &getImageData;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK) {
    Serial.printf("[ERROR] Inference failed with code: %d\n", err);
    Serial2.println("ERR:INFERENCE_FAILED");
    g_imageData = nullptr;
    return false;
  }

  Serial.printf("[INFO] Found %d raw detections\n", result.bounding_boxes_count);

  uint32_t rawCount = result.bounding_boxes_count;
  BoundingBox* boxes = nullptr;
  
  if (rawCount > 0) {
    boxes = new BoundingBox[rawCount];
    
    for (uint32_t i = 0; i < rawCount; i++) {
      ei_impulse_result_bounding_box_t raw = result.bounding_boxes[i];
      boxes[i].x = raw.x;
      boxes[i].y = raw.y;
      boxes[i].width = raw.width;
      boxes[i].height = raw.height;
      boxes[i].confidence = raw.value;
      strncpy(boxes[i].label, raw.label, 31);
      boxes[i].label[31] = '\0';
      boxes[i].suppressed = false;
      
      Serial.printf("[RAW] %s: %.2f @ (%d,%d,%d,%d)\n", 
                    boxes[i].label, boxes[i].confidence,
                    boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height);
    }

    // Apply all filters
    DetectionFilter::filterInvalid(boxes, rawCount);
    DetectionFilter::removeContained(boxes, rawCount);
    DetectionFilter::applyNMS(boxes, rawCount);
    DetectionFilter::keepBestPerClass(boxes, rawCount);
  }

  // Output result to Arduino via Serial2
  bool found = false;
  if (boxes) {
    for (uint32_t i = 0; i < rawCount; i++) {
      if (!boxes[i].suppressed) {
        Serial.printf("[RESULT] %s: %.2f\n", boxes[i].label, boxes[i].confidence);
        
        // Send to Arduino and echo to Serial Monitor
        char response[32];
        snprintf(response, 32, "OK:%s:%.2f", boxes[i].label, boxes[i].confidence);
        
        // Flush before sending to clear any garbage
        Serial2.flush();
        delay(10);
        
        Serial2.println(response);
        Serial2.flush();  // Wait for transmission to complete
        
        Serial.printf("[TX→Arduino] %s\n", response);
        
        found = true;
        break;
      }
    }
    delete[] boxes;
  }

  if (!found) {
    Serial.println("[RESULT] No valid detection");
    
    Serial2.flush();
    delay(10);
    
    Serial2.println("NONE");
    Serial2.flush();
    
    Serial.println("[TX→Arduino] NONE");
  }

  g_imageData = nullptr;
  return true;
}

// ============================================================================
// GLOBALS
// ============================================================================
CameraManager camera;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // UART0: Serial Monitor (USB-TTL)
  Serial.begin(115200);
  delay(100);
  
  // UART2: Arduino Communication
  Serial2.begin(UART2_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  delay(100);
  
  Serial.println("\n================================");
  Serial.println("ESP32-CAM Dual UART Sorter v2.2");
  Serial.println("================================");
  Serial.println("UART0 (Serial): 115200 baud - USB Monitor");
  Serial.println("UART2 (Serial2): 38400 baud - Arduino");
  Serial.printf("  TX2: GPIO%d -> Arduino RX\n", UART2_TX_PIN);
  Serial.printf("  RX2: GPIO%d <- Arduino TX\n", UART2_RX_PIN);
  Serial.println("================================");
  Serial.println("SAFE PINS - No strapping/PSRAM!");
  Serial.println("Waiting for INFER commands...");
  Serial.println("================================\n");

  if (!camera.initialize()) {
    Serial.println("[FATAL] Camera initialization failed!");
    Serial2.println("ERR:CAMERA_INIT");
    while(1) delay(1000);
  }

  delay(500);
  Serial.println("[READY] System initialized\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Listen for commands from Arduino on Serial2
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();
    
    Serial.printf("[RX←Arduino] Command: %s\n", cmd.c_str());
    
    if (cmd == "INFER") {
      Serial.println("[INFER] Starting capture...");
      
      if (!camera.capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT)) {
        Serial.println("[ERROR] Capture failed");
        Serial2.println("ERR:CAPTURE_FAILED");
        camera.release();
        return;
      }

      runInference(camera.getBuffer());
      camera.release();
    }
  }
  
  // ALSO listen for commands from Serial Monitor (for testing!)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    Serial.printf("[RX←Monitor] Command: %s\n", cmd.c_str());
    
    if (cmd == "INFER") {
      Serial.println("[INFER] Manual test - Starting capture...");
      
      if (!camera.capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT)) {
        Serial.println("[ERROR] Capture failed");
        camera.release();
        return;
      }

      runInference(camera.getBuffer());
      camera.release();
    }
  }
  
  delay(10);
}