// ============================================================================
// ARDUINO UNO 3-CLASS SORTER - UPDATED FOR ESP32 UART2
// ============================================================================
// Board: Arduino UNO
// Required Libraries: 
//   - LiquidCrystal_I2C (by Frank de Brabander)
//   - NeoSWSerial (by SlashDevin)
// UART Baud: 38400 (max supported by NeoSWSerial)
// NEW: Updated for ESP32 GPIO14/13 (safe pins)
// ============================================================================

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NeoSWSerial.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define TRIG_PIN 7
#define ECHO_PIN 6
#define SERVO1_PIN 9
#define SERVO2_PIN 10
// ESP32 UART2 connection (GPIO14/13 - safe, no strapping/PSRAM conflicts):
// ESP32 GPIO14 (TX2) -> Arduino D3 (RX)
// ESP32 GPIO13 (RX2) <- Arduino D2 (TX) via voltage divider (1kΩ + 2kΩ)
#define ESP_RX_PIN 3  // Arduino D3 <- ESP32 GPIO14 (TX2)
#define ESP_TX_PIN 2  // Arduino D2 -> ESP32 GPIO13 (RX2) via voltage divider!

// ============================================================================
// SERVO ANGLES
// ============================================================================
#define S1_HOME 0
#define S1_REPOSITION1 45   // First reposition angle
#define S1_REPOSITION2 90   // Second reposition angle
#define S1_MID 90
#define S1_FAR 180
#define S2_HOME 90
#define S2_RIGHT 20   // Full drop RIGHT (70° movement)
#define S2_LEFT 160   // Full drop LEFT (70° movement) - FIXED!

// ============================================================================
// TIMING (milliseconds)
// ============================================================================
#define SETTLE_MS 250
#define HOLD_MS 300
#define TIMEOUT_MS 2500
#define HC_DELAY_MS 50

// ============================================================================
// DETECTION PARAMETERS
// ============================================================================
#define DISTANCE_MIN 3   // cm - minimum detection distance
#define DISTANCE_MAX 33  // cm - maximum detection distance
#define MAX_VOTES 3
#define MAJORITY_VOTES 2
#define MIN_CONFIDENCE 0.65
#define MAX_REPOSITIONS 2  // Try repositioning twice before giving up

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================
Servo servo1;
Servo servo2;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // If LCD blank, try 0x3F
NeoSWSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);  // RX=3, TX=2

// ============================================================================
// STATE MACHINE
// ============================================================================
typedef enum {
  STATE_IDLE,
  STATE_DETECT,
  STATE_REQUEST,
  STATE_WAIT,
  STATE_REPOSITION,
  STATE_DECIDE,
  STATE_SORT,
  STATE_ERROR
} SystemState;

SystemState state = STATE_IDLE;

// ============================================================================
// VOTING DATA
// ============================================================================
uint8_t votesCan = 0;
uint8_t votesGlass = 0;
uint8_t votesBottle = 0;
float confCan = 0.0;
float confGlass = 0.0;
float confBottle = 0.0;
unsigned long deadline = 0;
uint8_t repositionAttempts = 0;
uint8_t consecutiveNones = 0;

// ============================================================================
// ULTRASONIC SENSOR
// ============================================================================
uint16_t measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  
  uint16_t cm = duration / 58;
  return (cm > 300) ? 999 : cm;
}

bool isObjectPresent() {
  uint16_t d1 = measureDistance();
  delay(HC_DELAY_MS);
  uint16_t d2 = measureDistance();
  
  // Object is present if BOTH readings are in valid range (3-33cm)
  bool inRange1 = (d1 >= DISTANCE_MIN && d1 <= DISTANCE_MAX);
  bool inRange2 = (d2 >= DISTANCE_MIN && d2 <= DISTANCE_MAX);
  
  return (inRange1 && inRange2);
}

// ============================================================================
// SERVO CONTROL
// ============================================================================
void moveServo1(int angle) {
  servo1.write(angle);
  delay(SETTLE_MS);
}

void moveServo2(int angle) {
  servo2.write(angle);
  delay(SETTLE_MS);
}

void performDrop(bool isRight) {
  if (isRight) {
    moveServo2(S2_RIGHT);
    delay(HOLD_MS);
    moveServo2(S2_HOME);
  } else {
    moveServo2(S2_LEFT);
    delay(HOLD_MS);
    moveServo2(S2_HOME);
  }
}

void sortToBin(uint8_t bin) {
  // Always return to home first
  moveServo1(S1_HOME);
  
  switch(bin) {
    case 1:  // CAN
      moveServo1(S1_HOME);
      performDrop(true);
      moveServo1(S1_HOME);
      break;
      
    case 2:  // GLASS
      moveServo1(S1_MID);
      performDrop(true);
      moveServo1(S1_HOME);
      break;
      
    case 3:  // BOTTLE
      moveServo1(S1_FAR);
      performDrop(true);
      moveServo1(S1_HOME);
      break;
      
    case 4:  // MIXED/UNKNOWN
    default:
      moveServo1(S1_MID);
      performDrop(false);
      moveServo1(S1_HOME);
      break;
  }
}

// ============================================================================
// LCD DISPLAY
// ============================================================================
void updateDisplay(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  if (line2 && line2[0]) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}

// ============================================================================
// VOTING SYSTEM
// ============================================================================
void resetVotes() {
  votesCan = 0;
  votesGlass = 0;
  votesBottle = 0;
  confCan = 0.0;
  confGlass = 0.0;
  confBottle = 0.0;
  repositionAttempts = 0;
  consecutiveNones = 0;
}

void recordVote(const char* className, float confidence) {
  if (strcmp(className, "CAN3") == 0) {
    votesCan++;
    confCan += confidence;
  } else if (strcmp(className, "GLASS3") == 0) {
    votesGlass++;
    confGlass += confidence;
  } else if (strcmp(className, "BOTTLE3") == 0) {
    votesBottle++;
    confBottle += confidence;
  }
  consecutiveNones = 0;  // Reset on successful detection
}

uint8_t getTotalVotes() {
  return votesCan + votesGlass + votesBottle;
}

bool hasMajority() {
  return (votesCan >= MAJORITY_VOTES || 
          votesGlass >= MAJORITY_VOTES || 
          votesBottle >= MAJORITY_VOTES);
}

uint8_t calculateBin() {
  // Check majority
  if (votesCan >= MAJORITY_VOTES) return 1;
  if (votesGlass >= MAJORITY_VOTES) return 2;
  if (votesBottle >= MAJORITY_VOTES) return 3;
  
  // No votes
  if (getTotalVotes() == 0) return 4;
  
  // Tie-break by confidence
  float maxConf = confCan;
  uint8_t bin = 1;
  
  if (confGlass > maxConf) {
    maxConf = confGlass;
    bin = 2;
  }
  
  if (confBottle > maxConf) {
    maxConf = confBottle;
    bin = 3;
  }
  
  // All zero
  if (maxConf < 0.01) return 4;
  
  return bin;
}

void showVotes() {
  char buffer[17];
  snprintf(buffer, 17, "C:%d G:%d B:%d", votesCan, votesGlass, votesBottle);
  updateDisplay("Voting...", buffer);
}

// ============================================================================
// ESP32 COMMUNICATION
// ============================================================================
void requestInference() {
  // Flush any leftover data from previous response
  while(espSerial.available()) {
    espSerial.read();
  }
  
  delay(10);
  
  espSerial.println("INFER");
  deadline = millis() + TIMEOUT_MS;
}

bool parseResponse(char* buffer, char* outClass, float* outConf) {
  // Empty buffer
  if (buffer[0] == '\0') {
    return false;
  }
  
  // Check "NONE"
  if (strncmp(buffer, "NONE", 4) == 0) {
    return false;
  }
  
  // Check "ERR:"
  if (strncmp(buffer, "ERR:", 4) == 0) {
    return false;
  }
  
  // Parse "OK:CLASS:CONF" - be flexible with garbage
  // Look for "OK" anywhere in the first few characters
  char* okPtr = strstr(buffer, "OK");
  if (okPtr) {
    // Skip past "OK" and any garbage, look for ':'
    char* colonPtr = strchr(okPtr + 2, ':');
    if (!colonPtr) return false;
    
    // Extract class name
    char* classPtr = colonPtr + 1;
    char* secondColonPtr = strchr(classPtr, ':');
    if (!secondColonPtr) return false;
    
    // Copy class name
    size_t classLen = secondColonPtr - classPtr;
    if (classLen > 15) classLen = 15;
    strncpy(outClass, classPtr, classLen);
    outClass[classLen] = '\0';
    
    // Extract confidence
    *outConf = atof(secondColonPtr + 1);
    
    return (*outConf >= MIN_CONFIDENCE);
  }
  
  return false;
}

// ============================================================================
// STATE MACHINE
// ============================================================================
void runStateMachine() {
  switch(state) {
    
    case STATE_IDLE:
      updateDisplay("Ready", "");
      if (isObjectPresent()) {
        Serial.println(F(">>> OBJECT DETECTED! Moving to DETECT state"));
        state = STATE_DETECT;
      }
      break;
    
    case STATE_DETECT:
      updateDisplay("Detecting", "Object present");
      Serial.println(F(">>> STATE_DETECT: Confirming object presence"));
      delay(100);
      
      if (isObjectPresent()) {
        Serial.println(F(">>> Object confirmed, resetting votes"));
        resetVotes();
        moveServo1(S1_HOME);  // Ensure starting position
        state = STATE_REQUEST;
      } else {
        Serial.println(F(">>> Object disappeared, back to IDLE"));
        state = STATE_IDLE;
      }
      break;
    
    case STATE_REQUEST:
      updateDisplay("Requesting", "Inference...");
      Serial.println(F(">>> STATE_REQUEST: Sending INFER to ESP32"));
      requestInference();
      state = STATE_WAIT;
      break;
    
    case STATE_WAIT:
      // Timeout check
      if (millis() > deadline) {
        Serial.println(F(">>> TIMEOUT waiting for ESP32 response"));
        state = STATE_ERROR;
        break;
      }
      
      // Check for response
      if (espSerial.available()) {
        char buffer[32];
        uint8_t idx = 0;
        
        // Wait a bit for full message to arrive
        delay(50);
        
        unsigned long readStart = millis();
        while (millis() - readStart < 200) {  // Increased timeout
          if (espSerial.available()) {
            char c = espSerial.read();
            if (c == '\n' || c == '\r') break;
            // Filter out obvious garbage (non-printable except digits, letters, colon, dot)
            if ((c >= '0' && c <= '9') || 
                (c >= 'A' && c <= 'Z') || 
                (c >= 'a' && c <= 'z') || 
                c == ':' || c == '.') {
              if (idx < 31) buffer[idx++] = c;
            }
          }
        }
        buffer[idx] = '\0';
        
        Serial.print(F(">>> RX from ESP32: ["));
        Serial.print(buffer);
        Serial.println(F("]"));
        
        // Skip empty responses
        if (idx == 0 || buffer[0] == '\0') {
          Serial.println(F(">>> Empty response, ignoring"));
          break;  // Stay in WAIT state
        }
        
        // Parse
        char className[16];
        float confidence = 0.0;
        
        if (parseResponse(buffer, className, &confidence)) {
          // DETECTION SUCCESS
          Serial.print(F(">>> PARSED OK: Class="));
          Serial.print(className);
          Serial.print(F(" Conf="));
          Serial.println(confidence);
          
          recordVote(className, confidence);
          showVotes();
          
          Serial.print(F(">>> Total votes: "));
          Serial.print(getTotalVotes());
          Serial.print(F(" (C:"));
          Serial.print(votesCan);
          Serial.print(F(" G:"));
          Serial.print(votesGlass);
          Serial.print(F(" B:"));
          Serial.print(votesBottle);
          Serial.println(F(")"));
          
          delay(500);
          
          // Decision check
          if (hasMajority() || getTotalVotes() >= MAX_VOTES) {
            Serial.println(F(">>> Decision reached, moving to DECIDE"));
            state = STATE_DECIDE;
          } else {
            Serial.println(F(">>> Need more votes, requesting again"));
            state = STATE_REQUEST;  // Get another vote
          }
        } else {
          // NO DETECTION ("NONE")
          Serial.println(F(">>> NO DETECTION (NONE or ERR)"));
          consecutiveNones++;
          
          // If we've already started repositioning, don't check object presence
          // (servo movement might have moved it out of ultrasonic range)
          if (repositionAttempts > 0) {
            Serial.println(F(">>> Already repositioning, continuing..."));
            
            if (repositionAttempts < MAX_REPOSITIONS) {
              state = STATE_REPOSITION;
            } else {
              // Gave up after max repositions
              Serial.println(F(">>> Max repositions reached, giving up -> Bin 4"));
              
              // IMPORTANT: Reset all votes so calculateBin() returns 4
              votesCan = 0;
              votesGlass = 0;
              votesBottle = 0;
              confCan = 0.0;
              confGlass = 0.0;
              confBottle = 0.0;
              
              updateDisplay("Can't detect", "-> Bin 4");
              delay(1000);
              state = STATE_DECIDE;  // Will go to Bin 4
            }
          } else {
            // Haven't repositioned yet - check if object is still present
            if (isObjectPresent()) {
              Serial.print(F(">>> Object present but can't detect. Reposition attempts: "));
              Serial.println(repositionAttempts);
              
              // Object present but camera can't see it
              if (repositionAttempts < MAX_REPOSITIONS) {
                state = STATE_REPOSITION;
              } else {
                // Gave up after max repositions
                Serial.println(F(">>> Max repositions reached, giving up -> Bin 4"));
                
                // IMPORTANT: Reset all votes so calculateBin() returns 4
                votesCan = 0;
                votesGlass = 0;
                votesBottle = 0;
                confCan = 0.0;
                confGlass = 0.0;
                confBottle = 0.0;
                
                updateDisplay("Can't detect", "-> Bin 4");
                delay(1000);
                state = STATE_DECIDE;  // Will go to Bin 4
              }
            } else {
              Serial.println(F(">>> Object disappeared, back to IDLE"));
              // Object disappeared
              state = STATE_IDLE;
            }
          }
        }
      }
      break;
    
    case STATE_REPOSITION:
      {
        repositionAttempts++;
        
        Serial.print(F(">>> STATE_REPOSITION: Attempt #"));
        Serial.println(repositionAttempts);
        
        char line1[17];
        char line2[17];
        snprintf(line1, 17, "Reposition #%d", repositionAttempts);
        strcpy(line2, "Trying again...");
        updateDisplay(line1, line2);
        
        // Move servo to different angle
        if (repositionAttempts == 1) {
          Serial.println(F(">>> Moving to REPOSITION1 angle (45°)"));
          moveServo1(S1_REPOSITION1);  // 45 degrees
        } else {
          Serial.println(F(">>> Moving to REPOSITION2 angle (90°)"));
          moveServo1(S1_REPOSITION2);  // 90 degrees
        }
        
        delay(300);  // Let object settle
        state = STATE_REQUEST;  // Try inference again
      }
      break;
    
    case STATE_DECIDE:
      {
        uint8_t bin = calculateBin();
        
        Serial.print(F(">>> STATE_DECIDE: Calculated Bin "));
        Serial.println(bin);
        
        char line1[17];
        char line2[17];
        
        snprintf(line1, 17, "Bin %d", bin);
        
        switch(bin) {
          case 1: strcpy(line2, "CAN"); break;
          case 2: strcpy(line2, "GLASS"); break;
          case 3: strcpy(line2, "BOTTLE"); break;
          default: strcpy(line2, "MIXED"); break;
        }
        
        updateDisplay(line1, line2);
        delay(1000);
        
        state = STATE_SORT;
      }
      break;
    
    case STATE_SORT:
      {
        uint8_t bin = calculateBin();
        Serial.print(F(">>> STATE_SORT: Sorting to bin "));
        Serial.println(bin);
        sortToBin(bin);
        Serial.println(F(">>> Sorting complete, back to IDLE"));
        state = STATE_IDLE;
      }
      break;
    
    case STATE_ERROR:
      Serial.println(F(">>> STATE_ERROR: Timeout or error occurred"));
      
      // Reset votes so it goes to Bin 4
      votesCan = 0;
      votesGlass = 0;
      votesBottle = 0;
      confCan = 0.0;
      confGlass = 0.0;
      confBottle = 0.0;
      
      updateDisplay("Error/Timeout", "-> Bin 4");
      delay(1500);
      moveServo1(S1_HOME);  // Return home first
      sortToBin(4);
      Serial.println(F(">>> Error handled, back to IDLE"));
      state = STATE_IDLE;
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Debug serial (optional - comment out to save ~100 bytes RAM)
  Serial.begin(9600);
  Serial.println(F("Arduino Sorter v3.1"));
  Serial.println(F("ESP32 UART2 Config"));
  Serial.println(F("RX: D3 <- ESP GPIO14"));
  Serial.println(F("TX: D2 -> ESP GPIO13"));
  Serial.println(F("Baud: 38400"));
  
  // ESP32 communication (UART2 of ESP32)
  espSerial.begin(38400);
  
  // GPIO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(S1_HOME);
  servo2.write(S2_HOME);
  
  // LCD
  lcd.init();
  lcd.backlight();
  updateDisplay("Initializing", "Wait 3 sec...");
  
  // Wait for ESP32
  delay(3000);
  
  updateDisplay("System Ready", "v3.1");
  delay(1000);
  
  Serial.println(F("Ready!"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  runStateMachine();
  delay(10);
}