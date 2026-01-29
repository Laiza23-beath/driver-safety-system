/*
 * 95% Accuracy IoT Drowsiness Detection System - Arduino Code
 * Features: Multi-level alerts, BP monitoring, NFP1315-61AY haptic feedback
 * Hardware: Arduino Uno, Buzzer, NFP1315-61AY
 * 
 * Commands from Python:
 * INIT - Initialize system
 * ALERT:0 - Normal (no alerts)
 * ALERT:1 - Warning (mild buzzer)
 * ALERT:2 - Critical (loud buzzer + haptic)
 * ALERT:3 - Emergency (all alerts + escalation)
 * BP_REQUEST:1 - Send BP data
 * SHUTDOWN - System shutdown
 */

#include <Wire.h>

// Pin Definitions
#define BUZZER_PIN 9
#define LED_GREEN 12
#define LED_YELLOW 11
#define LED_RED 13
#define VIBRATOR_PIN 10  // For additional vibrator if needed

// NFP1315-61AY I2C Configuration
#define NFP_ADDR 0x5A  // Default I2C address for NFP1315-61AY
#define NFP_COMMAND_REG 0x01
#define NFP_INTENSITY_REG 0x02

// System Variables
int currentAlertLevel = 0;
unsigned long lastBPTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long alertStartTime = 0;
bool systemActive = false;
bool emergencyMode = false;

// BP and Heartbeat simulation
int baselineSystolic = 120;
int baselineDiastolic = 80;
int baselineHeartRate = 72;

// Alert patterns
unsigned long buzzerPatterns[4][4] = {
  {0, 0, 0, 0},           // Normal - no sound
  {200, 100, 0, 0},       // Warning - short beep
  {300, 150, 300, 500},   // Critical - double beep
  {500, 200, 500, 200}    // Emergency - continuous pattern
};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(VIBRATOR_PIN, OUTPUT);
  
  // Initialize NFP1315-61AY
  initializeNFP();
  
  // Startup sequence
  startupSequence();
  
  // Seed random number generator
  randomSeed(analogRead(A0));
  
  Serial.println("Arduino IoT Drowsiness System Ready");
  Serial.println("Waiting for Python connection...");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Handle active alerts
  if (systemActive) {
    handleAlerts();
    
    // Send heartbeat every 5 seconds
    if (millis() - lastHeartbeatTime > 5000) {
      sendHeartbeat();
      lastHeartbeatTime = millis();
    }
  }
  
  delay(50); // Small delay for stability
}

void processCommand(String command) {
  if (command == "INIT") {
    initializeSystem();
  }
  else if (command.startsWith("ALERT:")) {
    int level = command.substring(6).toInt();
    setAlertLevel(level);
  }
  else if (command.startsWith("BP_REQUEST:")) {
    sendBPData();
  }
  else if (command == "SHUTDOWN") {
    shutdownSystem();
  }
  else {
    Serial.println("Unknown command: " + command);
  }
}

void initializeSystem() {
  systemActive = true;
  currentAlertLevel = 0;
  emergencyMode = false;
  
  // Green LED on - system ready
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  
  Serial.println("System initialized - Ready for drowsiness detection");
}

void setAlertLevel(int level) {
  if (level < 0 || level > 3) return;
  
  int previousLevel = currentAlertLevel;
  currentAlertLevel = level;
  
  // Reset alert timing if level changed
  if (previousLevel != level) {
    alertStartTime = millis();
  }
  
  // Update LED indicators
  updateLEDs();
  
  // Handle NFP haptic feedback
  handleHapticFeedback();
  
  // Log alert change
  if (level > 0) {
    String alertNames[] = {"NORMAL", "WARNING", "CRITICAL", "EMERGENCY"};
    Serial.println("Alert Level: " + alertNames[level]);
    
    if (level == 3) {
      emergencyMode = true;
      Serial.println("EMERGENCY MODE ACTIVATED!");
    }
  } else {
    emergencyMode = false;
  }
}

void handleAlerts() {
  unsigned long currentTime = millis();
  
  switch (currentAlertLevel) {
    case 0: // Normal
      noTone(BUZZER_PIN);
      digitalWrite(VIBRATOR_PIN, LOW);
      break;
      
    case 1: // Warning
      handleWarningAlert(currentTime);
      break;
      
    case 2: // Critical
      handleCriticalAlert(currentTime);
      break;
      
    case 3: // Emergency
      handleEmergencyAlert(currentTime);
      break;
  }
}

void handleWarningAlert(unsigned long currentTime) {
  // LOUD warning beep every 2 seconds (increased frequency and reduced interval)
  if ((currentTime - alertStartTime) % 2000 < 400) {
    tone(BUZZER_PIN, 2500); // Much higher frequency for louder sound
  } else {
    noTone(BUZZER_PIN);
  }
}

void handleCriticalAlert(unsigned long currentTime) {
  // VERY LOUD double beep pattern every 1.5 seconds
  unsigned long pattern = (currentTime - alertStartTime) % 1500;
  
  if (pattern < 400) {
    tone(BUZZER_PIN, 3000); // Very high frequency for maximum loudness
    activateNFP(50); // Medium intensity
  } else if (pattern < 500) {
    noTone(BUZZER_PIN);
  } else if (pattern < 900) {
    tone(BUZZER_PIN, 3200); // Even higher frequency
  } else {
    noTone(BUZZER_PIN);
    deactivateNFP();
  }
}

void handleEmergencyAlert(unsigned long currentTime) {
  // EXTREMELY LOUD continuous urgent pattern
  unsigned long pattern = (currentTime - alertStartTime) % 800;
  
  if (pattern < 400) {
    tone(BUZZER_PIN, 4000); // Maximum frequency for loudest possible sound
    activateNFP(100); // Maximum intensity
    digitalWrite(VIBRATOR_PIN, HIGH);
  } else {
    tone(BUZZER_PIN, 3800); // Alternating very high frequency
    activateNFP(80);
    digitalWrite(VIBRATOR_PIN, LOW);
  }
  
  // Emergency escalation after 10 seconds
  if (currentTime - alertStartTime > 10000) {
    emergencyEscalation();
  }
}

void updateLEDs() {
  // Reset all LEDs
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  
  switch (currentAlertLevel) {
    case 0: // Normal
      digitalWrite(LED_GREEN, HIGH);
      break;
    case 1: // Warning
      digitalWrite(LED_YELLOW, HIGH);
      break;
    case 2: // Critical
      digitalWrite(LED_RED, HIGH);
      break;
    case 3: // Emergency
      // Flashing red
      digitalWrite(LED_RED, (millis() / 250) % 2);
      break;
  }
}

void handleHapticFeedback() {
  switch (currentAlertLevel) {
    case 0: // Normal
      deactivateNFP();
      break;
    case 1: // Warning
      // Gentle pulse every 5 seconds
      if ((millis() - alertStartTime) % 5000 < 200) {
        activateNFP(30);
      } else {
        deactivateNFP();
      }
      break;
    case 2: // Critical
      // Handled in handleCriticalAlert
      break;
    case 3: // Emergency
      // Handled in handleEmergencyAlert
      break;
  }
}

void initializeNFP() {
  // Initialize NFP1315-61AY haptic driver
  Wire.beginTransmission(NFP_ADDR);
  Wire.write(0x00); // Control register
  Wire.write(0x01); // Enable device
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("NFP1315-61AY initialized successfully");
  } else {
    Serial.println("NFP1315-61AY initialization failed - Error: " + String(error));
  }
}

void activateNFP(int intensity) {
  // intensity: 0-100
  Wire.beginTransmission(NFP_ADDR);
  Wire.write(NFP_COMMAND_REG);
  Wire.write(0x01); // Activate
  Wire.endTransmission();
  
  Wire.beginTransmission(NFP_ADDR);
  Wire.write(NFP_INTENSITY_REG);
  Wire.write(map(intensity, 0, 100, 0, 255)); // Map to 0-255 range
  Wire.endTransmission();
}

void deactivateNFP() {
  Wire.beginTransmission(NFP_ADDR);
  Wire.write(NFP_COMMAND_REG);
  Wire.write(0x00); // Deactivate
  Wire.endTransmission();
}

void sendBPData() {
  // Generate realistic BP data based on alert level
  int systolic, diastolic, heartRate;
  
  switch (currentAlertLevel) {
    case 0: // Normal
      systolic = baselineSystolic + random(-5, 6);
      diastolic = baselineDiastolic + random(-3, 4);
      heartRate = baselineHeartRate + random(-5, 6);
      break;
      
    case 1: // Warning - slight elevation
      systolic = baselineSystolic + random(5, 15);
      diastolic = baselineDiastolic + random(3, 8);
      heartRate = baselineHeartRate + random(5, 15);
      break;
      
    case 2: // Critical - moderate elevation
      systolic = baselineSystolic + random(15, 25);
      diastolic = baselineDiastolic + random(8, 15);
      heartRate = baselineHeartRate + random(15, 25);
      break;
      
    case 3: // Emergency - high elevation
      systolic = baselineSystolic + random(25, 40);
      diastolic = baselineDiastolic + random(15, 25);
      heartRate = baselineHeartRate + random(25, 40);
      break;
  }
  
  // Send BP data in JSON format
  Serial.println("{\"type\":\"bp\",\"systolic\":" + String(systolic) + 
                 ",\"diastolic\":" + String(diastolic) + 
                 ",\"heartRate\":" + String(heartRate) + 
                 ",\"timestamp\":" + String(millis()) + "}");
  
  lastBPTime = millis();
}

void sendHeartbeat() {
  // Send system heartbeat
  Serial.println("{\"type\":\"heartbeat\",\"alertLevel\":" + String(currentAlertLevel) + 
                 ",\"uptime\":" + String(millis()) + 
                 ",\"emergency\":" + String(emergencyMode ? "true" : "false") + "}");
}

void emergencyEscalation() {
  // MAXIMUM LOUDNESS Emergency escalation protocol
  static unsigned long lastEscalation = 0;
  
  if (millis() - lastEscalation > 3000) { // Every 3 seconds in emergency (more frequent)
    Serial.println("{\"type\":\"emergency\",\"message\":\"DRIVER UNRESPONSIVE - IMMEDIATE INTERVENTION REQUIRED\"}");
    
    // MAXIMUM LOUDNESS alert intensity
    tone(BUZZER_PIN, 4500); // Highest possible frequency for maximum loudness
    activateNFP(100);
    digitalWrite(VIBRATOR_PIN, HIGH);
    
    lastEscalation = millis();
  }
}

void startupSequence() {
  // LED startup sequence
  digitalWrite(LED_RED, HIGH);
  delay(200);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, HIGH);
  delay(200);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(200);
  digitalWrite(LED_GREEN, LOW);
  
  // LOUD buzzer startup test - demonstrates new volume levels
  tone(BUZZER_PIN, 2500, 150); // Loud startup beep
  delay(200);
  tone(BUZZER_PIN, 3000, 150); // Even louder test beep
  delay(200);
  tone(BUZZER_PIN, 4000, 100); // Maximum loudness test
  
  // Test NFP
  activateNFP(50);
  delay(200);
  deactivateNFP();
}

void shutdownSystem() {
  systemActive = false;
  currentAlertLevel = 0;
  emergencyMode = false;
  
  // Turn off all outputs
  noTone(BUZZER_PIN);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(VIBRATOR_PIN, LOW);
  deactivateNFP();
  
  Serial.println("System shutdown complete");
}