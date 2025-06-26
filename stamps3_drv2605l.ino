/*
 * M5Stack StampS3 with DRV2605L Haptic Driver
 * 
 * Features:
 * - Deep sleep on startup after LED blink
 * - Button wake-up functionality
 * - LED blink patterns based on button pressed
 * - Haptic feedback using DRV2605L
 * 
 * Pin Configuration (Corrected):
 * - LED: G39
 * - BTN1: G10 (1 second blink)
 * - BTN2: G9  (2 second blink)
 * - BTN3: G8  (3 second blink)
 * - BTN4: G7  (haptic pattern + deep sleep)
 * - DRV2605L VCC: 5V
 * - DRV2605L GND: GND
 * - DRV2605L SDA: G44
 * - DRV2605L SCL: G42
 * - DRV2605L IN: G40
 * - DRV2605L EN: G41
 */

#include <Wire.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// Pin definitions (CORRECTED)
#define LED_PIN 39
#define BTN1_PIN 10  // 1 second blink
#define BTN2_PIN 9   // 2 second blink
#define BTN3_PIN 8   // 3 second blink
#define BTN4_PIN 7   // haptic pattern
#define DRV_EN_PIN 41
#define DRV_IN_PIN 40
#define SDA_PIN 44   // Corrected from G13 to G44
#define SCL_PIN 42   // Corrected from G15 to G42

// DRV2605L I2C address
#define DRV2605L_ADDR 0x5A

// DRV2605L registers
#define DRV2605L_REG_STATUS 0x00
#define DRV2605L_REG_MODE 0x01
#define DRV2605L_REG_RTPIN 0x02
#define DRV2605L_REG_LIBRARY 0x03
#define DRV2605L_REG_WAVESEQ1 0x04
#define DRV2605L_REG_WAVESEQ2 0x05
#define DRV2605L_REG_WAVESEQ3 0x06
#define DRV2605L_REG_WAVESEQ4 0x07
#define DRV2605L_REG_WAVESEQ5 0x08
#define DRV2605L_REG_WAVESEQ6 0x09
#define DRV2605L_REG_WAVESEQ7 0x0A
#define DRV2605L_REG_WAVESEQ8 0x0B
#define DRV2605L_REG_GO 0x0C
#define DRV2605L_REG_OVERDRIVE 0x0D
#define DRV2605L_REG_SUSTAINPOS 0x0E
#define DRV2605L_REG_SUSTAINNEG 0x0F
#define DRV2605L_REG_BREAK 0x10
#define DRV2605L_REG_AUDIOCTRL 0x11
#define DRV2605L_REG_AUDIOLVL 0x12
#define DRV2605L_REG_AUDIOMAX 0x13
#define DRV2605L_REG_RATEDV 0x16
#define DRV2605L_REG_CLAMPV 0x17
#define DRV2605L_REG_AUTOCALCOMP 0x18
#define DRV2605L_REG_AUTOCALEMP 0x19
#define DRV2605L_REG_FEEDBACK 0x1A
#define DRV2605L_REG_CONTROL1 0x1B
#define DRV2605L_REG_CONTROL2 0x1C
#define DRV2605L_REG_CONTROL3 0x1D
#define DRV2605L_REG_CONTROL4 0x1E
#define DRV2605L_REG_VBAT 0x21
#define DRV2605L_REG_LRARESON 0x22

// Wake-up reason tracking
RTC_DATA_ATTR int wakeup_reason = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(DRV_IN_PIN, OUTPUT);
  
  // Disable DRV2605L initially
  digitalWrite(DRV_EN_PIN, LOW);
  digitalWrite(DRV_IN_PIN, LOW);
  
  // Check wake-up reason
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  
  if (wakeup_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // First boot - blink LED once and go to sleep
    Serial.println("First boot - blinking LED and going to deep sleep");
    blinkLED(1, 200);
    goToDeepSleep();
  } else if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0) {
    // Woken up by button press
    handleButtonWakeup();
  }
}

void loop() {
  // Should never reach here due to deep sleep
  delay(1000);
}

void handleButtonWakeup() {
  // Small delay to debounce
  delay(50);
  
  // Check which button was pressed
  if (digitalRead(BTN1_PIN) == LOW) {
    Serial.println("BTN1 pressed - 1 second blink pattern");
    blinkPattern(1);
  } else if (digitalRead(BTN2_PIN) == LOW) {
    Serial.println("BTN2 pressed - 2 second blink pattern");
    blinkPattern(2);
  } else if (digitalRead(BTN3_PIN) == LOW) {
    Serial.println("BTN3 pressed - 3 second blink pattern");
    blinkPattern(3);
  } else if (digitalRead(BTN4_PIN) == LOW) {
    Serial.println("BTN4 pressed - haptic pattern");
    runHapticPattern();
    goToDeepSleep();
    return;
  }
  
  // After handling button, go back to deep sleep
  goToDeepSleep();
}

void blinkPattern(int seconds) {
  // Blink LED twice per second for the specified duration
  int totalBlinks = seconds * 2;
  for (int i = 0; i < totalBlinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}

void blinkLED(int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void runHapticPattern() {
  // Enable DRV2605L
  digitalWrite(DRV_EN_PIN, HIGH);
  delay(100); // Allow time for power-up
  
  // Initialize I2C with corrected pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize DRV2605L
  if (initDRV2605L()) {
    Serial.println("DRV2605L initialized successfully");
    
    // Run haptic patterns 50-123
    for (int pattern = 50; pattern <= 123; pattern++) {
      Serial.print("Playing haptic effect: ");
      Serial.println(pattern);
      playHapticEffect(pattern);
      delay(500); // Delay between patterns
    }
  } else {
    Serial.println("Failed to initialize DRV2605L");
    // Blink LED to indicate error
    blinkLED(5, 100);
  }
  
  // Disable DRV2605L
  digitalWrite(DRV_EN_PIN, LOW);
  digitalWrite(DRV_IN_PIN, LOW);
}

bool initDRV2605L() {
  // Check if device is present
  Wire.beginTransmission(DRV2605L_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("DRV2605L not found on I2C bus");
    return false;
  }
  
  // Set to standby mode
  writeRegister(DRV2605L_REG_MODE, 0x40);
  delay(10);
  
  // Set library to ERM (Eccentric Rotating Mass)
  writeRegister(DRV2605L_REG_LIBRARY, 1);
  
  // Set to internal trigger mode
  writeRegister(DRV2605L_REG_MODE, 0x00);
  
  // Set rated voltage (adjust based on your actuator - typical ERM value)
  writeRegister(DRV2605L_REG_RATEDV, 0x3E);
  
  // Set overdrive clamp voltage
  writeRegister(DRV2605L_REG_CLAMPV, 0x8C);
  
  // Set feedback control for ERM
  writeRegister(DRV2605L_REG_FEEDBACK, 0xB6);
  
  // Set control registers
  writeRegister(DRV2605L_REG_CONTROL1, 0x93);
  writeRegister(DRV2605L_REG_CONTROL2, 0xF5);
  writeRegister(DRV2605L_REG_CONTROL3, 0x80);
  
  Serial.println("DRV2605L registers configured");
  return true;
}

void playHapticEffect(uint8_t effect) {
  // Clear waveform sequencer
  for (int i = 0; i < 8; i++) {
    writeRegister(DRV2605L_REG_WAVESEQ1 + i, 0);
  }
  
  // Set the effect in the first waveform slot
  writeRegister(DRV2605L_REG_WAVESEQ1, effect);
  
  // Trigger the effect
  writeRegister(DRV2605L_REG_GO, 1);
  
  // Wait for effect to complete
  delay(50);
  uint8_t go_bit = 1;
  int timeout = 0;
  while (go_bit == 1 && timeout < 100) {
    go_bit = readRegister(DRV2605L_REG_GO);
    delay(10);
    timeout++;
  }
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(DRV2605L_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("I2C write error: ");
    Serial.println(result);
  }
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(DRV2605L_ADDR);
  Wire.write(reg);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("I2C read error: ");
    Serial.println(result);
    return 0;
  }
  
  Wire.requestFrom(DRV2605L_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void goToDeepSleep() {
  Serial.println("Going to deep sleep...");
  Serial.flush();
  
  // Configure wake-up sources (ESP32 ext0 can only use one pin, so we'll use ext1 for multiple pins)
  uint64_t ext_wakeup_pin_mask = 0;
  ext_wakeup_pin_mask |= (1ULL << BTN1_PIN);
  ext_wakeup_pin_mask |= (1ULL << BTN2_PIN);
  ext_wakeup_pin_mask |= (1ULL << BTN3_PIN);
  ext_wakeup_pin_mask |= (1ULL << BTN4_PIN);
  
  esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW);
  
  // Configure pins to maintain state during deep sleep
  rtc_gpio_pullup_en((gpio_num_t)BTN1_PIN);
  rtc_gpio_pullup_en((gpio_num_t)BTN2_PIN);
  rtc_gpio_pullup_en((gpio_num_t)BTN3_PIN);
  rtc_gpio_pullup_en((gpio_num_t)BTN4_PIN);
  
  // Ensure LED is off
  digitalWrite(LED_PIN, LOW);
  
  // Ensure DRV2605L is disabled
  digitalWrite(DRV_EN_PIN, LOW);
  digitalWrite(DRV_IN_PIN, LOW);
  
  // Enter deep sleep
  esp_deep_sleep_start();
}