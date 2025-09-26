#include <Arduino.h>
#include <Adafruit_TCA8418.h>
#include <Wire.h>
#include <math.h>
#include "IS31FL373x.h"

// Demo: IS31FL3737 LED driver + TCA8418 keypad controller
// Hardware: 4x4 LED matrix + 4x4 button matrix
// Modes: ANIMATED, INTERACTIVE, DEBUG

#define VERSION "1.0.1"

// Hardware configuration
#define LED_ROWS 4
#define LED_COLS 4
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

// Hardware objects
IS31FL3737 led_driver(ADDR::GND);
Adafruit_TCA8418 keypad;

// Mode management
int current_mode = 0;
const char* mode_names[] = {"ANIMATED", "INTERACTIVE", "CYCLE"};

// Button state
bool dots[KEYPAD_ROWS][KEYPAD_COLS] = {0};
unsigned long mode_debounce = 0;

void clear_leds() {
  led_driver.clear();
  led_driver.show();
}

// Animation variables
unsigned long animation_start = 0;
unsigned long debug_start = 0;
int debug_led = 0;

void switch_mode() {
  current_mode = (current_mode + 1) % 3;
  clear_leds();
  animation_start = millis();
  debug_led = 0;
  Serial.printf("Mode: %s\n", mode_names[current_mode]);
}

void check_mode_button() {
  static bool last_state = HIGH;
  bool current_state = digitalRead(0);
  
  if (last_state == HIGH && current_state == LOW) {
    unsigned long now = millis();
    if (now - mode_debounce > 500) {
      switch_mode();
      mode_debounce = now;
    }
  }
  last_state = current_state;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.printf("Lights and Buttons Demo v%s\n", VERSION);
  
  // Initialize mode button
  pinMode(0, INPUT_PULLUP);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(800000);
  
  // Initialize keypad driver
  if (!keypad.begin()) {
    Serial.println("TCA8418 keypad init failed!");
    return;
  }
  keypad.matrix(KEYPAD_ROWS, KEYPAD_COLS);
  Serial.printf("Keypad: %dx%d matrix\n", KEYPAD_ROWS, KEYPAD_COLS);
  
  // Initialize LED driver
  if (!led_driver.begin()) {
    Serial.println("IS31FL3737 LED init failed!");
    return;
  }
  led_driver.setGlobalCurrent(128);
  Serial.printf("LEDs: %dx%d matrix\n", LED_ROWS, LED_COLS);
  
  // Test LEDs
  led_driver.drawPixel(0, 0, 255);
  led_driver.drawPixel(1, 1, 255);
  led_driver.show();
  delay(1000);
  clear_leds();
  
  Serial.println("Ready! Press button to switch modes.");
  animation_start = millis();
}


void loop() {
  check_mode_button();
  
  // Handle keypad input
  if (keypad.available() > 0) {
    int k = keypad.getEvent();
    bool pressed = k & 0x80;
    k &= 0x7F;
    k--;
    
    // The TCA8418 numbers keys in column-major order (col*rows + row, with rows=10), 
    // but our matrix is row-major and columns are reversed for LED layout.
    // This calculation converts the TCA8418 key index (k) to our (row, col) coordinates:
    //   - col: reverse the column index and map to 0-3
    //   - row: use the remainder to get the row index
    int tca8418_rows = 10;
    int col = 3 - (k / tca8418_rows); 
    int row = k % tca8418_rows; 
    
    if (row >= 0 && row < KEYPAD_ROWS && col >= 0 && col < KEYPAD_COLS) {
      if (pressed) {
        Serial.printf("KEY PRESS (%d,%d) key=%d\n", row, col, k);
        
        if (current_mode == 1) { // INTERACTIVE mode
          dots[row][col] = !dots[row][col];
          Serial.printf("  -> LED %s\n", dots[row][col] ? "ON" : "OFF");
        }
      } else {
        Serial.printf("KEY RELEASE (%d,%d) key=%d\n", row, col, k);
      }
    }
  }
  
  // Update LEDs based on current mode
  if (current_mode == 0) { // ANIMATED
    unsigned long t = millis() - animation_start;
    for (int row = 0; row < LED_ROWS; row++) {
      for (int col = 0; col < LED_COLS; col++) {
        float phase = (t + (row * LED_COLS + col) * 200) % 3000 / 3000.0;
        int brightness = (sin(phase * 2 * PI) + 1) * 127;
        led_driver.drawPixel(col, row, brightness);
      }
    }
    led_driver.show();
  }
  else if (current_mode == 1) { // INTERACTIVE
    for (int row = 0; row < LED_ROWS; row++) {
      for (int col = 0; col < LED_COLS; col++) {
        led_driver.drawPixel(col, row, dots[row][col] ? 255 : 0);
      }
    }
    led_driver.show();
  }
  else if (current_mode == 2) { // CYCLE
    static unsigned long last_change = 0;
    unsigned long current_time = millis();
    
    // Calculate variable delay using sine wave (50-500ms range)
    float time_factor = (current_time % 10000) / 10000.0f; // 10 second cycle
    float sine_value = (sin(time_factor * 2 * PI) + 1) / 2; // 0 to 1
    unsigned long variable_delay = 50 + (unsigned long)(sine_value * 450); // 50-500ms
    
    if (current_time - last_change > variable_delay) {
      clear_leds();
      int row = debug_led / LED_COLS;
      int col = debug_led % LED_COLS;
      led_driver.drawPixel(col, row, 255);
      led_driver.show();
      Serial.printf("LED %d (%d,%d) - delay: %lums\n", debug_led, row, col, variable_delay);
      debug_led = (debug_led + 1) % (LED_ROWS * LED_COLS);
      last_change = current_time;
    }
  }
}