#include <Arduino.h>
#include <Adafruit_TCA8418.h>
#include <Wire.h>
#include <math.h>
#include "IS31FL373x.h"
#include "Ticker.h"

// This example demonstrates how to use the IS31FL3737 LED driver with the Adafruit TCA8418 keypad driver.
// The IS31FL3737 is a 12x12 LED matrix driver with PWM control for each LED. The Adafruit TCA8418 is a
// keypad matrix driver with buffering and debouncing. This example supports configurable matrix sizes:
// - LEDs: 12x12 matrix (144 LEDs) via IS31FL3737 I2C driver
// - Keypad: Up to 8x10 matrix (80 buttons) via TCA8418 I2C controller

// Configuration: Modify the constants below to change matrix sizes for your PCB
// - LED_MATRIX_ROWS/COLS: Set LED matrix dimensions (IS31FL3737 hardware: 12x12 matrix, 144 LEDs)
// - KEYPAD_ROWS/COLS: Set keypad matrix dimensions (max 8x10)

// The firmware supports multiple modes:
// - ANIMATED: Default mode with position-dependent fade patterns across all LEDs
// - INTERACTIVE: User presses keypad buttons to control corresponding LED animations
// - DEBUG: Cycles through each LED position in sequence to test wiring and identify issues
// 
// Hardware setup:
// - Mode button: Connect a momentary button between GPIO 0 and GND
// - Keypad: Matrix connected via TCA8418 I2C controller (configurable size)
// - LEDs: Matrix connected via IS31FL3737 I2C driver (12x12 matrix)
// - Built-in LED (GPIO 2): Used for error indication and keypad press diagnostics
//
// Usage:
// - Press and release the mode button (GPIO 0) to switch between modes
// - In INTERACTIVE mode, press any keypad button to control LEDs
// - Built-in LED lights up when any keypad button is pressed (diagnostic feature)

// for more info see https://github.com/somebox/lights-and-buttons

// Matrix configuration constants - modify these to match your hardware
#define LED_MATRIX_ROWS 12    // IS31FL3737 hardware: 12 rows
#define LED_MATRIX_COLS 12    // IS31FL3737 hardware: 12 columns (144 LEDs total)
#define KEYPAD_ROWS 8         // TCA8418 supports up to 8 rows
#define KEYPAD_COLS 10        // TCA8418 supports up to 10 columns

// === Simple demo state ===
// We intentionally keep just one optional animation toggle to show how users can add their own modes
bool animation_enabled = true;   // Press the MODE_BUTTON to toggle animation vs static-interactive display
unsigned long last_button_change = 0; // debounce helper
const unsigned long BUTTON_DEBOUNCE_MS = 250;

// Mode button configuration (can be repurposed by users)
#define MODE_BUTTON_PIN 0
bool last_button_state = HIGH; // pulled-up

// Error handling configuration
#define ONBOARD_LED_PIN 2  // GPIO pin for onboard LED (ESP32 DevKit usually uses GPIO 2)
bool system_error = false;
// I2C communication is handled internally by the IS31FL3737B driver

Adafruit_TCA8418 keypad;   // I2C default address is 0x34
Ticker timer;   // used for periodic status messages

// ---------------------------------------------------------------------------------------------

// I2C communication handled by IS31FL3737 driver
// CRITICAL: Use the correct driver class that matches your physical hardware chip
IS31FL3737 led_driver(ADDR::GND);  // IS31FL3737 chip with ADDR pin connected to GND

// Per-key timing / brightness seed used when a key is pressed.
// 0 means "inactive".
unsigned long key_activation_time[KEYPAD_ROWS][KEYPAD_COLS];

// Simple helper resets all key state
void clear_key_state(){
  for(int r=0;r<KEYPAD_ROWS;r++){
    for(int c=0;c<KEYPAD_COLS;c++){
      key_activation_time[r][c] = 0;
    }
  }
}

void clear_all_leds(){
  led_driver.clear();
  led_driver.show(); // Push changes to hardware
}

// === Demo animation ===
// A very small, easy to read animation: a soft global pulsing brightness factor.
// Users can replace this with any pattern they like.
uint8_t global_brightness = 180; // base brightness ceiling
uint8_t min_brightness = 20;
unsigned long animation_start_time = 0;
const unsigned long ANIMATION_CYCLE_TIME = 3000; // ms per full pulse

uint8_t pulse_brightness(){
  float phase = (millis() - animation_start_time) % ANIMATION_CYCLE_TIME / (float)ANIMATION_CYCLE_TIME; // 0..1
  float s = (sin(phase * TWO_PI) + 1.0f) * 0.5f; // 0..1
  return (uint8_t)(min_brightness + s * (global_brightness - min_brightness));
}

// === Key driven LED update ===
// For each pressed key we compute a decay brightness based on how long ago it was pressed.
// This demonstrates mapping key events to LED pixels and simple timing logic.
void refresh_leds(){
  // If animation is enabled we compute a global pulse factor.
  float pulse = animation_enabled ? (pulse_brightness() / 255.0f) : 1.0f;

  int rows = min(LED_MATRIX_ROWS, KEYPAD_ROWS);
  int cols = min(LED_MATRIX_COLS, KEYPAD_COLS);

  unsigned long now = millis();
  const unsigned long HOLD_PEAK_MS = 1200; // how long to decay
  for(int r=0;r<rows;r++){
    for(int c=0;c<cols;c++){
      unsigned long t = key_activation_time[r][c];
      if(t == 0){
        led_driver.drawPixel(c, r, 0);
        continue;
      }
      unsigned long age = now - t;
      if(age > HOLD_PEAK_MS){
        key_activation_time[r][c] = 0; // auto clear
        led_driver.drawPixel(c, r, 0);
        continue;
      }
      float fade = 1.0f - (age / (float)HOLD_PEAK_MS); // 1 -> 0
      int base = (int)(fade * 255);
      int value = (int)(base * pulse);
      led_driver.drawPixel(c, r, value);
    }
  }
  led_driver.show();
}

// A tiny diagnostic helper: briefly turn on a single LED (r,c) at full brightness
void flash_pixel(int r,int c){
  if(r<0||c<0||r>=LED_MATRIX_ROWS||c>=LED_MATRIX_COLS) return;
  led_driver.drawPixel(c,r,255);
  led_driver.show();
  delay(30);
  led_driver.drawPixel(c,r,0);
}

void timerStatusMessage(); // forward declaration

void handle_button_toggle(){
  bool reading = digitalRead(MODE_BUTTON_PIN);
  if(reading != last_button_state){
    unsigned long now = millis();
    if(now - last_button_change > BUTTON_DEBOUNCE_MS && reading == LOW){ // falling edge press
      animation_enabled = !animation_enabled;
      Serial.printf("Mode button: animation_enabled=%s\n", animation_enabled?"true":"false");
      if(animation_enabled){
        animation_start_time = millis();
      }
    }
    last_button_change = now;
    last_button_state = reading;
  }
}

// (Old multi-mode button logic removed in favor of simple toggle)

void flash_error_led() {
  // Flash the onboard LED to indicate system error
  static unsigned long last_flash = 0;
  static bool led_state = false;
  
  unsigned long current_time = millis();
  if (current_time - last_flash > 250) { // Flash every 250ms (4Hz)
    led_state = !led_state;
    digitalWrite(ONBOARD_LED_PIN, led_state);
    last_flash = current_time;
  }
}

void handle_system_error(const char* error_message) {
  Serial.println("=== SYSTEM ERROR ===");
  Serial.println(error_message);
  Serial.println("Flashing onboard LED. Check I2C connections and restart.");
  Serial.println("====================");
  
  system_error = true;
  
  // Flash LED indefinitely
  while (true) {
    flash_error_led();
    delay(10); // Small delay to prevent watchdog issues
  }
}

// Optional: very lightweight FPS counter for insight (no Ticker needed)
unsigned long last_fps_log = 0;
unsigned long frames = 0;
void maybeLogFPS(){
  frames++;
  unsigned long now = millis();
  if(now - last_fps_log > 5000){
    float fps = frames / ((now - animation_start_time)/1000.0f + 0.001f);
    Serial.printf("[INFO] approx fps=%.1f (frames=%lu)\n", fps, frames);
    last_fps_log = now;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(__FILE__);

  // Initialize mode / toggle button with internal pullup
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("Button initialized on GPIO %d (toggle animation)\n", MODE_BUTTON_PIN);

  // Initialize onboard LED for error indication and keypad diagnostics
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW); // Start with LED off
  Serial.printf("Onboard LED initialized on GPIO %d for error indication and keypad diagnostics\n", ONBOARD_LED_PIN);

  // Wire.begin(21,22);
  Wire.begin();
  Wire.setClock(800000); // use 800 kHz I2C
  Serial.print("I2C speed is ");
  Serial.println(Wire.getClock());

  Serial.println("Initializing TCA8418 Keypad driver");
  if (! keypad.begin(TCA8418_DEFAULT_ADDR, &Wire)) {
    handle_system_error("TCA8418 Keypad driver initialization failed! Check I2C wiring and pullups.");
  } else {
    Serial.println("keypad driver init at address 0x");
    Serial.println(TCA8418_DEFAULT_ADDR, HEX);
    keypad.matrix(KEYPAD_ROWS, KEYPAD_COLS);
    Serial.printf("Keypad matrix configured for %dx%d (%d total buttons)\n", KEYPAD_ROWS, KEYPAD_COLS, KEYPAD_ROWS * KEYPAD_COLS);
  }

  Serial.println("About to initialize IS31FL3737 LED driver...");
  Serial.flush(); // Ensure message is printed before potential crash
  
  bool init_success = false;
  try {
    Serial.println("Calling led_driver.begin()...");
    Serial.flush();
    init_success = led_driver.begin();
    Serial.printf("led_driver.begin() returned: %s\n", init_success ? "true" : "false");
  } catch (...) {
    Serial.println("Exception caught during led_driver.begin()!");
    handle_system_error("IS31FL3737 LED driver crashed during initialization!");
  }
  
  if (!init_success) {
    handle_system_error("IS31FL3737 LED driver initialization failed! Check I2C wiring and ADDR pin configuration.");
  }
  Serial.println("IS31FL3737 driver initialized successfully");
  
  // Debug: Check driver properties
  Serial.printf(" -> Driver matrix size: %dx%d\n", led_driver.getWidth(), led_driver.getHeight());
  Serial.printf(" -> PWM buffer size: %d bytes\n", led_driver.getPWMBufferSize());
  
  // Configure driver for optimal performance
  Serial.printf(" -> I2C Address: 0x%02X\n", led_driver.getI2CAddress());
  
  Serial.println(" -> Setting global current control");
  led_driver.setGlobalCurrent(128); // Set to ~50% brightness
  Serial.printf(" -> Configuring %dx%d LED matrix\n", LED_MATRIX_ROWS, LED_MATRIX_COLS);
  
  // Test basic LED functionality
  Serial.println(" -> Testing basic LED functionality...");
  led_driver.clear();
  Serial.println("   Cleared LEDs");
  
  // Try to light a few test LEDs
  Serial.println("   Setting test pixels...");
  led_driver.drawPixel(0, 0, 255);  // Top-left corner
  led_driver.drawPixel(1, 1, 255);  // Second position
  led_driver.drawPixel(5, 5, 255);  // Middle area
  led_driver.show();
  Serial.println("   Called show() - LEDs should be lit now");
  
  delay(2000); // Give time to see the test LEDs
  
  Serial.println(" -> Clearing test LEDs");
  led_driver.clear();
  led_driver.show();
  
  clear_key_state();
  animation_start_time = millis();
  
  Serial.println("=== MATRIX CONFIGURATION ===");
  Serial.printf("LED Matrix: %dx%d (%d total LEDs)\n", LED_MATRIX_ROWS, LED_MATRIX_COLS, LED_MATRIX_ROWS * LED_MATRIX_COLS);
  Serial.printf("Keypad Matrix: %dx%d (%d total buttons)\n", KEYPAD_ROWS, KEYPAD_COLS, KEYPAD_ROWS * KEYPAD_COLS);
  Serial.println("============================");
  Serial.println("Starting simplified demo (animation enabled)");
}


void loop()
{
  // If system error occurred, just flash LED and don't run normal operations
  if (system_error) {
    flash_error_led();
    return;
  }
  
  // Check button toggle
  handle_button_toggle();
  
  // Handle keypad input
  if (keypad.available() > 0)
  {
    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();
    bool pressed = k & 0x80;
    k &= 0x7F;
    k--;
    
    // Calculate row and column based on the configured keypad matrix size
    // TCA8418 uses row-major ordering: key_number = row * KEYPAD_COLS + col
    // The postion calculation will depend on how the buttons and leds are wired to the TCA8418.
    int col = 3 - (k / KEYPAD_COLS);
    int row = k % KEYPAD_COLS;
    
    // Bounds check to ensure we don't exceed our matrix dimensions
    if (row >= 0 && row < KEYPAD_ROWS && col >= 0 && col < KEYPAD_COLS) {
      if (pressed) {
        digitalWrite(ONBOARD_LED_PIN, HIGH); // any keypress indicator
        key_activation_time[row][col] = millis();
        Serial.printf("KEY PRESS r=%d c=%d (key=%d)\n", row, col, k);
      } else {
        digitalWrite(ONBOARD_LED_PIN, LOW);
        Serial.printf("KEY RELEASE r=%d c=%d (key=%d)\n", row, col, k);
      }
    } else {
      Serial.printf("❌ KEYPAD EVENT OUT OF BOUNDS: key %d -> row: %d, col: %d (max %dx%d)\n", k, row, col, KEYPAD_ROWS, KEYPAD_COLS);
    }
  }
  // Refresh LEDs based on key state + optional global animation
  refresh_leds();
  maybeLogFPS();
}