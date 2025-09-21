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

// Mode system
enum class Mode {
  ANIMATED = 0,
  INTERACTIVE = 1,
  DEBUG = 2,
  MODE_COUNT = 3
};

Mode current_mode = Mode::ANIMATED;
unsigned long mode_switch_debounce = 0;
const unsigned long DEBOUNCE_TIME = 500; // 500ms debounce for mode switching

// Mode button configuration
#define MODE_BUTTON_PIN 0  // GPIO pin for mode button (change this to your actual pin)
bool mode_button_pressed = false;
bool mode_button_last_state = HIGH;

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

int dots[KEYPAD_ROWS][KEYPAD_COLS];
#define RANDOM_RANGE 2000

// Animation timing for the animated mode
unsigned long animation_start_time = 0;
const unsigned long ANIMATION_CYCLE_TIME = 2000; // 2 seconds per cycle

void randomize_dots(){
  for (int i=0; i<KEYPAD_ROWS; i++){
    for (int j=0; j<KEYPAD_COLS; j++){
      dots[i][j] = random(RANDOM_RANGE)+1;
    }
  }
}

void clear_dots(){
  for (int i=0; i<KEYPAD_ROWS; i++){
    for (int j=0; j<KEYPAD_COLS; j++){
      dots[i][j] = 0;
    }
  }
}

void clear_all_leds(){
  led_driver.clear();
  led_driver.show(); // Push changes to hardware
}

void animate_mode_default(){
  // Position-dependent fade patterns where timing depends on row/col
  unsigned long current_time = millis();
  static unsigned long last_debug = 0;
  static bool debug_printed = false;
  
  for (int row=0; row<LED_MATRIX_ROWS; row++){
    for (int col=0; col<LED_MATRIX_COLS; col++){
      // Create a unique phase offset based on position
      // Each position gets a different timing pattern
      float total_leds = LED_MATRIX_ROWS * LED_MATRIX_COLS;
      float position_factor = (row * LED_MATRIX_COLS + col) / total_leds; // 0 to 1 based on position
      float time_offset = position_factor * ANIMATION_CYCLE_TIME;
      
      // Calculate the animation phase for this LED
      float phase = ((current_time + (unsigned long)time_offset) % ANIMATION_CYCLE_TIME) / (float)ANIMATION_CYCLE_TIME;
      
      // Create a fade in/out pattern using sine wave
      float brightness_factor = (sin(phase * 2 * PI) + 1) / 2; // 0 to 1
      
      // Create a pattern like "00100011" by using position-dependent frequency
      int pattern_frequency = 1 + (row + col) % 4; // 1-4 cycles per animation
      float pattern_phase = fmod(phase * pattern_frequency, 1.0);
      
      // Create digital pattern (on/off) with smooth transitions
      float pattern_value = (sin(pattern_phase * 2 * PI) > 0) ? 1.0 : 0.1;
      
      // Combine brightness and pattern
      int led_brightness = (int)(brightness_factor * pattern_value * 255); // 0-255 range
      
      led_driver.drawPixel(col, row, led_brightness);
      
      // Debug: Print first few LED values
      if (!debug_printed && row < 2 && col < 2) {
        Serial.printf("   LED[%d][%d] = %d\n", row, col, led_brightness);
      }
    }
  }
  
  // Debug info once per second
  if (current_time - last_debug > 1000) {
    if (!debug_printed) {
      Serial.println("animate_mode_default: Setting LEDs and calling show()");
      debug_printed = true;
    }
    last_debug = current_time;
  }
  
  led_driver.show(); // Push frame buffer to hardware
}

void animate_mode_interactive(){
  // Interactive mode - existing functionality where user presses buttons to control LEDs
  // Note: Only animate LEDs that correspond to actual keypad positions
  int max_rows = min(LED_MATRIX_ROWS, KEYPAD_ROWS);
  int max_cols = min(LED_MATRIX_COLS, KEYPAD_COLS);
  
  static unsigned long last_debug_print = 0;
  static bool debug_printed = false;
  
  // Print debug info once when entering interactive mode
  if (!debug_printed && millis() - last_debug_print > 1000) {
    Serial.printf("Interactive Animation Debug: max_rows=%d, max_cols=%d\n", max_rows, max_cols);
    debug_printed = true;
    last_debug_print = millis();
  }
  
  for (int col=0; col<max_cols; col++){
    for (int row=0; row<max_rows; row++){
      if (dots[row][col] == 0){
        led_driver.drawPixel(col, row, 0); // turn off segment
      } else {
        float offset = sin((10000+millis()) / (dots[row][col]*1.0));
        int brightness = (130+125*offset);
        led_driver.drawPixel(col, row, brightness); // turn on segment
        
        // Debug logging for specific positions (like key 6 which maps to row=0, col=6)
        if ((row == 0 && col == 6) && millis() - last_debug_print > 2000) {
          Serial.printf("🔍 LED Debug row=0,col=6: dots[0][6]=%d, brightness=%d\n", dots[0][6], brightness);
          last_debug_print = millis();
        }
        
        delay(3); // reduced delay for smoother animation
      }
    }
  }
  led_driver.show(); // Push frame buffer to hardware
  
  // Reset debug flag when switching away from interactive mode
  if (current_mode != Mode::INTERACTIVE) {
    debug_printed = false;
  }
}

void animate_mode_debug(){
  // Debug mode - cycles through each LED position in sequence to test wiring
  static unsigned long last_change = 0;
  static int current_led = 0;
  const unsigned long LED_CHANGE_INTERVAL = 1000; // 1000ms per LED
  // int max_to_test = LED_MATRIX_ROWS * LED_MATRIX_COLS;
  int max_to_test = 10;
  
  unsigned long current_time = millis();
  
  if (current_time - last_change > LED_CHANGE_INTERVAL) {
    // Clear all LEDs first
    clear_all_leds();
    
    // Calculate row and col from current_led index
    int row = current_led / LED_MATRIX_COLS;
    int col = current_led % LED_MATRIX_COLS;
    
    if (row < LED_MATRIX_ROWS && col < LED_MATRIX_COLS) {
      // Calculate SW/CS pins for IS31FL3737
      // For IS31FL3737: SW pins are 1-12 (rows), CS pins are 1-12 (columns)
      int sw_pin = row + 1;  // SW1-SW12 (1-based)
      int cs_pin = col + 1;  // CS1-CS12 (1-based)
      
      // Get I2C address from the driver
      uint8_t i2c_address = led_driver.getI2CAddress();
      
      // Turn on current LED and log immediately
      led_driver.drawPixel(col, row, 255);
      led_driver.show(); // Push to hardware immediately after setting pixel
      Serial.printf("🔵 DEBUG LED ON: LED#%d, row=%d, col=%d, SW%d, CS%d, I2C=0x%02X\n", 
                   current_led, row, col, sw_pin, cs_pin, i2c_address);
    }
    
    // Move to next LED
    current_led++;
    if (current_led >= max_to_test) {
      current_led = 0; // Wrap around
      Serial.println("🔄 DEBUG: Completed full LED matrix cycle");
    }
    
    last_change = current_time;
  }
}

void timerStatusMessage(); // forward declaration

void switch_mode(){
  // Switch to next mode
  current_mode = (Mode)(((int)current_mode + 1) % (int)Mode::MODE_COUNT);
  
  // Clear LEDs when switching modes
  clear_all_leds();
  clear_dots();
  
  // Manage timer based on mode
  timer.detach(); // Stop existing timer
  
  // Print mode change
  Serial.print("Switched to mode: ");
  switch(current_mode){
    case Mode::ANIMATED:
      Serial.println("ANIMATED");
      animation_start_time = millis();
      timer.attach(5, timerStatusMessage); // Re-enable fps logging (5 second period)
      break;
    case Mode::INTERACTIVE:
      Serial.println("INTERACTIVE");
      timer.attach(5, timerStatusMessage); // Re-enable fps logging (5 second period)
      break;
    case Mode::DEBUG:
      Serial.println("DEBUG - LED Matrix Test (FPS logging disabled)");
      Serial.printf("Will cycle through all %d LEDs (%dx%d matrix)\n", LED_MATRIX_ROWS * LED_MATRIX_COLS, LED_MATRIX_ROWS, LED_MATRIX_COLS);
      // No timer attachment - fps logging disabled in debug mode
      break;
  }
}

void check_mode_button(){
  // Read the current state of the mode button
  bool current_state = digitalRead(MODE_BUTTON_PIN);
  
  // Check for button press (transition from HIGH to LOW)
  if (mode_button_last_state == HIGH && current_state == LOW) {
    // Button was just pressed
    mode_button_pressed = true;
    Serial.println("Mode button pressed - waiting for release");
  }
  
  // Check for button release (transition from LOW to HIGH)
  if (mode_button_last_state == LOW && current_state == HIGH && mode_button_pressed) {
    // Button was released after being pressed
    unsigned long current_time = millis();
    if (current_time - mode_switch_debounce > DEBOUNCE_TIME) {
      Serial.println("Mode button released - switching mode");
      switch_mode();
      mode_switch_debounce = current_time;
    }
    mode_button_pressed = false;
  }
  
  mode_button_last_state = current_state;
}

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

#define TIMER_PERIOD 5
static long frame = 1;
static long last_frames = 0;
void timerStatusMessage(){
  Serial.println("-----");
  Serial.printf("frame: %d\n", frame);
  float fps = (frame - last_frames) / TIMER_PERIOD;
  Serial.printf("fps: %f\n", fps);
  last_frames = frame;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(__FILE__);

  // Initialize mode button with internal pullup
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("Mode button initialized on GPIO %d with pullup\n", MODE_BUTTON_PIN);

  // Initialize onboard LED for error indication and keypad diagnostics
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW); // Start with LED off
  Serial.printf("Onboard LED initialized on GPIO %d for error indication and keypad diagnostics\n", ONBOARD_LED_PIN);

  Wire.begin(22,21);
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
  
  // Only attach timer for fps logging in non-debug modes
  if (current_mode != Mode::DEBUG) {
    timer.attach(TIMER_PERIOD, timerStatusMessage);
  }
  
  clear_dots();
  animation_start_time = millis();
  
  Serial.println("=== MATRIX CONFIGURATION ===");
  Serial.printf("LED Matrix: %dx%d (%d total LEDs)\n", LED_MATRIX_ROWS, LED_MATRIX_COLS, LED_MATRIX_ROWS * LED_MATRIX_COLS);
  Serial.printf("Keypad Matrix: %dx%d (%d total buttons)\n", KEYPAD_ROWS, KEYPAD_COLS, KEYPAD_ROWS * KEYPAD_COLS);
  Serial.println("============================");
  Serial.println("Starting in ANIMATED mode (default)");
}


void loop()
{
  // If system error occurred, just flash LED and don't run normal operations
  if (system_error) {
    flash_error_led();
    return;
  }
  
  // Check mode button (separate GPIO pin)
  check_mode_button();
  
  // Handle keypad input (only for interactive mode)
  if (keypad.available() > 0)
  {
    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();
    bool pressed = k & 0x80;
    k &= 0x7F;
    k--;
    
    // Calculate row and column based on the configured keypad matrix size
    // TCA8418 uses row-major ordering: key_number = row * KEYPAD_COLS + col
    int row = k / KEYPAD_COLS;
    int col = k % KEYPAD_COLS;
    
    // Bounds check to ensure we don't exceed our matrix dimensions
    if (row >= 0 && row < KEYPAD_ROWS && col >= 0 && col < KEYPAD_COLS) {
      if (pressed) {
        // Diagnostic: Light up built-in LED when any button is pressed
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        Serial.printf("🔴 KEYPAD PRESS\t row: %d, col: %d (key %d) - LED ON\n", row, col, k);
        
        if (current_mode == Mode::INTERACTIVE) {
          // Handle button press in interactive mode
          dots[row][col] = millis();
          Serial.printf("   Interactive mode: Setting dots[%d][%d] = %lu\n", row, col, dots[row][col]);
        }
      } else {
        // Diagnostic: Turn off built-in LED when button is released
        digitalWrite(ONBOARD_LED_PIN, LOW);
        Serial.printf("⚫ KEYPAD RELEASE\t row: %d, col: %d (key %d) - LED OFF\n", row, col, k);
        
        if (current_mode == Mode::INTERACTIVE) {
          // Calculate how long the button was held
          unsigned long hold_time = millis() - dots[row][col];
          dots[row][col] = hold_time;
          Serial.printf("   Button held for %lu ms\n", hold_time);
        }
      }
    } else {
      Serial.printf("❌ KEYPAD EVENT OUT OF BOUNDS: key %d -> row: %d, col: %d (max %dx%d)\n", k, row, col, KEYPAD_ROWS, KEYPAD_COLS);
    }
  }
  
  // Run the appropriate animation based on current mode
  switch(current_mode) {
    case Mode::ANIMATED:
      animate_mode_default();
      break;
    case Mode::INTERACTIVE:
      animate_mode_interactive();
      break;
    case Mode::DEBUG:
      animate_mode_debug();
      // Don't increment frame counter in debug mode to avoid fps logging
      return;
  }
  
  frame++;
}