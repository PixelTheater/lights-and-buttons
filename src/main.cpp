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

// Animation mode abstraction
class AnimationMode {
public:
  virtual void begin() {}
  virtual void update() = 0;
  virtual const char* name() const = 0;
  virtual ~AnimationMode() {}
};

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

// Per-key timing / brightness seed used when a key is pressed.
// 0 means "inactive".
unsigned long key_activation_time[KEYPAD_ROWS][KEYPAD_COLS];

// Interactive mode state
uint8_t dots[KEYPAD_ROWS][KEYPAD_COLS] = {0}; // Track which keys are pressed

// Animation timing constants
unsigned long animation_start_time = 0;
const unsigned long ANIMATION_CYCLE_TIME = 3000; // 3 seconds per cycle

// FPS logging variables
unsigned long last_fps_log = 0;
unsigned long frames = 0;

// Simple helper resets all key state
void clear_key_state(){
  for(int r=0;r<KEYPAD_ROWS;r++){
    for(int c=0;c<KEYPAD_COLS;c++){
      key_activation_time[r][c] = 0;
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

// --- Animation Mode Implementations ---
class AnimatedMode : public AnimationMode {
  unsigned long animation_start_time = 0;
public:
  void begin() override {
    animation_start_time = millis();
  }
  void update() override {
    unsigned long current_time = millis();
    for (int row=0; row<LED_MATRIX_ROWS; row++){
      for (int col=0; col<LED_MATRIX_COLS; col++){
        float total_leds = LED_MATRIX_ROWS * LED_MATRIX_COLS;
        float position_factor = (row * LED_MATRIX_COLS + col) / total_leds;
        float time_offset = position_factor * ANIMATION_CYCLE_TIME;
        float phase = ((current_time + (unsigned long)time_offset) % ANIMATION_CYCLE_TIME) / (float)ANIMATION_CYCLE_TIME;
        float brightness_factor = (sin(phase * 2 * PI) + 1) / 2;
        int led_brightness = (int)(brightness_factor * 255);
        led_driver.drawPixel(col, row, led_brightness);
      }
    }
    led_driver.show();
  }
  const char* name() const override { return "ANIMATED"; }
};

class InteractiveMode : public AnimationMode {
public:
  void begin() override {
    clear_dots();
  }
  void update() override {
    int max_rows = min(LED_MATRIX_ROWS, KEYPAD_ROWS);
    int max_cols = min(LED_MATRIX_COLS, KEYPAD_COLS);
    for (int col=0; col<max_cols; col++){
      for (int row=0; row<max_rows; row++){
        if (dots[row][col] == 0){
          led_driver.drawPixel(col, row, 0);
        } else {
          float offset = sin((10000+millis()) / (dots[row][col]*1.0));
          int brightness = (130+125*offset);
          led_driver.drawPixel(col, row, brightness);
        }
      }
    }
    led_driver.show();
  }
  const char* name() const override { return "INTERACTIVE"; }
};
class DebugMode : public AnimationMode {
  unsigned long last_change = 0;
  int current_led = 0;
  const unsigned long LED_CHANGE_INTERVAL = 1000;
  const int max_to_test = 10;
public:
  void begin() override {
    current_led = 0;
    last_change = 0;
  }
  void update() override {
    unsigned long current_time = millis();
    if (current_time - last_change > LED_CHANGE_INTERVAL) {
      clear_all_leds();
      int row = current_led / LED_MATRIX_COLS;
      int col = current_led % LED_MATRIX_COLS;
      if (row < LED_MATRIX_ROWS && col < LED_MATRIX_COLS) {
        led_driver.drawPixel(col, row, 255);
        led_driver.show();
        Serial.printf("🔵 DEBUG LED ON: LED#%d, row=%d, col=%d\n", current_led, row, col);
      }
      current_led++;
      if (current_led >= max_to_test) {
        current_led = 0;
        Serial.println("🔄 DEBUG: Completed full LED matrix cycle");
      }
      last_change = current_time;
    }
  }
  const char* name() const override { return "DEBUG"; }
};

// --- Animation Mode Management ---
AnimatedMode animatedMode;
InteractiveMode interactiveMode;
DebugMode debugMode;
AnimationMode* animation_modes[] = { &animatedMode, &interactiveMode, &debugMode };
const int MODE_COUNT = sizeof(animation_modes)/sizeof(animation_modes[0]);
int current_mode = 0;
AnimationMode* current_animation = animation_modes[0];

void timerStatusMessage() {
  Serial.printf("Mode: %s | FPS: %.1f\n", current_animation->name(), frames / 5.0);
}

void switch_mode() {
  current_mode = (current_mode + 1) % MODE_COUNT;
  current_animation = animation_modes[current_mode];
  clear_all_leds();
  current_animation->begin();
  timer.detach();
  Serial.print("Switched to mode: ");
  Serial.println(current_animation->name());
  if (strcmp(current_animation->name(), "DEBUG") == 0) {
    Serial.println("DEBUG - LED Matrix Test (FPS logging disabled)");
    Serial.printf("Will cycle through all %d LEDs (%dx%d matrix)\n", LED_MATRIX_ROWS * LED_MATRIX_COLS, LED_MATRIX_ROWS, LED_MATRIX_COLS);
  } else {
    timer.attach(5, timerStatusMessage);
  }
}

void check_mode_button(){
  bool current_state = digitalRead(MODE_BUTTON_PIN);
  if (mode_button_last_state == HIGH && current_state == LOW) {
    mode_button_pressed = true;
    Serial.println("Mode button pressed - waiting for release");
  }
  if (mode_button_last_state == LOW && current_state == HIGH && mode_button_pressed) {
    unsigned long now = millis();
    if (now - mode_switch_debounce > DEBOUNCE_TIME) {
      Serial.println("Mode button released - switching mode");
      switch_mode();
      mode_switch_debounce = now;
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

// Optional: very lightweight FPS counter for insight (no Ticker needed)
void maybeLogFPS(){
  frames++;
  unsigned long now = millis();
  if(now - last_fps_log > 5000){
    float fps = frames / ((now - last_fps_log)/1000.0f + 0.001f);
    Serial.printf("[INFO] approx fps=%.1f (frames=%lu)\n", fps, frames);
    last_fps_log = now;
    frames = 0;
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
  clear_dots();
  
  // Initialize the first mode
  current_animation->begin();
  timer.attach(5, timerStatusMessage);
  
  Serial.println("=== MATRIX CONFIGURATION ===");
  Serial.printf("LED Matrix: %dx%d (%d total LEDs)\n", LED_MATRIX_ROWS, LED_MATRIX_COLS, LED_MATRIX_ROWS * LED_MATRIX_COLS);
  Serial.printf("Keypad Matrix: %dx%d (%d total buttons)\n", KEYPAD_ROWS, KEYPAD_COLS, KEYPAD_ROWS * KEYPAD_COLS);
  Serial.println("============================");
  Serial.printf("Starting in mode: %s\n", current_animation->name());
}


void loop()
{
  // If system error occurred, just flash LED and don't run normal operations
  if (system_error) {
    flash_error_led();
    return;
  }
  
  // Check mode button
  check_mode_button();
  
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
        // Update dots array for InteractiveMode
        if (strcmp(current_animation->name(), "INTERACTIVE") == 0) {
          dots[row][col] = 1;
        }
        Serial.printf("KEY PRESS r=%d c=%d (key=%d)\n", row, col, k);
      } else {
        digitalWrite(ONBOARD_LED_PIN, LOW);
        // Clear dots array for InteractiveMode
        if (strcmp(current_animation->name(), "INTERACTIVE") == 0) {
          dots[row][col] = 0;
        }
        Serial.printf("KEY RELEASE r=%d c=%d (key=%d)\n", row, col, k);
      }
    } else {
      Serial.printf("❌ KEYPAD EVENT OUT OF BOUNDS: key %d -> row: %d, col: %d (max %dx%d)\n", k, row, col, KEYPAD_ROWS, KEYPAD_COLS);
    }
  }
  
  // Run the current animation mode
  current_animation->update();
  if (strcmp(current_animation->name(), "DEBUG") == 0) return;
  maybeLogFPS();
}