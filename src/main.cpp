#include <Arduino.h>
#include <Adafruit_TCA8418.h>
#include <Wire.h>
#include <math.h>
#include "is31fl3733.hpp"
#include "Ticker.h"

// This example demonstrates how to use the IS31FL3733 LED driver with the Adafruit TCA8418 keypad driver.
// The IS31FL3733 is a 12x16 LED matrix driver with PWM control for each LED. The Adafruit TCA8418 is a
// keypad matrix driver with buffering and debouncing. This example uses the IS31FL3733 to drive a 
// 4x4 LED matrix and the TCA8418 to read a 4x4 keypad. 

// The firmware supports multiple modes:
// - ANIMATED: Default mode with position-dependent fade patterns
// - INTERACTIVE: User presses keypad buttons to control LED animations
// 
// Hardware setup:
// - Mode button: Connect a momentary button between GPIO 0 and GND
// - Keypad: 4x4 matrix connected via TCA8418 I2C controller
// - LEDs: 4x4 matrix connected via IS31FL3733 I2C driver
//
// Usage:
// - Press and release the mode button (GPIO 0) to switch between modes
// - In INTERACTIVE mode, press any keypad button to control LEDs

// for more info see https://github.com/somebox/lights-and-buttons

using namespace IS31FL3733;

// Mode system
enum class Mode {
  ANIMATED = 0,
  INTERACTIVE = 1,
  MODE_COUNT = 2
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
// Function prototypes for the read and write functions defined later in the file.
uint8_t i2c_read_reg(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t length);
uint8_t i2c_write_reg(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count);

Adafruit_TCA8418 keypad;   // I2C default address is 0x34
Ticker timer;   // used for periodic status messages

// ---------------------------------------------------------------------------------------------

uint8_t i2c_read_reg(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t length)
/**
 * @brief Read a buffer of data from the specified register.
 * 
 * @param i2c_addr I2C address of the device to read the data from.
 * @param reg_addr Address of the register to read from.
 * @param buffer Buffer to read the data into.
 * @param length Length of the buffer.
 * @return uint8_t 
 */
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  byte bytesRead = Wire.requestFrom(i2c_addr, length);
  for (int i = 0; i < bytesRead && i < length; i++)
  {
    buffer[i] = Wire.read();
  }
  return bytesRead;
}
uint8_t i2c_write_reg(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count)
/**
 * @brief Writes a buffer to the specified register. It is up to the caller to ensure the count of
 * bytes to write doesn't exceed 31, which is the Arduino's write buffer size (32) minus one byte for
 * the register address.
 * 
 * @param i2c_addr I2C address of the device to write the data to.
 * @param reg_addr Address of the register to write to.
 * @param buffer Pointer to an array of bytes to write.
 * @param count Number of bytes in the buffer.
 * @return uint8_t 0 if success, non-zero on error.
 */
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(buffer, count);
  return Wire.endTransmission();
}

IS31FL3733Driver led_driver = IS31FL3733Driver(ADDR::GND, ADDR::GND, &i2c_read_reg, &i2c_write_reg);

int dots[4][4];
#define RANDOM_RANGE 2000

// Animation timing for the animated mode
unsigned long animation_start_time = 0;
const unsigned long ANIMATION_CYCLE_TIME = 2000; // 2 seconds per cycle

void randomize_dots(){
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      dots[i][j] = random(RANDOM_RANGE)+1;
    }
  }
}

void clear_dots(){
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      dots[i][j] = 0;
    }
  }
}

void clear_all_leds(){
  for (int row=0; row<4; row++){
    for (int col=0; col<4; col++){
      led_driver.SetLEDSinglePWM(col, row, 0);
    }
  }
}

void animate_mode_default(){
  // Position-dependent fade patterns where timing depends on row/col
  unsigned long current_time = millis();
  
  for (int row=0; row<4; row++){
    for (int col=0; col<4; col++){
      // Create a unique phase offset based on position
      // Each position gets a different timing pattern
      float position_factor = (row * 4 + col) / 16.0; // 0 to 1 based on position
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
      int led_brightness = (int)(brightness_factor * pattern_value * 200 + 20); // 20-220 range
      
      led_driver.SetLEDSinglePWM(col, row, led_brightness);
    }
  }
}

void animate_mode_interactive(){
  // Interactive mode - existing functionality where user presses buttons to control LEDs
  static int n = 0;
  for (int b=0; b<4; b++){
    int d = 2;
    for (int pos=0; pos<4; pos++){
      if (dots[pos][b] == 0){
        led_driver.SetLEDSinglePWM(b, pos, 0); // turn off segment
      } else {
        float offset = sin((10000+millis()) / (dots[pos][b]*1.0));
        led_driver.SetLEDSinglePWM(b, pos, (130+125*offset)); // turn on segment
        delay(3); // reduced delay for smoother animation
      }
    }
  }
}

void switch_mode(){
  // Switch to next mode
  current_mode = (Mode)(((int)current_mode + 1) % (int)Mode::MODE_COUNT);
  
  // Clear LEDs when switching modes
  clear_all_leds();
  clear_dots();
  
  // Print mode change
  Serial.print("Switched to mode: ");
  switch(current_mode){
    case Mode::ANIMATED:
      Serial.println("ANIMATED");
      animation_start_time = millis();
      break;
    case Mode::INTERACTIVE:
      Serial.println("INTERACTIVE");
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

  // Initialize onboard LED for error indication
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW); // Start with LED off
  Serial.printf("Onboard LED initialized on GPIO %d for error indication\n", ONBOARD_LED_PIN);

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
    keypad.matrix(4, 4);
  }

  Serial.println("Initializing IS31FL3733 LED driver");
  led_driver.Init();
  Serial.print("\nIS31FL3733B driver init at address 0x");
  Serial.println(led_driver.GetI2CAddress(), HEX);
  
  // Test LED driver communication by trying to read a register
  uint8_t test_buffer;
  uint8_t read_result = i2c_read_reg(led_driver.GetI2CAddress(), 0x00, &test_buffer, 1);
  if (read_result == 0) {
    handle_system_error("IS31FL3733 LED driver initialization failed! Check I2C wiring and address jumpers.");
  }
  
  Serial.println(" -> Setting global current control");
  led_driver.SetGCC(235);
  Serial.println(" -> Setting PWM state for all LEDs to half power");
  led_driver.SetLEDMatrixPWM(200);
  Serial.println(" -> Setting state of all LEDs to OFF");
  led_driver.SetLEDMatrixState(LED_STATE::ON);
  led_driver.SetLEDMatrixPWM(50); // set brightness
  
  timer.attach(TIMER_PERIOD, timerStatusMessage);
  
  clear_dots();
  animation_start_time = millis();
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
    int col = 3-((k / 10)%4); // limit to 4 columns
    int row = (k % 10)%4; // limit to 4 rows
    
    if (pressed) {
      Serial.printf("KEYPAD PRESS\t row: %d, col: %d\n", row, col);
      
      if (current_mode == Mode::INTERACTIVE) {
        // Handle button press in interactive mode
        dots[row][col] = millis();
      }
    } else {
      Serial.printf("KEYPAD RELEASE\t row: %d, col: %d\n", row, col);
      
      if (current_mode == Mode::INTERACTIVE) {
        // Calculate how long the button was held
        dots[row][col] = millis() - dots[row][col];
      }
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
  }
  
  frame++;
}