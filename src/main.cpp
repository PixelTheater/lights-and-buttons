#include <Arduino.h>
#include <Adafruit_TCA8418.h>
#include <Wire.h>
#include "is31fl3733.hpp"
#include "Ticker.h"

// This example demonstrates how to use the IS31FL3733 LED driver with the Adafruit TCA8418 keypad driver.
// The IS31FL3733 is a 12x16 LED matrix driver with PWM control for each LED. The Adafruit TCA8418 is a
// keypad matrix driver with buffering and debouncing. This example uses the IS31FL3733 to drive a 
// 4x4 LED matrix and the TCA8418 to read a 4x4 keypad. 

// The keypad is used to set the brightness of the LEDs in the matrix.

// for more info see https://github.com/somebox/lights-and-buttons

using namespace IS31FL3733;
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

void animate_segments(int repeats=1, int speed=50){
  static int n = 0;
  for (int b=0; b<4; b++){
    int d = 2;
    for (int pos=0; pos<4; pos++){
      if (dots[pos][b] == 0){
        led_driver.SetLEDSinglePWM(b, pos, 0); // turn off segment
      } else {
        float offset = sin((10000+millis()) / (dots[pos][b]*1.0));
        led_driver.SetLEDSinglePWM(b, pos, (130+125*offset)); // turn on segment
        delay(speed);
      }
    }
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

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(__FILE__);

  Wire.begin();
  Wire.setClock(800000); // use 800 kHz I2C
  Serial.print("I2C speed is ");
  Serial.println(Wire.getClock());

  Serial.println("Initializing TCA8418 Keypad driver");
  if (! keypad.begin(TCA8418_DEFAULT_ADDR, &Wire)) {
    Serial.println("keypad not found, check wiring & pullups!");
    while (1);
  } else {
    Serial.println("keypad driver init at address 0x");
    Serial.println(TCA8418_DEFAULT_ADDR, HEX);
    keypad.matrix(4, 4);
  }

  Serial.println("Initializing IS31FL3733 LED driver");
  led_driver.Init();
  Serial.print("\nIS31FL3733B driver init at address 0x");
  Serial.println(led_driver.GetI2CAddress(), HEX);  
  Serial.println(" -> Setting global current control");
  led_driver.SetGCC(150);
  Serial.println(" -> Setting PWM state for all LEDs to half power");
  led_driver.SetLEDMatrixPWM(140);
  Serial.println(" -> Setting state of all LEDs to OFF");
  led_driver.SetLEDMatrixState(LED_STATE::ON);
  led_driver.SetLEDMatrixPWM(50); // set brightness
  
  timer.attach(TIMER_PERIOD, timerStatusMessage);
  
  clear_dots();
}


void loop()
{
  if (keypad.available() > 0)
  {
    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();
    bool pressed = k & 0x80;
    if (pressed) {
      Serial.print("PRESS\tR: ");
    } else {
      Serial.print("RELEASE\tR: ");
    }
    k &= 0x7F;
    k--;
    int col = 3-((k / 10)%4); // limit to 4 columns
    int row = (k % 10)%4; // limit to 4 rows
    Serial.printf(" row: %d, col: %d\n", row, col);
    if (pressed){
      dots[row][col] =millis();
    } else {
      dots[row][col] = millis() - dots[row][col];
    }
  }
  
  // fade LEDs
  animate_segments(1, 3);
  // if (random(100)==0){
  //   dots[random(4)][random(4)] = random(RANDOM_RANGE)+150;
  // }
  frame++;
}