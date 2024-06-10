# Lights-and-Buttons

A small I2C breakout board for controlling lots of LEDs and buttons/switches.
It features two fabulous ICs on the same PCB:

- the *TCA8418 keypad controller*: scans up to 80 separate contact points in an 8x10 matrix, and features queuing and debouncing. Well-supported driver from Adafruit.
- the *IS31FL3737 LED driver*: This versitile chip can control a matrix of 144 (12x12) LEDs with 256 levels of individual dimming. Here we use a compatible IS31FL3733 driver, but this should be adapted in the future.

Both I2C devices are present on the same board with the connections available. There's a jumper to set the I2C address of the LED driver. The keypad driver only supports one I2C address, so there's a jumper to disable it. Using these jumpers, multiple boards can be connected to support more LEDs if desired.

This code example shows the use of a 4x4 button panel (16 in total), where each button has an LED built-in, and an ESP32 driving the project. When the user presses a button and holds it down, the LED will begin to flash depending on how long the button was held.

![PCB Top](docs/pcb-top-3d.png)
![PCB Bottom](docs/pcb-bottom-3d.png)

I originally developed this project to create some art projects and one-off fidget toys, and it makes it easy to mock things up. Another nice use is for miniatures and models - you can connect a lot of LEDs and not need to worry about wiring in resistors or adjusting brightness, as the LED driver takes care of all of that. Plus there's a load of contacts that can be used for buttons. 

This solution was inspired by [Adafruit's keypad controller breakout](https://www.adafruit.com/product/4918#description), and their open source driver is used in this example. Please support them.

Here are some pictures of the test rig this code was used on. It's using standard 16mm LED buttons (search for "R16-503") which have four contacts each, I made a small PCB for them and soldered copper rods to the columns and rows of each, forming a 4x4 matrix. The edges of each side were then wired to JST cables for connection to the board. The ESP32 pictured is connected to a small breakout board to make the I2C cabling easier.

![example button board project](docs/board-example.jpeg)
![example button board project](docs/button-board.jpeg)
![hand-wiring of the button board](docs/button-board-back.jpeg)

