## Overview
This Arduino sketch interfaces with a keypad, an OLED display, and an LCD using I2C communication. It's designed to relay keypad inputs to a connected system and display messages from an external source. Due to RAM limitations on the Arduino, some logic processing is handled by a ROS (Robot Operating System) node.

## Setup

### Hardware

- **Keypad**: Connects to digital pins 2, 3, 4, 5 (rows) and 7, 6, 8, 9 (columns).
- **OLED Display**: Connected via I2C at address `0x3C`.
- **LCD Display**: Connected via I2C at address `0x27`.
- **I2C Address**: Arduino is configured to listen at address `0x08`.

### Libraries

- `Wire.h`: For I2C communication.
- `Keypad.h`: For keypad interfacing.
- `Adafruit_SSD1306` and `Adafruit_GFX`: For OLED display control.
- `LiquidCrystal_I2C`: For LCD display control.

## Logic Handling on ROS

Due to memory constraints, specific text processing and transport logic are handled by an external ROS node. This offloads complex operations from the Arduino, ensuring efficient performance.

## Key Functions

- `setup()`: Initializes I2C communication, displays, and keypad.
- `receiveEvent(int bytes)`: Receives and processes messages from the I2C bus.
- `sendKey()`: Sends the last pressed key over I2C.
- `displayText(const char* text, int textSize)`: Displays text on the OLED and/or LCD as specified.

## Usage

- Keypad presses are detected and displayed on the OLED and/or LCD.
- Received messages from ROS via I2C are parsed and directed to the specified display(s).