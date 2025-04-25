# UART ROS Node for DFRobot FireBeetle 2 ESP32-E

## Introduction

This ROS 2 node provides UART communication between a ROS-based system (e.g. Raspberry Pi) and the DFRobot FireBeetle 2 ESP32-E. It allows the ESP32 to send keypad input to ROS and receive display commands (for both an OLED and 16x2 LCD) from ROS.

**Note:** The ESP32 handles I2C communication internally with the displays and keypad. This ROS node only communicates via UART.

## Packages

- **ROS 2**: Compatible with Foxy, Humble, and other ROS 2 distributions.
- **Python Libraries**:
  - ```rclpy```
  - ```pyserial```
  - ```skyfield``` (for astronomy-based coordinate generation)

Install the required Python packages:

```
pip install pyserial skyfield
```

## Hardware that interacts with it

- **ESP32 (DFRobot FireBeetle 2)**
- **OLED Display (I2C)**
- **16x2 LCD Display (I2C)**
- **Keypad Matrix**
- **Serial UART (GPIO14/15)** to communicate with Raspberry Pi
- ESP32 uses I2C for internal peripheral control (not ROS-visible)

## Installation guide

1. Clone this package into your ROS 2 workspace:

```
cd ~/your_ros_workspace/src
git clone <repository-url>
```

2. Install Python dependencies:

```
pip install pyserial skyfield
```

3. Build your workspace:

```
cd ~/your_ros_workspace
colcon build
```

## Configuration options

- **Serial Port**: Defaults to ```/dev/ttyAMA0``` (Raspberry Pi UART). Edit in the code if needed.
- **Baud Rate**: ```115200```
- **Display Prefixes**: Use ```oled:``` or ```lcd:``` or ```both:``` in message strings to target specific displays.
- **Menu Logic**: Built-in menu system for selecting targets and start coordinates via keypad.

## Author

Created by **Luco Berkouwer**.  
Fork this repository to adapt it for your own robotics setup. Contributions are welcome via pull requests!

### Related ESP Code

The Arduino PlatformIO firmware that this node communicates with (handling I2C displays and keypad) is available here:  
[ti_es_keypad_screen_esp](https://github.com/Luco-Dev/ti_es_keypad_screen_esp/tree/ea5f72bb9a9d6f243bc620badf1091c040fc443f)
