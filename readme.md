# I2C ROS Node for DFRobot FireBeetle 2 ESP32-E

## Overview

This ROS 2 node facilitates communication between a ROS-based system and an **ESP32 (DFRobot FireBeetle 2 ESP32-E)** over I2C. It sends display commands and retrieves keypad inputs from the ESP, which interfaces with both an OLED and an LCD.

## Setup

### Dependencies

- **ROS 2** (e.g., Foxy, Humble)
- **Python Libraries**:
  - `rclpy`
  - `smbus2`  
    > On Ubuntu, install via:  
    ```
    sudo apt install python3-smbus2
    ```

### Installation

1. Clone the package into your ROS workspace:
   ```
   cd ~/your_ros_workspace/src
   git clone <repository-url>
   ```
2. Install required Python libraries (if needed):
   ```
   pip install smbus2
   ```
3. Build the workspace:
   ```
   cd ~/your_ros_workspace
   colcon build
   ```

## Node Details

- **Node Name**: `i2c_publisher`
- **Publisher**: `/ti/es/keypad_data` (`std_msgs/String`)  
  Publishes keypresses from the ESP keypad.
- **Subscriber**: `/ti/es/display_data` (`std_msgs/String`)  
  Subscribes to messages that will be sent to the ESP for display.

## Communication

- **I2C Address**: Defaults to `0x08`
- **Sending**: Messages are prefixed with:
  - `oled:`, `lcd:`, or `both:` to control which display the message appears on.
- **Batching**: Long messages are automatically split into 32-byte chunks before being sent.
- **Receiving**: Reads single-character keypad inputs from the ESP and republishes them to the ROS topic.

## Functions

- `send_to_arduino(msg)`: Formats and sends display messages to the ESP.
- `send_in_batches(i2c_addr, register, message)`: Splits long messages into I2C-friendly chunks (≤32 bytes).
- `read_keypad()`: Reads and publishes a single key press.

## Logic Handling

- Prefixes like `"oled:"`, `"lcd:"`, and `"both:"` help the ESP decide which display(s) to use.
- The ESP truncates text beyond display limits to avoid overflow (especially for the 16x2 LCD).
- ROS handles display logic to minimize memory usage on the microcontroller.

## Usage

1. Ensure the ESP32 is flashed with the Arduino sketch and powered on.
2. Run the ROS node:
   ```
   ros2 run <package_name> i2cpublisher
   ```
3. Use ROS topics to send and receive data:
   - Publish messages to `/ti/es/display_data` to display on the ESP.
   - Subscribe to `/ti/es/keypad_data` to receive keypad input from the ESP.