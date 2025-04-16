import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

SERIAL_PORT = '/dev/ttyAMA0'  # Pi UART (GPIO14/15)
BAUD_RATE = 115200

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(String, '/ti/es/keypad_data', 10)
        self.subscription = self.create_subscription(String, '/ti/es/display_data', self.send_to_arduino, 10)
        
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Connected to ESP on {SERIAL_PORT}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            raise

        self.timer = self.create_timer(0.1, self.read_from_serial)  # Poll every 100ms

        # Menu state setup
        self.menu_options = ['Moon', 'Polestar']
        self.menu_start_options = ['GPS', 'WIJNHAVEN', "CUSTOM"]
        self.menu_target_coordinates = [
    {"type": "coords", "ra": "12;30;00", "dec": "-01;00;00"},  # Moon (example)
    {"type": "coords", "ra": "02;31;49", "dec": "+89;15;51"},  # Polestar
]
        self.menu_index = 0
        self.menu_start_index = 0
        self.menu_active = True  # True while user is selecting
        self.menu_start_active = True
        self.selection_confirmed = False
        self.selection_start_confirmed = False
        self.display_current_menu()

    def display_current_menu(self):
        current = self.menu_options[self.menu_index]
        current_start = self.menu_start_options[self.menu_start_index]
        oled_msg = f"oled: Select Target:\n> {current}\n#=Next *=Start\n Select start:\n> {current_start}\nA=Next C=Start "
        lcd_msg = f"lcd: Target: {current} start: {current_start}"
        self.send_serial_message(oled_msg)
        self.send_serial_message(lcd_msg)

    def send_to_arduino(self, msg):
        text = msg.data
        try:
            if text.startswith("oled: "):
                self.send_serial_message(text)

            elif text.startswith("both: "):
                display_text = text[6:]
                if len(display_text) > 33:
                    self.send_serial_message("lcd: Message Could not fit lcd screen")
                    self.send_serial_message("oled: " + display_text)
                else:
                    self.send_serial_message(text)

            elif text.startswith("lcd: "):
                display_text = text[6:]
                if len(display_text) > 33:
                    self.send_serial_message("lcd: Message Could not fit screen")
                else:
                    self.send_serial_message(text)

            else:
                self.get_logger().warning(f'Unrecognized message format: {text}')
        except Exception as e:
            self.get_logger().error(f'Serial Write Error: {e}')

    def send_serial_message(self, message):
        try:
            self.serial_conn.write((message + '|').encode('utf-8'))
            self.get_logger().info(f'Sent to Arduino: {message.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial Send Error: {e}')

    def send_ROS_message(self, message):
        try:
            self.get_logger().info(f'sending info to ROS')
            self.publisher_.publish(message)
        except Exception as e:
            self.get_logger().error(f'ROS Send Error: {e}')

    def read_from_serial(self):
        try:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.readline().decode('utf-8').strip()
                if data:
                    # Menu navigation logic
                    if self.menu_active:
                        if data == '#':  # Next option
                            self.menu_index = (self.menu_index + 1) % len(self.menu_options)
                            self.display_current_menu()
                            return
                        elif data == '*':  # Confirm selection
                            self.menu_active = False
                            selected = self.menu_options[self.menu_index]
                            oled_msg = f"oled: Selected: {selected}\nPress * to Start"
                            lcd_msg = f"lcd: Selected: {selected}"
                            self.send_serial_message(oled_msg)
                            self.send_serial_message(lcd_msg)
                            self.selection_confirmed = True
                            return
                    
                    elif self.selection_confirmed and data == '*':
                        selected = self.menu_options[self.menu_index]
                        oled_msg = f"oled: Starting with: {selected}"
                        lcd_msg = f"lcd: Starting: {selected}"
                        self.send_serial_message(oled_msg)
                        self.send_serial_message(lcd_msg)
                        self.selection_confirmed = False  # Reset if needed
                        # Optional: publish to other nodes or perform action here
                        return
                    

                    if self.menu_start_active:
                        if data == 'A':  # Next start option
                            self.menu_start_index = (self.menu_start_index + 1) % len(self.menu_start_options)
                            self.display_current_menu()
                            return
                        elif data == 'C':  # Confirm start selection
                            selected_target = self.menu_options[self.menu_index]
                            selected_start = self.menu_start_options[self.menu_start_index]
                            oled_msg = f"oled: Starting {selected_target} with {selected_start}"
                            lcd_msg = f"lcd: Starting {selected_target} with {selected_start}"
                            self.send_serial_message(oled_msg)
                            self.send_serial_message(lcd_msg)
                            self.selection_start_confirmed = True
                            return

                    # Normal data publishing
                    msg = String()
                    msg.data = data
                    if data == '*':
                        m = String()
                        m.data = f"{self.menu_target_coordinates[self.menu_index]}"
                        self.send_ROS_message(m)
                    self.get_logger().info(f'Received from ESP: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Serial Read Error: {e}')

def main(args=None):
    print("Serial UART (GPIO14/15) communication is live...")
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
