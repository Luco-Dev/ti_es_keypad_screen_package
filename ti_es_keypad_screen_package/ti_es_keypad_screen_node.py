import rclpy
from rclpy.node import Node
import smbus2
import time
from std_msgs.msg import String

I2C_ADDR = 0x08  # Arduino I2C address

class I2CPublisher(Node):
    def __init__(self):
        super().__init__('i2c_publisher')
        self.publisher_ = self.create_publisher(String, '/keypad_input', 10)
        self.subscription = self.create_subscription(String, '/send_to_arduino', self.send_to_arduino, 10)
        self.i2c_bus = smbus2.SMBus(1)
        self.timer = self.create_timer(0.5, self.read_keypad)  # Read keypad every 0.5 seconds

    def send_to_arduino(self, msg):
        text = msg.data
        try:
            # Check for the specific prefix
            if text.startswith("oled: "):
                # Remove "oled: " and send the remaining text to Arduino
                message_to_send = text+'#'                # Send in batches if the message exceeds 32 bytes
                self.send_in_batches(I2C_ADDR, 0, message_to_send)

            elif text.startswith("both: "):
                # Remove "both: " and send the message to both OLED and LCD
                message_to_send = text+'#'
                message_to_check = text[6:] + '#'
                if len(message_to_check) >= 31:
                    self.send_in_batches(I2C_ADDR, 0, "lcd: Message Could not fit lcd screen#")
                    self.send_in_batches(I2C_ADDR, 0, "oled: "+ text[6:] + '#')
                else:
                    # For OLED (no limit on length), send in batches if needed
                    self.send_in_batches(I2C_ADDR, 0, message_to_send)

            elif text.startswith("lcd: "):
                # Remove "lcd: " and limit the text to 32 characters for LCD
                message_to_send = text+'#'
                message_to_check = text[6:] + '#'

                if len(message_to_check) >= 31:
                    message_to_send = "lcd: Message Could not fit screen#"
                # For OLED (no limit on length), send in batches if needed
                self.send_in_batches(I2C_ADDR, 0, message_to_send)

            else:
                # If no recognized prefix, log an error or handle as needed
                self.get_logger().warning(f'Unrecognized message format: {text}')
        
        except Exception as e:
            self.get_logger().error(f'I2C Write Error: {e}')


    def send_in_batches(self, i2c_addr, register, message):
        """
        Send the message in batches of 32 bytes over I2C to the specified register.
        """
        # Split message into chunks of 32 bytes
        chunk_size = 31
        for i in range(0, len(message), chunk_size):
            chunk = message[i:i + chunk_size]
            try:
                # Write the chunk to the I2C bus
                self.i2c_bus.write_i2c_block_data(i2c_addr, register, chunk.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino (I2C Addr: {i2c_addr}, Register: {register}): {chunk}')
            except Exception as e:
                self.get_logger().error(f'I2C Write Error while sending chunk: {e}')

        
        

    def read_keypad(self):
        try:
            key = self.i2c_bus.read_byte(I2C_ADDR)
            if key != 0:
                msg = String()
                msg.data = chr(key)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Received Key: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'I2C Read Error: {e}')

def main(args=None):
    print("Keypad - Screen - Communication is live...")
    rclpy.init(args=args)
    node = I2CPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
