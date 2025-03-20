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
            self.i2c_bus.write_i2c_block_data(I2C_ADDR, 0, text.encode('utf-8'))
            self.get_logger().info(f'Sent to Arduino: {text}')
        except Exception as e:
            self.get_logger().error(f'I2C Write Error: {e}')

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
    rclpy.init(args=args)
    node = I2CPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
