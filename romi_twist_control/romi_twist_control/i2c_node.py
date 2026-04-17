import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from .a_star import AStar

class I2CBridgeNode(Node):
    def __init__(self):
        super().__init__('i2c_bridge')
        
        # Initialize the hardware interface
        self.romi = AStar()

        # Subscribe to pwm commands (from PI node)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/wheel_pwm',
            self.pwm_callback,
            10
        )

        # Publisher for encoder ticks
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/encoder_ticks',
            10
        )

        # Read encoders at 50Hz
        self.timer = self.create_timer(0.02, self.read_encoders)
        
        # Safety: Ensure motors start at 0
        self.romi.motors(0, 0)

        self.get_logger().info("AStar I2C Interface Initialized.")

    def pwm_callback(self, msg):
        left_pwm = int(msg.data[0])
        right_pwm = int(msg.data[1])
        try:
            self.romi.motors(left_pwm, right_pwm)
        except OSError as e:
            self.get_logger().error(f"I2C PWM error: {e}")

    def read_encoders(self):
        #a_star.py unpacks using 'hh' returning 2 32-bit integers
        try:
            left_ticks, right_ticks = self.romi.read_encoders()

            msg = Int32MultiArray()
            msg.data = [int(left_ticks), int(right_ticks)]
            self.encoder_pub.publish(msg)
        except OSError as e:
            self.get_logger().error(f"I2C Encoder Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = I2CBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Stop motors when the node is killed
        node.romi.motors(0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()