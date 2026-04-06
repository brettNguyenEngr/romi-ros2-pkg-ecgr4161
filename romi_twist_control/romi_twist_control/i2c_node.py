import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from .a_star import AStar

# Import the custom WheelSpeeds message
# from romi_interfaces.msg import WheelSpeeds

class I2CBridgeNode(Node):
    def __init__(self):
        super().__init__('i2c_bridge')
        
        # Initialize the hardware interface
        self.romi = AStar()

        # Subscribe to pwm commands (from PI node)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'wheel_pwm',
            self.pwm_callback,
            10
        )

        # Publisher for encoder ticks
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/encoder_ticks',
            10
        ) 
        
        self.get_logger().info("AStar I2C Interface Initialized.")

        # Create the subscriber for the wheel speeds
        # NOTE: We are commenting out the actual subscription until we build the msg file
        # self.subscription = self.create_subscription(
        #     WheelSpeeds,
        #    '/wheel_speeds',
        #    self.speeds_callback,
        #    10
        #)
        
        # Safety: Ensure motors start at 0
        self.romi.motors(0, 0)

    def speeds_callback(self, msg):
        # Extract the speeds from the message
        # The 32U4 expects 16-bit integers (roughly -300 to 300 for raw PWM)
        left_speed = int(msg.left)
        right_speed = int(msg.right)
        
        try:
            # Send the speeds over I2C using Pololu's function
            self.romi.motors(left_speed, right_speed)
        except OSError as e:
            # I2C can occasionally drop packets, it's good practice to catch this
            self.get_logger().error(f"I2C Communication Error: {e}")

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