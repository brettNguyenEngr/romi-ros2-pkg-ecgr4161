import rclpy
from rclpy.node import Node

# Import the custom WheelSpeeds message
from romi_interfaces.msg import WheelSpeeds

# Import Pololu's I2C helper class
from .a_star import AStar

class I2CBridgeNode(Node):
    def __init__(self):
        super().__init__('i2c_bridge')
        
        # Initialize the hardware interface
        self.romi = AStar()
        self.get_logger().info("AStar I2C Interface Initialized.")

        # Create the subscriber for the wheel speeds
        # NOTE: We are commenting out the actual subscription until we build the msg file
        self.subscription = self.create_subscription(
            WheelSpeeds,
            '/wheel_speeds',
            self.speeds_callback,
            10
        )
        
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