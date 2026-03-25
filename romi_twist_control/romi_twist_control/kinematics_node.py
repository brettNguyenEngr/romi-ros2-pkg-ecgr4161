import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from romi_interfaces.msg import WheelSpeeds

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        # 1. Subscribe to the Twist commands (from teleop)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        # 2. Publish the WheelSpeeds (to the I2C node)
        self.publisher = self.create_publisher(
            WheelSpeeds,
            '/wheel_speeds',
            10
        )
        
        # 3. Robot Physical Parameters (for the Pololu Romi)
        self.wheel_base = 0.141 # Distance between wheels in meters (14.1 cm)
        
        # 4. Conversion Factors
        # The Romi motors accept values from -300 to 300.
        # We need to map meters/second to this raw PWM value.
        # Assuming a max speed of ~0.5 m/s corresponds to 300 PWM:
        self.mps_to_pwm = 300.0 / 0.5 
        
        self.get_logger().info("Kinematics Node Initialized.")

    def twist_callback(self, msg):
        # Extract desired linear (forward/back) and angular (rotation) velocities
        v = msg.linear.x    # m/s
        omega = msg.angular.z # rad/s
        
        # Calculate individual wheel velocities in m/s
        # V_left = v - (omega * L / 2)
        # V_right = v + (omega * L / 2)
        v_left_mps = v - (omega * self.wheel_base / 2.0)
        v_right_mps = v + (omega * self.wheel_base / 2.0)
        
        # Convert m/s to raw motor PWM (-300 to 300)
        left_pwm = int(v_left_mps * self.mps_to_pwm)
        right_pwm = int(v_right_mps * self.mps_to_pwm)
        
        # Clamp the values to ensure we don't send illegal values to the 32U4
        left_pwm = max(-300, min(300, left_pwm))
        right_pwm = max(-300, min(300, right_pwm))
        
        # Create and publish the custom message
        speed_msg = WheelSpeeds()
        speed_msg.left = left_pwm
        speed_msg.right = right_pwm
        
        self.publisher.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()