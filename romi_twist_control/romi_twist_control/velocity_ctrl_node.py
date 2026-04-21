import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int16MultiArray

class VelocityCtrlNode(Node):
    def __init__(self):
        super().__init__('velocity_ctrl_node')
        
        # --- ROS 2 INTERFACES ---
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        self.pwm_pub = self.create_publisher(Int16MultiArray, '/wheel_pwm', 10)
        
        # --- ROBOT CONSTANTS ---
        self.wheel_base = 0.141 
        self.wheel_circumference = math.pi * 0.07 
        self.ticks_per_rev_wheel = 12.0 * 120.0 # 12 cts/rev * 120 gear ratio = 1440 ticks
        
        # --- TUNING PARAMETERS ---
        self.Kp = 600.0
        self.Ki = 400.0
        
        # --- STATE VARIABLES ---
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_time = self.get_clock().now()
        
        self.integral_left = 0.0
        self.integral_right = 0.0
        
        self.get_logger().info("Velocity Controller (Kinematics + PI) Initialized.")

    def twist_callback(self, msg):
        """ Translates /cmd_vel into internal target speeds. """
        v = msg.linear.x
        omega = msg.angular.z
        
        self.target_left_mps = v - (omega * self.wheel_base / 2.0)
        self.target_right_mps = v + (omega * self.wheel_base / 2.0)

    def encoder_callback(self, msg):
        """ Main control loop running at 50Hz (driven by I2C node). """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        current_left = msg.data[0]
        current_right = msg.data[1]
        
        if self.prev_left_ticks is None:
            self.prev_left_ticks = current_left
            self.prev_right_ticks = current_right
            return
            
        # Calculate tick deltas with hardware rollover protection
        delta_left = self.calculate_delta(current_left, self.prev_left_ticks)
        delta_right = self.calculate_delta(current_right, self.prev_right_ticks)
        
        self.prev_left_ticks = current_left
        self.prev_right_ticks = current_right
        
        # Calculate actual velocities
        actual_left_mps = (delta_left / self.ticks_per_rev_wheel) * self.wheel_circumference / dt
        actual_right_mps = (delta_right / self.ticks_per_rev_wheel) * self.wheel_circumference / dt
        
        # Calculate errors
        error_left = self.target_left_mps - actual_left_mps
        error_right = self.target_right_mps - actual_right_mps
        
        # Calculate PI PWM
        pwm_left = self.calculate_pi_pwm(error_left, dt, 'left')
        pwm_right = self.calculate_pi_pwm(error_right, dt, 'right')
        
        # Publish to motors
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(pwm_left), int(pwm_right)]
        self.pwm_pub.publish(pwm_msg)

        # Dashboard Logging (for PI tuning)
        self.get_logger().info(
            f"L: {self.target_left_mps:.2f} / {actual_left_mps:.2f} / {pwm_left:4.0f} | "
            f"R: {self.target_right_mps:.2f} / {actual_right_mps:.2f} / {pwm_right:4.0f}"
        )
        
    def calculate_pi_pwm(self, error, dt, side):
        if side == 'left':
            self.integral_left += error * dt
            integral = self.integral_left
        else:
            self.integral_right += error * dt
            integral = self.integral_right
            
        pwm = (self.Kp * error) + (self.Ki * integral)
        
        # Anti-windup clamping
        if pwm > 300:
            pwm = 300.0
            if side == 'left': self.integral_left -= error * dt
            else: self.integral_right -= error * dt
        elif pwm < -300:
            pwm = -300.0
            if side == 'left': self.integral_left -= error * dt
            else: self.integral_right -= error * dt
            
        return pwm

    def calculate_delta(self, current, prev):
        delta = current - prev
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()