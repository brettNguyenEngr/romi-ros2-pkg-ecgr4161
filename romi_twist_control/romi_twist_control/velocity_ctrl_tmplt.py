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
        self.wheel_base = 0.141                     # Distance b/t wheels (m)
        self.wheel_circumference = math.pi * 0.07   # 70mm diameter
        self.ticks_per_rev_wheel = 12.0 * 120.0     # 12 cts/rev * 120 gear ratio = 1440 ticks
        
        # --- TUNING PARAMETERS ---
        # TODO 4: Tune these values once your math is correct!
        self.Kp = 0.0
        self.Ki = 0.0
        
        # --- STATE VARIABLES ---
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_time = self.get_clock().now()
        
        self.integral_left = 0.0
        self.integral_right = 0.0
        
        self.get_logger().info("Velocity Controller Template Initialized.")

    def twist_callback(self, msg):
        """ 
        Translates /cmd_vel (v, omega) into internal target speeds for each wheel. 
        """
        v = msg.linear.x
        omega = msg.angular.z
        
        # =========================================================================
        # TODO 1: DIFFERENTIAL DRIVE KINEMATICS
        # Game Plan: 
        # 1. Use the linear velocity (v), angular velocity (omega), and the 
        #    robot's wheel base (self.wheel_base) to calculate the target 
        #    speed for both the left and right wheels in meters per second.
        # =========================================================================
        
        self.target_left_mps = 0.0   # Replace 0.0 with your math
        self.target_right_mps = 0.0  # Replace 0.0 with your math

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
        
        # =========================================================================
        # TODO 2: TICK TO VELOCITY CONVERSION & ERROR CALCULATION
        # Game Plan:
        # 1. Convert 'delta_left' and 'delta_right' (ticks) into actual 
        #    velocities (meters per second). 
        #    Hint: You will need self.ticks_per_rev_wheel, self.wheel_circumference, and dt.
        # 2. Calculate the "error" (target speed - actual speed) for both wheels.
        # =========================================================================
        
        actual_left_mps = 0.0  # Replace with your math
        actual_right_mps = 0.0 # Replace with your math
        
        error_left = 0.0       # Replace with your math
        error_right = 0.0      # Replace with your math
        
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
        """
        Calculates the appropriate PWM output using a Proportional-Integral (PI) controller.
        """
        # =========================================================================
        # TODO 3: PI CONTROL LAW & ANTI-WINDUP
        # Game Plan:
        # 1. Update the appropriate integral accumulator (self.integral_left or 
        #    self.integral_right) by adding (error * dt).
        # 2. Calculate the raw PWM using the PI equation: (Kp * error) + (Ki * integral)
        # 3. Clamp the PWM output so it does not exceed the hardware limits 
        #    of the Romi motors (-300 to 300).
        # 4. Anti-Windup: If the PWM was clamped, "unwind" the integral by 
        #    subtracting the (error * dt) you just added to prevent it from growing 
        #    infinitely while the motors are maxed out.
        # 5. Return the final clamped PWM value.
        # =========================================================================
        
        pwm = 0.0 # Replace this entire section with your control logic
            
        return pwm

    def calculate_delta(self, current, prev):
        """ Handles the 16-bit integer rollover from the AStar board. """
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